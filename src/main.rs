//! CDC-ACM serial port example using cortex-m-rtic.
//! Target board: Blue Pill
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use rtic;
use cortex_m::asm::delay;
use heapless::spsc;
use heapless::spsc::{Producer, Queue};
use heapless::Vec;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::gpio::PinState;
use stm32f1xx_hal::gpio::{gpioc::PC13, OpenDrain, Output, PushPull, Alternate, Pin};
use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use systick_monotonic::{fugit::Duration, Systick};
use usb_device::prelude::*;
use stm32f1xx_hal::pac::I2C1;

const SENSOR_BOOTLOADER_I2C_ADDRESS: u8 = 0x42;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        i2c_buffer: Vec<u8, 512>,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        state: bool,

        prod: spsc::Producer<'static, u8, 512>,
        cons: spsc::Consumer<'static, u8, 512>,

        i2c: BlockingI2c<
            I2C1,
            (
                Pin<'B', 6, Alternate<OpenDrain>>,
                Pin<'B', 7, Alternate<OpenDrain>>,
            ),
        >,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init(local = [
        queue: Queue<u8, 512> = Queue::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        rtt_init_print!();
        rprintln!("init");

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();

        let mut gpioc = cx.device.GPIOC.split();
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::Low);

        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let mut afio = cx.device.AFIO.constrain();

        let i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400.kHz(),
                duty_cycle: DutyCycle::Ratio16to9,
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        let mono = Systick::new(cx.core.SYST, 36_000_000);

        let (prod, cons) = cx.local.queue.split();
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (
            Shared {
                usb_dev,
                serial,
                i2c_buffer: Vec::new(),
            },
            Local {
                led,
                state: false,
                prod,
                cons,
                i2c,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [cons, i2c], shared = [i2c_buffer, serial])]
    fn idle(cx: idle::Context) -> ! {
        let queue = cx.local.cons;
        let mut buffer = cx.shared.i2c_buffer;

        let mut i2c = cx.local.i2c;
        let mut serial = cx.shared.serial;

        loop {
            buffer.lock(|b| {
                if b.len() > 0 {
                    rprintln!("vec {:?}", b.as_slice());
                    i2c.write(SENSOR_BOOTLOADER_I2C_ADDRESS, b.as_slice());
                    b.clear();
                    let mut read_buff = [0u8; 1];

                    match i2c.read(SENSOR_BOOTLOADER_I2C_ADDRESS, &mut read_buff) {
                        Ok(_) => {
                            if read_buff[0] < 255 {
                                rprintln!("send back: {:?}", read_buff[0] as char);
                                serial.lock(|ser| {
                                    ser.write(&read_buff).ok();
                                })
                            }

                        },
                        Err(err) => {
                            rprintln!("Error i2c read {:?}", err);
                        },
                    }
                }
            });


            if queue.len() > 0 {
                while let Some(byte) = queue.dequeue() {
                    rprintln!("CMD {:?}", byte as char);
                }
            }

            core::hint::spin_loop();
        }
    }

    #[task(local = [led, state])]
    fn blink(cx: blink::Context) {
        // rprintln!("blink");
        if *cx.local.state {
            cx.local.led.set_high();
            *cx.local.state = false;
        } else {
            cx.local.led.set_low();
            *cx.local.state = true;
        }
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial, i2c_buffer], local = [prod])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        let producer = cx.local.prod;
        let mut buffer = cx.shared.i2c_buffer;

        (&mut usb_dev, &mut serial, &mut buffer).lock(|usb_dev, serial, buffer| {
            if super::usb_poll(usb_dev, serial) {
                super::usb_read(usb_dev, serial, producer, buffer);
            }
        });
    }
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
) -> bool {
    usb_dev.poll(&mut [serial])
}

fn usb_read<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
    queue: &mut Producer<u8, 512>,
    buffer: &mut Vec<u8, 512>,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 64];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            buffer.extend_from_slice(&buf[0..count]);
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                queue.enqueue(*c);

                // if 0x61 <= *c && *c <= 0x7a {
                //     *c &= !0x20;
                // }
            }

            // serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}
