//! CDC-ACM serial port example using cortex-m-rtic combined with i2c
//! Target board: Blue Pill
#![no_main]
#![no_std]

use cortex_m::asm::delay;
use heapless::Vec;
use panic_rtt_target as _;
use rtic;
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};
use stm32f1xx_hal::gpio::PinState;
use stm32f1xx_hal::gpio::{gpioc::PC13, Alternate, OpenDrain, Output, Pin, PushPull};
use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode};
use stm32f1xx_hal::pac::I2C1;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{device::TIM1, timer::Counter};
use systick_monotonic::fugit::Instant;
use systick_monotonic::{fugit::Duration, Systick};
use usb_device::prelude::*;

const SENSOR_BOOTLOADER_I2C_ADDRESS: u8 = 0x42;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        i2c_buffer: Vec<u8, 512>,
        timer: Counter<TIM1, 1000>,
        last_read: Option<Instant<u32, 1, 1000>>,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,

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

    fn init_rtt() {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 4096
                    mode: NoBlockSkip
                    name: "Print Output"
                }
            }
            down: {
                0: {
                    size: 80
                    mode: NoBlockSkip
                    name: "Command input"
                }
            }
        };

        let output = channels.up.0;
        set_print_channel(output);
    }

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        init_rtt();
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

        let mut timer = cx.device.TIM1.counter_ms(&clocks);
        timer.start(10.secs()).unwrap();

        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (
            Shared {
                usb_dev,
                serial,
                i2c_buffer: Vec::new(),
                timer,
                last_read: None,
            },
            Local { led, i2c },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [i2c], shared = [i2c_buffer, serial, timer, last_read])]
    fn idle(cx: idle::Context) -> ! {
        let i2c = cx.local.i2c;

        let mut buffer = cx.shared.i2c_buffer;
        let mut serial = cx.shared.serial;
        let mut timer = cx.shared.timer;
        let mut last_read = cx.shared.last_read;

        loop {
            let do_i2c_stuff = (&mut timer, &mut last_read).lock(|timer, last_read| {
                let timer_expired = if let Some(last_read) = last_read {
                    let duration = timer.now().checked_duration_since(*last_read);

                    if let Some(time) = duration {
                        // we wait until we didn't get any uart message for 2 ms until
                        // we bundle it and send it over i2c
                        time.to_millis() > 2
                    } else {
                        false
                    }
                } else {
                    false
                };

                if timer_expired {
                    last_read.take();
                }
                timer_expired
            });

            if do_i2c_stuff {
                buffer.lock(|buffer| {
                    rprintln!("send i2c {:X?}", buffer.as_slice());
                    match i2c.write(SENSOR_BOOTLOADER_I2C_ADDRESS, buffer.as_slice()) {
                        Ok(_) => {}
                        Err(err) => {
                            rprintln!("Error writing i2c: {:?}", err);
                        }
                    }
                    buffer.clear();

                    let mut read_buff = [0u8; 256];
                    // We'd like to read until NACK. But the HAL does not really expose that.
                    match i2c.read(SENSOR_BOOTLOADER_I2C_ADDRESS, &mut read_buff) {
                        Ok(_) => {
                            for byte in read_buff {
                                if byte < 255 {
                                    rprintln!("read i2c: {:X?}", byte);

                                    serial.lock(|ser| {
                                        ser.write(&[byte]).ok();
                                    })
                                }
                            }
                        }
                        Err(err) => {
                            rprintln!("Error i2c read {:?}", err);
                        }
                    }
                });
                blink::spawn().unwrap();
            }

            core::hint::spin_loop();
        }
    }

    #[task(local = [led])]
    fn blink(cx: blink::Context) {
        cx.local.led.toggle();
    }

    #[task(binds = USB_HP_CAN_TX, shared = [usb_dev, serial])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            super::usb_poll(usb_dev, serial);
        });
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_dev, serial, i2c_buffer, timer, last_read])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let timer = cx.shared.timer;
        let last_read = cx.shared.last_read;

        let mut buffer = cx.shared.i2c_buffer;

        (&mut usb_dev, &mut serial, &mut buffer).lock(|usb_dev, serial, buffer| {
            if super::usb_poll(usb_dev, serial) {
                let bytes_read = super::usb_read(serial, buffer);
                if bytes_read == true {
                    (timer, last_read).lock(|timer, last_read| last_read.replace(timer.now()));
                }
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
    serial: &mut usbd_serial::SerialPort<'static, B>,
    buffer: &mut Vec<u8, 512>,
) -> bool {
    let mut buf = [0u8; 64];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            buffer.extend_from_slice(&buf[0..count]).ok();
            true
        }
        _ => false,
    }
}
