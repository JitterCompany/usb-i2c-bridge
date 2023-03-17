# USB - I2C converter

This is project converts usb serial to i2c. It is specifically built for use with avrdude and xboot configured for i2c. However the project does not know anything about avr or xboot.

- stm32f103.

## Usage

[avrdude] ---*usb serial*--- [**this converter**] ---*i2c*--- [atxmega with xboot bootloader]

Flash via i2c bootloader

```bash
avrdude -v -V -D -p atxmega64a4u -P /dev/tty.usbmodemTEST1 -c avr109 -b 115200 -U flash:w:blinky500.hex
```
