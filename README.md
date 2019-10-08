# atsamd-usb-host

[![Documentation](https://docs.rs/atsamd-usb-host/badge.svg)](https://docs.rs/atsamd-usb-host)
[![Testing](https://api.travis-ci.org/repos/bjc/atsamd-usb-host.svg?branch=master)](https://travis-ci.org/bjc/atsamd-usb-host)

This is a [usb-host](https://github.com/bjc/usb-host) driver for
Atmel's SAMD series of chips.

# Feature flags

To enable support for your MCU, you need to enable its feature. One
of: `samd21g18a`, `samd21e18a`, `samd51g19a`, `samd51j19a`,
`samd51j20a`.

So far this library has been tested on the following boards:

  - [Adafruit Trinket M0](https://www.adafruit.com/product/3500)
    (samd21e18a)
  - [Adafruit Feather M0](https://www.adafruit.com/product/2772)
    (samd21g18a)

# Examples

You can find a project utilizing this library for a USB to I²C bridge
[here](https://github.com/bjc/bleusb/tree/master/usb).
