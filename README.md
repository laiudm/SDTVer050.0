# T41_SDR

Forked from T41-EP 20W SSB and CW SDR Transceiver software developed by Albert Peter and Jack Purdum, version SDTVer050.0, dated 4 July 2024 and downloaded from https://groups.io/g/SoftwareControlledHamRadio/files/SDTVer050.0.zip

Source code changes by M0JTS are intended to support his hardware/software variant:
* si5351 directly generates quadrature signals
* touch screen control
* 192kbps usb audio support

This is a work in progress. Use at your own risk.

## First Update:

- Added a very simple serial debug command interface. It's driven by timer interrupts so it will work even when the main loop() code "gets stuck".
- Added mock interfaces for the buttons and the rotary encoders. I don't want lots of hardware in my test setup at this early stage.
- Added debug code to track why receive audio wasn't working. Finally tracked down the problem to the digital volume gain being far too low. I've increased it x100 & it now works.

