# px4flow_bsp

Rust no_std embedded hal board support package for the PX4FLOW optical flow sensor hardware.

## Status

Work in progress

- [x] Does not overwrite default PX4FLOW bootloader 
- [x] Simple blinky example 
- [x] Support for spi2 (gyro)
- [x] Support for i2c1 (offboard i2c communication)
- [x] Support for i2c2 (MT9V034 configuration port)
- [ ] Support for UART serial comms
- [ ] Support for DMA with DMCI camera interface (waiting on stm32f4xx-hal support)

## Notes
- The only supported mode for debugging is RTT with the `rttdebug` feature. This is because 
the PX4FLOW v2.3 only makes the SWD interface available (no easy ITM solution).
- This has been tested with the CUAV PX4FLOW v2.3. On this particular board, the 
SWD and SWCLK pads noted on the bottom of the board appear to be swapped



