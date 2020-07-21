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

## MCU Pin Map

| Pin      | Configuration |
| :--- | :--- | 
| PA0      |  UART4_TX ("TIM5_CH1" - no connection)       |
| PA1      | "TIM5_CH2" (unused)        |
| PA2      | TIM5_CH3_EXPOSURE (pulled low)   |
| PA3      | TIM5_CH4_STANDBY  (pulled low) |
| PA4      | DCMI_HSYNC       |
| PA5      | CAM_NRESET (tied to high)       |
| PA6      | DCMI_PIXCK       |
| PB6      | DCMI_D5       |
| PB7      | DCMI_VSYNC       |
| PB8      | I2C1 SCL       |
| PB9      | I2C1 SDA       |
| PB10     | I2C2 SCL       |
| PB11     | I2C2 SDA       |
| PB12      | spi_cs_gyro       |
| PB13      | SPI2 SCLK       |
| PB14      | SPI2 CIPO       |
| PB15      | SPI2 COPI       |
| PC6      | DCMI_D0       |
| PC7      | DCMI_D1       |
| PC8      | XCLK       |
| PC9      | "TIM8_CH4_LED_OUT" (unused)     |
| PC10     | DCMI_D8       |
| PC11     | UART4_RX       |
| PC12     | DCMI_D9       |
| PD0      | TBD       |
| PD5      | TBD       |
| PD6      | TBD       |
| PD7      | TBD       |
| PD15      | TBD       |
| PE0      | DCMI_D2       |
| PE1      | DCMI_D3       |
| PE2      | user_led0       |
| PE3      | user_led1       |
| PE4      | DCMI_D4       |
| PE5      | DCMI_D6       |
| PE6      | DCMI_D7       |
| PE7      | user_led2       |

