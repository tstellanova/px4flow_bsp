# px4flow_bsp

Rust no_std embedded hal board support package for the PX4FLOW optical flow sensor hardware.

## Usage

See the [example](./examples/play.rs) to an example that has been tested with
the PX4FLOW hardware. 

### Interrupt Handling

Currently you need to configure your application to forward interrupts from app-level
interrupt handlers, ie:

```rust
/// should be called whenever DMA2 completes a transfer
#[interrupt]
fn DMA2_STREAM1() {
    dcmi::dma2_stream1_irqhandler();
}

/// should be called whenever DCMI completes a frame
#[interrupt]
fn DCMI() {
    dcmi::dcmi_irqhandler();
}
```

This assumes you are using the [cortex-m-rt crate](https://crates.io/crates/cortex-m-rt) 
to construct your embedded application, and using its `#[interrupt]` to handle interrupts.


## Status

Work in progress

- [x] Does not overwrite the default PX4FLOW bootloader that typically ships with the board
- [x] Example that sets up DCMI to read from the camera 
- [x] Support for spi2 (l3gd20 gyro)
- [x] Support for i2c1 (offboard i2c communication)
- [x] Support for i2c2 (MT9V034 configuration port, and eeprom)
- [x] Support for USART2, USART3, and UART4 (sonar)
- [x] Support for serial eeprom on i2c2
- [x] Initial setup of DCMI peripheral
- [x] Initial setup of DMA2 
- [x] Mostly working DCMI->DMA2-> image buffer pipeline
- [ ] Support configurable / full-frame image buffers (currently limited to 64x64)
- [ ] Support use of full 10 bpp grayscale resolution of MT9V034
- [ ] Support use of 120x120 flow frame (bin 4 of 480 height)

## Notes
- The only supported mode for debugging is RTT with the `rttdebug` feature. This is because 
the PX4FLOW 1.x and 2.x boards only make the SWD interface available (no easy ITM solution).
- The `breakout` feature is intended for library development and debugging purposes.
Currently it's setup to work with the "DevEBox STM32F4XX_M Ver:3.0" board, which does not
include a l3gd20 gyro or eeprom, and eg the Arducam MT9V034 breakout board ("UC-396 RevA")
- This has been tested with the CUAV PX4FLOW v2.3. On this particular board, the 
SWD and SWCLK pads noted on the bottom of the board appear to be swapped

## MCU Pin Map

| Pin      | Configuration |
| :--- | :--- | 
| PA0      |  UART4_TX ("TIM5_CH1" - N/C)       |
| PA1      | "TIM5_CH2" (unused - N/C)        |
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

