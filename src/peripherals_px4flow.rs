/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use stm32f4xx_hal as p_hal;

use p_hal::stm32 as pac;
use p_hal::stm32::I2C1;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::GpioExt;
use p_hal::rcc::RccExt;
use p_hal::time::{U32Ext};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// Initialize peripherals for Pixracer.
/// Pixracer chip is [STM32F407VGT6](https://www.mouser.com/datasheet/2/389/dm00037051-1797298.pdf)
pub fn setup_peripherals() -> (
    //  user LEDs:
    (
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
        impl OutputPin + ToggleableOutputPin,
    ),
    impl DelayMs<u8>,
    I2C1PortType,
    // Spi1PortType,
    Spi2PortType,
    SpiGyroCsn,
) {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(24.mhz()) // 24 MHz xtal
        .sysclk(168.mhz()) // HCLK
        .pclk1(42.mhz()) // APB1 clock is HCLK/4
        .pclk2(84.mhz()) // APB2 clock is HCLK/2
        .freeze();

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // let hclk = clocks.hclk();
    // let pll48clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // rprintln!("hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, pll48clk.0);

    // let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    // let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();


    let user_led1 = gpioe.pe2.into_push_pull_output(); //amber
    let user_led2 = gpioe.pe3.into_push_pull_output(); //blue
    let user_led3 = gpioe.pe7.into_push_pull_output(); //red

    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks)
    };

    // let spi1_port = {
    //     let sck = gpioa.pa5.into_alternate_af5();
    //     let miso = gpioa.pa6.into_alternate_af5();
    //     let mosi = gpioa.pa7.into_alternate_af5();
    //
    //     p_hal::spi::Spi::spi1(
    //         dp.SPI1,
    //         (sck, miso, mosi),
    //         embedded_hal::spi::MODE_3,
    //         8_000_000.hz(),
    //         clocks,
    //     )
    // };

    let spi2_port = {
        let sck = gpiob.pb13.into_alternate_af5();
        let miso = gpiob.pb14.into_alternate_af5();
        let mosi = gpiob.pb15.into_alternate_af5();

        p_hal::spi::Spi::spi2(
            dp.SPI2,
            (sck, miso, mosi),
            embedded_hal::spi::MODE_3,
            1_000_000.hz(),
            clocks,
        )
    };

    // SPI chip select
    let mut spi_cs_gyro = gpiob.pb12.into_push_pull_output();
    let _ = spi_cs_gyro.set_high();

    (
        (user_led1, user_led2, user_led3),
        delay_source,
        i2c1_port,
        // spi1_port,
        spi2_port,
        spi_cs_gyro,
    )
}

pub type I2C1PortType = p_hal::i2c::I2c<
    I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
    ),
>;


pub type Spi1PortType = p_hal::spi::Spi<
    pac::SPI1,
    (
        p_hal::gpio::gpioa::PA5<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK ?  CAM_NRESET
        p_hal::gpio::gpioa::PA6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO ? DCMI_PIXCK
        p_hal::gpio::gpioa::PA7<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
>;


pub type Spi2PortType = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpiob::PB13<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpiob::PB14<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpiob::PB15<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
>;

/// chip select pin for Gyro
pub type SpiGyroCsn =    p_hal::gpio::gpiob::PB12<p_hal::gpio::Output<p_hal::gpio::PushPull>>;


