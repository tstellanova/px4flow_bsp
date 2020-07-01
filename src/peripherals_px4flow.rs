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
use stm32f4xx_hal::gpio::Speed;

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
    Spi2PortType,
    SpiGyroCsn,
    DcmiCtrlPins,
    DcmiDataPins,
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

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();


    let user_led1 = gpioe.pe2.into_push_pull_output(); //amber
    let user_led2 = gpioe.pe3.into_push_pull_output(); //blue
    let user_led3 = gpioe.pe7.into_push_pull_output(); //red

    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 1000.khz(), clocks)
    };

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

    // SPI gyro chip select
    let mut spi_cs_gyro = gpiob.pb12.into_push_pull_output();
    let _ = spi_cs_gyro.set_high();

    // DCMI control pins
    let dcmi_ctrl_pins = {
        let pixck = gpioa.pa6 // DCMI_PIXCK 
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::High) //TODO s/b 100 MHz Pullup
            .into_push_pull_output();

        let hsync = gpioa.pa4 // DCMI_HSYNC
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::High) //TODO s/b 100 MHz Pullup
            .into_push_pull_output();

        (
            pixck,
            hsync,
            gpiob.pb7.into_alternate_af13(), // DCMI_VSYNC
        )
    };

    // DCMI digital camera interface pins (AF13)
    let dcmi_data_pins = (
        gpioc.pc6.into_alternate_af13(), // DCMI_D0
        gpioc.pc7.into_alternate_af13(), // DCMI_D1
        gpioe.pe0.into_alternate_af13(), // DCMI_D2
        gpioe.pe1.into_alternate_af13(), // DCMI_D3
        gpioe.pe4.into_alternate_af13(), // DCMI_D4
        gpiob.pb6.into_alternate_af13(), // DCMI_D5
        gpioe.pe5.into_alternate_af13(), // DCMI_D6
        gpioe.pe6.into_alternate_af13(), // DCMI_D7
        gpioc.pc10.into_alternate_af13(), // DCMI_D8
        gpioc.pc12.into_alternate_af13(), // DCMI_D9
        gpiob.pb5.into_alternate_af13(), // DCMI_D10 //TODO verify -- unused?
    );


    (
        (user_led1, user_led2, user_led3),
        delay_source,
        i2c1_port,
        spi2_port,
        spi_cs_gyro,
        dcmi_ctrl_pins,
        dcmi_data_pins,
    )
}

pub type I2C1PortType = p_hal::i2c::I2c<
    I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
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
pub type SpiGyroCsn =  p_hal::gpio::gpiob::PB12<p_hal::gpio::Output<p_hal::gpio::PushPull>>;


/// The camera interface has a configurable
/// - parallel data interface from 8 to 14 data lines,
/// - a pixel clock line DCMI_PIXCLK (rising / falling edge configuration),
/// - horizontal synchronization line, DCMI_HSYNC,
/// - vertical synchronization line,  DCMI_VSYNC, with a programmable polarity.
pub type DcmiCtrlPins = (
    p_hal::gpio::gpioa::PA6<p_hal::gpio::Output<p_hal::gpio::PushPull>>, //DCMI_PIXCK
    p_hal::gpio::gpioa::PA4<p_hal::gpio::Output<p_hal::gpio::PushPull>>, //DCMI_HSYNC
    p_hal::gpio::gpiob::PB7<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, //DCMI_VSYNC
);

pub type DcmiDataPins = (
    p_hal::gpio::gpioc::PC6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioc::PC7<p_hal::gpio::Alternate< p_hal::gpio::AF13>>,
    p_hal::gpio::gpioe::PE0<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioe::PE1<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioe::PE4<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpiob::PB6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioe::PE5<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioe::PE6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioc::PC10<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpioc::PC12<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
    p_hal::gpio::gpiob::PB5<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,
);

