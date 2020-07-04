/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use stm32f4xx_hal as p_hal;

use p_hal::stm32 as pac;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use p_hal::gpio::{GpioExt, Speed};
use p_hal::rcc::RccExt;
use p_hal::time::{U32Ext};
// use shared_bus::CortexMBusManager;
use embedded_hal::digital::{v1_compat::OldOutputPin};
use l3gd20::L3gd20;

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
    I2c1Port,
    I2c2Port,
    Spi2Port,
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


    let user_led0 = gpioe.pe2.into_push_pull_output(); //amber
    let user_led1 = gpioe.pe3.into_push_pull_output(); //blue
    let user_led2 = gpioe.pe7.into_push_pull_output(); //red

    //used for eg external (offboard) communication
    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks)
    };

    //used for eg MT9V034 configuration
    let i2c2_port = {
        let scl = gpiob.pb10.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb11.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c2(dp.I2C2, (scl, sda), 1000.khz(), clocks)
    };

    // used for gyro
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
            .into_pull_up_input();


        let hsync = gpioa.pa4 // DCMI_HSYNC
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::High) //TODO s/b 100 MHz Pullup
            .into_pull_up_input();

        let vsync = gpiob.pb7 // DCMI_VSYNC
            .into_alternate_af13()
            .into_pull_up_input();

        (
            pixck,
            hsync,
            vsync,
        )
    };

    // DCMI digital camera interface pins (AF13)
    // this board supports ten parallel lines D0-D9
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
    );


    (
        (user_led0, user_led1, user_led2),
        delay_source,
        i2c1_port,
        i2c2_port,
        spi2_port,
        spi_cs_gyro,
        dcmi_ctrl_pins,
        dcmi_data_pins,
    )
}

pub type I2c1Port = p_hal::i2c::I2c<
    pac::I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
    ),
>;

pub type I2c2Port = p_hal::i2c::I2c<
    pac::I2C2,
    (
        p_hal::gpio::gpiob::PB10<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB11<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
    ),
>;

pub type Spi2Port = p_hal::spi::Spi<
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
    p_hal::gpio::gpioa::PA6<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DCMI_PIXCK
    p_hal::gpio::gpioa::PA4<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DCMI_HSYNC
    p_hal::gpio::gpiob::PB7<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DCMI_VSYNC
);

pub type DcmiDataPins = (
    p_hal::gpio::gpioc::PC6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D0
    p_hal::gpio::gpioc::PC7<p_hal::gpio::Alternate< p_hal::gpio::AF13>>, // D1
    p_hal::gpio::gpioe::PE0<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D2
    p_hal::gpio::gpioe::PE1<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D3
    p_hal::gpio::gpioe::PE4<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D4
    p_hal::gpio::gpiob::PB6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D5
    p_hal::gpio::gpioe::PE5<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D6
    p_hal::gpio::gpioe::PE6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>,  // D7
    p_hal::gpio::gpioc::PC10<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D8
    p_hal::gpio::gpioc::PC12<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D9
);

pub type GyroType = l3gd20::L3gd20<Spi2Port, OldOutputPin<SpiGyroCsn>>;

// l3gd20::L3gd20<stm32f4xx_hal::spi::Spi<
//     stm32f4::stm32f407::SPI2,
//     (stm32f4xx_hal::gpio::gpiob::PB13<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>,
//     stm32f4xx_hal::gpio::gpiob::PB14<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>,
//     stm32f4xx_hal::gpio::gpiob::PB15<stm32f4xx_hal::gpio::Alternate<stm32f4xx_hal::gpio::AF5>>)>,
//
//     embedded_hal::digital::v1_compat::OldOutputPin<stm32f4xx_hal::gpio::gpiob::PB12<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>>>
//


// pub type ExternalI2c1Bus = shared_bus::proxy::BusManager<bare_metal::Mutex<core::cell::RefCell<I2c1Port>>, I2c1Port>;


pub struct Board {
    external_i2c1: I2c1Port,
    internal_i2c2: I2c2Port,
    gyro: Option<GyroType>,
}

impl Board {

    pub fn new() -> Self {

        let (
            _user_leds,
            _delay_source,
            i2c1_port,
            i2c2_port,
            spi2_port,
            spi_cs_gyro,
            _dcmi_ctrl_pins,
            _dmci_data_pins,
        ) = setup_peripherals();

        // TODO since any number of devices could sit on the external i2c1,
        // we treat it as a shared bus
        //let i2c1_bus = shared_bus::CortexMBusManager::new(i2c1_port);

        let old_gyro_csn = OldOutputPin::new(spi_cs_gyro);
        let mut gyro_opt: Option<_> = None;
        if let Ok(mut gyro) = L3gd20::new(spi2_port, old_gyro_csn) {
            if let Ok(device_id) = gyro.who_am_i() {
                if device_id == 0xD4 {
                    gyro_opt = Some(gyro)
                }
            }
        }

        Self {
            external_i2c1: i2c1_port,
            internal_i2c2: i2c2_port,
            gyro: gyro_opt,
        }
    }
}
