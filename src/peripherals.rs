/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::{DCMI, RCC};

use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::timer::CountDown;
use embedded_hal::PwmPin;
use p_hal::timer::{self, Timer};

use p_hal::gpio::{GpioExt, Output, PushPull, Speed};
use p_hal::pwm;
use p_hal::rcc::RccExt;
use p_hal::time::U32Ext;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

use shared_bus::{BusManager, BusProxy, CortexMBusManager};
use stm32f4xx_hal::timer::{PinC3, PinC4};
use stm32f4xx_hal::dwt::{Dwt,DwtExt};

/// Initialize peripherals for PX4FLOW.
/// PX4FLOW v2.3 chip is [STM32F407VGT6](https://www.mouser.com/datasheet/2/389/dm00037051-1797298.pdf)
pub fn setup_peripherals() -> (
    //  user LEDs:
    (LedOutputActivity, LedOutputComm, LedOutputError),
    DelaySource,
    Dwt,
    I2c1Port,
    I2c2Port,
    Spi2Port,
    SpiGyroCsn,
    Usart2Port,
    Usart3Port,
    Uart4Port,
    DcmiCtrlPins,
    DcmiDataPins,
    pac::DMA2,
    pac::DCMI,
) {
    let mut dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let mut rcc = dp.RCC.constrain();
    let mut clocks = rcc
        .cfgr
        .use_hse(24.mhz()) // 24 MHz xtal
        .sysclk(168.mhz()) // HCLK
        .pclk1(42.mhz()) // APB1 clock is HCLK/4
        .pclk2(84.mhz()) // APB2 clock is HCLK/2
        .freeze();

    let mut delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);
    let dwt = cp.DWT.constrain(cp.DCB, clocks);

    //enable DCMI and DMA clocks before configuring their pins
    let rcc2 = unsafe { &(*RCC::ptr()) };
    // enable peripheral clocks for DCMI and DMA2
    rcc2.ahb2enr.modify(|_, w| w.dcmien().set_bit());
    rcc2.ahb1enr.modify(|_, w| w.dma2en().set_bit());

    // let hclk = clocks.hclk();
    // let pll48clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // rprintln!("hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, pll48clk.0);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();


    //activity LED
    #[cfg(feature = "breakout")]
    let user_led0 = gpioa
        .pa1
        .into_push_pull_output()
        .set_speed(Speed::Low)
        .downgrade();
    #[cfg(not(feature = "breakout"))]
    let user_led0 = gpioe.pe2.into_push_pull_output().downgrade();

    // communications LED
    let user_led1 = gpioe.pe3.into_push_pull_output().downgrade();
    // error LED
    let user_led2 = gpioe.pe7.into_push_pull_output().downgrade();

    //i2c1 port used for eg external (offboard) communication
    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks)
    };

    // board-internal i2c2 port used for MT9V034 configuration
    // and serial EEPROM
    let i2c2_port = {
        // on the actual px4flow hw, there are external pullups on the i2c lines;
        // however, stm32f407 breakout boards do not have these, so we add an internal pullup.
        let scl = gpiob
            .pb10
            .into_alternate_af4()
            .internal_pull_up(true)
            .set_speed(Speed::Low)
            .set_open_drain(); //J2C2_SCL
        let sda = gpiob
            .pb11
            .into_alternate_af4()
            .internal_pull_up(true)
            .set_speed(Speed::Low)
            .set_open_drain(); //J2C2_SDA
        p_hal::i2c::I2c::i2c2(dp.I2C2, (scl, sda), 100.khz(), clocks)
    };

    let usart2_port = {
        //TODO usart2 has HW flow control
        let config =
            p_hal::serial::config::Config::default().baudrate(115200.bps());
        let tx = gpiod.pd5.into_alternate_af7();
        let rx = gpiod.pd6.into_alternate_af7();
        p_hal::serial::Serial::usart2(dp.USART2, (tx, rx), config, clocks)
            .unwrap()
    };

    let usart3_port = {
        //TODO usart3 has HW flow control
        let config =
            p_hal::serial::config::Config::default().baudrate(115200.bps());
        let tx = gpiod.pd8.into_alternate_af7();
        let rx = gpiod.pd9.into_alternate_af7();
        p_hal::serial::Serial::usart3(dp.USART3, (tx, rx), config, clocks)
            .unwrap()
    };

    let uart4_port = {
        let config =
            p_hal::serial::config::Config::default().baudrate(9600.bps());
        let tx = gpioa.pa0.into_alternate_af8(); // UART4_TX normally unused (no connection)
        let rx = gpioc.pc11.into_alternate_af8(); // UART4_RX
        p_hal::serial::Serial::uart4(dp.UART4, (tx, rx), config, clocks)
            .unwrap()
    };

    // used for gyro
    let spi2_port = {
        let sck = gpiob.pb13.into_alternate_af5();
        let cipo = gpiob.pb14.into_alternate_af5();
        let copi = gpiob.pb15.into_alternate_af5();

        p_hal::spi::Spi::spi2(
            dp.SPI2,
            (sck, cipo, copi),
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
        let pixck = gpioa
            .pa6 // DCMI_PIXCK
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh); //s/b 100 MHz Pullup

        let hsync = gpioa
            .pa4 // DCMI_HSYNC
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh); // s/b 100 MHz Pullup

        let vsync = gpiob
            .pb7 // DCMI_VSYNC
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh); //s/b 100 MHz Pullup

        (pixck, hsync, vsync)
    };

    // DCMI digital camera interface pins (AF13)
    // this board supports ten parallel lines D0-D9
    let dcmi_data_pins = (
        gpioc
            .pc6
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D0
        gpioc
            .pc7
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D1
        gpioe
            .pe0
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D2
        gpioe
            .pe1
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D3
        gpioe
            .pe4
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D4
        gpiob
            .pb6
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D5
        gpioe
            .pe5
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D6
        gpioe
            .pe6
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D7
        gpioc
            .pc10
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D8
        gpioc
            .pc12
            .into_pull_up_input()
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh), // DCMI_D9
    );

    let dcmi = dp.DCMI;
    let dma2 = dp.DMA2;

    //configure PA2, PA3 as EXPOSURE and STANDBY PP output lines 2MHz
    // TIM5_CH3_EXPOSURE
    let mut exposure_line =
        gpioa.pa2.into_push_pull_output().set_speed(Speed::Low);
    // TIM5_CH4_STANDBY
    let mut standby_line =
        gpioa.pa3.into_push_pull_output().set_speed(Speed::Low);
    //clear these lines:
    let _ = exposure_line.set_low();
    let _ = standby_line.set_low();
    //The sensor goes into standby mode by setting STANDBY to HIGH.
    //TODO export exposure and standby lines available on the Board struct?

    //CAM_NRESET / PA5  is unused

    // Supply an XCLK (external clock) signal to MT9V034 using PWM.
    // PX4FLOW schematic PC8 is marked TIM8_CH3_MASTERCLOCK,
    // but it actually uses TIM3 CH3
    let channels = (
        gpioc.pc8.into_alternate_af2(),
        gpioc.pc9.into_alternate_af2(), //unused
    );
    let (mut ch1, _ch2) = pwm::tim3(dp.TIM3, channels, clocks, 24u32.mhz());
    let max_duty = ch1.get_max_duty();
    let duty_avg = (max_duty / 2) + 1;

    #[cfg(feature = "rttdebug")]
    rprintln!("duty cycle: {} max: {}", duty_avg, max_duty);

    ch1.set_duty(duty_avg);
    ch1.enable();

    #[cfg(feature = "rttdebug")]
    rprintln!("TIM3 XCLK config done");

    (
        (user_led0, user_led1, user_led2),
        delay_source,
        dwt,
        i2c1_port,
        i2c2_port,
        spi2_port,
        spi_cs_gyro,
        usart2_port,
        usart3_port,
        uart4_port,
        dcmi_ctrl_pins,
        dcmi_data_pins,
        dma2,
        dcmi,
    )
}

/// I2C1 port used for external communication
pub type I2c1Port = p_hal::i2c::I2c<
    pac::I2C1,
    (
        p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
    ),
>;

/// board-internal I2C2 port used for MT9V034 configuration
/// and serial EEPROM
pub type I2c2Port = p_hal::i2c::I2c<
    pac::I2C2,
    (
        p_hal::gpio::gpiob::PB10<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
        p_hal::gpio::gpiob::PB11<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
    ),
>;

/// SPI2 port used for Gyro
pub type Spi2Port = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpiob::PB13<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpiob::PB14<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //CIPO
        p_hal::gpio::gpiob::PB15<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //COPI
    ),
>;

/// Chip select pin for Gyro (used with SPI2)
pub type SpiGyroCsn =
    p_hal::gpio::gpiob::PB12<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

/// The DCMI (camera interface) has
/// - a parallel data interface from 8 to 14 data lines,
/// - a pixel clock line DCMI_PIXCLK (rising / falling edge configuration),
/// - horizontal synchronization line, DCMI_HSYNC,
/// - vertical synchronization line,  DCMI_VSYNC, with a programmable polarity.
pub type DcmiCtrlPins = (
    p_hal::gpio::gpioa::PA6<DcmiControlPin>, //DCMI_PIXCK
    p_hal::gpio::gpioa::PA4<DcmiControlPin>, //DCMI_HSYNC
    p_hal::gpio::gpiob::PB7<DcmiControlPin>, //DCMI_VSYNC
);
pub type DcmiControlPin = p_hal::gpio::Alternate<p_hal::gpio::AF13>;

// pub type DcmiDataInnerPin = p_hal::gpio::Alternate<p_hal::gpio::AF13>;
pub type DcmiParallelDataPin = DcmiControlPin; //p_hal::gpio::Input<p_hal::gpio::PullUp>;

/// Parallel image data lines for DCMI:
/// for the PX4FLOW, only 10 are connected to the mt9v034 image sensor
pub type DcmiDataPins = (
    p_hal::gpio::gpioc::PC6<DcmiParallelDataPin>, // D0
    p_hal::gpio::gpioc::PC7<DcmiParallelDataPin>, // D1
    p_hal::gpio::gpioe::PE0<DcmiParallelDataPin>, // D2
    p_hal::gpio::gpioe::PE1<DcmiParallelDataPin>, // D3
    p_hal::gpio::gpioe::PE4<DcmiParallelDataPin>, // D4
    p_hal::gpio::gpiob::PB6<DcmiParallelDataPin>, // D5
    p_hal::gpio::gpioe::PE5<DcmiParallelDataPin>, // D6
    p_hal::gpio::gpioe::PE6<DcmiParallelDataPin>, // D7
    p_hal::gpio::gpioc::PC10<DcmiParallelDataPin>, // D8
    p_hal::gpio::gpioc::PC12<DcmiParallelDataPin>, // D9
);

pub type LedOutputPinA = p_hal::gpio::gpioa::PA<Output<PushPull>>;
pub type LedOutputPinE = p_hal::gpio::gpioe::PE<Output<PushPull>>;

#[cfg(feature = "breakout")]
pub type LedOutputActivity = LedOutputPinA; // blue on original px4flow
#[cfg(not(feature = "breakout"))]
pub type LedOutputActivity = LedOutputPinE; // blue on original px4flow
pub type LedOutputComm = LedOutputPinE; // amber on original px4flow
pub type LedOutputError = LedOutputPinE; // red on original px4flow

pub type DelaySource = p_hal::delay::Delay;

pub type UsartIoPin = p_hal::gpio::Alternate<p_hal::gpio::AF7>;

pub type Usart2Port = p_hal::serial::Serial<
    pac::USART2,
    (
        p_hal::gpio::gpiod::PD5<UsartIoPin>,
        p_hal::gpio::gpiod::PD6<UsartIoPin>,
    ),
>;
pub type Usart3Port = p_hal::serial::Serial<
    pac::USART3,
    (
        p_hal::gpio::gpiod::PD8<UsartIoPin>,
        p_hal::gpio::gpiod::PD9<UsartIoPin>,
    ),
>;

pub type UartIoPin = p_hal::gpio::Alternate<p_hal::gpio::AF8>;

pub type Uart4Port = p_hal::serial::Serial<
    pac::UART4,
    (
        p_hal::gpio::gpioa::PA0<UartIoPin>,
        p_hal::gpio::gpioc::PC11<UartIoPin>,
    ),
>;
