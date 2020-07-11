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

use p_hal::pwm;
use p_hal::gpio::{GpioExt, Output, PushPull, Speed};
use p_hal::rcc::RccExt;
use p_hal::time::U32Ext;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

use shared_bus::{BusManager, BusProxy, CortexMBusManager};
use stm32f4xx_hal::timer::{PinC3, PinC4};

/// Initialize peripherals for PX4FLOW.
/// PX4FLOW v2.3 chip is [STM32F407VGT6](https://www.mouser.com/datasheet/2/389/dm00037051-1797298.pdf)
pub fn setup_peripherals() -> (
    //  user LEDs:
    (LedOutputPin, LedOutputPin, LedOutputPin),
    DelaySource,
    I2c1Port,
    I2c2Port,
    Spi2Port,
    SpiGyroCsn,
    DcmiCtrlPins,
    DcmiDataPins,
) {
    let mut dp = pac::Peripherals::take().unwrap();
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

    let mut delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // let hclk = clocks.hclk();
    // let pll48clk = clocks.pll48clk().unwrap_or(0u32.hz());
    // let pclk1 = clocks.pclk1();
    // rprintln!("hclk: {} /16: {} pclk1: {} rng_clk: {}", hclk.0, hclk.0 / 16, pclk1.0, pll48clk.0);

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    let user_led0 = gpioe.pe2.into_push_pull_output().downgrade(); //amber
    let user_led1 = gpioe.pe3.into_push_pull_output().downgrade(); //blue
    let user_led2 = gpioe.pe7.into_push_pull_output().downgrade(); //red

    //i2c1 port used for eg external (offboard) communication
    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks)
    };

    // board-internal i2c2 port used for MT9V034 configuration
    // and serial EEPROM
    let i2c2_port = {
        //TODO on discovery board we need pullup
        let scl = gpiob.pb10.into_alternate_af4()
            .internal_pull_up(true)
            .set_speed(Speed::Low)
            .set_open_drain(); //J2C2_SCL
        let sda = gpiob.pb11.into_alternate_af4()
            .internal_pull_up(true)
            .set_speed(Speed::Low)
            .set_open_drain(); //J2C2_SDA
        p_hal::i2c::I2c::i2c2(dp.I2C2, (scl, sda), 100.khz(), clocks)
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
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh) // 100 MHz Pullup
            .into_pull_up_input();

        let hsync = gpioa
            .pa4 // DCMI_HSYNC
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh) // s/b 100 MHz Pullup
            .into_pull_up_input();

        let vsync = gpiob
            .pb7 // DCMI_VSYNC
            .into_alternate_af13()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh) //s/b 100 MHz Pullup
            .into_pull_up_input();

        (pixck, hsync, vsync)
    };

    // DCMI digital camera interface pins (AF13)
    // this board supports ten parallel lines D0-D9
    let dcmi_data_pins = (
        gpioc.pc6.into_alternate_af13(),  // DCMI_D0
        gpioc.pc7.into_alternate_af13(),  // DCMI_D1
        gpioe.pe0.into_alternate_af13(),  // DCMI_D2
        gpioe.pe1.into_alternate_af13(),  // DCMI_D3
        gpioe.pe4.into_alternate_af13(),  // DCMI_D4
        gpiob.pb6.into_alternate_af13(),  // DCMI_D5
        gpioe.pe5.into_alternate_af13(),  // DCMI_D6
        gpioe.pe6.into_alternate_af13(),  // DCMI_D7
        gpioc.pc10.into_alternate_af13(), // DCMI_D8
        gpioc.pc12.into_alternate_af13(), // DCMI_D9
    );

    //configure PA2, PA3 as EXPOSURE and STANDBY PP output lines 2MHz
    let mut exposure_line = gpioa
        .pa2 // TIM5_CH3_EXPOSURE
        .into_alternate_af2() // AF2 -> TIM5_CH3
        .into_push_pull_output()
        .set_speed(Speed::Low);
    let mut standby_line = gpioa
        .pa3 // TIM5_CH4_STANDBY
        .into_alternate_af2() // AF2 -> TIM5_CH4
        .into_push_pull_output()
        .set_speed(Speed::Low);
    //clear these lines
    let _ = exposure_line.set_low();
    let _ = standby_line.set_low();
    //The sensor goes into standby mode by setting STANDBY to HIGH.

    // let mut cam_nreset_line = gpioa
    //     .pa5 // CAM_NRESET
    //     .into_push_pull_output()
    //     .set_speed(Speed::Low);
    // let _  = cam_nreset_line.set_high();
    // delay_source.delay_us(2u8);
    // let _ = cam_nreset_line.set_low();
    // delay_source.delay_us(2u8);
    // let _ = cam_nreset_line.set_high();
    // delay_source.delay_us(2u8);
    // let _ = cam_nreset_line.set_low();

    //TODO check TIM5 clock rate
    let mut tim5 = Timer::tim5(dp.TIM5, 2.mhz(), clocks);
    tim5.start(2.mhz());
    core::mem::forget(tim5);

    // Supply an XLCK clock signal to MT9V034:
    // PX4FLOW schematic PC8 is marked TIM8_CH3_MASTERCLOCK, but this is a typo:
    // it actually uses TIM3 CH3

    let channels = (
        gpioc.pc8.into_alternate_af2(),
        gpioc.pc9.into_alternate_af2(), //unused
    );
    let pwm = pwm::tim3(dp.TIM3, channels, clocks, 24u32.mhz());
    let (mut ch1, _ch2) = pwm;
    let max_duty = ch1.get_max_duty();
    let duty_avg = (max_duty / 2) + 1;

    #[cfg(feature = "rttdebug")]
    rprintln!("duty cycle: {} max: {}", duty_avg, max_duty);

    ch1.set_duty(duty_avg);
    ch1.enable();
    core::mem::forget(ch1);//free running forevermore

    #[cfg(feature = "rttdebug")]
    rprintln!("TIM3 XCLK config done");

    // TODO configure DCMI peripheral for continuous capture
    // NOTE(unsafe) This executes only during initialization
    // unsafe {
    //     //basic DCMI configuration
    //     &(*pac::DCMI::ptr()).cr.write(|w| {
    //         w.cm()
    //             .clear_bit() // capture mode: continuous
    //             .ess()
    //             .clear_bit() // synchro mode: hardware
    //             .pckpol()
    //             .clear_bit() // PCK polarity: falling
    //             .vspol()
    //             .clear_bit() // VS polarity: low
    //             .hspol()
    //             .clear_bit() // HS polarity: low
    //             .fcrc()
    //             .bits(0x00) // capture rate: every frame
    //             .edm()
    //             .bits(0x00)
    //     }); // extended data mode: 8 bit
    //
    //     //enable clock for DCMI peripheral
    //     &(*pac::RCC::ptr())
    //         .ahb2enr
    //         .modify(|_, w| w.dcmien().enabled());
    //
    //     //TODO verify this is how we enable capturing
    //     &(*pac::DCMI::ptr())
    //         .cr
    //         .write(|w| w.capture().set_bit().enable().set_bit());
    // }

    // unsafe {
    //     let mut chan1 = &(*pac::DMA2::ptr()).st[1];
    //     //configure DMA2, stream 1, channel 1 for DCMI
    //     chan1.cr.write(|w| {
    //         w.chsel()
    //             .bits(1) // ch1
    //             .dir()
    //             .peripheral_to_memory() // transferring peripheral to memory
    //             .pinc()
    //             .fixed() // do not increment peripheral
    //             .minc()
    //             .incremented() // increment memory
    //             // TODO psize
    //             // TODO msize
    //             .circ()
    //             .enabled() // enable circular mode
    //             .pl()
    //             .high() // high priority
    //             .mburst()
    //             .single() // single memory burst
    //             .pburst()
    //             .single() // single peripheral burst
    //     });
    //     chan1.fcr.write(|w| {
    //         w.dmdis()
    //             .disabled() // disable fifo mode
    //             .fth()
    //             .full() // fifo threshold full
    //     });
    //
    //     // TODO set NDT (number of items to transfer -- number of 32 bit words)
    //     // chan1.ndtr.write(|w| { w
    //     //
    //     // });
    //
    //     // TODO set base addresses
    //     // chan1.m0ar = mem0 base address
    //     // chan1.m1ar = mem1 base address
    //
    //     //TODO wire dcmi_ctrl_pins and dcmi_data_pins to DMA:
    //     // DMA2: Stream1, Channel_1 -> DCMI
    //     // DoubleBufferMode
    //
    //     //TODO enable DMA2 clock
    //     // &(*pac::RCC::ptr()).ahb1enr.write(|w| w.dma2en().enabled() );
    // }

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
    p_hal::gpio::gpioa::PA6<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DCMI_PIXCK
    p_hal::gpio::gpioa::PA4<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DCMI_HSYNC
    p_hal::gpio::gpiob::PB7<p_hal::gpio::Input<p_hal::gpio::PullUp>>, //DCMI_VSYNC
);

/// Parallel image data lines for DCMI
pub type DcmiDataPins = (
    p_hal::gpio::gpioc::PC6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D0
    p_hal::gpio::gpioc::PC7<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D1
    p_hal::gpio::gpioe::PE0<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D2
    p_hal::gpio::gpioe::PE1<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D3
    p_hal::gpio::gpioe::PE4<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D4
    p_hal::gpio::gpiob::PB6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D5
    p_hal::gpio::gpioe::PE5<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D6
    p_hal::gpio::gpioe::PE6<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D7
    p_hal::gpio::gpioc::PC10<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D8
    p_hal::gpio::gpioc::PC12<p_hal::gpio::Alternate<p_hal::gpio::AF13>>, // D9
);

pub type LedOutputPin = p_hal::gpio::gpioe::PE<Output<PushPull>>;
pub type DelaySource = p_hal::delay::Delay;


