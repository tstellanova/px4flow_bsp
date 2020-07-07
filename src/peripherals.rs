/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::{DCMI, RCC};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::timer::CountDown;
use p_hal::timer::{self, Timer};

use p_hal::gpio::{GpioExt, Output, PushPull, Speed};
use p_hal::rcc::RccExt;
use p_hal::time::U32Ext;
use embedded_hal::digital::v1_compat::OldOutputPin;
use l3gd20::L3gd20;
use mt9v034_i2c::Mt9v034;
use eeprom24x::{Eeprom24x, SlaveAddr};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

use core::borrow::BorrowMut;
use stm32f4xx_hal::timer::{PinC3, PinC4};
use shared_bus::{BusProxy, CortexMBusManager, BusManager};

use core::sync::atomic::{AtomicPtr, Ordering};
use lazy_static::lazy_static;

lazy_static! {
    /// this is how we share peripherals between multiple threads
    static ref I2C2_BUS_PTR: AtomicPtr<I2c2BusManagerType> = AtomicPtr::default();
}

/// Initialize peripherals for PX4FLOW.
/// PX4FLOW v2.3 chip is [STM32F407VGT6](https://www.mouser.com/datasheet/2/389/dm00037051-1797298.pdf)
pub fn setup_peripherals() -> (
    //  user LEDs:
    (LedOutputPin, LedOutputPin, LedOutputPin),
    impl DelayMs<u8>,
    I2c1Port,
    //I2c2Port,
    I2c2BusManagerType,
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
        let scl = gpiob.pb10.into_alternate_af4().set_speed(Speed::Low).set_open_drain();//J2C2_SCL
        let sda = gpiob.pb11.into_alternate_af4().set_speed(Speed::Low).set_open_drain();//J2C2_SDA
        p_hal::i2c::I2c::i2c2(dp.I2C2, (scl, sda), 100.khz(), clocks)
    };

    let mut i2c2_bus: I2c2BusManagerType  = shared_bus::CortexMBusManager::new(i2c2_port);

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
    let _exposure_line = gpioa
        .pa2 // TIM5_CH3_EXPOSURE
        .into_alternate_af2() // AF2 -> TIM5_CH3
        .into_push_pull_output()
        .set_speed(Speed::Low);
    let _standby_line = gpioa
        .pa3 // TIM5_CH4_STANDBY
        .into_alternate_af2() // AF2 -> TIM5_CH4
        .into_push_pull_output()
        .set_speed(Speed::Low);

    //TODO check TIM5 clock rate
    let mut tim5 = Timer::tim5(dp.TIM5, 2.mhz(), clocks);
    tim5.start(2.mhz());
    core::mem::forget(tim5);

    // Supply a clock signal to MT9V034:
    // PX4FLOW schematic is marked TIM8_CH3_MASTERCLOCK, but this is a typo:
    // actually uses TIM3 CH3
    let masterclock_line = gpioc
        .pc8
        .into_alternate_af2() // AF2 -> TIM3
        .internal_pull_up(true)
        .into_push_pull_output()
        .set_speed(Speed::VeryHigh); // 100 MHz
    core::mem::forget(masterclock_line);

    // Init TIM3 Channel3
    // NOTE(unsafe) This executes only during initialization
    unsafe {
        // TIM3 clock enable
        &(*pac::RCC::ptr())
            .apb1enr
            .modify(|_, w| w.tim3en().enabled());

        dp.TIM3.cr1.modify(|_, w| {
            w.ckd()
                .div1() // clock division
                .dir()
                .up() // count up
        });

        dp.TIM3.psc.write(|w| w.bits(0)); //prescaler
        dp.TIM3.arr.modify(|_, w| {
            w.arr().bits(3) //Auto-reload value (period)
        });

        dp.TIM3.ccer.modify(|_, w| {
            w.cc3p()
                .clear_bit() //polarity high
                .cc3e()
                .set_bit() // outputstate enable
        });

        dp.TIM3.ccmr2_output_mut().modify(|_, w| {
            w.oc3pe()
                .enabled() //output compare preload enable
                .oc3m()
                .pwm_mode1() // output compare mode pwm1
        });

        dp.TIM3.ccr3.write(|w| {
            w.bits(2) // pulse -- divide period by 2
        });

        dp.TIM3.cr1.modify(|_, w| {
            w.arpe()
                .enabled() // Auto-reload preload enable
                .cen()
                .enabled() // TIM3 counter enable
        });
    }

    #[cfg(feature = "rttdebug")]
    rprintln!("TIM3 config done");

    // configure DCMI for continuous capture
    // NOTE(unsafe) This executes only during initialization
    unsafe {
        //basic DCMI configuration
        &(*pac::DCMI::ptr()).cr.write(|w| {
            w.cm()
                .clear_bit() // capture mode: continuous
                .ess()
                .clear_bit() // synchro mode: hardware
                .pckpol()
                .clear_bit() // PCK polarity: falling
                .vspol()
                .clear_bit() // VS polarity: low
                .hspol()
                .clear_bit() // HS polarity: low
                .fcrc()
                .bits(0x00) // capture rate: every frame
                .edm()
                .bits(0x00)
        }); // extended data mode: 8 bit

        //enable clock for DCMI peripheral
        &(*pac::RCC::ptr())
            .ahb2enr
            .modify(|_, w| w.dcmien().enabled());

        //TODO verify this is how we enable capturing
        &(*pac::DCMI::ptr())
            .cr
            .write(|w| w.capture().set_bit().enable().set_bit());
    }

    unsafe {
        let mut chan1 = &(*pac::DMA2::ptr()).st[1];
        //configure DMA2, stream 1, channel 1 for DCMI
        chan1.cr.write(|w| {
            w.chsel()
                .bits(1) // ch1
                .dir()
                .peripheral_to_memory() // transferring peripheral to memory
                .pinc()
                .fixed() // do not increment peripheral
                .minc()
                .incremented() // increment memory
                // TODO psize
                // TODO msize
                .circ()
                .enabled() // enable circular mode
                .pl()
                .high() // high priority
                .mburst()
                .single() // single memory burst
                .pburst()
                .single() // single peripheral burst
        });
        chan1.fcr.write(|w| {
            w.dmdis()
                .disabled() // disable fifo mode
                .fth()
                .full() // fifo threshold full
        });

        // TODO set NDT (number of items to transfer -- number of 32 bit words)
        // chan1.ndtr.write(|w| { w
        //
        // });

        // TODO set base addresses
        // chan1.m0ar = mem0 base address
        // chan1.m1ar = mem1 base address

        //TODO wire dcmi_ctrl_pins and dcmi_data_pins to DMA:
        // DMA2: Stream1, Channel_1 -> DCMI
        // DoubleBufferMode

        //TODO enable DMA2 clock
        // &(*pac::RCC::ptr()).ahb1enr.write(|w| w.dma2en().enabled() );
    }

    (
        (user_led0, user_led1, user_led2),
        delay_source,
        i2c1_port,
        // i2c2_port,
        i2c2_bus,
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
use core::pin::Pin;

pub type Spi2Port = p_hal::spi::Spi<
    pac::SPI2,
    (
        p_hal::gpio::gpiob::PB13<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //SCLK
        p_hal::gpio::gpiob::PB14<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MISO
        p_hal::gpio::gpiob::PB15<p_hal::gpio::Alternate<p_hal::gpio::AF5>>, //MOSI
    ),
>;

/// chip select pin for Gyro
pub type SpiGyroCsn =
    p_hal::gpio::gpiob::PB12<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

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

pub type GyroType = l3gd20::L3gd20<Spi2Port, OldOutputPin<SpiGyroCsn>>;
pub type LedOutputPin = p_hal::gpio::gpioe::PE<Output<PushPull>>;


//type MutexInner = core::cell::RefCell<I2c>;
// type Mutex = cortex_m::interrupt::Mutex<MutexInner>;
// type BusManager = shared_bus::CortexMBusManager<MutexInner, I2c>;
// type BusProxy = shared_bus::proxy::BusProxy<'static, Mutex, I2c>;

pub type I2c2BusManagerType =
    shared_bus::proxy::BusManager<
        cortex_m::interrupt::Mutex<core::cell::RefCell<I2c2Port>>,
        I2c2Port>;

pub type I2c2BusProxyType<'a> =
    shared_bus::proxy::BusProxy<'a,
        cortex_m::interrupt::Mutex<core::cell::RefCell<I2c2Port>>, I2c2Port>;

pub type EepromType<'a> = eeprom24x::Eeprom24x<
    I2c2BusProxyType<'a>,
    eeprom24x::page_size::B64,
    eeprom24x::addr_size::TwoBytes>;

pub type CameraConfigType<'a> = Mt9v034<I2c2BusProxyType<'a>>;
// pub type CameraConfigType = Mt9v034<I2c2Port>;


pub struct Board<'a> {
    pub external_i2c1: I2c1Port,
    // pub internal_i2c2: I2c2BusManagerType,
    pub cam_config: Option<CameraConfigType<'a>>,
    pub gyro_opt: Option<GyroType>,
    pub user_leds: [LedOutputPin; 3],
    pub eeprom: Option<EepromType<'a>>,
}

impl Board<'static> {
    pub fn new() -> Self {
        let (
            raw_user_leds,
            mut delay_source,
            i2c1_port,
            mut i2c2_bus,
            // i2c2_port,
            spi2_port,
            spi_cs_gyro,
            _dcmi_ctrl_pins,
            _dmci_data_pins,
        ) = setup_peripherals();

        //TODO verify we are safe to forget the DCMI pins after configuration
        core::mem::forget(_dcmi_ctrl_pins);
        core::mem::forget(_dmci_data_pins);

        // TODO since any number of devices could sit on the external i2c1 port,
        //  we should treat it as a shared bus
        //let i2c1_bus = shared_bus::CortexMBusManager::new(i2c1_port);

        let old_gyro_csn = OldOutputPin::new(spi_cs_gyro);
        let mut gyro_opt: Option<_> = None;
        if let Ok(mut gyro) = L3gd20::new(spi2_port, old_gyro_csn) {
            if let Ok(device_id) = gyro.who_am_i() {
                if device_id == 0xD4 {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("gyro setup done");
                    gyro_opt = Some(gyro)
                }
            }
        }

        //store the one-and-only i2c2 bus to a static
        I2C2_BUS_PTR.store(&mut i2c2_bus, Ordering::Relaxed);

        #[cfg(feature = "rttdebug")]
        rprintln!("eeprom setup start");
        let eeprom_i2c_address = SlaveAddr::default();
        const PARAM_ADDRESS: u32 = 0x1234;

        let proxy1 = unsafe {
            I2C2_BUS_PTR.load(Ordering::SeqCst).as_mut().unwrap().acquire()
        };

        let mut eeprom = Eeprom24x::new_24x128(proxy1, eeprom_i2c_address);
        eeprom.write_byte(PARAM_ADDRESS, 0xAA).unwrap();
        delay_source.delay_ms(5u8);

        let read_data = eeprom.read_byte(PARAM_ADDRESS).unwrap();
        #[cfg(feature = "rttdebug")]
        rprintln!("eeprom data: 0x{:X}", read_data);
        let eeprom_opt = Some(eeprom);

        let proxy2 = unsafe {
            I2C2_BUS_PTR.load(Ordering::SeqCst).as_mut().unwrap().acquire()
        };
        let mut cam_config =
            Mt9v034::new(proxy2, mt9v034_i2c::DEFAULT_I2C_ADDRESS);
        cam_config.setup().unwrap();
        let cam_opt = Some(cam_config);

       Self {
            external_i2c1: i2c1_port,
            // internal_i2c2: i2c2_bus,
            cam_config: cam_opt,
            gyro_opt: gyro_opt,
            user_leds: [raw_user_leds.0, raw_user_leds.1, raw_user_leds.2],
            eeprom: eeprom_opt,
        }

    }


}
