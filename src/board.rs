/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use crate::peripherals::*;

use embedded_hal::blocking::delay::DelayMs;
use stm32f4xx_hal as p_hal;
use p_hal::dwt::Dwt;
use p_hal::stm32 as pac;

use eeprom24x::Eeprom24x;
use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::OutputPin;
use l3gd20::L3gd20;
use mt9v034_i2c::{BinningFactor, Mt9v034, ParamContext};

use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::singleton;

use crate::dcmi::{DcmiWrapper, SQ_DIM_120, SQ_DIM_64};
#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use stm32f4xx_hal::dwt::StopWatch;

/// The main Board support type:
/// This contains both pre-initialized drivers for
/// onboard devices as well as bus ports for external ports peripherals.
pub struct Board<'a> {
    pub activity_led: LedOutputActivity,
    pub comms_led: LedOutputComm,
    pub error_led: LedOutputError,

    pub delay_source: DelaySource,
    pub dwt: Dwt,
    pub external_i2c1: I2c1BusManager,
    pub camera_config: Option<CameraConfigType<'a>>,
    pub gyro: Option<GyroType>,
    pub eeprom: Option<EepromType<'a>>,
    pub dma2: pac::DMA2,
    pub dcmi_wrap: Option<DcmiWrapper<'a>>,
    pub usart2: Usart2Port,
    pub usart3: Usart3Port,
    pub uart4: Uart4Port,

    stopwatch: StopWatch<'a>,
}

impl Default for Board<'_> {
    fn default() -> Self {
        #[cfg(feature = "rttdebug")]
        rprintln!("new board");

        let (
            raw_user_leds,
            mut delay_source,
            dwt,
            i2c1_port,
            i2c2_port,
            spi2_port,
            mut spi_cs_gyro,
            usart2,
            usart3,
            uart4,
            dcmi_ctrl_pins,
            dcmi_data_pins,
            dma2,
            dcmi,
        ) = setup_peripherals();

        //We are safe to forget the DCMI pins after configuration
        core::mem::forget(dcmi_ctrl_pins);
        core::mem::forget(dcmi_data_pins);


        static mut STOPWATCH_BUF: [u32; 2] = [0u32; 2];
        let mut stopwatch = dwt.stopwatch(unsafe { &mut STOPWATCH_BUF });

        // Since any number of devices could sit on the external i2c1 port,
        //  we should treat it as a shared bus
        let i2c1_bus_mgr = shared_bus::CortexMBusManager::new(i2c1_port);

        let mut gyro_opt: Option<_> = None;
        #[cfg(not(feature = "breakout"))]
        {
            let _ = spi_cs_gyro.set_high(); // initially deselected
            let old_gyro_csn = OldOutputPin::new(spi_cs_gyro);

            if let Ok(mut gyro) = L3gd20::new(spi2_port, old_gyro_csn) {
                if let Ok(device_id) = gyro.who_am_i() {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("gyro ID: 0x{:X}", device_id);
                    if device_id == 0xD4 {
                        #[cfg(feature = "rttdebug")]
                        rprintln!("gyro setup done");
                        gyro_opt = Some(gyro)
                    }
                }
            }
        }

        //store the one-and-only i2c2 bus to a static
        let i2c2_bus_mgr: &'static mut I2c2BusManager =
            singleton!(:I2c2BusManager =
                shared_bus::CortexMBusManager::new(i2c2_port)
            )
            .unwrap();

        let mut eeprom_opt = None;
        #[cfg(not(feature = "breakout"))]
        {
            #[cfg(feature = "rttdebug")]
            rprintln!("eeprom setup start");
            let eeprom_i2c_address = eeprom24x::SlaveAddr::default();
            const PARAM_ADDRESS: u32 = 0x1234;

            let mut eeprom = Eeprom24x::new_24x128(
                i2c2_bus_mgr.acquire(),
                eeprom_i2c_address,
            );
            eeprom.write_byte(PARAM_ADDRESS, 0xAA).unwrap();
            delay_source.delay_ms(5u8);

            let read_data = eeprom.read_byte(PARAM_ADDRESS).unwrap();
            #[cfg(feature = "rttdebug")]
            rprintln!("eeprom data: 0x{:X}", read_data);
            let eeprom_opt = Some(eeprom);
        }

        // option A: select max aspect ration with row and column bin 4 (188x120)
        let mut dcmi_wrap = DcmiWrapper::default(dcmi);
        // option B: square-120 images
        // let mut dcmi_wrap = DcmiWrapper::new(dcmi, SQ_DIM_120, SQ_DIM_120, 8);

        dcmi_wrap.setup(&dma2);

        #[cfg(feature = "breakout")]
        let base_i2c_address = mt9v034_i2c::ARDUCAM_BREAKOUT_ADDRESS;
        #[cfg(not(feature = "breakout"))]
        let base_i2c_address = mt9v034_i2c::PX4FLOW_CAM_ADDRESS;
        let mut cam_config =
            Mt9v034::new(i2c2_bus_mgr.acquire(), base_i2c_address);

        // configure image sensor with two distinct contexts:
        // - Context A: 480x480 window, binning 4 -> 120x120 output images (square-120)
        // - Context B: 752x480 window, binning 4 -> 188x120 output images
        const BINNING_A: BinningFactor = BinningFactor::Four;
        const BINNING_B: BinningFactor = BinningFactor::Four;
        const WINDOW_W_A: u16 = 480;
        const WINDOW_H_A: u16 = 480;
        const WINDOW_W_B: u16 = 752;
        const WINDOW_H_B: u16 = 480;


        cam_config
            .setup_with_dimensions(
                WINDOW_W_A,
                WINDOW_H_A,
                BINNING_A,
                BINNING_A,
                WINDOW_W_B,
                WINDOW_H_B,
                BINNING_B,
                BINNING_B,
                ParamContext::ContextB,
            )
            .expect("could not configure MT9V034");

        // Note that we do not call dcmi_wrap.enable_capture() here --
        // instead we allow the board user to do that if desired.

        let mut result = Self {
            activity_led: raw_user_leds.0,
            comms_led: raw_user_leds.1,
            error_led: raw_user_leds.2,
            external_i2c1: i2c1_bus_mgr,
            camera_config: Some(cam_config),
            gyro: gyro_opt,
            delay_source,
            eeprom: eeprom_opt,
            dma2: dma2,
            dcmi_wrap: Some(dcmi_wrap),
            usart2,
            usart3,
            uart4,
            dwt,
            stopwatch
        };

        result
    }
}


impl Board<'_> {
    /// Call this on the DMA2_STREAM1 interrupt
    pub fn handle_dma2_stream1_interrupt(&mut self) {
        if let Some(dcmi_wrap) = self.dcmi_wrap.as_mut() {
            dcmi_wrap.dma2_stream1_irqhandler();

            #[cfg(feature = "rttdebug")]
            {
                self.stopwatch.lap();
                if let Some(period) = self.stopwatch.lap_time(1) {
                    let unread = dcmi_wrap.unread_frames();
                    let period_secs = period.as_secs_f32();
                    // if (period_secs > 0.03) || (unread > 1) {
                    //     rprintln!("dma {:.5} {}", period.as_secs_f32(), dcmi_wrap.unread_frames());
                    // }
                }
                self.stopwatch.reset();
            }
        }
    }

    /// Call this on the DCMI interrupt
    pub fn handle_dcmi_interrupt(&mut self) {
        if let Some(dcmi_wrap) = self.dcmi_wrap.as_mut() {
            dcmi_wrap.dcmi_irqhandler();
        }
    }
}

pub type BusManager<Port> = shared_bus::proxy::BusManager<
    cortex_m::interrupt::Mutex<core::cell::RefCell<Port>>,
    Port,
>;

pub type BusProxy<'a, Port> = shared_bus::proxy::BusProxy<
    'a,
    cortex_m::interrupt::Mutex<core::cell::RefCell<Port>>,
    Port,
>;

pub type I2c1BusManager = BusManager<I2c1Port>;
pub type I2c1BusProxy<'a> = BusProxy<'a, I2c1Port>;

pub type I2c2BusManager = BusManager<I2c2Port>;
pub type I2c2BusProxy<'a> = BusProxy<'a, I2c2Port>;

/// Concrete type for gyro driver
pub type GyroType = l3gd20::L3gd20<Spi2Port, OldOutputPin<SpiGyroCsn>>;

/// Concrete type for serial EEPROM driver
pub type EepromType<'a> = eeprom24x::Eeprom24x<
    I2c2BusProxy<'a>,
    eeprom24x::page_size::B64,
    eeprom24x::addr_size::TwoBytes,
>;

/// Concrete type for camera configuration driver
pub type CameraConfigType<'a> = Mt9v034<I2c2BusProxy<'a>>;


