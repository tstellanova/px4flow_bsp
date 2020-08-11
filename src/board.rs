/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use crate::peripherals::*;

use embedded_hal::blocking::delay::DelayMs;
use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use eeprom24x::Eeprom24x;
use embedded_hal::digital::v1_compat::OldOutputPin;
use l3gd20::L3gd20;
use mt9v034_i2c::{BinningFactor, Mt9v034, ParamContext};

use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::singleton;

use crate::dcmi::{DcmiWrapper, ImageFrameBuf, SQ_FRAME_BUF_LEN};
#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// The main Board support type:
/// This contains both pre-initialized drivers for
/// onboard devices as well as bus ports for external ports peripherals.
pub struct Board<'a> {
    pub activity_led: LedOutputActivity,
    pub comms_led: LedOutputComm,
    pub error_led: LedOutputError,

    pub delay_source: DelaySource,
    pub external_i2c1: I2c1BusManager,
    pub camera_config: Option<CameraConfigType<'a>>,
    pub gyro: Option<GyroType>,
    pub eeprom: Option<EepromType<'a>>,
    pub dma2: pac::DMA2,
    pub dcmi_wrap: Option<DcmiWrapper<'a>>,
    pub usart2: Usart2Port,
    pub usart3: Usart3Port,
    pub uart4: Uart4Port,
}

impl Default for Board<'_> {
    fn default() -> Self {
        #[cfg(feature = "rttdebug")]
        rprintln!("new board");

        let (
            raw_user_leds,
            mut delay_source,
            i2c1_port,
            i2c2_port,
            spi2_port,
            spi_cs_gyro,
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

        // Since any number of devices could sit on the external i2c1 port,
        //  we should treat it as a shared bus
        let i2c1_bus_mgr = shared_bus::CortexMBusManager::new(i2c1_port);

        let mut gyro_opt: Option<_> = None;
        #[cfg(not(feature = "breakout"))]
        {
            let old_gyro_csn = OldOutputPin::new(spi_cs_gyro);
            if let Ok(mut gyro) = L3gd20::new(spi2_port, old_gyro_csn) {
                if let Ok(device_id) = gyro.who_am_i() {
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

        let dma_buf0: &'static mut ImageFrameBuf =
            singleton!(:ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN]).unwrap();

        let mut dcmi_wrap = DcmiWrapper::default(dcmi);
        dcmi_wrap.setup(&dma2);

        #[cfg(feature = "breakout")]
        let base_i2c_address = mt9v034_i2c::ARDUCAM_BREAKOUT_ADDRESS;
        #[cfg(not(feature = "breakout"))]
        let base_i2c_address = mt9v034_i2c::PX4FLOW_CAM_ADDRESS;
        let mut cam_config =
            Mt9v034::new(i2c2_bus_mgr.acquire(), base_i2c_address);

        // configure image sensor with two distinct contexts:
        // - Context A: 256x256 window, binning 4 -> 64x64 output images (flow-64)
        // - Context B: 480x480 window, binning 4 -> 120x120 output images (flow-120)
        const BINNING_A: BinningFactor = BinningFactor::Four;
        const BINNING_B: BinningFactor = BinningFactor::Four;
        const WINDOW_W_A: u16 = 256;
        const WINDOW_H_A: u16 = 256;
        const WINDOW_W_B: u16 = 480;
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
                ParamContext::ContextA,
            )
            .expect("could not configure MT9V034");

        // Note that we do not call dcmi_wrap.enable_capture() here --
        // instead we allow the board user to do that if desired.

        Self {
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
        }
    }
}

impl Board<'_> {
    /// Call this on the DMA2_STREAM1 interrupt
    pub fn handle_dma2_stream1_interrupt(&mut self) {
        if let Some(dcmi_wrap) = self.dcmi_wrap.as_mut() {
            dcmi_wrap.dma2_stream1_irqhandler();
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
