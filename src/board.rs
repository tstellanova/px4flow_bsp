use crate::peripherals::*;
use embedded_hal::blocking::delay::DelayMs;

use eeprom24x::{Eeprom24x};
use embedded_hal::digital::v1_compat::OldOutputPin;
use l3gd20::L3gd20;
use mt9v034_i2c::Mt9v034;

use core::sync::atomic::{AtomicPtr, Ordering};
use cortex_m::singleton;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;




/// The main Board support type:
/// This contains both pre-initialized drivers for
/// onboard devices as well as bus ports for external ports peripherals.
pub struct Board<'a> {
    pub user_leds: [LedOutputPin; 3],
    pub external_i2c1: I2c1BusManager,
    pub camera_config: Option<CameraConfigType<'a>>,
    pub gyro: Option<GyroType>,
    pub eeprom: Option<EepromType<'a>>,
    //TODO add external UARTs
}

impl Board<'_> {
    pub fn new() -> Self {
        let (
            raw_user_leds,
            mut delay_source,
            i2c1_port,
            i2c2_port,
            spi2_port,
            spi_cs_gyro,
            _dcmi_ctrl_pins,
            _dmci_data_pins,
        ) = setup_peripherals();

        //TODO verify we are safe to forget the DCMI pins after configuration
        core::mem::forget(_dcmi_ctrl_pins);
        core::mem::forget(_dmci_data_pins);

        // Since any number of devices could sit on the external i2c1 port,
        //  we should treat it as a shared bus
        let i2c1_bus_mgr = shared_bus::CortexMBusManager::new(i2c1_port);

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
        let i2c2_bus_mgr: &'static mut I2c2BusManager =
            singleton!(:I2c2BusManager =
                shared_bus::CortexMBusManager::new(i2c2_port)
            ).unwrap();

        #[cfg(feature = "rttdebug")]
        rprintln!("eeprom setup start");
        let eeprom_i2c_address = eeprom24x::SlaveAddr::default();
        const PARAM_ADDRESS: u32 = 0x1234;

        let mut eeprom = Eeprom24x::new_24x128(i2c2_bus_mgr.acquire(), eeprom_i2c_address);
        eeprom.write_byte(PARAM_ADDRESS, 0xAA).unwrap();
        delay_source.delay_ms(5u8);

        let read_data = eeprom.read_byte(PARAM_ADDRESS).unwrap();
        #[cfg(feature = "rttdebug")]
        rprintln!("eeprom data: 0x{:X}", read_data);
        let eeprom_opt = Some(eeprom);

        #[cfg(feature = "breakout")]
            let base_i2c_address = mt9v034_i2c::ARDUCAM_BREAKOUT_ADDRESS;
        #[cfg(not(feature = "breakout"))]
            let base_i2c_address=    mt9v034_i2c::PX4FLOW_CAM_ADDRESS;

        let mut cam_config =
            Mt9v034::new(i2c2_bus_mgr.acquire(), 0x48);
        cam_config.setup().unwrap();
        let cam_opt = Some(cam_config);

        Self {
            external_i2c1: i2c1_bus_mgr,
            camera_config: cam_opt,
            gyro: gyro_opt,
            user_leds: [raw_user_leds.0, raw_user_leds.1, raw_user_leds.2],
            eeprom: eeprom_opt,
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
