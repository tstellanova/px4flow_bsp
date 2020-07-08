use crate::peripherals::*;
use embedded_hal::blocking::delay::DelayMs;

use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal::digital::v1_compat::OldOutputPin;
use l3gd20::L3gd20;
use mt9v034_i2c::Mt9v034;

use core::sync::atomic::{AtomicPtr, Ordering};
use lazy_static::lazy_static;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

lazy_static! {
    /// this is how we share peripherals between multiple threads
    static ref I2C2_BUS_PTR: AtomicPtr<I2c2BusManagerType> = AtomicPtr::default();
}

/// Concrete type for gyro driver
pub type GyroType = l3gd20::L3gd20<Spi2Port, OldOutputPin<SpiGyroCsn>>;

/// Concrete type for serial EEPROM driver
pub type EepromType<'a> = eeprom24x::Eeprom24x<
    I2c2BusProxyType<'a>,
    eeprom24x::page_size::B64,
    eeprom24x::addr_size::TwoBytes,
>;

/// Concrete type for camera configuration driver
pub type CameraConfigType<'a> = Mt9v034<I2c2BusProxyType<'a>>;

pub struct Board<'a> {
    pub external_i2c1: I2c1Port,
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



        let proxy2 = unsafe {
            I2C2_BUS_PTR
                .load(Ordering::SeqCst)
                .as_mut()
                .unwrap()
                .acquire()
        };
        let mut cam_config =
            Mt9v034::new(proxy2, mt9v034_i2c::DEFAULT_I2C_ADDRESS);
        //cam_config.setup().unwrap();
        let cam_opt = Some(cam_config);


        #[cfg(feature = "rttdebug")]
        rprintln!("eeprom setup start");
        let eeprom_i2c_address = SlaveAddr::default();
        const PARAM_ADDRESS: u32 = 0x1234;

        let proxy1 = unsafe {
            I2C2_BUS_PTR
                .load(Ordering::SeqCst)
                .as_mut()
                .unwrap()
                .acquire()
        };
        let mut eeprom = Eeprom24x::new_24x128(proxy1, eeprom_i2c_address);
        // eeprom.write_byte(PARAM_ADDRESS, 0xAA).unwrap();
        // delay_source.delay_ms(5u8);
        //
        // let read_data = eeprom.read_byte(PARAM_ADDRESS).unwrap();
        #[cfg(feature = "rttdebug")]
        rprintln!("eeprom data: 0x{:X}", read_data);
        let eeprom_opt = Some(eeprom);

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
