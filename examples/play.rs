/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::digital::{v1_compat::OldOutputPin};

const GYRO_REPORTING_RATE_HZ: u16 = 380;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use l3gd20::L3gd20;

use px4flow_bsp::peripherals;
use px4flow_bsp::peripherals_px4flow::{Spi2PortType, SpiGyroCsn};

// use peripherals::{Spi2PortType, SpiGyroCsn};


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (
        mut user_leds,
        mut delay_source,
        i2c1_port,
        // spi1_port,
        spi2_port,
        spi_cs_gyro
    ) = peripherals::setup_peripherals();

    // let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    // let spi_bus2 = shared_bus::CortexMBusManager::new(spi2_port);
    let _i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

    let old_gyro_csn = OldOutputPin::new(spi_cs_gyro);
    let mut gyro_opt: Option<GyroType> = None;
    if let Ok(mut gyro) = L3gd20::new(spi2_port, old_gyro_csn) {
        if let Ok(device_id) = gyro.who_am_i() {
            if device_id == 0xD4 {
                gyro_opt = Some(gyro)
            }
        }
    }

    delay_source.delay_ms(250u8);


    let loop_interval = GYRO_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = user_leds.0.set_high();
    let _ = user_leds.1.set_low();
    let _ = user_leds.2.set_high();

    loop {

        if gyro_opt.is_some() {
            if let Ok(sample) = gyro_opt.as_mut().unwrap().gyro() {
                rprintln!("gyro {}, {}, {}", sample.x, sample.y, sample.z);
            }
        }


        let _ = user_leds.0.toggle();
        let _ = user_leds.1.toggle();
        //let _ = user_leds.2.toggle();
    }
}


type GyroType = L3gd20<Spi2PortType, OldOutputPin<SpiGyroCsn>>;

