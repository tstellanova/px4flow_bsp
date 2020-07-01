/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rprint, rtt_init_print};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::PwmPin;

const GYRO_REPORTING_RATE_HZ: u16 = 200;
const GYRO_REPORTING_INTERVAL_MS: u16 = (1000 / GYRO_REPORTING_RATE_HZ);

use l3gd20::L3gd20;

use px4flow::peripherals;
use core::cmp::max;

use peripherals::{Spi2PortType, SpiCsGyro};


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (
        mut user_leds,
        mut delay_source,
        mut rng,
        i2c1_port,
        spi1_port,
        spi2_port,
        spi_cs_gyro
    ) = peripherals::setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let spi_bus2 = shared_bus::CortexMBusManager::new(spi2_port);
    let _i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);

    let mut gyro_opt = None;
    if let Ok(mut gyro) = L3gd20::new(spi_bus2, spi_cs_imu) {
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

        }
        for _ in 0..10 {
            for _ in 0..10 {
                if mpu_opt.is_some() {
                    if let Ok(marg_all) = mpu_opt.as_mut().unwrap().all::<[f32; 3]>() {
                        //rprintln!("imu az: {:.02}", marg_all.accel[2]);
                    }
                }

                if tdk_opt.is_some() {
                    // if let Ok(gyro_sample) = tdk_6dof.get_gyro() {
                    //     //rprintln!("gyro: {:?}", gyro_sample);
                    // }
                    if let Ok(sample) = tdk_opt.as_mut().unwrap().get_scaled_accel() {
                        //rprintln!("tdk az: {}", sample[2]);
                    }
                }

                delay_source.delay_ms(loop_interval);
            }

            if mag_int_opt.is_some() {
                if let Ok(mag_sample) = mag_int_opt.as_mut().unwrap().get_mag_vector() {
                    // rprintln!("mag_i_0 {}", mag_sample[0]);
                }
            }
        }

        if baro_int_opt.is_some() {
            if let Ok(sample) = baro_int_opt.as_mut().unwrap().get_second_order_sample(Oversampling::OS_2048, &mut delay_source) {
                // rprintln!("baro: {} ", sample.pressure);
            }
        }

        let _ = user_leds.0.toggle();
        let _ = user_leds.1.toggle();
        //let _ = user_leds.2.toggle();
    }
}

