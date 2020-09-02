/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

//! This example shows how to initialize the PX4FLOW board
//! and access the onboard gyroscope.
//! See the l3gd20 crate for more details on customizing the gyro configuration.
//!
use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::blocking::delay::DelayMs;

const GYRO_REPORTING_RATE_HZ: u16 = 50;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use px4flow_bsp::board::Board;


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let mut board = Board::default();

    let loop_interval = GYRO_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {} ms", loop_interval);

    let _ = board.activity_led.set_high();
    let _ = board.comms_led.set_high();
    let _ = board.error_led.set_high();

    let mut overrun_count = 0;
    let mut delay_interval_ms = GYRO_REPORTING_INTERVAL_MS;
    loop {
        // read the gyro as fast as possible
        if let Some(gyro) = board.gyro.as_mut() {
            for _ in 0..10 {
                if let Ok(status) = gyro.status() {
                    if status.overrun {
                        rprintln!("overrun! {} ms", delay_interval_ms);
                        overrun_count += 1;
                        if overrun_count > 2 && delay_interval_ms > 1 {
                            delay_interval_ms -= 1;
                            overrun_count = 0;
                        }
                    }
                    if status.new_data || status.overrun {
                        if let Ok(sample) = gyro.gyro() {
                            rprintln!("gyro {}, {}, {}", sample.x, sample.y, sample.z );
                            let _ = board.comms_led.toggle();
                        }
                    }
                    board.delay_source.delay_ms(delay_interval_ms);
                }
            }
        }
        let _ = board.activity_led.toggle();
    }
}
