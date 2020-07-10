/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;

use panic_rtt_core::{self, rprintln, rtt_init_print};

// use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const GYRO_REPORTING_RATE_HZ: u16 = 380;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use px4flow_bsp::board::Board;

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let mut board = Board::new();

    let loop_interval = GYRO_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    for led in board.user_leds.iter_mut() {
        let _ = led.set_high();
    }

    loop {
        for _ in 0..10 {
            for _ in 0..10 {
                for _ in 0..10 {
                    if board.gyro_opt.is_some() {
                        if let Ok(_sample) =
                            board.gyro_opt.as_mut().unwrap().gyro()
                        {
                            // rprintln!(
                            //     "gyro {}, {}, {}",
                            //     sample.x,
                            //     sample.y,
                            //     sample.z
                            // );
                        }
                    }

                    let _ = board.user_leds[0].toggle(); //amber
                }
            }
            let _ = board.user_leds[1].toggle(); //blue
        }
        let _ = board.user_leds[2].toggle(); //red
    }
}
