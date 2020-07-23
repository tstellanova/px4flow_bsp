/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::{entry};
use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::interrupt;

use panic_rtt_core::{self, rprintln, rtt_init_print};

// use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const GYRO_REPORTING_RATE_HZ: u16 = 380;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use px4flow_bsp::{board::Board, dcmi};
use px4flow_bsp::dcmi::DcmiWrapper;
use cortex_m::asm::{bkpt};

/// should be called whenever DMA2 completes a transfer
#[interrupt]
fn DMA2_STREAM1() {
    dcmi::dma2_stream1_irqhandler();
    bkpt();
}

/// should be called whenever DCMI completes a frame
#[interrupt]
fn DCMI() {
    dcmi::dcmi_irqhandler();
    bkpt();
}


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

    //for now we turn on a gray test pattern
    // let _ = board.camera_config.as_mut().unwrap().
    //     enable_pixel_test_pattern(true, 0x3000);

    loop {
        for _ in 0..10 {
            for _ in 0..10 {
                for _ in 0..10 {
                    if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {

                        let ris =  dcmi_wrap.dcmi_raw_status();
                        if 0 != ris {
                            rprintln!("cap ris: {:#b}",ris);
                        }

                        let lisr = dcmi_wrap.dma2_raw_lisr();
                        if 0 != lisr {
                            rprintln!("lisr: {:#b}",lisr);
                        }

                        let hisr = dcmi_wrap.dma2_raw_hisr();
                        if 0 != hisr {
                            rprintln!("hisr: {:#b}",hisr);
                        }

                        dcmi_wrap.dump_status();
                    }

                    //DcmiWrapper::dump_counts();

                    if board.gyro.is_some() {
                        if let Ok(_sample) =  board.gyro.as_mut().unwrap().gyro()
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

        //DcmiWrapper::dump_imgbuf1();
        let _ = board.user_leds[2].toggle(); //red
    }
}
