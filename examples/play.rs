/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use p_hal::stm32 as pac;
use rt::entry;
use stm32f4xx_hal as p_hal;

use pac::interrupt;

use panic_rtt_core::{self, rprint, rprintln, rtt_init_print};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const GYRO_REPORTING_RATE_HZ: u16 = 95;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use mt9v034_i2c::PixelTestPattern;

use px4flow_bsp::dcmi::{DcmiWrapper, ImageFrameBuf, IMG_FRAME_BUF_LEN};
use px4flow_bsp::{board::Board, dcmi};

/// should be called whenever DMA2 completes a transfer
#[interrupt]
fn DMA2_STREAM1() {
    // forward to DCMI's interrupt handler
    dcmi::dma2_stream1_irqhandler();
}

/// should be called whenever DCMI completes a frame
#[interrupt]
fn DCMI() {
    // forward to DCMI's interrupt handler
    dcmi::dcmi_irqhandler();
}

/// Setup close-coupled RAM buffers for faster image manipulation
#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG0: ImageFrameBuf = [0u8; IMG_FRAME_BUF_LEN];
#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG1: ImageFrameBuf = [0u8; IMG_FRAME_BUF_LEN];

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let mut board = Board::default();

    let loop_interval = GYRO_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = board.activity_led.set_high();
    let _ = board.comms_led.set_high();
    let _ = board.error_led.set_high();

    // This is how we can enable the grayscale test pattern
    let _ = board.camera_config.as_mut().unwrap().
        enable_pixel_test_pattern(true, PixelTestPattern::DiagonalShade);

    if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {
        dcmi_wrap.enable_capture();
    }

    let mut flow_img_idx = 0;
    loop {
        for _ in 0..10 {
            for _ in 0..10 {
                // read the 6dof frequently
                if let Some(six_dof) = board.gyro.as_mut() {
                    if let Ok(_sample) = six_dof.gyro() {
                        //rprintln!("gyro {}, {}, {}", _sample.x, _sample.y, _sample.z );
                    }
                }
            }
            if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {
                let avail_frames = dcmi_wrap.available_frame_count();
                if avail_frames > 0 {
                    //rprintln!("avail: {}", avail_frames);
                    let dst = if flow_img_idx == 0 {
                        unsafe { &mut FAST_IMG0 }
                    } else {
                        unsafe { &mut FAST_IMG1 }
                    };
                    dcmi_wrap.copy_image_buf(dst);

                    // this simply dumps all of the received pixel data --
                    // in a real application we'd do something more substantial with the data
                    let _ = board.activity_led.toggle();
                    rprintln!("--- buf {} ---", flow_img_idx);
                    for i in 0..4096 {
                        rprint!("{:x},", dst[i]);
                    }
                    rprintln!("\n---");
                    flow_img_idx = (flow_img_idx + 1) % 2;

                }
            }
        }
        let _ = board.comms_led.toggle();
    }
}
