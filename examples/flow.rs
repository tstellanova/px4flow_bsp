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

use correlation_flow::fwht;

const GYRO_REPORTING_RATE_HZ: u16 = 95;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use mt9v034_i2c::PixelTestPattern;

use base64::display::Base64Display;
use core::sync::atomic::{AtomicPtr, Ordering};
use px4flow_bsp::board::Board;
use px4flow_bsp::dcmi::{  ImageFrameBuf, FRAME_BUF_LEN};

static mut BOARD_PTR: AtomicPtr<Board> = AtomicPtr::new(core::ptr::null_mut());
/// should be called whenever DMA2 completes a transfer
#[interrupt]
fn DMA2_STREAM1() {
    // forward to DCMI's interrupt handler
    unsafe {
        (*BOARD_PTR.load(Ordering::SeqCst)).handle_dma2_stream1_interrupt();
    }
}

/// should be called whenever DCMI completes a frame
#[interrupt]
fn DCMI() {
    // forward to DCMI's interrupt handler
    unsafe {
        (*BOARD_PTR.load(Ordering::SeqCst)).handle_dcmi_interrupt();
    }
}



/// Setup core-coupled RAM buffers for faster image manipulation
#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG0: ImageFrameBuf = [0u8; FRAME_BUF_LEN];

#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG1: ImageFrameBuf = [0u8; FRAME_BUF_LEN];


#[entry]
fn main() -> ! {
    rtt_init_print!(BlockIfFull);
    rprintln!("-- > MAIN --");

    let mut board = Board::default();
    // this provides the interrupt handler access to the shared Board struct
    unsafe {
        BOARD_PTR.store(&mut board, Ordering::SeqCst);
    }

    let loop_interval = GYRO_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    let _ = board.activity_led.set_high();
    let _ = board.comms_led.set_high();
    let _ = board.error_led.set_high();

    let fast_img_bufs: [_; 2] = unsafe { [&mut FAST_IMG0, &mut FAST_IMG1] };
    // This is how we can enable a grayscale test pattern on the MT9V034
    // let _ = board.camera_config.as_mut().unwrap().
    //     enable_pixel_test_pattern(true, PixelTestPattern::DiagonalShade);

    let mut correlator: fwht::HadamardCorrelator = fwht::HadamardCorrelator::new(64, 64);

    if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {
        dcmi_wrap.enable_capture(&board.dma2);
    }
    let mut img_count: u32 = 0;
    let mut flow_img_idx = 0;
    loop {
        for _ in 0..10 {
            // read the gyro
            if let Some(gyro) = board.gyro.as_mut() {
                if let Ok(check_status) = gyro.status() {
                    if check_status.overrun || check_status.new_data {
                        if let Ok(_sample) = gyro.gyro() {
                            //rprintln!("gyro {}, {}, {}", _sample.x, _sample.y, _sample.z );
                        }
                    }
                }
            }
            // read and process any pending image data
            if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {
                let dst = fast_img_bufs[flow_img_idx].as_mut();
                if let Ok(read_len) = dcmi_wrap.read_available(dst) {
                    if read_len > 0 {
                        let new_frame = fast_img_bufs[flow_img_idx].as_ref();
                        flow_img_idx = (flow_img_idx + 1) % 2;
                        let old_frame = fast_img_bufs[flow_img_idx].as_ref();

                        //TODO this image frame we're providing is currently wrong:
                        // We're given something like a 188x120 BIN4 image
                        // (in row-major order in the slice)
                        // and we're treating it as a 64x64 image
                        let (dx, dy) = correlator.measure_translation(new_frame, old_frame);
                        rprintln!("{} ({}, {})", img_count, dx, dy);

                        let _ = board.comms_led.toggle();
                        img_count += 1;
                    }
                }
            }
        }
        let _ = board.activity_led.toggle();
    }
}


