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

use base64::display::Base64Display;
use core::sync::atomic::{AtomicPtr, Ordering};
use px4flow_bsp::board::Board;
use px4flow_bsp::dcmi::{
    ImageFrameBuf,
    FRAME_BUF_LEN,
    // Sq120FrameBuf, SQ120_FRAME_BUF_LEN,
};

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
// static mut FAST_IMG0: Sq120FrameBuf = [0u8; SQ120_FRAME_BUF_LEN];

#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG1: ImageFrameBuf = [0u8; FRAME_BUF_LEN];
// static mut FAST_IMG1: Sq120FrameBuf = [0u8; SQ120_FRAME_BUF_LEN];

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

    let mut fast_img_bufs: [&mut ImageFrameBuf; 2] =
        unsafe { [&mut FAST_IMG0, &mut FAST_IMG1] };
    // This is how we can enable a grayscale test pattern on the MT9V034
    // let _ = board.camera_config.as_mut().unwrap().
    //     enable_pixel_test_pattern(true, PixelTestPattern::DiagonalShade);

    if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {
        dcmi_wrap.enable_capture(&board.dma2);
    }
    let mut img_count: u32 = 0;
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
                if let Ok(read_len) = dcmi_wrap
                    .read_available(fast_img_bufs[flow_img_idx].as_mut())
                {
                    if read_len > 0 {
                        dump_change_events(
                            img_count,
                            flow_img_idx,
                            &mut fast_img_bufs,
                        );
                        let last_idx = (flow_img_idx + 1) % 2;
                        flow_img_idx = last_idx;

                        // In this example we dump pixel data as base64 encoded
                        // raw 8-bit values to the rtt console.
                        // In a real application we'd do something more substantial
                        // with the data, such as calcualte optical flow
                        //dump_pixels(img_count, dst);

                        let _ = board.activity_led.toggle();
                        img_count += 1;
                    }
                }
            }
        }
        let _ = board.comms_led.toggle();
    }
}

fn calc_fft(samples: &mut ImageFrameBuf) {
    let spectrum = microfft::real::rfft_16(&mut samples);
    rprintln!("\n--- {:?}", spectrum);
}

/// calculate differences between pixels
/// differences < 16 are filtered out
fn dump_change_events(
    timestamp: u32,
    flow_img_idx: usize,
    fast_img_bufs: &mut [&mut ImageFrameBuf; 2],
) {
    let last_idx = (flow_img_idx + 1) % 2;

    for i in 0..fast_img_bufs[last_idx].len() {
        let last_val = fast_img_bufs[last_idx][i];
        let cur_val = fast_img_bufs[flow_img_idx][i];
        let event_val = if cur_val == last_val {
            0
        } else if cur_val > last_val {
            //map 255 diff max to four bits
            let inter_val = (((cur_val - last_val) as u16 * 15) / 255) as u8;
            inter_val << 4
        } else {
            //map 255 diff max to four bits
            let inter_val = (((last_val - cur_val) as u16 * 15) / 255) as u8;
            inter_val
        };

        fast_img_bufs[last_idx][i] = event_val;
    }
    dump_pixels(timestamp, fast_img_bufs[last_idx]);
}

/// output image data as 8-bit raw pixels in base64 encoded format, to RTT
fn dump_pixels(timestamp: u32, buf: &[u8]) {
    rprintln!("\n--- {}", timestamp);

    //process input chunks that are multiples of 12 bytes (for base64 continuity)
    const CHUNK_SIZE: usize = 24;
    let total_len = buf.len();
    let mut read_idx = 0;
    while read_idx < total_len {
        let max_idx = total_len.min(read_idx + CHUNK_SIZE);
        let wrapper = Base64Display::with_config(
            &buf[read_idx..max_idx],
            base64::STANDARD,
        );
        rprint!("{}", wrapper);
        read_idx += CHUNK_SIZE;
    }
}
