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

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;

const GYRO_REPORTING_RATE_HZ: u16 = 95;
const GYRO_REPORTING_INTERVAL_MS: u16 = 1000 / GYRO_REPORTING_RATE_HZ;

use px4flow_bsp::{board::Board, dcmi};
use px4flow_bsp::dcmi::{DcmiWrapper, ImageFrameBuf, IMG_FRAME_BUF_LEN};
use embedded_hal::blocking::delay::{DelayMs};

use core::sync::atomic::{Ordering, AtomicPtr};


use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};

use ssd1306::{prelude::*, Builder};

/// should be called whenever DMA2 completes a transfer
#[interrupt]
fn DMA2_STREAM1() {
    dcmi::dma2_stream1_irqhandler();
}

/// should be called whenever DCMI completes a frame
#[interrupt]
fn DCMI() {
    dcmi::dcmi_irqhandler();
}


#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG0:ImageFrameBuf = [0u8; IMG_FRAME_BUF_LEN];
#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG1:ImageFrameBuf = [0u8; IMG_FRAME_BUF_LEN];

// static FAST_IMG0_PTR: AtomicPtr<ImageFrameBuf> = AtomicPtr::new(core::ptr::null_mut());
// static FAST_IMG1_PTR: AtomicPtr<ImageFrameBuf> = AtomicPtr::new(core::ptr::null_mut());


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    // let img0 = &unsafe { FAST_IMG0 } as *mut ImageFrameBuf;
    // let img1 = &unsafe { FAST_IMG1 } as *mut ImageFrameBuf;
    // rprintln!("FAST_IMG0: 0x{:x}\nFAST_IMG1: 0x{:x}", img0 as usize, img1 as usize);

    // FAST_IMG0_PTR.store(img0, Ordering::Relaxed);
    // FAST_IMG1_PTR.store(img1, Ordering::Relaxed);


    let mut board = Board::new();

    rprintln!("create display...");
    let mut disp: GraphicsMode<_> =
        Builder::new().connect_i2c(board.external_i2c1.acquire()).into();

    rprintln!("init display...");
    disp.init().unwrap();

    rprintln!("load image...");
    let raw: ImageRaw<BinaryColor> =
        ImageRaw::new(include_bytes!("./rust.raw"), 64, 64);
    let mut x_pos = 0;
    let mut im = Image::new(&raw, Point::new(x_pos, 0));
    im.draw(&mut disp).unwrap();
    disp.flush().unwrap();

    let loop_interval = GYRO_REPORTING_INTERVAL_MS as u8;
    rprintln!("loop_interval: {}", loop_interval);

    for led in board.user_leds.iter_mut() {
        let _ = led.set_high();
    }

    //for now we turn on a gray test pattern
    let _ = board.camera_config.as_mut().unwrap().
        enable_pixel_test_pattern(true, 0x3000);

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
                if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {

                    let avail_frames = dcmi_wrap.available_frame_count();
                    if avail_frames > 0 {
                        //rprintln!("avail: {}", avail_frames);
                        if 0 == flow_img_idx {
                            dcmi_wrap.copy_image_buf(unsafe { &mut FAST_IMG0 });
                        } else {
                            dcmi_wrap.copy_image_buf(unsafe { &mut FAST_IMG1 });
                        }
                        flow_img_idx = (flow_img_idx + 1) % 2;
                        // TODO calculate flow diff between two images
                        let _ = board.user_leds[1].toggle(); //blue
                    }
                }
                let _ = board.user_leds[0].toggle(); //amber
            }
        }

        //DcmiWrapper::dump_counts();
        //DcmiWrapper::dump_imgbuf1();
        let _ = board.user_leds[2].toggle(); //red
    }
}
