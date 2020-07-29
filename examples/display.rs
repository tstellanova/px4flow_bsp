/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::{entry, exception};
use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use px4flow_bsp::{dcmi, peripherals, board::Board};
use px4flow_bsp::dcmi::{ImageFrameBuf, IMG_FRAME_BUF_LEN};
use pac::{interrupt};

use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
};

use ssd1306::{prelude::*, Builder, I2CDIBuilder};
// use embedded_hal::blocking::delay::{DelayMs, DelayUs};
// use embedded_graphics::pixelcolor::Gray8;
use cortex_m::asm::wfi;
use cortex_m_rt::ExceptionFrame;

#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG0:ImageFrameBuf = [0u8; IMG_FRAME_BUF_LEN];
#[link_section = ".ccmram.IMG_BUFS"]
static mut FAST_IMG1:ImageFrameBuf = [0u8; IMG_FRAME_BUF_LEN];


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

// #[exception]
// fn HardFault(ef: &ExceptionFrame) -> ! {
//     panic!("{:#?}", ef);
// }

const ONE_BPP_BUF_LEN: usize = ( 64 * 64 ) / 8;

#[rustfmt::skip]
const TEST_RAW_IMG_DATA: [u8; ONE_BPP_BUF_LEN] = [
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,

    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,

    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,

    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,

    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,

    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,

    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
    0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,

    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
    0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,

];


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

    let mut board = Board::new();

    rprintln!("create display...");
    let interface = I2CDIBuilder::new().init(board.external_i2c1.acquire());
    let mut disp: GraphicsMode<_, _> = Builder::new()
        .size(DisplaySize128x64)
        .connect(interface)
        .into();
    rprintln!("init display...");
    disp.init().unwrap();
    disp.clear();
    disp.flush().unwrap();

    rprintln!("load image...");
    // let raw: ImageRaw<BinaryColor> =
    //     ImageRaw::new(include_bytes!("./rust.raw"), 64, 64);
    // let mut start_img = Image::new(&raw, Point::new(32, 0));

    let raw_start: ImageRaw<BinaryColor> = ImageRaw::new(&TEST_RAW_IMG_DATA, 64, 64);
    let start_img = Image::new(&raw_start, Point::new(32, 0));
    // let mut one_bit_buf:[u8; ONE_BPP_BUF_LEN] = [0; ONE_BPP_BUF_LEN];

    rprintln!("draw image...");
    start_img.draw(&mut disp).unwrap();
    disp.flush().unwrap();

    if let Some(dcmi_wrap) = board.dcmi_wrap.as_mut() {
        //rprintln!("enabling dcmi");
        dcmi_wrap.enable_capture();

       let _ = board.activity_led.set_high();

        let mut flipper = 0;
        // let mut x_pos = 0;
        rprintln!("start loop...");
        loop {
            let avail = dcmi_wrap.available_frame_count();
            if avail > 0 {
                if flipper == 0 {
                    dcmi_wrap.copy_image_buf(unsafe { &mut FAST_IMG0 });
                }
                else {
                    dcmi_wrap.copy_image_buf(unsafe { &mut FAST_IMG1 });
                }

                let src =
                    if flipper == 0  { unsafe { &mut FAST_IMG0 } }
                    else  { unsafe { &mut FAST_IMG1 }};
                flipper = (flipper + 1) % 1;

                // one_bit_buf[0] = if TEST_RAW_IMG_DATA[0] > 0 { 0 } else {0xFF};
                // one_bit_buf[ONE_BPP_BUF_LEN-1] = if TEST_RAW_IMG_DATA[ONE_BPP_BUF_LEN-1]

                // for y in 0..64 {
                //     for x  in 0..64 {
                //         let src_idx = (x + y*64) as usize;
                //         let dst_idx = src_idx / 8;
                //         let mut dst_val = one_bit_buf[dst_idx];
                //         let bit_idx = src_idx % 8;
                //         let bitmask = 0xFF ^ (1 << bit_idx);
                //         let bit_val =  if src[src_idx] > 127 { 1 } else {0};
                //         dst_val = (dst_val & bitmask) | (bit_val << bit_idx);
                //         one_bit_buf[dst_idx] = dst_val;
                //     }
                // }

                // let snap: ImageRaw<BinaryColor> = ImageRaw::new(&one_bit_buf, 64, 64);
                // let snap_img = Image::new(&snap, Point::new(32, 0));
                // start_img.draw(&mut disp).unwrap();
                // let _ = disp.flush();
                //
                let _ = board.activity_led.toggle();
            }
            else {
                let _ = board.activity_led.set_high();
                if start_img.draw(&mut disp).is_ok() {
                    let _ = disp.flush();
                    wfi(); //wait for next DMCI-DMA completion
                    //board.delay_source.delay_us(2000u32);
                }
            }
        }
    }

    loop {}

}


