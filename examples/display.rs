/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_main]
#![no_std]

use cortex_m_rt as rt;
use rt::entry;
use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use panic_rtt_core::{self, rprintln, rtt_init_print};

use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use px4flow_bsp::{dcmi, peripherals, board::Board};
use pac::interrupt;

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


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

    let mut board = Board::new();


    rprintln!("create display...");
    let mut disp: GraphicsMode<_> =
        Builder::new().connect_i2c(board.external_i2c1.acquire()).into();

    rprintln!("init display...");
    disp.init().unwrap();
    disp.clear();

    rprintln!("load image...");
    let raw: ImageRaw<BinaryColor> =
        ImageRaw::new(include_bytes!("./rust.raw"), 64, 64);
    let mut im = Image::new(&raw, Point::new(32, 0));

    rprintln!("draw image...");
    im.draw(&mut disp).unwrap();
    disp.flush().unwrap();

    rprintln!("start loop...");
    loop {


    }
}
