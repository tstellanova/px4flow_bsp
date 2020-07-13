use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::{DCMI, RCC};

// use core::pin::Pin;
use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

//TODO maybe these frame constants should be configurable?
pub const MAX_FRAME_HEIGHT: usize = 480;
pub const MAX_FRAME_WIDTH: usize = 752;
pub const FULL_FRAME_ROW_WIDTH: usize = MAX_FRAME_WIDTH/4;
pub const FULL_FRAME_COL_HEIGHT: usize = MAX_FRAME_HEIGHT/4;
pub const FULL_FRAME_PIXEL_COUNT: usize = FULL_FRAME_ROW_WIDTH*FULL_FRAME_COL_HEIGHT;
pub const FRAME_XFER_WORD_COUNT: u32 = (FULL_FRAME_PIXEL_COUNT / 4) as u32;
pub type ImageFrameBuf = [u8; FULL_FRAME_PIXEL_COUNT];

pub struct DcmiWrapper {
    image_buf1: ImageFrameBuf,
    image_buf2: ImageFrameBuf,
    image_buf3: ImageFrameBuf,
}


impl DcmiWrapper {

    pub fn new() -> Self {
        let mut inst =
            Self {
                image_buf1: [0; FULL_FRAME_PIXEL_COUNT],
                image_buf2: [0; FULL_FRAME_PIXEL_COUNT],
                image_buf3: [0; FULL_FRAME_PIXEL_COUNT],
            };


        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi setup start");

        //NOTE(unsafe) This executes only once during initialization
        unsafe {
            Self::setup_dcmi();
            inst.setup_dma2();
        }

        // enable interrupts for DMA2 transfer and DCMI capture completion
        // cortex_m::interrupt::free(|_| {
        //     pac::NVIC::unpend(pac::Interrupt::DMA2_STREAM1);
        //     pac::NVIC::unpend(pac::Interrupt::DCMI);
        //     unsafe {
        //         pac::NVIC::unmask(pac::Interrupt::DMA2_STREAM1);
        //         pac::NVIC::unmask(pac::Interrupt::DCMI);
        //     }
        // });

        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi setup done");

        inst
    }

    /// Configure DMA2 for DCMI peripheral -> memory transfer
    unsafe fn setup_dma2(&mut self) {
        //configure DMA2, stream 1, channel 1 for DCMI peripheral -> memory
        let mut stream1_chan1 = &(*pac::DMA2::ptr()).st[1];
        stream1_chan1.cr.write(|w| {
            w.chsel() // ch1
                .bits(1)
                .dir() // transferring peripheral to memory
                .peripheral_to_memory()
                .pinc()// do not increment peripheral
                .fixed()
                .minc()// increment memory
                .incremented()
                // TODO psize
                // TODO msize
                .circ()// enable circular mode
                .enabled()
                .pl()// high priority
                .high()
                .mburst()// single memory burst
                .single()
                .pburst()// single peripheral burst
                .single()
        });
        stream1_chan1.fcr.write(|w| {
            w.dmdis()// disable fifo mode
                .disabled()
                .fth()// fifo threshold full
                .full()
        });
        // Set number of items to transfer: number of 32 bit words
        stream1_chan1.ndtr.write(|w| {
            w.bits(FRAME_XFER_WORD_COUNT)
        });

        // TODO this is probably unacceptable as the buffer address is not pinned?
        let mem0_addr: u32 = (&self.image_buf1 as *const ImageFrameBuf) as u32;
        let mem1_addr: u32 = (&self.image_buf2 as *const ImageFrameBuf) as u32;

        stream1_chan1.m0ar.write(|w| w.bits(mem0_addr));
        stream1_chan1.m1ar.write(|w| w.bits(mem1_addr));

        //TODO wire dcmi_ctrl_pins and dcmi_data_pins to DMA:
        // DMA2: Stream1, Channel_1 -> DCMI
        // DoubleBufferMode

        //Enable DMA2 clock
        &(*pac::RCC::ptr()).ahb1enr.write(|w| w.dma2en().enabled() );

    }

    /// Configure the DCMI peripheral for continuous capture
    unsafe fn setup_dcmi()
    {
        let mut dcmi_periph = &(*pac::DCMI::ptr());
        //basic DCMI configuration
        dcmi_periph.cr.modify(|_, w| { w
            .cm()// capture mode: continuous
            .clear_bit()
            .ess()// synchro mode: hardware
            .clear_bit()
            .pckpol()// PCK polarity: falling
            .clear_bit()
            .vspol()// vsync polarity: low
            .clear_bit()
            .hspol()// hsync polarity: low
            .clear_bit()
            .fcrc()// capture rate: every frame
            .bits(0x00)
            .edm() // extended data mode: 8 bit
            .bits(0x00)
        });

        //enable clock for DCMI peripheral
        &(*pac::RCC::ptr()).ahb2enr
            .modify(|_, w| w.dcmien().enabled());

        // enable interrupt on frame capture completion
        dcmi_periph.ier.modify(|_, w| w.frame_ie().set_bit());

        //TODO verify this is how we enable capturing
        dcmi_periph.cr.modify(|_, w| w.capture().set_bit().enable().set_bit());

    }
}

pub static DCMI_DMA_IT_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Call this from DMA2_Stream1_IRQHandler interrupt
pub fn dma2_stream1_irqhandler()
{
    // dma2 transfer from DCMI to memory completed
    DCMI_DMA_IT_COUNT.fetch_add(1, Ordering::Relaxed);

    unsafe {
        // Clear DMA_IT_TCIF1
        //CTCIFx: Streamx clear transfer complete interrupt flag (x=3..0)
        // Writing 1 to this bit clears the corresponding TCIFx flag in the DMA_LISR register
        &(*pac::DMA2::ptr()).lifcr.write(|w| w.ctcif1().set_bit());

        //TODO process frame data somehow? move to next buffer ?

    }

}

pub static DCMI_CAP_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Call this from DCMI_IRQHandler interrupt
pub fn dcmi_irqhandler()
{
    DCMI_CAP_COUNT.fetch_add(1, Ordering::Relaxed);

    unsafe {
        &(*pac::DCMI::ptr()).icr.write(|w| {
            //clear dcmi capture complete interrupt flag
            w.frame_isc().set_bit()
        });
    }

}





