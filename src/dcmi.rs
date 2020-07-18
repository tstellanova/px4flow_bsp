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
    frame_width: usize,
    frame_height: usize,
    dcmi: pac::DCMI,
    dma2: pac::DMA2,
}


static mut IMG_BUF1: ImageFrameBuf = [0; FULL_FRAME_PIXEL_COUNT];
static mut IMG_BUF2: ImageFrameBuf = [0; FULL_FRAME_PIXEL_COUNT];
static mut IMG_BUF3: ImageFrameBuf = [0; FULL_FRAME_PIXEL_COUNT];


impl DcmiWrapper {

    pub fn new(dcmi: pac::DCMI, dma2: pac::DMA2) -> Self {
        Self {
            frame_width: FULL_FRAME_ROW_WIDTH,
            frame_height: FULL_FRAME_COL_HEIGHT,
            dcmi,
            dma2
        }
    }

    /// Dump count of captures and dma transfers to rtt
    pub fn dump_counts() {
        #[cfg(feature = "rttdebug")]
        {
            let cap_count = DCMI_CAP_COUNT.load(Ordering::Relaxed);
            let xfer_count = DCMI_DMA_IT_COUNT.load(Ordering::Relaxed);
            if cap_count > 0 {
                rprintln!("caps: {} xfers: {}", cap_count, xfer_count);
            }
        }
    }

    /// Setup dcmi and associated DMA
    pub fn setup(&mut self) {
        //NOTE(unsafe) This executes only once during initialization

        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi::setup start: {}, {}",FULL_FRAME_PIXEL_COUNT, FRAME_XFER_WORD_COUNT);

        unsafe {
            self.setup_dcmi();
            self.setup_dma2();
            self.enable_dcmi_capture();
            self.enable_dma2();

            #[cfg(feature = "rttdebug")]
            {
                rprintln!("dcmi cr: 0x{:x}",self.dcmi.cr.read().bits());
            }

        }

        cortex_m::interrupt::free(|_| {
            // enable interrupts for DMA2 transfer and DCMI capture completion
            pac::NVIC::unpend(pac::Interrupt::DMA2_STREAM1);
            pac::NVIC::unpend(pac::Interrupt::DCMI);
            unsafe {
                pac::NVIC::unmask(pac::Interrupt::DMA2_STREAM1);
                pac::NVIC::unmask(pac::Interrupt::DCMI);
            }
        });

        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi::setup done");
    }

    /// Configure DMA2 for DCMI peripheral -> memory transfer
    unsafe fn setup_dma2(&mut self) {
        //configure DMA2, stream 1, channel 1 for DCMI peripheral -> memory
        let mut stream1_chan1 = &self.dma2.st[1];
        //configure double-buffer mode
        stream1_chan1.cr.modify(|_, w| w
            // enable double-buffer mode
            .dbm().enabled()
            // select Memory0 initially
            .ct().memory0());

        // TODO this is probably unacceptable as the buffer address is not pinned?
        let mem0_addr: u32 = (&IMG_BUF1 as *const ImageFrameBuf) as u32;
        let mem1_addr: u32 = (&IMG_BUF2 as *const ImageFrameBuf) as u32;

        // currently we can't easily coerce pointers to u32 in const context,
        // but here's how we would calculate the DCMI peripheral address for DMA:
        //const DCMI_BASE: *const pac::dcmi::RegisterBlock = pac::DCMI::ptr(); //0x5005_0000
        //const DCMI_PERIPH_ADDR: u32 = DCMI_BASE.wrapping_offset(0x28) as u32;// "0x28 - data register"
        const DCMI_PERIPH_ADDR: u32 = 0x5005_0028;

        stream1_chan1.m0ar.write(|w| w.bits(mem0_addr));
        stream1_chan1.m1ar.write(|w| w.bits(mem1_addr));
        stream1_chan1.par.write(|w| w.bits(DCMI_PERIPH_ADDR));

        // init dma2 stream1
        stream1_chan1.cr.modify(|_, w| { w
            // select ch1
            .chsel().bits(1)
            // transferring peripheral to memory
            .dir().peripheral_to_memory()
            // do not increment peripheral address
            .pinc().fixed()
            // increment memory address
            .minc().incremented()
            // 32 bit (word) peripheral data size DMA_PeripheralDataSize_Word
            .psize().bits32()
            // 32 bit (word) memory data size DMA_MemoryDataSize_Word
            .msize().bits32()
            // enable circular mode
            .circ().enabled()
            // high priority
            .pl().high()
            // single memory burst
            .mburst().single()
            // single peripheral burst
            .pburst().single()
        });
        stream1_chan1.fcr.modify(|_, w| { w
            // disable fifo mode
            .dmdis().disabled()
            // fifo threshold full
            .fth().full()
        });
        // Set number of items to transfer: number of 32 bit words
        stream1_chan1.ndtr.write(|w| {
            w.bits(FRAME_XFER_WORD_COUNT)
        });

    }

    /// Configure the DCMI peripheral for continuous capture
    unsafe fn setup_dcmi(&mut self)
    {
        //basic DCMI configuration
        self.dcmi.cr.modify(|_, w| { w
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

        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi cr: 0x{:x}",self.dcmi.cr.read().bits());

    }

    unsafe fn enable_dcmi_capture(&mut self) {

        // enable interrupt on frame capture completion
        self.dcmi.ier.modify(|_, w|  w
            .frame_ie().set_bit()
            .ovr_ie().set_bit()
            .vsync_ie().set_bit()
            .line_ie().set_bit()
        );

        self.dcmi.cr.modify(|_, w|
            w.capture().set_bit().enable().set_bit());
    }

    unsafe fn enable_dma2(&mut self) {
        let mut stream1_chan1 = &self.dma2.st[1];
        stream1_chan1.cr.modify(|_, w| w
            // enable stream
            .en().enabled()
            // Transfer complete interrupt enable
            .tcie().enabled()
            // Half transfer interrupt enable
            .htie().enabled()
        );
    }

    pub fn dcmi_capture_finished(&mut self) -> bool {
        self.dcmi.ris.read().frame_ris().bit_is_set()
    }

    pub fn dma_transfer_finished(&mut self) -> bool {
        self.dma2.lisr.read().tcif1().bit_is_set()
    }

    pub fn dcmi_raw_status(&mut self) -> u32 {

        self.dcmi.ris.read().bits()
    }
}

pub static DCMI_DMA_IT_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Call this from DMA2_STREAM1 interrupt
pub fn dma2_stream1_irqhandler()
{
    // dma2 transfer from DCMI to memory completed
    DCMI_DMA_IT_COUNT.fetch_add(1, Ordering::Relaxed);

    unsafe {
        //clear any interrupt pending bits
        // Writing 1 to this bit clears the corresponding TCIFx flag in the DMA_LISR register
        &(*pac::DMA2::ptr()).lifcr.write(|w| w
            .ctcif1().set_bit()
            .chtif1().set_bit()
        );

        //TODO process frame data somehow? move to next buffer ?
    }

}

pub static DCMI_CAP_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Call this from DCMI interrupt
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





