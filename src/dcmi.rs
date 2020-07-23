use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::{DCMI, RCC};

// use core::pin::Pin;
use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use core::ops::Deref;

//TODO make frame constants configurable?

// pub const MAX_FRAME_HEIGHT: usize = 480;
// pub const MAX_FRAME_WIDTH: usize = 752;
// pub const BINNING_FACTOR: usize = 4;
// pub const FULL_FRAME_ROW_WIDTH: usize = MAX_FRAME_WIDTH/BINNING_FACTOR;
// pub const FULL_FRAME_COL_HEIGHT: usize = MAX_FRAME_HEIGHT/BINNING_FACTOR;



//These are mt9v034 constants, not generally applicable to all cameras
const MAX_FRAME_WIDTH: usize = 752;
const MAX_FRAME_HEIGHT: usize = 480;
const BINNING_FACTOR: usize = 4;
const FULL_FRAME_WIDTH: usize = MAX_FRAME_WIDTH / BINNING_FACTOR; // 188
const FULL_FRAME_HEIGHT: usize = MAX_FRAME_HEIGHT / BINNING_FACTOR; // 120
const FULL_FRAME_PIXEL_COUNT: usize = FULL_FRAME_WIDTH * FULL_FRAME_HEIGHT; //22560

pub const FLOW_IMG_HEIGHT: usize = 64;
pub const FLOW_IMG_WIDTH: usize = 64;
pub const FLOW_FRAME_PIXEL_COUNT: usize = FLOW_IMG_HEIGHT*FLOW_IMG_WIDTH;
pub const FRAME_XFER_WORD_COUNT: u32 = (FLOW_FRAME_PIXEL_COUNT / 4) as u32;

/// Buffer to store image data; note this is larger than the actual size read
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
            frame_width: FLOW_IMG_WIDTH,
            frame_height: FLOW_IMG_HEIGHT,
            dcmi,
            dma2
        }
    }

    /// Setup DCMI and associated DMA
    pub fn setup(&mut self) {

        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi::setup start: {}, {}",FULL_FRAME_PIXEL_COUNT, FRAME_XFER_WORD_COUNT);

        //NOTE(unsafe) This executes only once during initialization
        unsafe {
            self.deinit_dma2();
            self.toggle_dcmi(false);
            self.init_dcmi();
            self.init_dma2();
        }
        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi::setup done");
    }

    pub fn enable_capture(&mut self) {
        unsafe {
            self.enable_dcmi_and_dma();
            #[cfg(feature = "rttdebug")]
            {
                rprintln!("dcmi cr: 0x{:x}",self.dcmi.cr.read().bits());
            }
        }


    }

    unsafe fn deinit_dma2(&mut self) {

        self.toggle_dma2_stream1(false);
        let mut stream1_chan1 = &self.dma2.st[1];

        stream1_chan1.cr.write(|w| w.bits(0) );
        stream1_chan1.ndtr.write(|w| w.bits(0) );
        stream1_chan1.par.write(|w| w.bits(0) );
        stream1_chan1.m0ar.write(|w| w.bits(0) );
        stream1_chan1.m1ar.write(|w| w.bits(0) );
        //fifo control
        stream1_chan1.fcr.write(|w| w.bits(0x00000021) ); //TODO verify value

        //clear all interrupt enable flags
        self.clear_dma2_interrupts();
    }

    /// Clear all pending interrupts from DMA2 stream 1
    unsafe fn clear_dma2_interrupts(&mut self) {
        // Setting these bits clears the corresponding TCIFx flag in the DMA_LISR register
        self.dma2.lifcr.write(|w| w
            .cfeif1().set_bit()
            .cdmeif1().set_bit()
            .cteif1().set_bit()
            .chtif1().set_bit()
            .ctcif1().set_bit()
        );
    }

    /// Configure DMA2 for DCMI peripheral -> memory transfer
    unsafe fn init_dma2(&mut self) {

        //configure DMA2, stream 1, channel 1 for DCMI peripheral -> memory
        let mut stream1_chan1 = &self.dma2.st[1];

        #[cfg(feature = "rttdebug")]
        rprintln!("00 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());

        //configure double-buffer mode
        stream1_chan1.cr.modify(|_, w| w
            // enable double-buffer mode
            .dbm().enabled()
            // select Memory0 initially
            .ct().memory0());

        // TODO verify acceptable if the buffer address is not pinned
        let mem0_addr: u32 = (&IMG_BUF1 as *const ImageFrameBuf) as u32;
        let mem1_addr: u32 = (&IMG_BUF2 as *const ImageFrameBuf) as u32;



        //stream1_chan1.m0ar.write(|w| w.bits(mem0_addr));
        stream1_chan1.m1ar.write(|w| w.bits(mem1_addr));
        stream1_chan1.par.write(|w| w.bits(Self::DCMI_PERIPH_ADDR));

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

        #[cfg(feature = "rttdebug")]
        rprintln!("00 dma2_fcr: {:#b}", stream1_chan1.fcr.read().bits());
        stream1_chan1.fcr.modify(|_, w| { w
            // disable direct mode
            .dmdis().enabled() //TODO verify
            // fifo threshold full
            .fth().full()
        });

        #[cfg(feature = "rttdebug")]
        rprintln!("00 dma2_ndtr: {:#b}", stream1_chan1.ndtr.read().bits());
        // Set number of items to transfer: number of 32 bit words
        stream1_chan1.ndtr.write(|w| {
            w.bits(FRAME_XFER_WORD_COUNT)
        });

        #[cfg(feature = "rttdebug")]
        rprintln!("post-init dma2 CR = {}, NDTR = {}, PAR = {}, M0AR = {}, M1AR = {}, FCR = {}",
            stream1_chan1.cr.read().bits(),
            stream1_chan1.ndtr.read().bits(),
            stream1_chan1.par.read().bits(),
            stream1_chan1.m0ar.read().bits(),
            stream1_chan1.m1ar.read().bits(),
            stream1_chan1.fcr.read().bits(),
        );

        //exp CR = 33969408, NDTR = 1024, PAR = 1342505000, M0AR = 0, M1AR = 536905744, FCR = 35
        //got CR = 33969408, NDTR = 1024, PAR = 1342505000, M0AR = 0, M1AR = 536894552, FCR = 35
    }

    /// Configure the DCMI peripheral for continuous capture
    unsafe fn init_dcmi(&mut self)
    {
        #[cfg(feature = "rttdebug")]
        rprintln!("04 dcmi_cr: {:#b}",self.dcmi.cr.read().bits());

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
        rprintln!("05 dcmi_cr: {:#b}", self.dcmi.cr.read().bits());
    }

    /// Enable DMA2 and DCMI after setup
    unsafe fn enable_dcmi_and_dma(&mut self) {
        self.toggle_dma2_stream1(true);
        self.toggle_dcmi(true);
        self.enable_dma_interrupts();
        self.enable_dcmi_interrupts();

        #[cfg(feature = "rttdebug")]
            {
                let mut stream1_chan1 = &self.dma2.st[1];
                rprintln!("post-enable dma2 CR = {}, NDTR = {}, PAR = {}, M0AR = {}, M1AR = {}, FCR = {}",
                    stream1_chan1.cr.read().bits(),
                    stream1_chan1.ndtr.read().bits(),
                    stream1_chan1.par.read().bits(),
                    stream1_chan1.m0ar.read().bits(),
                    stream1_chan1.m1ar.read().bits(),
                    stream1_chan1.fcr.read().bits(),
                );
            }
    }


    unsafe fn toggle_dma2_stream1(&mut self, enable: bool) {
        let mut stream1_chan1 = &self.dma2.st[1];
        #[cfg(feature = "rttdebug")]
        rprintln!("08 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());

        if enable {
            stream1_chan1.cr.modify(|_, w| w.en().enabled());
        }
        else {
            stream1_chan1.cr.modify(|_, w| w.en().disabled());
        }

        #[cfg(feature = "rttdebug")]
        rprintln!("09 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());
    }

    unsafe fn toggle_dcmi(&mut self, enable: bool) {
        #[cfg(feature = "rttdebug")]
        rprintln!("toggle dcmi_cr: {:#b}",self.dcmi.cr.read().bits());

        if enable {
            // enable the interface:
            self.dcmi.cr.modify(|_, w| w.enable().set_bit());
            // enable capturing:
            self.dcmi.cr.modify(|_, w| w.capture().set_bit());
        }
        else {
            // disable the interface:
            self.dcmi.cr.modify(|_, w| w.enable().clear_bit());
            // disable capturing:
            self.dcmi.cr.modify(|_, w| w.capture().clear_bit());
        }

        #[cfg(feature = "rttdebug")]
        rprintln!("toggle dcmi_cr: {:#b}",self.dcmi.cr.read().bits());
    }

    unsafe fn enable_dcmi_interrupts(&mut self) {
        cortex_m::interrupt::free(|_| {
            // enable interrupts DCMI capture completion
            pac::NVIC::unpend(pac::Interrupt::DCMI);
            pac::NVIC::unmask(pac::Interrupt::DCMI);
        });

        self.dcmi.ier.modify(|_, w|  w
            .err_ie().set_bit()
            .ovr_ie().set_bit()
            .vsync_ie().set_bit()
            // line valid
            .line_ie().set_bit()
            // frame capture completion interrupt
            .frame_ie().set_bit()
        );
    }

    unsafe fn enable_dma_interrupts(&mut self) {
        cortex_m::interrupt::free(|_| {
            // enable interrupts for DMA2 transfer completion
            pac::NVIC::unpend(pac::Interrupt::DMA2_STREAM1);
            pac::NVIC::unmask(pac::Interrupt::DMA2_STREAM1);
        });

        let mut stream1_chan1 = &self.dma2.st[1];

        #[cfg(feature = "rttdebug")]
        rprintln!("04 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());

        stream1_chan1.cr.modify(|_, w| w
            // Half transfer interrupt enable
            .htie().enabled()
            // Transfer complete interrupt enable
            .tcie().enabled()
        );

        #[cfg(feature = "rttdebug")]
        rprintln!("05 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());
    }

    pub fn dma2_raw_lisr(&mut self) -> u32 {
        self.dma2.lisr.read().bits()
    }

    pub fn dma2_raw_hisr(&mut self) -> u32 {
        self.dma2.hisr.read().bits()
    }

    pub fn dcmi_raw_status(&mut self) -> u32 {
        self.dcmi.ris.read().bits()
    }

    pub fn dump_status(&mut self) {
        let p = Self::DCMI_PERIPH_ADDR as *const u32;
        let val = unsafe { core::ptr::read(p) };
        // let val: u32 = unsafe { *(Self::DCMI_PERIPH_ADDR as *const u32) };

        if val != 0 {
            #[cfg(feature = "rttdebug")]
            rprintln!("DCMI_DR: {:#b}", val);
        }

        let dcmi_en = unsafe { &(*pac::RCC::ptr()).ahb2enr.read().dcmien().bit_is_set() } ;
        if !dcmi_en {
            #[cfg(feature = "rttdebug")]
            rprintln!("dcmi_en false?");
        }

        let dcmi_mis = self.dcmi.mis.read().bits();
        if 0 != dcmi_mis {
            #[cfg(feature = "rttdebug")]
            rprintln!("dcmi_mis {:#b}", dcmi_mis);
        }


    }

    /// Dump count of captures and dma transfers to rtt
    pub fn dump_counts() {
        #[cfg(feature = "rttdebug")]
        {
            let cap_count = DCMI_CAP_COUNT.load(Ordering::Relaxed);
            let xfer_count = DCMI_DMA_IT_COUNT.load(Ordering::Relaxed);
            if xfer_count > 0 || cap_count > 0 {
                rprintln!("caps: {} xfers: {}", cap_count, xfer_count);
            }
        }
    }

    #[cfg(feature = "rttdebug")]
    pub fn dump_imgbuf1() {
        unsafe {
            rprintln!("imgbuf1 {:x?}",&IMG_BUF1[0..4]);
        }
    }

    // currently we can't easily coerce pointers to u32 in const context,
    // but here's how we would calculate the DCMI peripheral address for DMA:
    //const DCMI_BASE: *const pac::dcmi::RegisterBlock = pac::DCMI::ptr(); //0x5005_0000
    //const DCMI_PERIPH_ADDR: u32 = DCMI_BASE.wrapping_offset(0x28) as u32;// "0x28 - data register DR"
    const DCMI_PERIPH_ADDR: u32 = 0x5005_0028;
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





