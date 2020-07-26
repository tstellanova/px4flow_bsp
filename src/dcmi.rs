use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::{DCMI, RCC};

use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use core::ops::Deref;

//TODO make frame constants configurable?


//These are mt9v034 constants, not generally applicable to all cameras
const MAX_FRAME_WIDTH: usize = 752;
const MAX_FRAME_HEIGHT: usize = 480;
const BINNING_FACTOR: usize = 4;
const FULL_FRAME_WIDTH: usize = MAX_FRAME_WIDTH / BINNING_FACTOR; // 188
const FULL_FRAME_HEIGHT: usize = MAX_FRAME_HEIGHT / BINNING_FACTOR; // 120
const FULL_FRAME_PIXEL_COUNT: usize = 4096; //TODO: FULL_FRAME_WIDTH * FULL_FRAME_HEIGHT; //22560

pub const FLOW_IMG_HEIGHT: usize = 64;
pub const FLOW_IMG_WIDTH: usize = 64;
pub const FLOW_FRAME_PIXEL_COUNT: usize = FLOW_IMG_HEIGHT*FLOW_IMG_WIDTH;
pub const FRAME_XFER_WORD_COUNT: u32 = (FLOW_FRAME_PIXEL_COUNT / 4) as u32;

pub const IMG_FRAME_BUF_LEN: usize = FULL_FRAME_PIXEL_COUNT;
/// Buffer to store image data; note this is larger than the actual size read
pub type ImageFrameBuf = [u8; IMG_FRAME_BUF_LEN];

pub struct DcmiWrapper {
    frame_width: usize,
    frame_height: usize,
    dcmi: pac::DCMI,
    dma2: pac::DMA2,
}


static mut IMG_BUF0: ImageFrameBuf = [0u8; FULL_FRAME_PIXEL_COUNT];
static mut IMG_BUF1: ImageFrameBuf = [0u8; FULL_FRAME_PIXEL_COUNT];
static mut IMG_BUF2: ImageFrameBuf = [0u8; FULL_FRAME_PIXEL_COUNT];

static BUF0_ADDR: AtomicUsize = AtomicUsize::new(0);
static BUF1_ADDR: AtomicUsize = AtomicUsize::new(0);
static BUF2_ADDR: AtomicUsize = AtomicUsize::new(0);


/// the buffer index of the frame buffer that is currently unused by DMA
static UNUSED_BUF_IDX: AtomicUsize = AtomicUsize::new(2);
/// the buffer index of the buffer currently selected for M0AR
static MEM0_BUF_IDX: AtomicUsize = AtomicUsize::new(0);
/// the buffer index of the buffer currently selected for M1AR
static MEM1_BUF_IDX: AtomicUsize = AtomicUsize::new(1);

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

        init_dma_buffers(stream1_chan1);
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

        self.dcmi.ier.write(|w|  w
            // watch for data buffer overrun occurred
            .ovr_ie().set_bit()
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

    #[cfg(feature = "rttdebug")]
    pub fn dump_status(&mut self) {
        static LAST_FRAME_COUNT:AtomicUsize = AtomicUsize::new(0);
        let cur_frame_count = DCMI_CAP_COUNT.load(Ordering::Relaxed);
        let diff = cur_frame_count - LAST_FRAME_COUNT.load(Ordering::Relaxed);
        LAST_FRAME_COUNT.store(cur_frame_count, Ordering::Relaxed);

        if diff > 0 {
            rprintln!("DCMI frames captured: {}", diff);
        }

        // let dcmi_dr = Self::DCMI_PERIPH_ADDR as *const u32;
        // let dcmi_dr_val = unsafe { core::ptr::read(dcmi_dr) };
        // if dcmi_dr_val != 0 {
        //     #[cfg(feature = "rttdebug")]
        //     rprintln!("DCMI_DR: {:#b}", dcmi_dr_val);
        // }

    }

    /// Dump count of captures and dma transfers to rtt
    #[cfg(feature = "rttdebug")]
    pub fn dump_counts() {
        let cap_count = DCMI_CAP_COUNT.load(Ordering::Relaxed);
        let xfer_count = DCMI_DMA_IT_COUNT.load(Ordering::Relaxed);
        if xfer_count > 0 || cap_count > 0 {
            rprintln!("caps: {} xfers: {}", cap_count, xfer_count);
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

    let dma2 = unsafe { &(*pac::DMA2::ptr()) };

    // clear any pending interrupt bits by writing to LIFCR:
    // this clears the corresponding TCIFx flag in the DMA2 LISR register
    dma2.lifcr.write(|w| w
        .ctcif1().set_bit()
        .chtif1().set_bit()
    );

    let stream1_chan1 = &dma2.st[1];
    swap_idle_and_unused_buf(stream1_chan1);
}


/// initialize the buffers used for double-buffering with DMA2
fn init_dma_buffers(stream1_chan1: &pac::dma2::ST) {
    MEM0_BUF_IDX.store(0, Ordering::SeqCst);
    MEM1_BUF_IDX.store(1, Ordering::SeqCst);
    UNUSED_BUF_IDX.store(2, Ordering::SeqCst);

    let buf0_addr = (&unsafe { IMG_BUF0 } as *const ImageFrameBuf) as usize;
    let buf1_addr = (&unsafe { IMG_BUF1 } as *const ImageFrameBuf) as usize;
    let buf2_addr = (&unsafe { IMG_BUF2 } as *const ImageFrameBuf) as usize;
    BUF0_ADDR.store(buf0_addr, Ordering::Relaxed);
    BUF1_ADDR.store(buf1_addr, Ordering::Relaxed);
    BUF2_ADDR.store(buf2_addr, Ordering::Relaxed);
    #[cfg(feature = "rttdebug")]
    rprintln!("buf0 {:x} buf1 {:x} buf2 {:x}",buf0_addr, buf1_addr, buf2_addr);

    stream1_chan1.m0ar.write(|w| unsafe { w.bits(buf0_addr as u32) });
    stream1_chan1.m1ar.write(|w| unsafe { w.bits(buf1_addr as u32) });
}


/// Update "next" DMA buffer selection to the unused buffer:
/// This is essential to operating DMA in double buffering mode.
fn swap_idle_and_unused_buf(stream1_chan1: &pac::dma2::ST) {
    // is DMA2 currently writing to memory0 ?
    let targ_is_mem0 = stream1_chan1.cr.read().ct().is_memory0();
    let ndtr = stream1_chan1.ndtr.read().bits();
    let m0ar = stream1_chan1.m0ar.read().bits();
    let m1ar = stream1_chan1.m1ar.read().bits();


    let cur_unused = UNUSED_BUF_IDX.load(Ordering::Acquire);
    let new_target = match cur_unused {
        // 0 => (&unsafe { IMG_BUF0 } as *const ImageFrameBuf) as u32,
        // 1 => (&unsafe { IMG_BUF1 } as *const ImageFrameBuf) as u32,
        // 2 => (&unsafe { IMG_BUF2 } as *const ImageFrameBuf) as u32,
        0 =>  BUF0_ADDR.load(Ordering::SeqCst) as u32,
        1 =>  BUF1_ADDR.load(Ordering::SeqCst) as u32,
        2 =>  BUF2_ADDR.load(Ordering::SeqCst) as u32,
        _ => panic!("invalid cur_unused")
    };

    #[cfg(feature = "rttdebug")]
    rprintln!("mem0 {} ndtr {} m0ar {:x} m1ar {:x} new {:x}",targ_is_mem0, ndtr, m0ar, m1ar, new_target);

    if targ_is_mem0 {
        let cur_mem1 = MEM1_BUF_IDX.load(Ordering::SeqCst);
        // #[cfg(feature = "rttdebug")]
        // rprintln!("mem1 idle: {} unused: {} ", cur_mem1, cur_unused);
        //memory1 is idle, so swap an unused buffer into DMA_S2M1AR
        UNUSED_BUF_IDX.store(cur_mem1, Ordering::SeqCst);
        MEM1_BUF_IDX.store(cur_unused, Ordering::SeqCst);
        stream1_chan1.m1ar.write(|w| unsafe { w.bits(new_target) } );
    }
    else {
        let cur_mem0 = MEM0_BUF_IDX.load(Ordering::SeqCst);
        // #[cfg(feature = "rttdebug")]
        // rprintln!("mem0 idle: {} unused: {} ", cur_mem0, cur_unused);
        //memory0 is idle, so swap an unused buffer into DMA_S2M0AR
        UNUSED_BUF_IDX.store(cur_mem0, Ordering::SeqCst);
        MEM0_BUF_IDX.store(cur_unused, Ordering::SeqCst);
        stream1_chan1.m0ar.write(|w| unsafe { w.bits(new_target) });
    }
}


pub static DCMI_CAP_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Call this from DCMI interrupt
pub fn dcmi_irqhandler()
{
    DCMI_CAP_COUNT.fetch_add(1, Ordering::Relaxed);

    let dcmi = unsafe { &(*pac::DCMI::ptr()) };
    #[cfg(feature = "rttdebug")]
    {
        let ris_val = dcmi.ris.read().bits();
        // ordinarily we expect this interrupt on frame capture completion
        if 0b11001 != ris_val {
            rprintln!("dcmi ris: {:#b}",dcmi.ris.read().bits());
        }
    }

    dcmi.icr.write(|w| { w
        //clear dcmi capture complete interrupt flag
        .frame_isc().set_bit()
        // clear overflow flag
        .ovr_isc().set_bit()
    });
}







