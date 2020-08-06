use p_hal::stm32 as pac;
use stm32f4xx_hal as p_hal;

use pac::{DCMI, RCC};

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m::singleton;

use core::ops::Deref;
#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;
use stm32f4xx_hal::stm32::dma2::ST;

pub const SQ_DIM_64: usize = 64;
pub const SQ_DIM_120: usize = 120;
pub const SQ_64_PIX_COUNT: usize = SQ_DIM_64 * SQ_DIM_64;
pub const SQ_120_PIX_COUNT: usize = SQ_DIM_120 * SQ_DIM_120;
pub const MT9V034_PIX_COUNT: usize = 752 * 480;
pub const MT9V034_BIN4_PIX_COUNT: usize = MT9V034_PIX_COUNT/16;

//TODO this assumes 8 bpp
pub const SQ_FRAME_BUF_LEN: usize = SQ_120_PIX_COUNT;


/// Buffer to store image data
pub type ImageFrameBuf = [u8; SQ_FRAME_BUF_LEN];


struct DmaRotatingTransfer<'a> {
    buf0: &'a mut ImageFrameBuf,
    buf1: &'a mut ImageFrameBuf,
    buf2: &'a mut ImageFrameBuf,
    buf0_addr: u32,
    buf1_addr: u32,
    buf2_addr: u32,
    /// the buffer index of the frame buffer that is currently unused by DMA
    unused_buf_idx: AtomicUsize,
    /// the buffer index of the buffer currently selected for M0AR
    mem0_buf_idx: AtomicUsize,
    /// the buffer index of the buffer currently selected for M1AR
    mem1_buf_idx: AtomicUsize,
}


/// Wrapper for reading DCMI
pub struct DcmiWrapper<'a> {
    frame_height: usize,
    frame_width: usize,
    pixel_count: usize,
    bits_per_pixel: u8, //TODO convert to enum
    dcmi: pac::DCMI,
    transfer: DmaRotatingTransfer<'a>,
}

static mut IMG_BUF0: ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN];
static mut IMG_BUF1: ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN];
static mut IMG_BUF2: ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN];

static mut BUF0_PTR: *const ImageFrameBuf = core::ptr::null();
static mut BUF1_PTR: *const ImageFrameBuf = core::ptr::null();
static mut BUF2_PTR: *const ImageFrameBuf = core::ptr::null();

/// the buffer index of the frame buffer that is currently unused by DMA
static UNUSED_BUF_IDX: AtomicUsize = AtomicUsize::new(2);
/// the buffer index of the buffer currently selected for M0AR
static MEM0_BUF_IDX: AtomicUsize = AtomicUsize::new(0);
/// the buffer index of the buffer currently selected for M1AR
static MEM1_BUF_IDX: AtomicUsize = AtomicUsize::new(1);

impl DcmiWrapper<'_> {
    /// New wrapper ready for DCMI with a 64x64, 8 bits per pixel capture
    pub fn default(dcmi: pac::DCMI, dma2: &pac::DMA2) -> Self {
        //Self::new(dcmi, dma2, SQ_DIM_64, SQ_DIM_64, 8)
        Self::new(dcmi, dma2, SQ_DIM_120, SQ_DIM_120, 8)
    }

    ///
    pub fn new(
        dcmi: pac::DCMI,
        dma2: &pac::DMA2,
        frame_height: usize,
        frame_width: usize,
        bits_per_pixel: u8,
    ) -> Self {
        let pixel_count = frame_height * frame_width;

        let buf0_addr = unsafe { (&IMG_BUF0 as *const ImageFrameBuf)};
        let buf1_addr = unsafe { (&IMG_BUF1 as *const ImageFrameBuf)};
        let buf2_addr = unsafe { (&IMG_BUF2 as *const ImageFrameBuf)};

        #[cfg(feature = "rttdebug")]
        rprintln!(
            "buf0 {:x} buf1 {:x} buf2 {:x}",
            buf0_addr as u32,
            buf1_addr as u32,
            buf2_addr as u32);

        Self {
            frame_height,
            frame_width,
            pixel_count,
            bits_per_pixel,
            dcmi,
            transfer: DmaRotatingTransfer {
                buf0: unsafe { &mut IMG_BUF0 } ,
                buf1: unsafe { &mut IMG_BUF1 },
                buf2: unsafe { &mut IMG_BUF2 },
                buf0_addr: buf0_addr as u32,
                buf1_addr: buf1_addr as u32,
                buf2_addr: buf2_addr as u32,
                /// the buffer index of the frame buffer that is currently unused by DMA
                unused_buf_idx: AtomicUsize::new(2),
                /// the buffer index of the buffer currently selected for M0AR
                mem0_buf_idx: AtomicUsize::new(0),
                /// the buffer index of the buffer currently selected for M1AR
                mem1_buf_idx: AtomicUsize::new(1),
            }
        }
    }

    /// Setup DCMI and associated DMA
    pub fn setup(&mut self, dma2: &pac::DMA2) {
        #[cfg(feature = "rttdebug")]
        rprintln!(
            "dcmi::setup start: {}, {}",
            self.pixel_count,
            self.bits_per_pixel
        );

        //NOTE(unsafe) This executes only once during initialization
        unsafe {
            self.deinit_dma2(dma2);
            self.toggle_dcmi(false);
            self.init_dcmi();
            self.init_dma2();
        }
        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi::setup done");
    }

    /// Call this after `setup` to begin capture and start interrupts
    pub fn enable_capture(&mut self) {
        unsafe {
            self.enable_dcmi_and_dma();
            #[cfg(feature = "rttdebug")]
            rprintln!("dcmi cr: 0x{:x}", self.dcmi.cr.read().bits());
        }
    }

    fn deinit_dma2(&mut self, dma2: &pac::DMA2) {
        self.toggle_dma2_stream1(false);
        let mut stream1_chan1 = dma2.st[1];

        unsafe {
            stream1_chan1.cr.write(|w| w.bits(0));
            stream1_chan1.ndtr.write(|w| w.bits(0));
            stream1_chan1.par.write(|w| w.bits(0));
            stream1_chan1.m0ar.write(|w| w.bits(0));
            stream1_chan1.m1ar.write(|w| w.bits(0));
            //fifo control
            stream1_chan1.fcr.write(|w| w.bits(0x00000021)); //TODO verify value
        };

        //clear all interrupt enable flags
        Self::clear_dma2_interrupts(dma2);
    }

    /// Clear all pending interrupts from DMA2 stream 1
    fn clear_dma2_interrupts(dma2: &pac::DMA2) {
        // Setting these bits clears the corresponding TCIFx flag in the DMA_LISR register
        dma2.lifcr.write(|w| {
            w.cfeif1()
                .set_bit()
                .cdmeif1()
                .set_bit()
                .cteif1()
                .set_bit()
                .chtif1()
                .set_bit()
                .ctcif1()
                .set_bit()
        });
    }

    /// Configure DMA2 for DCMI peripheral -> memory transfer
    fn init_dma2(&mut self, dma2: &pac::DMA2) {
        //configure DMA2, stream 1, channel 1 for DCMI peripheral -> memory

        // #[cfg(feature = "rttdebug")]
        // rprintln!("00 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());

        //configure double-buffer mode
        dma2.st[1].cr.modify(|_, w| {
            w
                // enable double-buffer mode
                .dbm()
                .enabled()
                // select Memory0 initially
                .ct()
                .memory0()
        });

        self.init_dma_buffers(dma2);
        dma2.st[1]
            .par
            .write(|w| unsafe { w.bits(Self::DCMI_PERIPH_ADDR) });

        // init dma2 stream1
        dma2.st[1].cr.modify(|_, w| {
            w
                // select ch1
                .chsel()
                .bits(1)
                // transferring peripheral to memory
                .dir()
                .peripheral_to_memory()
                // do not increment peripheral address
                .pinc()
                .fixed()
                // increment memory address
                .minc()
                .incremented()
                // 32 bit (word) peripheral data size DMA_PeripheralDataSize_Word
                .psize()
                .bits32()
                // 32 bit (word) memory data size DMA_MemoryDataSize_Word
                .msize()
                .bits32()
                // enable circular mode
                .circ()
                .enabled()
                // high priority
                .pl()
                .high()
                // single memory burst
                .mburst()
                .single()
                // single peripheral burst
                .pburst()
                .single()
        });

        dma2.st[1].fcr.modify(|_, w| {
            w
                // disable direct mode
                .dmdis()
                .enabled()
                // fifo threshold full
                .fth()
                .full()
        });

        // Set number of items to transfer: number of 32 bit words
        let bytes_per_pixel = Self::bytes_per_pixel(self.bits_per_pixel);
        let word_count = ((self.pixel_count * bytes_per_pixel) / 4) as u32;
        dma2.st[1].ndtr.write(|w| unsafe { w.bits(word_count) });

        #[cfg(feature = "rttdebug")]
        {
            let stream1_chan1 = &dma2.st[1];
            rprintln!("post-init dma2 CR = {}, NDTR = {}, PAR = {}, M0AR = {}, M1AR = {}, FCR = {}",
                stream1_chan1.cr.read().bits(),
                stream1_chan1.ndtr.read().bits(),
                stream1_chan1.par.read().bits(),
                stream1_chan1.m0ar.read().bits(),
                stream1_chan1.m1ar.read().bits(),
                stream1_chan1.fcr.read().bits(),
            );
        }

        //sample: CR = 33969408, NDTR = 1024, PAR = 1342505000, M0AR = 0, M1AR = 536894552, FCR = 35
    }

    /// Initializes the buffers used for double-buffering with DMA2
    fn init_dma_buffers(&mut self, dma2: &pac::DMA2) {
        self.transfer.mem0_buf_idx.store(0, Ordering::SeqCst);
        self.transfer.mem1_buf_idx.store(1, Ordering::SeqCst);
        self.transfer.unused_buf_idx.store(2, Ordering::SeqCst);

        unsafe {
            //set the initial buffers to be used by DMA
            dma2.st[1].m0ar.write(|w| w.bits(self.transfer.buf0_addr));
            dma2.st[1].m1ar.write(|w| w.bits(self.transfer.buf1_addr));
        }
    }

    /// calculate the number of bytes needed to represent a single pixel
    fn bytes_per_pixel(bits_per_pixel: u8) -> usize {
        if bits_per_pixel > 8 {
            if bits_per_pixel > 16 {
                if bits_per_pixel > 24 {
                    4
                } else {
                    3
                }
            } else {
                2
            }
        } else {
            1
        }
    }

    /// Configure the DCMI peripheral for continuous capture
    fn init_dcmi(&mut self) {
        // #[cfg(feature = "rttdebug")]
        // rprintln!("04 dcmi_cr: {:#b}", self.dcmi.cr.read().bits());

        //basic DCMI configuration
        //TODO use self.bits_per_pixel to configure EDM
        self.dcmi.cr.modify(|_, w| unsafe {
            w.cm() // capture mode: continuous
                .clear_bit()
                .ess() // synchro mode: hardware
                .clear_bit()
                .pckpol() // PCK polarity: falling
                .clear_bit()
                .vspol() // vsync polarity: low
                .clear_bit()
                .hspol() // hsync polarity: low
                .clear_bit()
                .fcrc() // capture rate: every frame
                .bits(0x00)
                .edm() // extended data mode: 8 bit
                .bits(0x00)
        });

        #[cfg(feature = "rttdebug")]
        rprintln!("05 dcmi_cr: {:#b}", self.dcmi.cr.read().bits());
    }

    /// Enable DMA2 and DCMI after setup
    fn enable_dcmi_and_dma(&mut self, dma2: &pac::DMA2) {
        self.toggle_dma2_stream1(true);
        self.toggle_dcmi(true);
        self.enable_dma_interrupts();
        self.enable_dcmi_interrupts();

        #[cfg(feature = "rttdebug")]
        {
            let mut stream1_chan1 = &dma2.st[1];
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

    /// Enable or disable the DMA2 stream
    fn toggle_dma2_stream1(&mut self, dma2: &pac::DMA2, enable: bool) {
        let mut stream1_chan1 = &dma2.st[1];
        #[cfg(feature = "rttdebug")]
        rprintln!("08 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());

        if enable {
            stream1_chan1.cr.modify(|_, w| w.en().enabled());
        } else {
            stream1_chan1.cr.modify(|_, w| w.en().disabled());
        }

        #[cfg(feature = "rttdebug")]
        rprintln!("09 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());
    }

    /// Enable or disable the DCMI interface
    fn toggle_dcmi(&mut self, enable: bool) {
        #[cfg(feature = "rttdebug")]
        rprintln!("toggle dcmi_cr: {:#b}", self.dcmi.cr.read().bits());

        if enable {
            // enable the interface:
            self.dcmi.cr.modify(|_, w| w.enable().set_bit());
            // enable capturing:
            self.dcmi.cr.modify(|_, w| w.capture().set_bit());
        } else {
            // disable the interface:
            self.dcmi.cr.modify(|_, w| w.enable().clear_bit());
            // disable capturing:
            self.dcmi.cr.modify(|_, w| w.capture().clear_bit());
        }

        #[cfg(feature = "rttdebug")]
        rprintln!("toggle dcmi_cr: {:#b}", self.dcmi.cr.read().bits());
    }

    /// Enable `DCMI` interrupts
    fn enable_dcmi_interrupts(&mut self) {
        cortex_m::interrupt::free(|_| {
            // enable interrupts DCMI capture completion
            pac::NVIC::unpend(pac::Interrupt::DCMI);
            unsafe {
                pac::NVIC::unmask(pac::Interrupt::DCMI);
            }
        });

        self.dcmi.ier.write(|w| {
            w
                // watch for data buffer overrun occurred
                .ovr_ie()
                .set_bit()
                // frame capture completion interrupt
                .frame_ie()
                .set_bit()
        });
    }

    /// Enable `DMA2_STREAM1` interrupts
    fn enable_dma_interrupts(&mut self, dma2: &pac::DMA2) {
        cortex_m::interrupt::free(|_| {
            // enable interrupts for DMA2 transfer completion
            pac::NVIC::unpend(pac::Interrupt::DMA2_STREAM1);
            unsafe {
                pac::NVIC::unmask(pac::Interrupt::DMA2_STREAM1);
            }
        });

        let mut stream1_chan1 = &dma2.st[1];

        // #[cfg(feature = "rttdebug")]
        // rprintln!("04 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());

        stream1_chan1.cr.modify(|_, w| {
            w
                // Note: add this to enable Half transfer interrupt:
                //.htie().enabled()
                // Transfer complete interrupt enable:
                .tcie()
                .enabled()
        });

        #[cfg(feature = "rttdebug")]
        rprintln!("05 dma2_cr: {:#b}", stream1_chan1.cr.read().bits());
    }

    /// Copy data from the current DMA-idle buffer into the provided slice
    pub fn copy_image_buf(&mut self, dest: &mut [u8]) {
        let cur_unused = UNUSED_BUF_IDX.load(Ordering::SeqCst);
        // reset the counter of transferred frames
        UNREAD_FRAMES_COUNT.store(0, Ordering::SeqCst);

        let raw_source = unsafe {
            match cur_unused {
                0 => BUF0_PTR,
                1 => BUF1_PTR,
                2 => BUF2_PTR,
                _ => panic!("invalid cur_unused"),
            }
        };

        let source = unsafe { raw_source.as_ref().unwrap() };
        dest.copy_from_slice(&source[0..dest.len()]);
    }

    #[cfg(feature = "rttdebug")]
    pub fn dump_status(&mut self) {
        static LAST_FRAME_COUNT: AtomicUsize = AtomicUsize::new(0);
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

    /// Returns the number of frames that have been transferred since the last `copy_image_buf`
    pub fn available_frame_count(&mut self) -> usize {
        return UNREAD_FRAMES_COUNT.load(Ordering::SeqCst);
    }

    /// Call this from DMA2_STREAM1 interrupt
    pub fn dma2_stream1_irqhandler(&mut self) {
        // dma2 transfer from DCMI to memory completed
        DCMI_DMA_IT_COUNT.fetch_add(1, Ordering::SeqCst);
        UNREAD_FRAMES_COUNT.fetch_add(1, Ordering::SeqCst);

        // clear any pending interrupt bits by writing to LIFCR:
        // this clears the corresponding TCIFx flag in the DMA2 LISR register
        dma2.lifcr.write(|w| w.ctcif1().set_bit());
        //NOTE add .chtif1().set_bit() in order to clear half transfer interrupt

        self.swap_idle_and_unused_buf(&dma2.st[1]);
    }

    /// Update "next" DMA buffer selection to the unused buffer:
    /// This is essential to operating DMA in double buffering mode.
    fn swap_idle_and_unused_buf(&mut self, stream1_chan1: &ST) {

        // is DMA2 currently writing to memory0 ?
        let targ_is_mem0 = stream1_chan1.cr.read().ct().is_memory0();
        //let ndtr = stream1_chan1.ndtr.read().bits();
        // let m0ar = stream1_chan1.m0ar.read().bits();
        // let m1ar = stream1_chan1.m1ar.read().bits();

        let cur_unused = self.transfer.unused_buf_idx.load(Ordering::SeqCst);
        let new_target = match cur_unused {
            0 => self.transfer.buf0_addr,
            1 => self.transfer.buf1_addr,
            2 => self.transfer.buf2_addr,
            _ => panic!("invalid cur_unused"),
        };

        // #[cfg(feature = "rttdebug")]
        // rprintln!("mem0 {} ndtr {} m0ar {:x} m1ar {:x} new {:x}", targ_is_mem0, ndtr, m0ar, m1ar, new_target);

        if targ_is_mem0 {
            //memory1 is idle, so swap an unused buffer into DMA_S2M1AR
            let cur_mem1 = self.transfer.mem1_buf_idx.load(Ordering::SeqCst);
            self.transfer.unused_buf_idx.store(cur_mem1, Ordering::SeqCst);
            self.transfer.mem1_buf_idx.store(cur_unused, Ordering::SeqCst);
            stream1_chan1.m1ar.write(|w| unsafe { w.bits(new_target) });
        } else {
            //memory0 is idle, so swap an unused buffer into DMA_S2M0AR
            let cur_mem0 = self.transfer.mem0_buf_idx.load(Ordering::SeqCst);
            self.transfer.unused_buf_idx.store(cur_mem0, Ordering::SeqCst);
            self.transfer.mem0_buf_idx.store(cur_unused, Ordering::SeqCst);
            stream1_chan1.m0ar.write(|w| unsafe { w.bits(new_target) });
        }
    }

    /// Dump count of captures and dma transfers to rtt
    #[cfg(feature = "rttdebug")]
    pub fn dump_counts() {
        let cap_count = DCMI_CAP_COUNT.load(Ordering::SeqCst);
        let xfer_count = DCMI_DMA_IT_COUNT.load(Ordering::SeqCst);
        if xfer_count > 0 || cap_count > 0 {
            rprintln!("caps: {} xfers: {}", cap_count, xfer_count);
        }
    }

    // currently we can't easily coerce pointers to u32 in const context,
    // but here's how we would calculate the DCMI peripheral address for DMA:
    //const DCMI_BASE: *const pac::dcmi::RegisterBlock = pac::DCMI::ptr(); //0x5005_0000
    //const DCMI_PERIPH_ADDR: u32 = DCMI_BASE.wrapping_offset(0x28) as u32;// "0x28 - data register DR"
    const DCMI_PERIPH_ADDR: u32 = 0x5005_0028;
}

/// Counts the number of DMA interrupts
static DCMI_DMA_IT_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Stores the number of frames that have been transferred since `copy_image_buf` was last called.
static UNREAD_FRAMES_COUNT: AtomicUsize = AtomicUsize::new(0);

// Call this from DMA2_STREAM1 interrupt
// pub fn dma2_stream1_irqhandler() {
//     // dma2 transfer from DCMI to memory completed
//     DCMI_DMA_IT_COUNT.fetch_add(1, Ordering::SeqCst);
//     UNREAD_FRAMES_COUNT.fetch_add(1, Ordering::SeqCst);
//
//     let dma2 = unsafe { &(*pac::DMA2::ptr()) };
//
//     // clear any pending interrupt bits by writing to LIFCR:
//     // this clears the corresponding TCIFx flag in the DMA2 LISR register
//     dma2.lifcr.write(|w| w.ctcif1().set_bit());
//     //NOTE add .chtif1().set_bit() in order to clear half transfer interrupt
//
//     let stream1_chan1 = &dma2.st[1];
//     swap_idle_and_unused_buf(stream1_chan1);
// }

// /// Initializes the buffers used for double-buffering with DMA2
// fn init_dma_buffers(stream1_chan1: &pac::dma2::ST) {
//     MEM0_BUF_IDX.store(0, Ordering::SeqCst);
//     MEM1_BUF_IDX.store(1, Ordering::SeqCst);
//     UNUSED_BUF_IDX.store(2, Ordering::SeqCst);
//
//     unsafe {
//         let buf0_addr = (&IMG_BUF0 as *const ImageFrameBuf);
//         let buf1_addr = (&IMG_BUF1 as *const ImageFrameBuf);
//         let buf2_addr = (&IMG_BUF2 as *const ImageFrameBuf);
//         #[cfg(feature = "rttdebug")]
//         rprintln!(
//             "buf0 {:x} buf1 {:x} buf2 {:x}",
//             buf0_addr as usize,
//             buf1_addr as usize,
//             buf2_addr as usize
//         );
//
//         //store these static addresses once and then use them repeatedly
//         BUF0_PTR = buf0_addr;
//         BUF1_PTR = buf1_addr;
//         BUF2_PTR = buf2_addr;
//
//         //set the initial buffers to be used by DMA
//         stream1_chan1.m0ar.write(|w| w.bits(buf0_addr as u32));
//         stream1_chan1.m1ar.write(|w| w.bits(buf1_addr as u32));
//     }
// }



// /// Update "next" DMA buffer selection to the unused buffer:
// /// This is essential to operating DMA in double buffering mode.
// fn swap_idle_and_unused_buf(stream1_chan1: &pac::dma2::ST) {
//     // is DMA2 currently writing to memory0 ?
//     let targ_is_mem0 = stream1_chan1.cr.read().ct().is_memory0();
//     //let ndtr = stream1_chan1.ndtr.read().bits();
//     // let m0ar = stream1_chan1.m0ar.read().bits();
//     // let m1ar = stream1_chan1.m1ar.read().bits();
//
//     let cur_unused = UNUSED_BUF_IDX.load(Ordering::SeqCst);
//     let new_target = match cur_unused {
//         0 => (unsafe { BUF0_PTR }) as u32,
//         1 => (unsafe { BUF1_PTR }) as u32,
//         2 => (unsafe { BUF2_PTR }) as u32,
//         _ => panic!("invalid cur_unused"),
//     };
//
//     // #[cfg(feature = "rttdebug")]
//     // rprintln!("mem0 {} ndtr {} m0ar {:x} m1ar {:x} new {:x}", targ_is_mem0, ndtr, m0ar, m1ar, new_target);
//
//     if targ_is_mem0 {
//         //memory1 is idle, so swap an unused buffer into DMA_S2M1AR
//         let cur_mem1 = MEM1_BUF_IDX.load(Ordering::SeqCst);
//         UNUSED_BUF_IDX.store(cur_mem1, Ordering::SeqCst);
//         MEM1_BUF_IDX.store(cur_unused, Ordering::SeqCst);
//         stream1_chan1.m1ar.write(|w| unsafe { w.bits(new_target) });
//     } else {
//         //memory0 is idle, so swap an unused buffer into DMA_S2M0AR
//         let cur_mem0 = MEM0_BUF_IDX.load(Ordering::SeqCst);
//         UNUSED_BUF_IDX.store(cur_mem0, Ordering::SeqCst);
//         MEM0_BUF_IDX.store(cur_unused, Ordering::SeqCst);
//         stream1_chan1.m0ar.write(|w| unsafe { w.bits(new_target) });
//     }
// }

/// Stores the number of DCMI transfer completed interrupts
static DCMI_CAP_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Call this from DCMI interrupt
pub fn dcmi_irqhandler() {
    DCMI_CAP_COUNT.fetch_add(1, Ordering::SeqCst);

    let dcmi = unsafe { &(*pac::DCMI::ptr()) };
    #[cfg(feature = "rttdebug")]
    {
        let ris_val = dcmi.ris.read().bits();
        // ordinarily we expect this interrupt on frame capture completion
        if 0b11001 != ris_val {
            rprintln!("error dcmi ris: {:#b}", dcmi.ris.read().bits());
        }
    }

    dcmi.icr.write(|w| {
        w
            //clear dcmi capture complete interrupt flag
            .frame_isc()
            .set_bit()
            // clear overflow flag
            .ovr_isc()
            .set_bit()
    });
}
