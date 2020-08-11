/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

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
pub const MT9V034_BIN4_PIX_COUNT: usize = MT9V034_PIX_COUNT / 16;

//TODO this assumes 8 bpp
pub const SQ_FRAME_BUF_LEN: usize = SQ_120_PIX_COUNT;

/// Buffer to store image data
pub type ImageFrameBuf = [u8; SQ_FRAME_BUF_LEN];

/// Stores the buffers and indices required to operate DMA in "double buffer" mode,
/// where two buffers are in service to DMA at any given time,
/// and a third is available for our consumer to read data from.
struct DmaRotatingTransfer<'a> {
    buf_refs: [&'a mut ImageFrameBuf; 3],
    buf_addresses: [u32; 3],
    /// the buffer index of the frame buffer that is currently unused by DMA
    /// and available for reading by our consumer
    available_buf_idx: AtomicUsize,
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
    dma_it_count: AtomicUsize,
    unread_frames_count: AtomicUsize,
    dcmi_capture_count: AtomicUsize,
    dcmi: pac::DCMI,
    transfer: DmaRotatingTransfer<'a>,
}

static mut IMG_BUF0: ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN];
static mut IMG_BUF1: ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN];
static mut IMG_BUF2: ImageFrameBuf = [0u8; SQ_FRAME_BUF_LEN];

impl DcmiWrapper<'_> {
    /// New wrapper ready for DCMI with a 64x64, 8 bits per pixel capture
    pub fn default(dcmi: pac::DCMI) -> Self {
        Self::new(dcmi, SQ_DIM_120, SQ_DIM_120, 8)
    }

    ///
    pub fn new(
        dcmi: pac::DCMI,
        frame_height: usize,
        frame_width: usize,
        bits_per_pixel: u8,
    ) -> Self {
        let pixel_count = frame_height * frame_width;

        let buf0_addr = unsafe { (&IMG_BUF0 as *const ImageFrameBuf) } as u32;
        let buf1_addr = unsafe { (&IMG_BUF1 as *const ImageFrameBuf) } as u32;
        let buf2_addr = unsafe { (&IMG_BUF2 as *const ImageFrameBuf) } as u32;

        #[cfg(feature = "rttdebug")]
        rprintln!(
            "buf0 {:x} buf1 {:x} buf2 {:x}",
            buf0_addr,
            buf1_addr,
            buf2_addr
        );

        Self {
            frame_height,
            frame_width,
            pixel_count,
            bits_per_pixel,
            dma_it_count: AtomicUsize::new(0),
            unread_frames_count: AtomicUsize::new(0),
            dcmi_capture_count: AtomicUsize::new(0),
            dcmi,
            transfer: DmaRotatingTransfer {
                buf_refs: unsafe {
                    [&mut IMG_BUF0, &mut IMG_BUF1, &mut IMG_BUF2]
                },
                buf_addresses: [buf0_addr, buf1_addr, buf2_addr],
                available_buf_idx: AtomicUsize::new(2),
                mem0_buf_idx: AtomicUsize::new(0),
                mem1_buf_idx: AtomicUsize::new(1),
            },
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
            self.init_dma2(dma2);
        }
        #[cfg(feature = "rttdebug")]
        rprintln!("dcmi::setup done");
    }

    /// Call this after `setup` to begin capture and start interrupts
    pub fn enable_capture(&mut self, dma2: &pac::DMA2) {
        unsafe {
            self.enable_dcmi_and_dma(dma2);
            #[cfg(feature = "rttdebug")]
            rprintln!("dcmi cr: 0x{:x}", self.dcmi.cr.read().bits());
        }
    }

    fn deinit_dma2(&mut self, dma2: &pac::DMA2) {
        self.toggle_dma2_stream1(dma2, false);
        let mut stream1_chan1 = &dma2.st[1];

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
        self.transfer.available_buf_idx.store(2, Ordering::SeqCst);

        unsafe {
            //set the initial buffers to be used by DMA
            dma2.st[1]
                .m0ar
                .write(|w| w.bits(self.transfer.buf_addresses[0]));
            dma2.st[1]
                .m1ar
                .write(|w| w.bits(self.transfer.buf_addresses[1]));
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
        self.toggle_dma2_stream1(dma2, true);
        self.toggle_dcmi(true);
        self.enable_dma_interrupts(dma2);
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
    /// Copy available image data into the provided slice
    pub fn read_available(&mut self, dest: &mut [u8]) -> Result<usize, ()> {
        let unread = self.unread_frames_count.swap(0, Ordering::SeqCst);
        if unread != 0 {
            let cur_available =
                self.transfer.available_buf_idx.load(Ordering::SeqCst);
            let raw_source = &self.transfer.buf_refs[cur_available];
            dest.copy_from_slice(raw_source[0..dest.len()].as_ref());
            return Ok(dest.len());
        }
        return Ok(0);
    }

    /// Returns the number of frames that have been transferred since the last `read_available`
    pub fn unread_frames(&mut self) -> usize {
        return self.unread_frames_count.load(Ordering::SeqCst);
    }

    /// Call this from DMA2_STREAM1 interrupt
    pub fn dma2_stream1_irqhandler(&mut self) {
        // DMA2 transfer from DCMI to memory completed
        self.dma_it_count.fetch_add(1, Ordering::SeqCst);
        self.unread_frames_count.fetch_add(1, Ordering::SeqCst);

        let dma2 = unsafe { &(*pac::DMA2::ptr()) };

        // clear any pending interrupt bits by writing to LIFCR:
        // this clears the corresponding TCIFx flag in the DMA2 LISR register
        dma2.lifcr.write(|w| w.ctcif1().set_bit());
        //NOTE add .chtif1().set_bit() in order to clear half transfer interrupt

        self.swap_idle_and_unused_buf(&dma2.st[1]);
    }

    /// Update the "next" active DMA buffer selection to be the buffer currently unused by DMA,
    /// and mark the buffer most recently written to by DMA as unused (not serving DMA),
    /// so that our consumer is free to read from that buffer.
    /// This function is essential to operating DMA in double buffering mode.
    fn swap_idle_and_unused_buf(&mut self, stream1_chan1: &ST) {
        // is DMA2 currently writing to memory0 ?
        let targ_is_mem0 = stream1_chan1.cr.read().ct().is_memory0();
        let cur_available =
            self.transfer.available_buf_idx.load(Ordering::SeqCst);
        let new_target = self.transfer.buf_addresses[cur_available];

        // #[cfg(feature = "rttdebug")]
        // {
        //     let ndtr = stream1_chan1.ndtr.read().bits();
        //     let m0ar = stream1_chan1.m0ar.read().bits();
        //     let m1ar = stream1_chan1.m1ar.read().bits();
        //     rprintln!("mem0 {} ndtr {} m0ar {:x} m1ar {:x} new {:x}",
        //     targ_is_mem0, ndtr, m0ar, m1ar, new_target);
        // }

        if targ_is_mem0 {
            //memory1 is idle, so swap the available buffer with DMA_S2M1AR
            let cur_mem1 = self.transfer.mem1_buf_idx.load(Ordering::SeqCst);
            self.transfer
                .available_buf_idx
                .store(cur_mem1, Ordering::SeqCst);
            self.transfer
                .mem1_buf_idx
                .store(cur_available, Ordering::SeqCst);
            stream1_chan1.m1ar.write(|w| unsafe { w.bits(new_target) });
        } else {
            //memory0 is idle, so swap the available buffer with DMA_S2M0AR
            let cur_mem0 = self.transfer.mem0_buf_idx.load(Ordering::SeqCst);
            self.transfer
                .available_buf_idx
                .store(cur_mem0, Ordering::SeqCst);
            self.transfer
                .mem0_buf_idx
                .store(cur_available, Ordering::SeqCst);
            stream1_chan1.m0ar.write(|w| unsafe { w.bits(new_target) });
        }
    }

    /// Call this from DCMI interrupt
    pub fn dcmi_irqhandler(&mut self) {
        self.dcmi_capture_count.fetch_add(1, Ordering::SeqCst);
        // let dcmi = unsafe { &(*pac::DCMI::ptr()) };
        #[cfg(feature = "rttdebug")]
        {
            let ris_val = self.dcmi.ris.read().bits();
            // ordinarily we expect this interrupt on frame capture completion
            if 0b11001 != ris_val {
                rprintln!("error dcmi ris: {:#b}", ris_val);
            }
        }

        self.dcmi.icr.write(|w| {
            w
                //clear dcmi capture complete interrupt flag
                .frame_isc()
                .set_bit()
                // clear overflow flag
                .ovr_isc()
                .set_bit()
        });
    }

    // currently we can't easily coerce pointers to u32 in const context,
    // but here's how we would calculate the DCMI peripheral address for DMA:
    //const DCMI_BASE: *const pac::dcmi::RegisterBlock = pac::DCMI::ptr(); //0x5005_0000
    //const DCMI_PERIPH_ADDR: u32 = DCMI_BASE.wrapping_offset(0x28) as u32;// "0x28 - data register DR"
    const DCMI_PERIPH_ADDR: u32 = 0x5005_0028;
}
