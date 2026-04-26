//! Cache coherency operations for DWC2 internal-DMA transfers.
//!
//! The Synopsys DWC2 USB controller used on STM32 H7/F7 etc. has an internal AHB
//! DMA engine that reads/writes USB packets directly to system memory. When the
//! Cortex-M7 D-Cache is enabled the application *must* invalidate (after RX) and
//! clean (before TX) the buffer ranges to avoid stale data.
//!
//! For chips without D-Cache (Cortex-M0/M3/M4 or RISC-V with coherent memory)
//! these helpers degrade to no-ops. Detection is done at runtime via the
//! standard ARMv7-M SCB.CCR.DC bit so the same binary works whether the cache
//! is on or off.

#![cfg(feature = "dma")]

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
mod imp {
    use core::arch::asm;
    use core::ptr::{read_volatile, write_volatile};

    /// Cortex-M7 D-Cache line size (fixed by ARM architecture).
    pub const CACHE_LINE_SIZE: usize = 32;

    /// SCB.CCR (Configuration and Control Register).
    const SCB_CCR: *const u32 = 0xE000_ED14 as *const u32;
    /// SCB.DCIMVAC (D-Cache Invalidate by MVA to PoC).
    const SCB_DCIMVAC: *mut u32 = 0xE000_EF5C as *mut u32;
    /// SCB.DCCMVAC (D-Cache Clean by MVA to PoC).
    const SCB_DCCMVAC: *mut u32 = 0xE000_EF68 as *mut u32;
    /// SCB.DCCIMVAC (D-Cache Clean+Invalidate by MVA to PoC).
    const SCB_DCCIMVAC: *mut u32 = 0xE000_EF70 as *mut u32;
    /// CCR.DC bit – D-Cache enable (Cortex-M7 only; reads as 0 on M3/M4).
    const CCR_DC_BIT: u32 = 1 << 16;

    #[inline(always)]
    fn dcache_enabled() -> bool {
        unsafe { read_volatile(SCB_CCR) & CCR_DC_BIT != 0 }
    }

    #[inline(always)]
    fn dsb() {
        unsafe { asm!("dsb sy", options(nostack, preserves_flags)) };
    }

    #[inline(always)]
    fn isb() {
        unsafe { asm!("isb sy", options(nostack, preserves_flags)) };
    }

    fn align_range(start: usize, len: usize) -> (usize, usize) {
        let end = start + len;
        let s = start & !(CACHE_LINE_SIZE - 1);
        let e = (end + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);
        (s, e)
    }

    /// Invalidate D-Cache for the range. Call AFTER a DMA write into memory
    /// before the CPU reads it.
    #[inline]
    pub unsafe fn dcache_invalidate_range(start: usize, len: usize) {
        if len == 0 || !dcache_enabled() {
            return;
        }
        dsb();
        let (mut addr, end) = align_range(start, len);
        while addr < end {
            write_volatile(SCB_DCIMVAC, addr as u32);
            addr += CACHE_LINE_SIZE;
        }
        dsb();
        isb();
    }

    /// Clean D-Cache for the range. Call BEFORE handing the buffer to a DMA
    /// engine that will read it.
    #[inline]
    pub unsafe fn dcache_clean_range(start: usize, len: usize) {
        if len == 0 || !dcache_enabled() {
            return;
        }
        dsb();
        let (mut addr, end) = align_range(start, len);
        while addr < end {
            write_volatile(SCB_DCCMVAC, addr as u32);
            addr += CACHE_LINE_SIZE;
        }
        dsb();
        isb();
    }

    /// Clean and invalidate D-Cache for the range.
    #[inline]
    #[allow(dead_code)]
    pub unsafe fn dcache_clean_invalidate_range(start: usize, len: usize) {
        if len == 0 || !dcache_enabled() {
            return;
        }
        dsb();
        let (mut addr, end) = align_range(start, len);
        while addr < end {
            write_volatile(SCB_DCCIMVAC, addr as u32);
            addr += CACHE_LINE_SIZE;
        }
        dsb();
        isb();
    }
}

#[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
mod imp {
    pub const CACHE_LINE_SIZE: usize = 32;

    #[inline]
    pub unsafe fn dcache_invalidate_range(_start: usize, _len: usize) {}

    #[inline]
    pub unsafe fn dcache_clean_range(_start: usize, _len: usize) {}

    #[inline]
    #[allow(dead_code)]
    pub unsafe fn dcache_clean_invalidate_range(_start: usize, _len: usize) {}
}

pub use imp::*;
