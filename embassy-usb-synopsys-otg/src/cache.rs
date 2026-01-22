//! Cache coherency operations for DMA transfers.
//!
//! This module provides cache clean and invalidate operations required for
//! proper DMA operation on platforms with data caches.
//!
//! On AR8030 (T-HEAD RISC-V), the cache operations use T-HEAD extension
//! instructions which operate on physical addresses.

/// Cache line size in bytes (32 for AR8030)
#[cfg(feature = "ar8030")]
pub const CACHE_LINE_SIZE: usize = 32;

/// Invalidate data cache for the given address range.
///
/// This must be called AFTER DMA RX completes to ensure the CPU sees
/// the data written by DMA, not stale cached data.
///
/// # Safety
/// - The address range must be valid memory
/// - Concurrent writes to this memory region may be lost
#[cfg(feature = "ar8030")]
#[inline]
pub unsafe fn dcache_invalidate_range(start: usize, len: usize) {
    if len == 0 {
        return;
    }

    let end = start + len;
    let start_aligned = start & !(CACHE_LINE_SIZE - 1);
    let end_aligned = (end + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);

    // Memory barrier before cache invalidation
    core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags));

    let mut addr = start_aligned;
    while addr < end_aligned {
        // T-HEAD extension: dcache.ipa (invalidate by physical address)
        // Instruction encoding: imm12 = 0x02a
        // NOTE: dcache.iva (0x26) does NOT work on AR8030, must use dcache.ipa!
        core::arch::asm!(
            ".insn i 0x0b, 0, x0, {0}, 0x02a",
            in(reg) addr,
            options(nostack, preserves_flags)
        );
        addr += CACHE_LINE_SIZE;
    }

    // Memory barrier after invalidation to ensure completion
    core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags));
}

/// Clean (write back) data cache for the given address range.
///
/// This must be called BEFORE DMA TX to ensure the DMA controller sees
/// the latest data written by the CPU.
///
/// # Safety
/// - The address range must be valid memory
#[cfg(feature = "ar8030")]
#[inline]
pub unsafe fn dcache_clean_range(start: usize, len: usize) {
    if len == 0 {
        return;
    }

    let end = start + len;
    let start_aligned = start & !(CACHE_LINE_SIZE - 1);
    let end_aligned = (end + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);

    // Memory barrier before cache clean
    core::arch::asm!("fence", options(nostack, preserves_flags));

    let mut addr = start_aligned;
    while addr < end_aligned {
        // T-HEAD extension: dcache.cpa (clean by physical address)
        // Instruction encoding: imm12 = 0x029
        core::arch::asm!(
            ".insn i 0x0b, 0, x0, {0}, 0x029",
            in(reg) addr,
            options(nostack, preserves_flags)
        );
        addr += CACHE_LINE_SIZE;
    }

    // Memory barrier after clean to ensure completion
    core::arch::asm!("fence", options(nostack, preserves_flags));
}

/// Clean and invalidate data cache for the given address range.
///
/// This is useful for bidirectional DMA buffers.
///
/// # Safety
/// - The address range must be valid memory
/// - Concurrent writes to this memory region may be lost
#[cfg(feature = "ar8030")]
#[inline]
#[allow(dead_code)]
pub unsafe fn dcache_clean_invalidate_range(start: usize, len: usize) {
    if len == 0 {
        return;
    }

    let end = start + len;
    let start_aligned = start & !(CACHE_LINE_SIZE - 1);
    let end_aligned = (end + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);

    // Memory barrier before cache operations
    core::arch::asm!("fence", options(nostack, preserves_flags));

    let mut addr = start_aligned;
    while addr < end_aligned {
        // T-HEAD extension: dcache.cipa (clean + invalidate by physical address)
        // Instruction encoding: imm12 = 0x02b
        core::arch::asm!(
            ".insn i 0x0b, 0, x0, {0}, 0x02b",
            in(reg) addr,
            options(nostack, preserves_flags)
        );
        addr += CACHE_LINE_SIZE;
    }

    // Memory barrier after operations to ensure completion
    core::arch::asm!("fence", options(nostack, preserves_flags));
}

// No-op implementations for non-AR8030 platforms
// These will be optimized away by the compiler

#[cfg(not(feature = "ar8030"))]
pub const CACHE_LINE_SIZE: usize = 32;

#[cfg(not(feature = "ar8030"))]
#[inline]
pub unsafe fn dcache_invalidate_range(_start: usize, _len: usize) {
    // No-op: platform doesn't need cache management or uses coherent DMA
}

#[cfg(not(feature = "ar8030"))]
#[inline]
pub unsafe fn dcache_clean_range(_start: usize, _len: usize) {
    // No-op: platform doesn't need cache management or uses coherent DMA
}

#[cfg(not(feature = "ar8030"))]
#[inline]
#[allow(dead_code)]
pub unsafe fn dcache_clean_invalidate_range(_start: usize, _len: usize) {
    // No-op: platform doesn't need cache management or uses coherent DMA
}
