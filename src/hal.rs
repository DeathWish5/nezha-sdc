use core::fmt;
use core::ops::{Deref, DerefMut};

pub const PAGE_SIZE: usize = 4096;

pub type VirtAddr = usize;
pub type PhysAddr = usize;

pub struct Reg(volatile_register::RW<u32>);

impl fmt::Debug for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!("Reg[RW] {}", self.0.read()))
    }
}

/// #Safety: the safety of register should be ensured by busy-bit of status register
unsafe impl Sync for Reg {}

pub struct Wapper<'a, T> {
    ptr: &'a mut T,
}

impl<T> Wapper<'_, T> {
    pub fn from_raw(base_addr: PhysAddr) -> Self {
        Self {
            ptr: unsafe { &mut *(phys_to_virt(base_addr) as *mut T) },
        }
    }
}

impl<T> Deref for Wapper<'_, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        self.ptr
    }
}

impl<T> DerefMut for Wapper<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.ptr
    }
}

impl Reg {
    pub fn write(&mut self, reg: u32) {
        unsafe { self.0.write(reg) };
    }

    pub fn read(&self) -> u32 {
        self.0.read()
    }

    pub fn modify<F: Fn(u32) -> u32>(&mut self, f: F) {
        unsafe { self.0.modify(f) };
    }
}

pub struct Timer;

impl Timer {
    pub fn get_arch_counter(&self) -> u64 {
        let mut mtime: u64 = 0;
        #[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
        unsafe {
            asm!("csrr {}, time", out(reg) mtime)
        };
        #[cfg(not(any(target_arch = "riscv32", target_arch = "riscv64")))]
        compile_error!("Unsupported architecture, please check build target");
        mtime
    }

    pub fn get_us(&self) -> u64 {
        self.get_arch_counter() / 24
    }

    pub fn udelay(&self, us: u64) {
        let time = self.get_arch_counter() + us * 24;
        while self.get_arch_counter() < time {
            core::hint::spin_loop();
        }
    }

    pub fn mdelay(&self, ms: u64) {
        self.udelay(ms * 1000)
    }
}

pub fn phys_to_virt(paddr: PhysAddr) -> VirtAddr {
    unsafe { sdcard_phys_to_virt(paddr) }
}

pub fn virt_to_phys(vaddr: VirtAddr) -> PhysAddr {
    unsafe { sdcard_virt_to_phys(vaddr) }
}

extern "C" {
    pub fn sdcard_dma_alloc(pages: usize) -> PhysAddr;
    pub fn sdcard_dma_dealloc(paddr: PhysAddr, pages: usize) -> i32;
    pub fn sdcard_phys_to_virt(paddr: PhysAddr) -> VirtAddr;
    pub fn sdcard_virt_to_phys(vaddr: VirtAddr) -> PhysAddr;
}

// JUST FOR DEBUG
#[inline]
pub fn read_reg<T>(addr: usize, offset: usize) -> T {
    unsafe { core::ptr::read_volatile((phys_to_virt(addr + offset)) as *const T) }
}

#[inline]
pub fn intr_get() -> bool {
    #[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
    {
        use riscv::register::{sie, sstatus};
        let sie = sstatus::read().sie();
        let ext = sie::read().sext();
        ext && sie
    }
    #[cfg(not(any(target_arch = "riscv32", target_arch = "riscv64")))]
    {
        false
    }
}

pub fn wfi() {
    #[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
    {
        use riscv::{asm, register::sstatus};
        unsafe {
            // enable interrupt and disable
            let intr = sstatus::read().sie();
            if intr == false {
                sstatus::set_sie();
            }
            asm::wfi();
            if intr == false {
                sstatus::clear_sie();
            }
        }
    }
    #[cfg(not(any(target_arch = "riscv32", target_arch = "riscv64")))]
    {
        // DO NOTHING
    }
}
