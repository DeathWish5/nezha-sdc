pub struct Timer;

impl Timer {
    pub fn get_arch_counter(&self) -> u64 {
        let mtime: u64;
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
        let mut t1: u64;
        let t2: u64;
        t1 = self.get_arch_counter();
        t2 = t1 + us * 24;
        loop {
            t1 = self.get_arch_counter();
            if t2 < t1 {
                break;
            }
        }
    }

    pub fn mdelay(&self, ms: u64) {
        self.udelay(ms * 1000)
    }
}
