use super::*;
use alloc::sync::Arc;
// use spin::Mutex;

mod ccmu;
mod dma;
mod host;
mod shmc;

use dma::*;
pub use shmc::*;
pub use ccmu::*;
pub use host::MmcHost;

pub const SHMC0_BASE_ADDR: PhysAddr = 0x04020000;
pub const SHMC1_BASE_ADDR: PhysAddr = 0x04021000;
pub const SHMC2_BASE_ADDR: PhysAddr = 0x04022000;
pub const SHMC_SIZE: usize = PAGE_SIZE;

pub fn host(ccmu_addr: usize, shmc_addr: usize) -> Arc<MmcHost> {
    let mmc = MmcHost::new(ccmu_addr, shmc_addr);
    Arc::new(mmc)
}

pub fn init(host: &Arc<MmcHost>) {
    host.init_card().unwrap();
}