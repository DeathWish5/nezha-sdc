#![no_std]
#![feature(asm)]
#![feature(concat_idents)]
#![allow(non_upper_case_globals)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]

extern crate alloc;
#[macro_use]
extern crate log;
#[macro_use]
extern crate static_assertions;

mod error;
mod gpio;
mod mmc;
#[macro_use]
pub mod hal;

use alloc::sync::Arc;

use error::*;
use hal::*;
pub use mmc::MmcHost;

pub fn primary_init<F: Fn(usize, usize) -> Option<usize>>(mapper: F) -> Arc<MmcHost> {
    mapper(gpio::GPIO_BASE, gpio::GPIO_SIZE).unwrap();
    mapper(mmc::SUNXI_CCM_BASE, mmc::SUNXI_CCM_SIZE).unwrap();
    mapper(mmc::SHMC0_BASE_ADDR, mmc::SHMC_SIZE).unwrap();
    mmc::host(mmc::SUNXI_CCM_BASE, mmc::SHMC0_BASE_ADDR)
}

pub fn init(host: &Arc<MmcHost>) {
    gpio::init(gpio::GPIO_BASE);
    mmc::init(host);
}
