use super::*;

/* #REF: D1 User Manual v0.1 section 9.7 (https://open.allwinnertech.com/#/doc?menuID=2) */

pub const GPIO_BASE: PhysAddr = 0x2000000;
pub const GPIO_SIZE: usize = PAGE_SIZE;
const GPIO_BANK_NR: usize = 0x30;
const GPIO_BANKS: usize = 6;

#[repr(C)]
struct GpioBank {
    cfg: [Reg; 4],
    dat: [Reg; 1],
    drv: [Reg; 4],
    pull: [Reg; 2],
    res0: [Reg; 1],
}

// TODO: GPIO Interrupts

// #[repr(C)]
// struct GpioInt {
//     // TODO
// }

#[repr(C)]
struct SunxiGpioRegs {
    res0: [u8; 0x30],
    banks: [GpioBank; GPIO_BANKS],
    // res1: [u8; ...]
    // interrupts: &'a mut [GpioInt; GPIO_BANKS],
}

#[repr(usize)]
enum GpioBankIdx {
    // PA = NotSupported,
    PB = 0,
    PC = 1,
    PD = 2,
    PE = 3,
    PF = 4,
    PG = 5,
}

// struct SunxiGpioRegs<'a> {
//     ptr: &'a mut  SunxiGpioRegs,
// }

// impl SunxiGpioRegs {
//     fn from_raw(base_addr: PhysAddr) -> Self {
//         Self {
//             ptr: &*(base_addr as *mut _ as *mut  SunxiGpioRegs)
//         }
//     }
// }

// impl Deref for SunxiGpioRegs {
//     type Target =  SunxiGpioRegs;
//     fn deref(&self) -> &Self::Target {
//         self.ptr
//     }
// }

/*
*   #REF: D1 User Manual v0.1 section 9.7.5.28 (https://open.allwinnertech.com/#/doc?menuID=2)
*   config register 32bit: pin 0 ~ 6 select:
*   --- 31:28 --- : reserved
*   --- 27:24 --- : pin 6 select
*   ---  ...  --- : pin 5~1 select, every 4 bit select 1 pin
*   ---  3:0  --- : pin 0 select
*/

#[repr(u32)]
enum PF_CFG_SELECT {
    INPUT = 0x0000000,
    OUTPUT = 0x1111111,
    SDC0 = 0x222222,
    // JTAG = 0x333333,
    // R_JTAG = 0x444444,
    // I2S2_DOUT = 0x555555,
    // I2S2_DIN = 0x666666,
    // EINT = 0xeeeeeee,
    // DISABLE = 0xfffffff,
}

/*
*   #REF: D1 User Manual v0.1 section 9.7.5.31 (https://open.allwinnertech.com/#/doc?menuID=2)
*   driving register 32bit: pin 0~6 multi driving select
*   --- 31:28 --- : reserved
*   --- 27:24 --- : pin 6 select
*   ---  ...  --- : pin 5~1 select, every 4 bit select 1 pin
*   ---  3:0  --- : pin 0 select
*/

#[repr(u32)]
enum PF_DRI_SELECT {
    LEVEL0 = 0x0000000,
    LEVEL1 = 0x1111111,
    LEVEL2 = 0x2222222,
    LEVEL3 = 0x3333333,
}

/*
*   #REF: D1 User Manual v0.1 section 9.7.5.31 (https://open.allwinnertech.com/#/doc?menuID=2)
*   pull register 32bit: pin 0~6 pull up or down select
*   --- 31:14 --- : reserved
*   --- 13:12 --- : pin 6 select
*   ---  ...  --- : pin 5~1 select, every 2 bit select 1 pin
*   ---  1:0  --- : pin 0 select
*/

#[repr(u32)]
enum PF_PULL_SELECT {
    DISABLE = 0x000,
    PULL_UP = 0x555,
    PULL_DOWN = 0xaaa,
}

pub fn init(base_addr: usize) {
    info!("GPIO_BANK sizeof = 0x{:x}", core::mem::size_of::<GpioBank>());
    // const_assert_eq!(core::mem::size_of::<GpioBank>(), GPIO_BANK_NR);
    let mut GPIO = Wapper::<SunxiGpioRegs>::from_raw(base_addr);
    let PF = &mut GPIO.banks[GpioBankIdx::PF as usize];
    PF.cfg[0].write(PF_CFG_SELECT::SDC0 as u32);
    PF.drv[0].write(PF_DRI_SELECT::LEVEL2 as u32);
    PF.pull[0].write(PF_PULL_SELECT::PULL_UP as u32);
    info!("gpio inited! pin select = {:x}, dri = {:x}, pull = {:x}", PF_CFG_SELECT::SDC0 as u32, 
            PF_DRI_SELECT::LEVEL2 as u32, PF_PULL_SELECT::PULL_UP as u32);
}
