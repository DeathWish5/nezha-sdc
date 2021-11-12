#![no_std]
#![no_main]
#![feature(asm)]
extern crate alloc;
use crate::{
    generic::Peripherals,
    serial::{read_reg, write_reg},
};
use alloc::boxed::Box;
use bitflags::bitflags;
use core::fmt::{self, Error};
use embedded_hal::prelude::_embedded_hal_serial_Write;
use fat32::volume::Volume;
use serial::Serial;
use spin::Mutex;

const GPIO_BASE: usize = 0x02000000;
const PF_CFG0: usize = 0xF0;
const PF_DAT: usize = 0x100;
const PF_DRV0: usize = 0x104;
const PF_PULL0: usize = 0x114;

const hclkbase: usize = 0x2001000 + 0x84c;
const mclkbase: usize = 0x2001000 + 0x830;

pub const SHMC_BASE: usize = 0x04020000;
const gctrl: usize = 0x0;
const clkcr: usize = 0x04;
const time_out: usize = 0x08;
const width: usize = 0x0C;
const blksz: usize = 0x10;
const bytecnt: usize = 0x14;
pub const cmd: usize = 0x18;
pub const arg: usize = 0x1C;
const intmask: usize = 0x30;
pub const rint: usize = 0x38;
pub const status: usize = 0x3C;
const dbgc: usize = 0x50;
const csdc: usize = 0x54;
const ntsr: usize = 0x5c;
const hwrst: usize = 0x78;
const thldc: usize = 0x100;
const drv_dl: usize = 0x140;
const fifo: usize = 0x200;

bitflags! {
    struct MmcResp:u32{
        const MMC_RSP_PRESENT = 1 << 0;
        const MMC_RSP_136     = 1 << 1;
        const MMC_RSP_CRC     = 1 << 2;
        const MMC_RSP_BUSY    = 1 << 3;
        const MMC_RSP_OPCODE  = 1 << 4;
        const MMC_RSP_NONE    = 0;
        const MMC_RSP_R1      = MmcResp::MMC_RSP_PRESENT.bits
            | MmcResp::MMC_RSP_CRC.bits
            | MmcResp::MMC_RSP_OPCODE.bits;
        const MMC_RSP_R1B     = MmcResp::MMC_RSP_PRESENT.bits
            | MmcResp::MMC_RSP_CRC.bits
            | MmcResp::MMC_RSP_OPCODE.bits
            | MmcResp::MMC_RSP_BUSY.bits;
        const MMC_RSP_R2      = MmcResp::MMC_RSP_PRESENT.bits
            | MmcResp::MMC_RSP_136.bits
            | MmcResp::MMC_RSP_CRC.bits;
        const MMC_RSP_R3      = MmcResp::MMC_RSP_PRESENT.bits;
        const MMC_RSP_R6      = MmcResp::MMC_RSP_PRESENT.bits
            | MmcResp::MMC_RSP_CRC.bits
            | MmcResp::MMC_RSP_OPCODE.bits;
        const MMC_RSP_R7      = MmcResp::MMC_RSP_PRESENT.bits
            | MmcResp::MMC_RSP_CRC.bits
            | MmcResp::MMC_RSP_OPCODE.bits;
    }
    struct MmcFlags:u32{
        const MMC_DATA_NONE  = 0;
        const MMC_DATA_READ  = 1 << 0;
        const MMC_DATA_WRITE = 1 << 1;
        const MMC_CMD_MANUAL = 1;
    }
    struct SdVersion:u32{
        const SD_VERSION_SD      = 0x20000;
        const SD_VERSION_SD_2    = 0x20000 | 0x20;
        const SD_VERSION_1_0     = 0x20000 | 0x10;
        const SD_VERSION_SD_1_10 = 0x20000 | 0x1a;
    }
}
lazy_static::lazy_static! {
    static ref LEGACY_STDIO: Mutex<Option<Box<Serial>>> =
        Mutex::new(Some(Box::new(Serial::new(0x0250_0000))));
}
struct Stdout;

impl fmt::Write for Stdout {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        if let Some(stdio) = LEGACY_STDIO.lock().as_mut() {
            for byte in s.as_bytes() {
                stdio.write(*byte).unwrap();
            }
        }
        Ok(())
    }

    fn write_char(&mut self, c: char) -> fmt::Result {
        self.write_str(c.encode_utf8(&mut [0; 4]))
    }

    fn write_fmt(mut self: &mut Self, args: fmt::Arguments<'_>) -> fmt::Result {
        fmt::write(&mut self, args)
    }
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    use fmt::Write;
    Stdout.write_fmt(args).unwrap();
}

#[macro_export(local_inner_macros)]
macro_rules! println {
    ($fmt: literal $(, $($arg: tt)+)?) => {
        _print(core::format_args!(core::concat!($fmt, "\r\n") $(, $($arg)+)?));
    }
}
#[macro_export(local_inner_macros)]
macro_rules! print {
    ($($arg:tt)*) => ({
        _print(core::format_args!($($arg)*));
    });
}

pub struct MMC {
    pub OCR: u32,
    pub CID: [u32; 4],
    pub CSD: [u32; 4],
    pub RCA: u32,
    pub SCR: [u32; 2],
    pub read_bl_len: u32,
    pub write_bl_len: u32,
    pub tran_speed: u32
}
pub struct MMC_CMD {
    cmdidx: u32,
    cmdarg: u32,
    resptype: MmcResp,
    resp0: [u32; 4],
    resp1: usize,
    flags: MmcFlags,
    blocks: u32,
    blocksize: u32,
}
impl MMC_CMD {
    pub unsafe fn send(&mut self, data: bool) {
        println!("\n[CMD] SEND CMD {}", self.cmdidx & 0x3F);
        let mut cmdval = 0x8000_0000u32;
        /*
         * CMDREG
         * CMD[5:0]     : Command index
         * CMD[6]       : Has response
         * CMD[7]       : Long response
         * CMD[8]       : Check response CRC
         * CMD[9]       : Has data
         * CMD[10]      : Write
         * CMD[11]      : Steam mode
         * CMD[12]      : Auto stop
         * CMD[13]      : Wait previous over
         * CMD[14]      : About cmd
         * CMD[15]      : Send initialization
         * CMD[21]      : Update clock
         * CMD[31]      : Load cmd
         */
        if self.resptype.contains(MmcResp::MMC_RSP_PRESENT) {
            cmdval |= 1 << 6;
        }
        if self.resptype.contains(MmcResp::MMC_RSP_136) {
            cmdval |= 1 << 7;
        }
        if self.resptype.contains(MmcResp::MMC_RSP_CRC) {
            cmdval |= 1 << 8;
        }
        if data {
            cmdval |= (1 << 9) | (1 << 13);
            if self.flags.contains(MmcFlags::MMC_DATA_WRITE) {
                cmdval |= 1 << 10
            }
            if self.blocks > 1 {
                cmdval |= 1 << 12
            }
            write_reg(SHMC_BASE, blksz, self.blocksize);
            write_reg(SHMC_BASE, bytecnt, self.blocks * self.blocksize);
        } else {
            if (self.cmdidx == 12) && self.flags.contains(MmcFlags::MMC_CMD_MANUAL) {
                cmdval |= 1 << 14;
                cmdval &= !(1 << 13);
            }
        }

        write_reg(SHMC_BASE, arg, self.cmdarg);
        if !data {
            write_reg(SHMC_BASE, cmd, self.cmdidx | cmdval);
        }

        if data {
            write_reg(
                SHMC_BASE,
                gctrl,
                read_reg::<u32>(SHMC_BASE, gctrl) | 0x80000000u32,
            );
            write_reg(SHMC_BASE, cmd, self.cmdidx | cmdval);
            self.trans_data_by_cpu();
            println!("CMD: 0x{:x}", self.cmdidx | cmdval);
            println!("GCTRL: 0x{:x}", read_reg::<u32>(SHMC_BASE, gctrl));
            println!("CMD: 0x{:x}", read_reg::<u32>(SHMC_BASE, blksz));
            println!("CMD: 0x{:x}", read_reg::<u32>(SHMC_BASE, bytecnt));
        }
        loop {
            if read_reg::<u32>(SHMC_BASE, rint) & 0x4 != 0 {
                break;
            }
        }
        if data {
            let mut done;
            loop {
                let staval = read_reg::<u32>(SHMC_BASE, rint);
                if staval & 0xbbc2 != 0 {
                    println!("timeout 0x{:x}", staval);
                    return;
                }
                done = if self.blocks > 1 {
                    staval & (1 << 14)
                } else {
                    staval & (1 << 3)
                };
                if done != 0 {
                    break;
                }
            }
        }
        if self.resptype.bits & MmcResp::MMC_RSP_BUSY.bits != 0 {
            let timeout = crate::peripheral::Timer.get_us() + 0x4ffffff;
            loop {
                if crate::peripheral::Timer.get_us() > timeout {
                    panic!("TIMEOUT");
                }
                if read_reg::<u32>(SHMC_BASE, status) & (1 << 9) == 0 {
                    break;
                }
            }
        }
        if self.resptype.bits & MmcResp::MMC_RSP_136.bits != 0 {
            self.resp0[0] = read_reg::<u32>(SHMC_BASE, 0x2C);
            self.resp0[1] = read_reg::<u32>(SHMC_BASE, 0x28);
            self.resp0[2] = read_reg::<u32>(SHMC_BASE, 0x24);
            self.resp0[3] = read_reg::<u32>(SHMC_BASE, 0x20);
        } else {
            self.resp0[0] = read_reg::<u32>(SHMC_BASE, 0x20);
        }
        println!("SHMC_RINT:   0x{:x}", read_reg::<u32>(SHMC_BASE, rint));
        write_reg(SHMC_BASE, rint, 0xFFFFFFFFu32);
        read_reg::<u32>(SHMC_BASE, 0x20);
    }
    pub unsafe fn trans_data_by_cpu(&mut self) {
        let mut timeout = crate::peripheral::Timer.get_us() + 0xffffff;
        let byte_cnt = (self.blocks * self.blocksize) as usize;
        println!("bytecnt {:x}", byte_cnt >> 2);
        println!("SHMC_status: 0x{:x}", read_reg::<u32>(SHMC_BASE, status));
        let dst = self.resp1 as *mut u32;
        let status_bit = match self.flags {
            MmcFlags::MMC_DATA_READ => 1 << 2,
            MmcFlags::MMC_DATA_WRITE => 1 << 3,
            _ => 0,
        };
        for i in 0..(byte_cnt >> 2) {
            loop {
                if read_reg::<u32>(SHMC_BASE, status) & status_bit == 0 {
                    break;
                }
                if crate::peripheral::Timer.get_us() > timeout {
                    panic!("translate data timeout")
                }
            }
            match self.flags {
                MmcFlags::MMC_DATA_READ => dst
                    .offset(i as isize)
                    .write(read_reg::<u32>(SHMC_BASE, fifo)),
                MmcFlags::MMC_DATA_WRITE => {
                    write_reg(SHMC_BASE, fifo, dst.offset(i as isize).read())
                }
                _ => panic!("Error flag {:x}", self.flags),
            }
            timeout = crate::peripheral::Timer.get_us() + 0xffffff;
        }
        println!("SHMC_status: 0x{:x}", read_reg::<u32>(SHMC_BASE, status));
    }
    pub unsafe fn ptr2val<T>(&self, offset: isize) -> T {
        (self.resp1 as *const T).offset(offset).read()
    }
    pub unsafe fn read_block(&mut self, start: u32, blkcnt: u32) {
        self.cmdidx = match blkcnt {
            1 => 17,
            _ => 18,
        };
        println!("CMDIDX: {}", self.cmdidx);

        self.resptype = MmcResp::MMC_RSP_R1;
        self.cmdarg = start;
        self.blocks = blkcnt;
        self.blocksize = 512;
        self.flags = MmcFlags::MMC_DATA_READ;
        self.send(true);
        println!("0x{:?}", self.resp1);
        self.print();
    }
    pub unsafe fn write_block(&mut self, start: u32, blkcnt: u32) {
        self.cmdidx = match blkcnt {
            1 => 24,
            _ => 25,
        };
        self.cmdarg = start;
        self.resptype = MmcResp::MMC_RSP_R1;
        self.blocks = blkcnt;
        self.blocksize = 512;
        self.flags = MmcFlags::MMC_DATA_WRITE;
        self.send(true);
    }
    pub unsafe fn print(&self) {
        println!("Count:0x{:x}", self.blocks * self.blocksize * 4);
        let index = self.resp1 as *mut u8;
        for i in 0..(self.blocks * self.blocksize) as isize {
            print!("0x{:<02x}, ", index.offset(i).read());
            if (i + 1) % 16 == 0 {
                println!("");
            }
        }
    }

    pub unsafe fn printinfo(&self) {
        println!("SHMC STATUS: 0x{:x}", read_reg::<u32>(SHMC_BASE, 0x3C));
        println!("SHMC CMD:    0x{:x}", read_reg::<u32>(SHMC_BASE, cmd));
        println!(
            "RESPONSES:   [0x{:x}, 0x{:x}, 0x{:x}, 0x{:x}]",
            self.resp0[0], self.resp0[1], self.resp0[2], self.resp0[3]
        );
    }
    pub unsafe fn clear(&mut self) {
        self.resp0.iter_mut().for_each(|f| *f = 0);

        println!("Count:0x{:x}", self.blocks * self.blocksize * 4);
        for i in 0..(self.blocks * self.blocksize) as usize {
            *((self.resp1 + i) as *mut u8) = 0;
        }
    }
}
pub unsafe fn _get_pll_periph0() -> u32 {
    let regv = read_reg::<u32>(0x02001000, 0x20);
    let n = ((regv >> 8) & 0xff) + 1;
    let m = ((regv >> 1) & 0x01) + 1;
    let p = ((regv >> 16) & 0x07) + 1;
    24 * n / m / p / 2
}
pub unsafe fn mmc_updata_clk() {
    let timeout = crate::peripheral::Timer.get_us() + 0xfffff;
    write_reg(
        SHMC_BASE,
        clkcr,
        read_reg::<u32>(SHMC_BASE, clkcr) | (1 << 31),
    );
    write_reg(SHMC_BASE, cmd, (1u32 << 31) | (1 << 21) | (1 << 13));
    loop {
        if read_reg::<u32>(SHMC_BASE, cmd) & 0x8000_0000u32 == 0
            && crate::peripheral::Timer.get_us() < timeout
        {
            break;
        }
    }
    if read_reg::<u32>(SHMC_BASE, cmd) & 0x8000_0000u32 != 0 {
        panic!("update clk fail");
    }
    write_reg(
        SHMC_BASE,
        clkcr,
        read_reg::<u32>(SHMC_BASE, clkcr) & !(1 << 31),
    );
    write_reg(SHMC_BASE, rint, 0xFFFFFFFFu32);
}
pub unsafe fn mmc_get_clock() -> u32 {
    let rval = read_reg::<u32>(mclkbase, 0);
    let m = rval & 0xf;
    let n = (rval >> 8) & 0x3;
    let src = (rval >> 24) & 0x3;
    let sclk_hz;
    if src == 0 {
        sclk_hz = 24000000;
    } else if src == 2 {
        sclk_hz = _get_pll_periph0() * 2 * 1000000; /* use 2x pll6 */
    } else {
        panic!("wrong clock source");
    }
    sclk_hz / (1 << n) / (m + 1)
}
pub unsafe fn mmc_set_clock(clock: u32) {
    /* disable card clock */
    let rval = read_reg::<u32>(SHMC_BASE, clkcr) & !(1 << 16);
    write_reg(SHMC_BASE, clkcr, rval);

    /* updata clock */
    mmc_updata_clk();

    /* disable mclk */
    write_reg(mclkbase, 0, 0u32);
    let rval = read_reg::<u32>(SHMC_BASE, ntsr) | (1 << 31);
    write_reg(SHMC_BASE, ntsr, rval);

    let src: u32;
    let sclk_hz: u32;
    if clock <= 4000000 {
        src = 0;
        sclk_hz = 24000000;
    } else {
        src = 2;
        sclk_hz = _get_pll_periph0() * 2 * 1000000;
    }
    let m;
    let n;
    let mut div = (2 * sclk_hz + clock) / (2 * clock);
    div = if div == 0 { 1 } else { div };
    if div > 128 {
        m = 1;
        n = 0;
    } else if div > 64 {
        n = 3;
        m = div >> 3;
    } else if div > 32 {
        n = 2;
        m = div >> 2;
    } else if div > 16 {
        n = 1;
        m = div >> 1;
    } else {
        n = 0;
        m = div;
    }

    write_reg(mclkbase, 0, (src << 24) | (n << 8) | (m - 1));

    /* re-enable mclk */
    write_reg(mclkbase, 0, read_reg::<u32>(mclkbase, 0) | (1u32 << 31));
    let rval = read_reg::<u32>(SHMC_BASE, clkcr) & !(0xff);
    write_reg(SHMC_BASE, clkcr, rval);

    /* update clock */
    mmc_updata_clk();

    /* config delay */
    let odly = 0;
    let sdly = 0;
    let mut rval = read_reg::<u32>(SHMC_BASE, drv_dl) & !(0x3 << 16);
    rval |= ((odly & 0x1) << 16) | ((odly & 0x1) << 17);
    write_reg(mclkbase, 0, read_reg::<u32>(mclkbase, 0) & !(1 << 31));
    write_reg(SHMC_BASE, drv_dl, rval);
    write_reg(mclkbase, 0, read_reg::<u32>(mclkbase, 0) | (1 << 31));
    let mut rval = read_reg::<u32>(SHMC_BASE, ntsr);
    rval &= !(0x3 << 4);
    rval |= (sdly & 0x30) << 4;
    write_reg(SHMC_BASE, ntsr, rval);
    /* Re-enable card clock */
    write_reg(
        SHMC_BASE,
        clkcr,
        read_reg::<u32>(SHMC_BASE, clkcr) | (0x1 << 16),
    );
    /* update clock */
    mmc_updata_clk();
}

unsafe fn gpio_init() {
    let mut cfg = read_reg::<u32>(GPIO_BASE, PF_CFG0) & 0x11000000 | 0x222222;
    cfg &= !0x1000000;
    cfg |= 0x222222;
    write_reg(GPIO_BASE, PF_CFG0, cfg);
    let mut drv = read_reg::<u32>(GPIO_BASE, PF_DRV0);
    drv &= !0x1000000;
    drv |= 0x222222;
    write_reg(GPIO_BASE, PF_DRV0, drv);
    let pull = read_reg::<u32>(GPIO_BASE, PF_PULL0) & 0x11111000 | 0x555;
    write_reg(GPIO_BASE, PF_PULL0, pull);
    println!("CFG: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_CFG0));
    println!("DAT: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_DAT));
    println!("DRV: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_DRV0));
    println!("PUL: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_PULL0));
}

unsafe fn __mmc_be32_to_cpu(x: u32) -> u32 {
    (0x000000ff & ((x) >> 24))
        | (0x0000ff00 & ((x) >> 8))
        | (0x00ff0000 & ((x) << 8))
        | (0xff000000 & ((x) << 24))
}

pub(crate) unsafe fn mmc_core_init() {
    let shmc = Peripherals::steal().SHMC;
    // step 1
    let rval = read_reg::<u32>(hclkbase, 0) | (1u32 << 16);
    write_reg(hclkbase, 0, rval);
    crate::peripheral::Timer.mdelay(1);
    let rval = read_reg::<u32>(hclkbase, 0) | (1u32);
    write_reg(hclkbase, 0, rval);
    let rval = (1u32 << 31) | (2 << 8) | 14;
    write_reg(mclkbase, 0, rval);
    shmc.thldc
        .write(|w| w.bits((512 << 16) | (1u32 << 2) | (1 << 0)));

    // step 2
    shmc.gctrl.write(|w| w.bits(0x7));
    loop {
        if shmc.gctrl.read().bits() & 0x7 == 0 {
            break;
        }
    }
    shmc.hwrst.write(|w| w.bits(1));
    shmc.hwrst.write(|w| w.bits(0));
    peripheral::Timer.udelay(1000);
    shmc.hwrst.write(|w| w.bits(1));
    peripheral::Timer.udelay(1000);
    shmc.thldc
        .write(|w| w.bits((512 << 16) | (1u32 << 2) | (1u32 << 0)));
    shmc.csdc.write(|w| w.bits(3));
    shmc.dbgc.write(|w| w.bits(0xdebu32));
    shmc.imask.write(|w| w.bits(0xFFCEu32));
    shmc.rint.write(|w| w.bits(0xFFFFFFFFu32));

    // step3
    let rval = shmc.clkcr.read().bits() | (0x1u32 << 16) | (0x1 << 31);
    shmc.clkcr.write(|w| w.bits(rval));
    shmc.cmd.write(|w| w.bits(0x80202000u32));
    crate::peripheral::Timer.mdelay(1);
    shmc.clkcr
        .write(|w| w.bits(shmc.clkcr.read().bits() & !(0x1 << 31)));
    shmc.timeout.write(|w| w.bits(0xFFFFFFFFu32));

    let rval = read_reg::<u32>(SHMC_BASE, gctrl) & !(1u32 << 10);

    write_reg(mclkbase, 0, read_reg::<u32>(mclkbase, 0) & !(1 << 31));
    write_reg(SHMC_BASE, gctrl, rval);
    write_reg(mclkbase, 0, read_reg::<u32>(mclkbase, 0) | (1 << 31));
    // step4

    // 0 -> 8 -> 55, 41 -> 2 ? -> 3 -> 9 -> 7 ->55, 6 -> 9 -> 13 -> 7
    let buf = [0u32; 128 * 4];

    let mut cmdtmp = MMC_CMD {
        cmdidx: 0x8000_8000u32,
        cmdarg: 0,
        resptype: MmcResp::MMC_RSP_NONE,
        resp0: [0; 4],
        flags: MmcFlags::MMC_DATA_NONE,
        resp1: buf.as_ptr() as usize,
        blocks: 0,
        blocksize: 0,
    };
    let mut mmc = MMC {
        OCR: 0,
        RCA: 0,
        CID: [0; 4],
        CSD: [0; 4],
        SCR: [0; 2],
        read_bl_len: 0,
        write_bl_len: 0,
        tran_speed: 0
    };
    cmdtmp.send(false);

    cmdtmp.cmdidx = 8;
    cmdtmp.cmdarg = (1u32 << 8) | 0xaa;
    cmdtmp.resptype = MmcResp::MMC_RSP_R7;
    cmdtmp.send(false);
    loop {
        cmdtmp.cmdidx = 55;
        cmdtmp.cmdarg = 0;
        cmdtmp.resptype = MmcResp::MMC_RSP_R1;
        cmdtmp.send(false);

        cmdtmp.cmdidx = 41;
        cmdtmp.cmdarg = 0xfe0000 & 0xff8000 | 0x40000000;
        cmdtmp.resptype = MmcResp::MMC_RSP_R3;
        cmdtmp.send(false);
        if cmdtmp.resp0[0] & 0x8000_0000u32 != 0 {
            break;
        }
    }
    mmc.OCR = cmdtmp.resp0[0];

    // mmc_startup
    /* Put the Card in Identify Mode */
    cmdtmp.cmdidx = 2;
    cmdtmp.cmdarg = 0;
    cmdtmp.resptype = MmcResp::MMC_RSP_R2;
    cmdtmp.send(false);
    cmdtmp.printinfo();
    mmc.CID = cmdtmp.resp0;
    cmdtmp.clear();
    /*
	 * For MMC cards, set the Relative Address.
	 * For SD cards, get the Relatvie Address.
	 * This also puts the cards into Standby State
	 */
    cmdtmp.cmdidx = 3;
    cmdtmp.cmdarg = 0;
    cmdtmp.resptype = MmcResp::MMC_RSP_R6;
    cmdtmp.send(false);
    cmdtmp.printinfo();
    mmc.RCA = cmdtmp.resp0[0] >> 16 & 0xFFFF;
    cmdtmp.clear();

    /* Get the Card-Specific Data */
    cmdtmp.cmdidx = 9;
    cmdtmp.cmdarg = mmc.RCA << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R2;
    cmdtmp.send(false);
    cmdtmp.printinfo();
    mmc.CSD = cmdtmp.resp0;
    let frep = [10000, 100000, 1000000, 10000000][(cmdtmp.resp0[0] & 0x7) as usize];
    let mult = [0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80][((cmdtmp.resp0[0] >> 3) & 0x7) as usize];
    mmc.read_bl_len = (mmc.CSD[1] >> 16) & 0xf;
    mmc.write_bl_len = mmc.read_bl_len;
    mmc.tran_speed = frep * mult;
    println!("read_bl_len: 0x{:x}", (mmc.CSD[1] >> 16) & 0xf);
    cmdtmp.clear();

    /* Waiting for the ready status */
    cmdtmp.cmdidx = 13;
    cmdtmp.cmdarg = mmc.RCA << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    loop {
        cmdtmp.send(false);
        cmdtmp.printinfo();
        if cmdtmp.resp0[0] & (1u32 << 8) != 0 {
            break;
        }
        crate::peripheral::Timer.mdelay(1);
        if cmdtmp.resp0[0] & !0x0206BF7F != 0 {
            panic!("mmc_status_error")
        }
        cmdtmp.clear();
    }

    /* Select the card, and put it into Transfer Mode */
    cmdtmp.cmdidx = 7;
    cmdtmp.cmdarg = mmc.RCA << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1B;
    cmdtmp.send(false);
    cmdtmp.printinfo();
    mmc_set_clock(25000000);
    crate::peripheral::Timer.mdelay(1000);


    // /* change freq */
    cmdtmp.cmdidx = 55;
    cmdtmp.cmdarg = mmc.RCA << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.clear();
    cmdtmp.send(false);

    cmdtmp.cmdidx = 51;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.cmdarg = 0;
    cmdtmp.blocks = 1;
    cmdtmp.blocksize = 8;
    cmdtmp.flags = MmcFlags::MMC_DATA_READ;
    cmdtmp.send(true);
    mmc.SCR[0] = __mmc_be32_to_cpu(cmdtmp.ptr2val::<u32>(0));
    mmc.SCR[1] = __mmc_be32_to_cpu(cmdtmp.ptr2val::<u32>(1));

    if mmc.SCR[0] & 0x40000 != 0 {
        println!("MMC_MODE_BIT 0x{:x}", mmc.SCR[0]);
    }

    println!("Version： {}", (mmc.SCR[0] >> 24) & 0xf);
    println!("MODEBIT:  0x{:x} 0x{:x}", mmc.SCR[0] & 0x40000, mmc.SCR[0]);

    for _ in 0..4 {
        cmdtmp.cmdidx = 6;
        cmdtmp.resptype = MmcResp::MMC_RSP_R1;
        cmdtmp.cmdarg = 0 << 31 | 0xffffff;
        cmdtmp.cmdarg &= !(0xf << (0 * 4));
        cmdtmp.cmdarg |= 1 << (0 * 4);

        cmdtmp.blocks = 1;
        cmdtmp.blocksize = 64;
        cmdtmp.flags = MmcFlags::MMC_DATA_READ;
        cmdtmp.send(true);
        println!("0x{:?}", cmdtmp.resp1);
        println!("{:?}", cmdtmp.resp0);
        println!("[CMDARG] 0x{:x}", cmdtmp.cmdarg);
        cmdtmp.print();
        if cmdtmp.ptr2val::<u32>(16) & 0xF == 1 {
            break;
        }
    }

    cmdtmp.cmdidx = 6;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.cmdarg = 1 << 31 | 0xffffff;
    cmdtmp.cmdarg &= !(0xf << (0 * 4));
    cmdtmp.cmdarg |= 1 << (0 * 4);
    cmdtmp.blocks = 1;
    cmdtmp.blocksize = 64;
    cmdtmp.flags = MmcFlags::MMC_DATA_READ;
    cmdtmp.send(true);
    println!("0x{:?}", cmdtmp.resp1);
    println!("{:?}", cmdtmp.resp0);
    println!("[CMDARG] 0x{:x}", cmdtmp.cmdarg);
    cmdtmp.print();
    println!("Ret： 0x{:x}", cmdtmp.ptr2val::<u8>(16));
    if cmdtmp.ptr2val::<u8>(16) != 1 {
        panic!("Make China Great Again!")
    }

    mmc_updata_clk();

    println!("Ret： 0x{:x}", cmdtmp.ptr2val::<u8>(16));
    println!("Ret： 0x{:x}", cmdtmp.ptr2val::<u32>(4));

    // set bus width
    cmdtmp.cmdidx = 55;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.cmdarg = mmc.RCA << 16;
    cmdtmp.send(false);

    cmdtmp.cmdidx = 6;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.cmdarg = 0;
    cmdtmp.send(false);

    let ind = cmdtmp.resp1 as * mut u32;
    for i in 0isize..128{
        ind.offset(i).write(0x43997a7a)
    }
    for i in 128isize..256{
        ind.offset(i).write(0xbbbbbbbb)
    }
    // //singal sector test
    // {
    //     cmdtmp.write_block(0, 1);
    //     cmdtmp.read_block(0, 1);
    // }
    // // mutlty sector test
    // {
    //     cmdtmp.write_block(0,2);
    //     cmdtmp.write_block(2,2);
    //     cmdtmp.read_block(0, 4);
    // }
    
    {
        cmdtmp.read_block(32832, 1);
    }

    println!("\n\n\n=======================================");
    println!("SHMC_THLDC: 0x{:x}", shmc.thldc.read().bits());
    println!("SHMC_BGR_REG: 0x{:x}", read_reg::<u32>(hclkbase, 0));
    println!("SHMC_CLK_REG: 0x{:x}", read_reg::<u32>(mclkbase, 0));
    println!("SHMC_GCTRL:   0x{:x}", read_reg::<u32>(SHMC_BASE, gctrl));
    println!("SHMC_CLKCR:   0x{:x}", read_reg::<u32>(SHMC_BASE, clkcr));
    println!("Source: {}", _get_pll_periph0() * 2 * 1000000);
    println!("clock: {}", mmc_get_clock());
    println!("CFG: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_CFG0));
    println!("DAT: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_DAT));
    println!("DRV: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_DRV0));
    println!("PUL: 0x{:x}", read_reg::<u32>(GPIO_BASE, PF_PULL0));

    // let vol = Volume::new(Card);
    // drop(buf);
    // let mut root = vol.root_dir();
    // root.create_file("carefile").unwrap();
    // let mut file = root.open_file("carefile").unwrap();
    // file.write(&[0x40u8; 10], fat32::file::WriteType::Append)
    //     .unwrap();
    // let mut buf = [0u8; 128];
    // println!("Origin {:?}", buf);
    // file.read(&mut buf).unwrap();
    // println!("{:?}", buf);
}

pub unsafe fn mmc_init_test() {
    gpio_init();
    mmc_core_init();
}
mod generic {
    use core::marker;
    use core::marker::PhantomData;
    use core::ops::Deref;
    pub trait Readable {}
    pub trait Writable {}

    pub trait ResetValue {
        ///Register size
        type Type;
        ///Reset value of the register
        fn reset_value() -> Self::Type;
    }
    pub struct Reg<U, REG> {
        register: vcell::VolatileCell<U>,
        _marker: marker::PhantomData<REG>,
    }
    unsafe impl<U: Send, REG> Send for Reg<U, REG> {}
    impl<U, REG> Reg<U, REG>
    where
        Self: Readable,
        U: Copy,
    {
        #[inline(always)]
        pub fn read(&self) -> R<U, Self> {
            R {
                bits: self.register.get(),
                _reg: marker::PhantomData,
            }
        }
    }
    impl<U, REG> Reg<U, REG>
    where
        Self: ResetValue<Type = U> + Writable,
        U: Copy,
    {
        #[inline(always)]
        pub fn reset(&self) {
            self.register.set(Self::reset_value())
        }
    }
    impl<U, REG> Reg<U, REG>
    where
        Self: ResetValue<Type = U> + Writable,
        U: Copy,
    {
        #[inline(always)]
        pub fn write<F>(&self, f: F)
        where
            F: FnOnce(&mut W<U, Self>) -> &mut W<U, Self>,
        {
            self.register.set(
                f(&mut W {
                    bits: Self::reset_value(),
                    _reg: marker::PhantomData,
                })
                .bits,
            );
        }
    }
    impl<U, REG> Reg<U, REG>
    where
        Self: Writable,
        U: Copy + Default,
    {
        #[inline(always)]
        pub fn write_with_zero<F>(&self, f: F)
        where
            F: FnOnce(&mut W<U, Self>) -> &mut W<U, Self>,
        {
            self.register.set(
                f(&mut W {
                    bits: U::default(),
                    _reg: marker::PhantomData,
                })
                .bits,
            );
        }
    }
    impl<U, REG> Reg<U, REG>
    where
        Self: Readable + Writable,
        U: Copy,
    {
        #[inline(always)]
        pub fn modify<F>(&self, f: F)
        where
            for<'w> F: FnOnce(&R<U, Self>, &'w mut W<U, Self>) -> &'w mut W<U, Self>,
        {
            let bits = self.register.get();
            self.register.set(
                f(
                    &R {
                        bits,
                        _reg: marker::PhantomData,
                    },
                    &mut W {
                        bits,
                        _reg: marker::PhantomData,
                    },
                )
                .bits,
            );
        }
    }
    pub struct R<U, T> {
        pub(crate) bits: U,
        _reg: marker::PhantomData<T>,
    }
    impl<U, T> R<U, T>
    where
        U: Copy,
    {
        ///Create new instance of reader
        #[inline(always)]
        pub(crate) fn new(bits: U) -> Self {
            Self {
                bits,
                _reg: marker::PhantomData,
            }
        }
        ///Read raw bits from register/field
        #[inline(always)]
        pub fn bits(&self) -> U {
            self.bits
        }
    }
    impl<U, T, FI> PartialEq<FI> for R<U, T>
    where
        U: PartialEq,
        FI: Copy + Into<U>,
    {
        #[inline(always)]
        fn eq(&self, other: &FI) -> bool {
            self.bits.eq(&(*other).into())
        }
    }
    impl<FI> R<bool, FI> {
        ///Value of the field as raw bits
        #[inline(always)]
        pub fn bit(&self) -> bool {
            self.bits
        }
        ///Returns `true` if the bit is clear (0)
        #[inline(always)]
        pub fn bit_is_clear(&self) -> bool {
            !self.bit()
        }
        ///Returns `true` if the bit is set (1)
        #[inline(always)]
        pub fn bit_is_set(&self) -> bool {
            self.bit()
        }
    }
    pub struct W<U, REG> {
        ///Writable bits
        pub(crate) bits: U,
        _reg: marker::PhantomData<REG>,
    }
    impl<U, REG> W<U, REG> {
        ///Writes raw bits to the register
        #[inline(always)]
        pub unsafe fn bits(&mut self, bits: U) -> &mut Self {
            self.bits = bits;
            self
        }
    }
    ///Used if enumerated values cover not the whole range
    #[derive(Clone, Copy, PartialEq)]
    pub enum Variant<U, T> {
        ///Expected variant
        Val(T),
        ///Raw bits
        Res(U),
    }

    pub struct SHMC {
        _marker: PhantomData<*const ()>,
    }
    unsafe impl Send for SHMC {}
    impl SHMC {
        pub const fn ptr() -> *const shmc::RegisterBlock {
            0x04020000u32 as *const _
        }
    }
    impl Deref for SHMC {
        type Target = shmc::RegisterBlock;

        fn deref(&self) -> &Self::Target {
            unsafe { &*SHMC::ptr() }
        }
    }
    pub mod shmc {
        use super::{Readable, Reg, Writable};
        #[repr(C)]
        pub struct RegisterBlock {
            pub gctrl: Register,
            pub clkcr: Register,
            pub timeout: Register,
            pub width: Register,
            pub blksz: Register,
            pub bytecnt: Register,
            pub cmd: Register,
            pub arg: Register,
            pub resp0: Register,
            pub resp1: Register,
            pub resp2: Register,
            pub resp3: Register,
            pub imask: Register,
            pub mint: Register,
            pub rint: Register,
            pub status: Register,
            pub ftrglevel: Register,
            pub funcsel: Register,
            pub cbcr: Register,
            pub bbcr: Register,
            pub dbgc: Register,
            pub csdc: Register,
            pub a12a: Register,
            pub ntsr: Register,
            pub res1: [Register; 6],
            pub hwrst: Register,
            pub res2: Register,
            pub dmac: Register,
            pub dlba: Register,
            pub idst: Register,
            pub idie: Register,
            pub chda: Register,
            pub cbda: Register,
            pub res3: [Register; 26],
            pub thldc: Register,
            pub sfc: Register,
            pub res4: Register,
            pub dsbd: Register,
            pub res5: [Register; 12],
            pub drv_dl: Register,
            pub samp_dl: Register,
            pub ds_dl: Register,
            pub res6: [Register; 45],
            pub fifo: Register,
        }
        pub struct _REGISTER;
        type Register = Reg<u32, _REGISTER>;
        impl Readable for Register {}
        impl Writable for Register {}
        pub mod register {
            use crate::generic::ResetValue;

            use super::Register;

            pub type R = crate::generic::R<u32, super::Register>;
            pub type W = crate::generic::W<u32, super::Register>;
            impl ResetValue for Register {
                type Type = u32;
                fn reset_value() -> Self::Type {
                    0
                }
            }
            impl R {}
            impl W {}
        }
    }
    pub struct Peripherals {
        pub SHMC: SHMC,
    }
    impl Peripherals {
        pub unsafe fn steal() -> Self {
            Peripherals {
                SHMC: SHMC {
                    _marker: PhantomData,
                },
            }
        }
    }
}
mod peripheral {
    pub struct Timer;
    impl Timer {
        pub fn get_arch_counter(&self) -> u64 {
            let mtime: u64;
            unsafe { asm!("csrr {}, time", out(reg) mtime) }
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
}
mod serial {
    use core::{
        convert::Infallible,
        ptr::{read_volatile, write_volatile},
    };
    use embedded_hal::serial::{Read, Write};
    pub const UART_THR: usize = 0;
    pub const UART_RBR: usize = 0;
    pub const UART_LSR: usize = 0x14;
    pub const UART_USR: usize = 0x7c;
    #[inline]
    pub unsafe fn write_reg<T>(addr: usize, offset: usize, val: T) {
        write_volatile((addr + offset) as *mut T, val);
    }

    #[inline]
    pub unsafe fn read_reg<T>(addr: usize, offset: usize) -> T {
        read_volatile((addr + offset) as *const T)
    }
    const SUNXI_UART_USR_RFNE: u32 = 0x04;
    pub struct Serial {
        uart: usize,
    }
    impl Serial {
        pub fn new(base: usize) -> Self {
            Self { uart: base }
        }
    }
    impl Read<u8> for Serial {
        type Error = Infallible;

        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            if unsafe { (read_reg::<u32>(self.uart, UART_LSR) & (1 << 0)) != 0 } {
                Ok(unsafe { (read_reg::<u32>(self.uart, UART_RBR) & 0xff) as u8 })
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
    impl Write<u8> for Serial {
        type Error = Infallible;

        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            while unsafe { read_reg::<u32>(self.uart, UART_USR) & SUNXI_UART_USR_RFNE } == 0 {}
            unsafe { write_reg::<u32>(self.uart, UART_THR, word as u32) }
            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            while unsafe { read_reg::<u32>(self.uart, UART_USR) & SUNXI_UART_USR_RFNE } == 0 {}
            Ok(())
        }
    }
}
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
