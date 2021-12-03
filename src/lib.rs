#![no_std]
#![feature(asm)]

extern crate alloc;
#[macro_use]
extern crate log;

use spin::Mutex;

mod error;
mod generic;
mod peripheral;

use error::*;
use generic::{Peripherals, SHMC};
use peripheral::Timer;

const GPIO_BASE: usize = 0x02000000;
const PF_CFG0: usize = 0xF0;
const PF_DRV0: usize = 0x104;
const PF_PULL0: usize = 0x114;
const HCLKBASE: usize = 0x2001000 + 0x84c;
const MCLKBASE: usize = 0x2001000 + 0x830;

bitflags::bitflags! {
    pub struct SunxiMmcRing:u32 {
        const SUNXI_MMC_RINT_RESP_ERROR		= 1 << 1;
        const SUNXI_MMC_RINT_COMMAND_DONE	=	1 << 2;
        const SUNXI_MMC_RINT_DATA_OVER	=	1 << 3;
        const SUNXI_MMC_RINT_TX_DATA_REQUEST	=	1 << 4;
        const SUNXI_MMC_RINT_RX_DATA_REQUEST	=	1 << 5;
        const SUNXI_MMC_RINT_RESP_CRC_ERROR	=	1 << 6;
        const SUNXI_MMC_RINT_DATA_CRC_ERROR	=   1 << 7;
        const SUNXI_MMC_RINT_RESP_TIMEOUT	=	1 << 8;
        const SUNXI_MMC_RINT_DATA_TIMEOUT	=	1 << 9;
        const SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE=	1 << 10;
        const SUNXI_MMC_RINT_FIFO_RUN_ERROR	=	1 << 11;
        const SUNXI_MMC_RINT_HARD_WARE_LOCKED	=	1 << 12;
        const SUNXI_MMC_RINT_START_BIT_ERROR	=	1 << 13;
        const SUNXI_MMC_RINT_AUTO_COMMAND_DONE=	1 << 14;
        const SUNXI_MMC_RINT_END_BIT_ERROR	=	1 << 15;
        const SUNXI_MMC_RINT_SDIO_INTERRUPT	=	1 << 16;
        const SUNXI_MMC_RINT_CARD_INSERT	=	1 << 30;
        const SUNXI_MMC_RINT_CARD_REMOVE	=	1 << 31;
        const SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT =
            SunxiMmcRing::SUNXI_MMC_RINT_RESP_ERROR.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_RESP_CRC_ERROR.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_DATA_CRC_ERROR.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_RESP_TIMEOUT.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_DATA_TIMEOUT.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_VOLTAGE_CHANGE_DONE.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_FIFO_RUN_ERROR.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_HARD_WARE_LOCKED.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_START_BIT_ERROR.bits |
            SunxiMmcRing::SUNXI_MMC_RINT_END_BIT_ERROR.bits;
    }

    struct MmcResp:u32{
        const MMC_RSP_PRESENT = 1 << 0;
        const MMC_RSP_136     = 1 << 1;
        const MMC_RSP_CRC     = 1 << 2;
        const MMC_RSP_BUSY    = 1 << 3;
        const MMC_RSP_OPCODE  = 1 << 4;
        const MMC_RSP_NONE    = 0;
        const MMC_RSP_R1      =
            MmcResp::MMC_RSP_PRESENT.bits |
            MmcResp::MMC_RSP_CRC.bits |
            MmcResp::MMC_RSP_OPCODE.bits;
        const MMC_RSP_R1B     =
            MmcResp::MMC_RSP_PRESENT.bits |
            MmcResp::MMC_RSP_CRC.bits |
            MmcResp::MMC_RSP_OPCODE.bits |
            MmcResp::MMC_RSP_BUSY.bits;
        const MMC_RSP_R2      =
            MmcResp::MMC_RSP_PRESENT.bits |
            MmcResp::MMC_RSP_136.bits |
            MmcResp::MMC_RSP_CRC.bits;
        const MMC_RSP_R3      = MmcResp::MMC_RSP_PRESENT.bits;
        const MMC_RSP_R6      =
            MmcResp::MMC_RSP_PRESENT.bits |
            MmcResp::MMC_RSP_CRC.bits |
            MmcResp::MMC_RSP_OPCODE.bits;
        const MMC_RSP_R7      =
            MmcResp::MMC_RSP_PRESENT.bits |
            MmcResp::MMC_RSP_CRC.bits |
            MmcResp::MMC_RSP_OPCODE.bits;
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
    static ref RCA: Mutex<u32> = Mutex::new(0);
}

pub struct MMC {
    pub ocr: u32,
    pub cid: [u32; 4],
    pub csd: [u32; 4],
    pub rca: u32,
    pub scr: [u32; 2],
    pub read_bl_len: u32,
    pub write_bl_len: u32,
    pub tran_speed: u32,
}

impl MMC {
    pub fn new() -> Self {
        MMC {
            ocr: 0,
            rca: 0,
            cid: [0; 4],
            csd: [0; 4],
            scr: [0; 2],
            read_bl_len: 0,
            write_bl_len: 0,
            tran_speed: 0,
        }
    }
}

impl Default for MmcHost {
    fn default() -> Self {
        Self::new()
    }
}

pub struct MmcHost {
    cmdidx: u32,
    cmdarg: u32,
    resptype: MmcResp,
    resp0: [u32; 4],
    resp1: usize,
    flags: MmcFlags,
    blocks: u32,
    blocksize: u32,
    rca: u32,
    shmc: SHMC,
}

impl MmcHost {
    pub fn new() -> Self {
        Self {
            cmdidx: 0,
            cmdarg: 0,
            resptype: MmcResp::MMC_RSP_NONE,
            resp0: [0; 4],
            resp1: 0,
            flags: MmcFlags::MMC_DATA_NONE,
            blocks: 0,
            blocksize: 0,
            shmc: unsafe { Peripherals::steal().shmc },
            rca: 0,
        }
    }

    pub fn set_data(&mut self, data: usize) {
        self.resp1 = data;
    }

    pub unsafe fn mmc_rint_wait(
        &mut self,
        done_flag: SunxiMmcRing,
        timeout_msecs: u64,
    ) -> SdResult {
        let timeout = Timer.get_us() + timeout_msecs;
        loop {
            let status_bits = self.shmc.rint.read().bits();
            if Timer.get_us() > timeout {
                return Err(SdError::RintTimeout);
            }
            if status_bits & SunxiMmcRing::SUNXI_MMC_RINT_INTERRUPT_ERROR_BIT.bits() != 0 {
                self.error_recover();
                return Err(SdError::RintError);
            }
            if status_bits & done_flag.bits() != 0 {
                break;
            }
        }

        Ok(())
    }

    pub unsafe fn error_recover(&mut self) {
        self.shmc.gctrl.write(|w| w.bits(0x7));
        loop {
            if self.shmc.gctrl.read().bits() & 0x7 == 0 {
                break;
            }
        }
        self.shmc.gctrl.write(|w| w.bits(0x7));
        mmc_updata_clk().unwrap();
        self.shmc.rint.write(|w| w.bits(0xFFFFFFFF));
        self.shmc
            .gctrl
            .write(|w| w.bits(self.shmc.gctrl.read().bits() | (1 << 1)));
    }

    pub unsafe fn send(&mut self, data: bool) -> SdResult {
        debug!("\n[CMD] SEND CMD {}", self.cmdidx & 0x3F);
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
        if self.cmdidx == 0 {
            cmdval |= 1 << 15;
        }
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
            self.shmc.blksz.write(|w| w.bits(self.blocksize));
            self.shmc
                .bytecnt
                .write(|w| w.bits(self.blocks * self.blocksize));
        } else if (self.cmdidx == 12) && self.flags.contains(MmcFlags::MMC_CMD_MANUAL) {
            cmdval |= 1 << 14;
            cmdval &= !(1 << 13);
        }
        self.shmc.arg.write(|w| w.bits(self.cmdarg));

        if data {
            self.shmc
                .gctrl
                .write(|w| w.bits(self.shmc.gctrl.read().bits() | 0x80000000u32));
            self.shmc.cmd.write(|w| w.bits(self.cmdidx | cmdval));
            self.trans_data_by_cpu().unwrap();
            debug!("GCTRL: 0x{:x}", self.shmc.gctrl.read().bits());
            debug!("CMD: 0x{:x}", self.shmc.cmd.read().bits());
            debug!("BtCnt: 0x{:x}", self.shmc.bytecnt.read().bits());
        } else {
            debug!("CMDIDX: 0x{:x}", self.cmdidx | cmdval);
            self.shmc.cmd.write(|w| w.bits(self.cmdidx | cmdval));
        }

        self.mmc_rint_wait(SunxiMmcRing::SUNXI_MMC_RINT_COMMAND_DONE, 0xffffff)?;

        if data {
            self.mmc_rint_wait(
                if self.blocks > 1 {
                    SunxiMmcRing::SUNXI_MMC_RINT_AUTO_COMMAND_DONE
                } else {
                    SunxiMmcRing::SUNXI_MMC_RINT_DATA_OVER
                },
                0xffff,
            )?
        }
        if self.resptype.contains(MmcResp::MMC_RSP_BUSY) {
            let timeout = Timer.get_us() + 0x4ffffff;
            loop {
                if Timer.get_us() > timeout {
                    return Err(SdError::MMCSendTimeout);
                }

                if self.shmc.status.read().bits() & (1 << 9) == 0 {
                    break;
                }
            }
        }
        if self.resptype.contains(MmcResp::MMC_RSP_136) {
            self.resp0[0] = self.shmc.resp3.read().bits();
            self.resp0[1] = self.shmc.resp2.read().bits();
            self.resp0[2] = self.shmc.resp1.read().bits();
            self.resp0[3] = self.shmc.resp0.read().bits();
        } else {
            self.resp0[0] = self.shmc.resp0.read().bits();
        }
        debug!("SHMC_RINT:   0x{:x}", self.shmc.rint.read().bits());
        self.shmc.rint.write(|w| w.bits(0xFFFFFFFF));
        self.shmc
            .gctrl
            .write(|w| w.bits(self.shmc.gctrl.read().bits() | (1 << 1)));
        Ok(())
    }

    pub unsafe fn trans_data_by_cpu(&mut self) -> SdResult<usize> {
        let shmc = Peripherals::steal().shmc;
        let mut timeout = Timer.get_us() + 0xffffff;
        let byte_cnt = (self.blocks * self.blocksize) as usize;
        debug!("bytecnt {:x}", byte_cnt >> 2);
        debug!("SHMC_status: 0x{:x}", self.shmc.status.read().bits());
        let dst = self.resp1 as *mut u32;
        let status_bit = match self.flags {
            MmcFlags::MMC_DATA_READ => 1 << 2,
            MmcFlags::MMC_DATA_WRITE => 1 << 3,
            _ => 0,
        };
        for i in 0..(byte_cnt >> 2) {
            loop {
                if shmc.status.read().bits() & status_bit == 0 {
                    break;
                }
                if Timer.get_us() > timeout {
                    return Err(SdError::TranslateDataTimeout);
                }
            }
            match self.flags {
                MmcFlags::MMC_DATA_READ => {
                    dst.offset(i as isize)
                        .write_volatile(shmc.fifo.read().bits());
                }
                MmcFlags::MMC_DATA_WRITE => {
                    shmc.fifo
                        .write(|w| w.bits(dst.offset(i as isize).read_volatile()));
                }
                _ => return Err(SdError::WrongDataFlag),
            }
            timeout = Timer.get_us() + 0xffffff;
        }
        Ok(byte_cnt)
    }

    pub unsafe fn ptr2val<T>(&self, offset: isize) -> T {
        (self.resp1 as *const T).offset(offset).read_volatile()
    }

    pub unsafe fn mmc_send_status(&mut self) -> SdResult {
        // Timer.mdelay(10);
        self.cmdidx = 13;
        self.resptype = MmcResp::MMC_RSP_R1;
        self.cmdarg = *RCA.lock() << 16;
        let mut retries = 10;
        loop {
            if let Ok(_) = self.send(false) {
                if self.resp0[0] & 1 << 8 != 0 && self.resp0[0] & (0xf << 9) != 7 << 9 {
                    return Ok(());
                }
            } else {
                retries = retries - 1;
                if retries < 0 {
                    return Err(SdError::SdcardBusy);
                }
            }
            Timer.udelay(1000)
        }
    }

    pub unsafe fn read_block(&mut self, start: u32, blkcnt: u32) {
        self.cmdidx = match blkcnt {
            1 => 17,
            _ => 18,
        };
        debug!("CMDIDX: {} start: {}", self.cmdidx, start);
        self.resptype = MmcResp::MMC_RSP_R1;
        self.cmdarg = start;
        self.blocks = blkcnt;
        self.blocksize = 512;
        self.flags = MmcFlags::MMC_DATA_READ;
        self.send(true).unwrap();
        // self.dump();
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
        self.send(true).unwrap();
        Timer.mdelay(10);
        self.mmc_send_status().unwrap()
    }

    pub unsafe fn dump(&self) {
        debug!("Count:0x{:x}", self.blocks * self.blocksize * 4);
        let index = self.resp1 as *const u8;
        for i in 0..(self.blocks * self.blocksize) as isize {
            debug!("0x{:<02x}, ", index.offset(i).read());
        }
    }

    pub unsafe fn dumpinfo(&self) {
        debug!("SHMC STATUS: 0x{:x}", self.shmc.status.read().bits());
        debug!("SHMC CMD:    0x{:x}", self.shmc.cmd.read().bits());
        debug!(
            "RESPONSES:   [0x{:x}, 0x{:x}, 0x{:x}, 0x{:x}]",
            self.resp0[0], self.resp0[1], self.resp0[2], self.resp0[3]
        );
    }

    pub unsafe fn clear(&mut self) {
        self.resp0.iter_mut().for_each(|f| *f = 0);

        debug!("Count:0x{:x}", self.blocks * self.blocksize * 4);
        for i in 0..(self.blocks * self.blocksize) as usize {
            *((self.resp1 + i) as *mut u8) = 0;
        }
    }
}

#[inline]
pub unsafe fn write_reg<T>(addr: usize, offset: usize, val: T) {
    core::ptr::write_volatile((addr + offset) as *mut T, val);
}

#[inline]
pub unsafe fn read_reg<T>(addr: usize, offset: usize) -> T {
    core::ptr::read_volatile((addr + offset) as *const T)
}

pub unsafe fn _get_pll_periph0() -> u32 {
    let regv = read_reg::<u32>(0x02001000, 0x20);
    let n = ((regv >> 8) & 0xff) + 1;
    let m = ((regv >> 1) & 0x01) + 1;
    let p = ((regv >> 16) & 0x07) + 1;
    24 * n / m / p / 2
}

pub unsafe fn mmc_clk_io_onoff(onoff: bool, clk: u32) {
    if onoff {
        let rval = read_reg::<u32>(HCLKBASE, 0) | 1;
        write_reg(HCLKBASE, 0, rval);
        let rval = read_reg::<u32>(HCLKBASE, 0) | 1 << 16;
        write_reg(HCLKBASE, 0, rval);
        let rval = read_reg::<u32>(MCLKBASE, 0) | 1 << 31;
        write_reg(MCLKBASE, 0, rval);
    } else {
        let rval = read_reg::<u32>(MCLKBASE, 0) & !(1 << 31);
        write_reg(MCLKBASE, 0, rval);
        let rval = read_reg::<u32>(HCLKBASE, 0) & !1;
        write_reg(HCLKBASE, 0, rval);
        let rval = read_reg::<u32>(HCLKBASE, 0) & !(1 << 16);
        write_reg(HCLKBASE, 0, rval);
    }
    if clk > 0 {
        let rval = read_reg::<u32>(MCLKBASE, 0) & !(0x7fffffff);
        write_reg(MCLKBASE, 0, rval);
    }
}

pub unsafe fn mmc_updata_clk() -> SdResult {
    let shmc = Peripherals::steal().shmc;
    let timeout = Timer.get_us() + 0xfffff;
    shmc.clkcr
        .write(|w| w.bits(shmc.clkcr.read().bits() | (1 << 31)));
    shmc.cmd
        .write(|w| w.bits((1u32 << 31) | (1 << 21) | (1 << 13)));
    loop {
        if shmc.cmd.read().bits() & 0x8000_0000u32 == 0 && Timer.get_us() < timeout {
            break;
        }
    }
    if shmc.cmd.read().bits() & 0x8000_0000u32 != 0 {
        return Err(SdError::UpdateClockFail);
    }
    shmc.clkcr
        .write(|w| w.bits(shmc.clkcr.read().bits() & !(1 << 31)));
    shmc.rint.write(|w| w.bits(0xFFFFFFFFu32));
    Ok(())
}

pub unsafe fn mmc_get_clock() -> u32 {
    let rval = read_reg::<u32>(MCLKBASE, 0);
    let m = rval & 0xf;
    let n = (rval >> 8) & 0x3;
    let src = (rval >> 24) & 0x3;
    let sclk_hz = match src {
        0 => 24000000,
        2 => _get_pll_periph0() * 2 * 1000000, /* use 2x pll6 */
        _ => {
            return 0;
        }
    };
    sclk_hz / (1 << n) / (m + 1)
}

pub unsafe fn mmc_set_clock(clock: u32) {
    let shmc = Peripherals::steal().shmc;
    /* disable card clock */
    let rval = shmc.clkcr.read().bits() & !(1 << 16);
    shmc.clkcr.write(|w| w.bits(rval));
    /* updata clock */
    mmc_updata_clk().unwrap();

    /* disable mclk */
    write_reg(MCLKBASE, 0, 0u32);
    shmc.ntsr
        .write(|w| w.bits(shmc.ntsr.read().bits() | (1 << 31)));
    let (src, sclk_hz): (u32, u32) = if clock <= 4000000 {
        (0, 24000000)
    } else {
        (2, _get_pll_periph0() * 2 * 1000000)
    };
    let mut div = (2 * sclk_hz + clock) / (2 * clock);
    if div == 0 {
        div = 1;
    }
    let (m, n) = if div > 128 {
        (1, 0)
    } else {
        let n = if div > 64 {
            3
        } else if div > 32 {
            2
        } else if div > 16 {
            1
        } else {
            0
        };
        (div >> n, n)
    };

    write_reg(MCLKBASE, 0, (src << 24) | (n << 8) | (m - 1));

    /* re-enable mclk */
    write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) | (1u32 << 31));
    shmc.clkcr
        .write(|w| w.bits(shmc.clkcr.read().bits() & !(0xff)));
    /* update clock */
    mmc_updata_clk().unwrap();

    /* config delay */
    let odly = 0;
    let sdly = 0;
    let mut rval = shmc.drv_dl.read().bits();
    rval |= ((odly & 0x1) << 16) | ((odly & 0x1) << 17);
    write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) & !(1 << 31));
    shmc.drv_dl.write(|w| w.bits(rval));
    write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) | (1 << 31));

    let mut rval = shmc.ntsr.read().bits();
    rval &= !(0x3 << 4);
    rval |= (sdly & 0x30) << 4;
    shmc.ntsr.write(|w| w.bits(rval));
    /* Re-enable card clock */
    shmc.clkcr
        .write(|w| w.bits(shmc.clkcr.read().bits() | (0x1 << 16)));
    /* update clock */
    mmc_updata_clk().unwrap();
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
}

unsafe fn __mmc_be32_to_cpu(x: u32) -> u32 {
    (0x000000ff & ((x) >> 24))
        | (0x0000ff00 & ((x) >> 8))
        | (0x00ff0000 & ((x) << 8))
        | (0xff000000 & ((x) << 24))
}

pub(crate) unsafe fn mmc_core_init() -> SdResult {
    let shmc = Peripherals::steal().shmc;
    // step 1
    write_reg(HCLKBASE, 0, 0x10000);
    Timer.mdelay(1);
    write_reg(HCLKBASE, 0, read_reg::<u32>(HCLKBASE, 0) | (1u32));
    write_reg(MCLKBASE, 0, (1u32 << 31) | (2 << 8) | 14);
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
    Timer.mdelay(1);
    shmc.clkcr
        .write(|w| w.bits(shmc.clkcr.read().bits() & !(0x1 << 31)));
    let rval = shmc.gctrl.read().bits() & !(1u32 << 10);
    write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) & !(1 << 31));
    shmc.gctrl.write(|w| w.bits(rval));
    write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) | (1 << 31));
    // step4

    // 0 -> 8 -> 55, 41 -> 2 ? -> 3 -> 9 -> 7 ->55, 6 -> 9 -> 13 -> 7
    let buf = [0u8; 512 * 4];
    let mut cmdtmp = MmcHost::new();
    cmdtmp.set_data(buf.as_ptr() as *const _ as usize);
    let mut mmc = MMC::new();
    cmdtmp.send(false).unwrap();

    cmdtmp.cmdidx = 8;
    cmdtmp.cmdarg = (1u32 << 8) | 0xaa;
    cmdtmp.resptype = MmcResp::MMC_RSP_R7;
    cmdtmp.send(false).unwrap();
    loop {
        cmdtmp.cmdidx = 55;
        cmdtmp.cmdarg = 0;
        cmdtmp.resptype = MmcResp::MMC_RSP_R1;
        cmdtmp.send(false).unwrap();

        cmdtmp.cmdidx = 41;
        cmdtmp.cmdarg = 0xfe0000 & 0xff8000 | 0x40000000;
        cmdtmp.resptype = MmcResp::MMC_RSP_R3;
        cmdtmp.send(false).unwrap();
        if cmdtmp.resp0[0] & 0x8000_0000u32 != 0 {
            break;
        }
    }
    mmc.ocr = cmdtmp.resp0[0];

    // mmc_startup
    /* Put the Card in Identify Mode */
    cmdtmp.cmdidx = 2;
    cmdtmp.cmdarg = 0;
    cmdtmp.resptype = MmcResp::MMC_RSP_R2;
    cmdtmp.send(false).unwrap();
    cmdtmp.dumpinfo();
    mmc.cid = cmdtmp.resp0;
    cmdtmp.clear();
    /*
     * For MMC cards, set the Relative Address.
     * For SD cards, get the Relatvie Address.
     * This also puts the cards into Standby State
     */
    cmdtmp.cmdidx = 3;
    cmdtmp.cmdarg = 0;
    cmdtmp.resptype = MmcResp::MMC_RSP_R6;
    cmdtmp.send(false).unwrap();
    cmdtmp.dumpinfo();
    mmc.rca = cmdtmp.resp0[0] >> 16 & 0xFFFF;
    *RCA.lock() = mmc.rca;
    cmdtmp.rca = mmc.rca;
    cmdtmp.clear();

    /* Get the Card-Specific Data */
    cmdtmp.cmdidx = 9;
    cmdtmp.cmdarg = mmc.rca << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R2;
    cmdtmp.send(false).unwrap();
    cmdtmp.dumpinfo();
    mmc.csd = cmdtmp.resp0;
    let frep = [10000, 100000, 1000000, 10000000][(cmdtmp.resp0[0] & 0x7) as usize];
    let mult = [
        0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
    ][((cmdtmp.resp0[0] >> 3) & 0x7) as usize];
    mmc.read_bl_len = 1 << ((mmc.csd[1] >> 16) & 0xf);
    mmc.write_bl_len = mmc.read_bl_len;
    mmc.tran_speed = frep * mult;

    cmdtmp.clear();

    /* Waiting for the ready status */
    cmdtmp.cmdidx = 13;
    cmdtmp.cmdarg = *RCA.lock() << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    loop {
        cmdtmp.send(false).unwrap();
        cmdtmp.dumpinfo();
        if cmdtmp.resp0[0] & (1u32 << 8) != 0 {
            break;
        }
        Timer.mdelay(1);
        if cmdtmp.resp0[0] & !0x0206BF7F != 0 {
            return Err(SdError::MMCStatusError);
        }
        cmdtmp.clear();
    }

    /* Select the card, and put it into Transfer Mode */
    cmdtmp.cmdidx = 7;
    cmdtmp.cmdarg = mmc.rca << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1B;
    cmdtmp.send(false).unwrap();
    cmdtmp.dumpinfo();
    mmc_set_clock(25000000);
    Timer.mdelay(1000);

    // /* change freq */
    cmdtmp.cmdidx = 55;
    cmdtmp.cmdarg = mmc.rca << 16;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.clear();
    cmdtmp.send(false).unwrap();

    cmdtmp.cmdidx = 51;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.cmdarg = 0;
    cmdtmp.blocks = 1;
    cmdtmp.blocksize = 8;
    cmdtmp.flags = MmcFlags::MMC_DATA_READ;
    cmdtmp.send(true).unwrap();
    mmc.scr[0] = __mmc_be32_to_cpu(cmdtmp.ptr2val::<u32>(0));
    mmc.scr[1] = __mmc_be32_to_cpu(cmdtmp.ptr2val::<u32>(1));

    if mmc.scr[0] & 0x40000 != 0 {
        debug!("MMC_MODE_BIT 0x{:x}", mmc.scr[0]);
    }
    for _ in 0..4 {
        cmdtmp.cmdidx = 6;
        cmdtmp.resptype = MmcResp::MMC_RSP_R1;
        cmdtmp.cmdarg = 0 << 31 | 0xffffff; // ???
        cmdtmp.cmdarg &= !(0xf << (0 * 4)); // ???
        cmdtmp.cmdarg |= 1 << (0 * 4); // ???
        cmdtmp.blocks = 1;
        cmdtmp.blocksize = 64;
        cmdtmp.flags = MmcFlags::MMC_DATA_READ;
        cmdtmp.send(true).unwrap();
        if cmdtmp.ptr2val::<u32>(16) & 0xF == 1 {
            break;
        }
    }

    cmdtmp.cmdidx = 6;
    cmdtmp.resptype = MmcResp::MMC_RSP_R1;
    cmdtmp.cmdarg = 1 << 31 | 0xffffff;
    cmdtmp.cmdarg &= !(0xf << (0 * 4)); // ???
    cmdtmp.cmdarg |= 1 << (0 * 4); // ???
    cmdtmp.blocks = 1;
    cmdtmp.blocksize = 64;
    cmdtmp.flags = MmcFlags::MMC_DATA_READ;
    cmdtmp.send(true).unwrap();
    if cmdtmp.ptr2val::<u8>(16) & 0xf != 1 {
        return Err(SdError::SdcardBusy);
    }

    mmc_updata_clk().unwrap();

    #[cfg(feature = "iotest")]
    {
        let ind = cmdtmp.resp1 as *mut u32;
        for i in 0isize..128 {
            ind.offset(i).write(0x41424344)
        }
        for i in 128isize..256 {
            ind.offset(i).write(0xABCDEF)
        }
        {
            cmdtmp.write_block(0, 1);
            cmdtmp.read_block(0, 1);
        }
        {
            cmdtmp.write_block(0, 2);
            cmdtmp.write_block(2, 2);
            cmdtmp.read_block(0, 4);
        }
        for _ in 0..10 {
            cmdtmp.write_block(100, 1);
            cmdtmp.read_block(100, 1);
        }
    }
    Ok(())
}

pub unsafe fn sdcard_init() {
    gpio_init();
    mmc_core_init().unwrap();
}

#[cfg(test)]
mod tests {
    use super::sdcard_init;

    #[test]
    fn it_works() {
        unsafe { sdcard_init() };
    }
}
