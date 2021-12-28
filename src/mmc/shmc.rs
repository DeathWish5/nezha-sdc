use super::*;

/* #REF: D1 User Manual v0.1 section 7.2 (https://open.allwinnertech.com/#/doc?menuID=2) */

#[repr(C)]
pub struct ShmcRegs {
    pub gctrl: Reg,
    pub clkcr: Reg,
    pub timeout: Reg,
    pub width: Reg,
    pub blksz: Reg,
    pub bytecnt: Reg,
    pub cmd: Reg,
    pub arg: Reg,
    pub resp0: Reg,
    pub resp1: Reg,
    pub resp2: Reg,
    pub resp3: Reg,
    pub imask: Reg,
    pub mint: Reg,
    pub rint: Reg,
    pub status: Reg,
    pub ftrglevel: Reg,
    pub funcsel: Reg,
    pub cbcr: Reg,
    pub bbcr: Reg,
    pub dbgc: Reg,
    pub csdc: Reg,
    pub a12a: Reg,
    pub ntsr: Reg,
    pub res1: [Reg; 6],
    pub hwrst: Reg,
    pub res2: Reg,
    pub dmac: Reg,
    pub dlba: Reg,
    pub idst: Reg,
    pub idie: Reg,
    pub chda: Reg,
    pub cbda: Reg,
    pub res3: [Reg; 26],
    pub thldc: Reg,
    pub sfc: Reg,
    pub res4: Reg,
    pub dsbd: Reg,
    pub res5: [Reg; 12],
    pub drv_dl: Reg,
    pub samp_dl: Reg,
    pub ds_dl: Reg,
    pub res6: [Reg; 45],
    pub fifo: Reg,
}

pub(crate) const SHMC_RINT: usize = 0x38;
pub(crate) const SHMC_MINT: usize = 0x34;

bitflags::bitflags! {
    pub struct GctrlReg : u32 {
        const SOFT_RST = 1 << 0;
        const FIFO_RST = 1 << 1;
        const DMA_RST = 1 << 2;
        const GINT_ENABLE = 1 << 4;
        const DMA_ENABLE = 1 << 5;
        const CARD_DECT_ENABLE = 1 << 8;
        const DDR_MODE = 1 << 10; // SDR_MODE = 0 << 10;
        const TIME_UINT_DAT = 1 << 11;
        const TIME_UINT_CMD = 1 << 12;
        const FIFO_AC_MOD = 1 << 31;

        const RESET_ALL = GctrlReg::SOFT_RST.bits() | GctrlReg::FIFO_RST.bits() | GctrlReg::DMA_RST.bits();
    }

    pub struct IntMask : u32 {
        const RESP_ERROR		= 1 << 1;
        const COMMAND_DONE	=	1 << 2;
        const DATA_OVER	=	1 << 3;
        const TX_DATA_REQUEST	=	1 << 4;
        const RX_DATA_REQUEST	=	1 << 5;
        const RESP_CRC_ERROR	=	1 << 6;
        const DATA_CRC_ERROR	=   1 << 7;
        const RESP_TIMEOUT	=	1 << 8;
        const DATA_TIMEOUT	=	1 << 9;
        const VOLTAGE_CHANGE_DONE=	1 << 10;
        const FIFO_RUN_ERROR	=	1 << 11;
        const HARD_WARE_LOCKED	=	1 << 12;
        const START_BIT_ERROR	=	1 << 13;
        const AUTO_COMMAND_DONE=	1 << 14;
        const END_BIT_ERROR	=	1 << 15;
        const SDIO_INTERRUPT	=	1 << 16;
        const CARD_INSERT	=	1 << 30;
        const CARD_REMOVE	=	1 << 31;

        const DEFAULT_MASK = 0xFFCE;
        const DEFAULT_ERROR = 0xBBC2;
        const ALL = 0xFFFFFFFF;
    }

    pub struct ClkCtrlReg : u32 {
        // 7:0 - card clock divider
        const CLK_ENABLE = 1 << 16;
        const CLK_OUTPUT_CTR = 1 << 17;
        const MASK_DATA0 = 1 << 31;
    }

    pub struct ShmcCmd : u32 {
        // 5:0 - cmd index
        const HAS_RESP = 1 << 6;
        const LONG_RESP = 1 << 7;
        const RESP_CRC = 1 << 8;
        // 9 - with data transfer or not
        const HAS_DATA = 1 << 9;
        // 10 - data direction
        const DATA_READ = 0 << 10;
        const DATA_WRITE = 1 << 10;
        // 11 - transfer mode
        const BLOCK_MODE = 0 << 11;
        const STEAM_MODE = 1 << 11;
        const STOP_AUTO = 1 << 12;
        const SYNC_TRANS = 1 << 13;
        const STOP_ABORT = 1 << 14;
        const INIT_CMD = 1 << 15;
        const CLK_CHANGE = 1 << 21;
        // 25:24 - boot mode, zero -> not a boot cmd
        const MAND_BOOT = 0x1 << 24;
        const ALT_BOOT = 0x2 << 24;
        const EXP_BOOT_ACK = 1 << 26;
        const BOOT_ABORT = 1 << 27;
        const VOL_CHANGE = 1 << 28;
        const CMD_START = 1 << 31;
    }

    pub struct DmaCtrlReg : u32 {
        const DMA_RST = 1 << 0;
        const FIXED_BURST = 1 << 1;
        const DMA_ENABLE = 1 << 7;
        const CHECK_VALID = 1 << 31;
    }

    pub struct DmaIntMaskReg : u32 {
        const WINT_ENABLE = 1 << 0;
        const RINT_ENABLE = 1 << 1;
        const FBUS_ERR_INT_ENBALE = 1 << 2;
        const DES_UNAVAIL_INT_ENABLE = 1 << 4;
        const ERR_SUM_INT_ENABLE = 1 << 5;
    }

    pub struct FifoTriLevel : u32 {
        // 7:0 - transmit level
        // 23:16 - receive level
        // 30:28 - burst size of trans
        const T1 = 0 << 28;
        const T4 = 1 << 28;
        const T8 = 2 << 28;
        const T16 = 3 << 28;

        const DEFAULT = 240 << TRANSMIT_LV | 15 << RECEIVE_LV | FifoTriLevel::T16.bits();
    }

    pub struct DmaIntStatus : u32 {
        const WINT = 1 << 0;
        const RINT = 1 << 1;
        const FBUS_ERR = 1 << 2;
        const DES_UNAVAIL = 1 << 4;
        const ERR_INT_SUM = 1 << 5;
        const NOR_INT_SUM = 1 << 8;
        const ANOR_INT_SUM = 1 << 9;
        // 12:10 - error bits
        const ABORT_TRANS = 1 << 10;
        const ABORT_RECV = 1 << 11;

        const DEFAULT_ERROR = 0x234;
        const CLEAR_INT = 0x337;
    }

    pub struct StatusReg : u32 {
        // TODO: ...
        const FIFO_EMPTY = 1 << 2;
        const CARD_BUSY = 1 << 9;

        const STATUS_ERR_MASK = !0x0206BF7F;
    }

    pub struct SwitchStatus : u32 {
        const HIGH_SPEED_BUSY = 0x00020000;
        const HIGH_SPEED_SUPPORT = 0x00020000;
    }
}

/* FifoTriLevel shift*/
const TRANSMIT_LV: usize = 0;
const RECEIVE_LV: usize = 16;
