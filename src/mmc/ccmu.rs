use super::*;

/* #REF: D1 User Manual v0.1 section 3.2 (https://open.allwinnertech.com/#/doc?menuID=2) */

pub const SUNXI_CCM_BASE: PhysAddr = 0x2001000;
pub const SUNXI_CCM_SIZE: usize = PAGE_SIZE;

/* pll list */
const CCMU_PLL_CPUX_CTRL_REG: PhysAddr = 0x0;
const CCMU_PLL_DDR0_CTRL_REG: PhysAddr = 0x10;
const CCMU_PLL_PERI0_CTRL_REG: PhysAddr = 0x20;
const CCMU_PLL_VIDE00_CTRL_REG: PhysAddr = 0x40;
const CCMU_PLL_VIDE01_CTRL_REG: PhysAddr = 0x48;
const CCMU_PLL_VE_CTRL_REG: PhysAddr = 0x58;
const CCMU_PLL_AUDIO0_CTRL_REG: PhysAddr = 0x78;
const CCMU_PLL_AUDIO1_CTRL_REG: PhysAddr = 0x80;
const CCMU_GPADC_BGR_REG: PhysAddr = 0x9EC;
const CCMU_AUDIO_CODEC_BGR_REG: PhysAddr = 0xA5C;

/* pattern list */
const CCMU_PLL_AUDIO0_PAT0_REG: PhysAddr = 0x178;

/* cfg list */
const CCMU_CPUX_AXI_CFG_REG: PhysAddr = 0xD00;
const CCMU_PSI_AHB1_AHB2_CFG_REG: PhysAddr = 0x510;
const CCMU_APB1_CFG_GREG: PhysAddr = 0x520;
const CCMU_APB2_CFG_GREG: PhysAddr = 0x524;
const CCMU_MBUS_CFG_REG: PhysAddr = 0x540;

const CCMU_CE_CLK_REG: PhysAddr = 0x680;
const CCMU_CE_BGR_REG: PhysAddr = 0x68C;

/*SYS*/
const CCMU_DMA_BGR_REG: PhysAddr = 0x70C;
const CCMU_AVS_CLK_REG: PhysAddr = 0x740;

/* storage */
const CCMU_DRAM_CLK_REG: PhysAddr = 0x800;
const CCMU_MBUS_MST_CLK_GATING_REG: PhysAddr = 0x804;
const CCMU_DRAM_BGR_REG: PhysAddr = 0x80C;

const CCMU_NAND_CLK_REG: PhysAddr = 0x810;
const CCMU_NAND_BGR_REG: PhysAddr = 0x82C;

/* MMC */
pub const CCMU_SDMMC0_CLK_REG: PhysAddr = 0x830;
pub const CCMU_SDMMC1_CLK_REG: PhysAddr = 0x834;
pub const CCMU_SDMMC2_CLK_REG: PhysAddr = 0x838;
pub const CCMU_SMHC_BGR_REG: PhysAddr = 0x84c;

/*normal interface*/
const CCMU_UART_BGR_REG: PhysAddr = 0x90C;
const CCMU_TWI_BGR_REG: PhysAddr = 0x91C;

const CCMU_SCR_BGR_REG: PhysAddr = 0x93C;

const CCMU_SPI0_CLK_REG: PhysAddr = 0x940;
const CCMU_SPI1_CLK_REG: PhysAddr = 0x944;
const CCMU_SPI_BGR_CLK_REG: PhysAddr = 0x96C;
const CCMU_USB0_CLK_REG: PhysAddr = 0xA70;
const CCMU_USB_BGR_REG: PhysAddr = 0xA8C;

/*DMA*/
const DMA_GATING_BASE: PhysAddr = CCMU_DMA_BGR_REG;

#[repr(C)]
pub struct MmcClk {
    pub(super) sdmmc0: Reg,
    pub(super) sdmmc1: Reg,
    pub(super) sdmmc2: Reg,
    res0: [Reg; 4],
    pub(super) shmc_bgr: Reg,
}

bitflags::bitflags! {
    /* section 3.2.6.63 : SMHC Bus Gating Reset Register */
    pub struct ShmcBgrReg : u32 {
        const SHMC0_GATING = 1 << 0;
        const SHMC1_GATING = 1 << 1;
        const SHMC2_GATING = 1 << 2;
        const SHMC0_RST = 1 << 16;
        const SHMC1_RST = 1 << 17;
        const SHMC2_RST = 1 << 18;
    }

    pub struct MmcClkReg : u32 {
        /* bit 31 : clock gating*/
        const CLK_GATING = 1 << 31;
        /* bit 26:24 : clock src select */
        const HOSC = 0 << 24;
        const PLL_PERIx1 = 1 << 24;
        const PLL_PERIx2 = 2 << 24;
        const PLL_AUDIO1 = 3 << 24;
        /* bit 9:8 : factor N*/
        const N_1 = 0 << 8;
        const N_2 = 1 << 8;
        const N_4 = 2 << 8;
        const N_8 = 3 << 8;
        /* bit 3:0 : factor M*/
        // M = bit[3:0] + 1
    }
}

// SAFETY: Only used in init
pub fn get_pll_periph0(ccmu_addr: PhysAddr) -> u32 {
    let reg: Wapper<'static, Reg> = Wapper::from_raw(ccmu_addr + CCMU_PLL_PERI0_CTRL_REG);
    let regv = reg.read();
    let n = ((regv >> 8) & 0xff) + 1;
    let m = ((regv >> 1) & 0x01) + 1;
    let p = ((regv >> 16) & 0x07) + 1;
    24 * n / m / p / 2
}
