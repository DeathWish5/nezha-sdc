use super::*;
use aligned::{Aligned, A4};
use ccmu::*;
use core::sync::atomic::{fence, Ordering};
use spin::Mutex;

pub struct MmcHost {
    ccmu_addr: usize,
    shmc_addr: usize,
    inner: Mutex<MmcHostInner>,
}

macro_rules! polling_reg_clear {
    ($reg:expr, $rst:expr) => {
        while ($reg.read() & $rst.bits()) != 0 {
            // check time out ?
        }
    };
}

const BLOCK_SIZE: usize = 512;
const MAX_BLOCK_ID: usize = u32::MAX as usize; // TODO: ...

const DATA_TIMEOUT: usize = 0xFFFFFF;
const SHORT_TIMEOUT: usize = 0xFFFF;
const RESP_TIMEOUT: usize = 0xFFFF;

const FREQ_BASE: [u32; 4] = [10000, 100000, 1000000, 10000000];
const TRANS_SPEED_BASE: [u32; 16] = [
    0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80,
];

impl MmcHost {
    pub fn new(ccmu_addr: PhysAddr, shmc_addr: PhysAddr) -> Self {
        Self {
            ccmu_addr,
            shmc_addr,
            inner: Mutex::new(MmcHostInner::new(ccmu_addr, shmc_addr)),
        }
    }

    pub fn init_card(&self) -> SdResult {
        //info!("try to init card!");
        let mut inner = self.inner.lock();
        debug!("STEP1");
        inner.init_clock_primary();
        debug!("STEP2");
        /* Reset controller */
        inner.shmc.gctrl.write(GctrlReg::RESET_ALL.bits());
        polling_reg_clear!(inner.shmc.gctrl, GctrlReg::RESET_ALL);
        inner
            .shmc
            .timeout
            .write(((DATA_TIMEOUT << 8) | RESP_TIMEOUT) as u32);
        inner.shmc.thldc.write((512 << 16) | (1 << 2) | (1 << 0));
        inner.shmc.csdc.write(3);
        inner.shmc.dbgc.write(0xdeb);

	// Enable Interrupts
	inner.shmc.gctrl.modify(|x| x | GctrlReg::GINT_ENABLE.bits());
	inner.shmc.imask.write(0xc001ffceu32); // Enable many types of Interrupt
	/* DMA Interrupts
	inner.shmc.dmac.write(0x82);
	inner.shmc.idie.write(0x37);
	*/
	inner.shmc.rint.write(0xffffffffu32); // Reset interrupt status bits

	// release eMMC reset signal
        inner.shmc.hwrst.write(1);
        inner.shmc.hwrst.write(0);
        Timer.mdelay(1);
        inner.shmc.hwrst.write(1);
        Timer.mdelay(1);

        debug!("STEP3");
        //println!!(" STEP3");
        inner.init_clock_and_intr();
        Timer.mdelay(1);

        debug!("STEP4");
        //println!!(" STEP4");
        // TODO ...
        // 0 -> 8 -> 55, 41 -> 2 ? -> 3 -> 9 -> 7 ->55, 6 -> 9 -> 13 -> 7
        /* Reset the Card */
        let mut config = MmcCmdConfig::new(); // 0
        inner.send(&mut config).unwrap();
        Timer.mdelay(2);
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SEND_EXT_CSD); // 8
        config.cmdarg = (1u32 << 8) | 0xaa;
        config.resptype = MmcResp::R7;
        inner.send(&mut config).unwrap();
        Timer.mdelay(2);
        loop {
            config.cmdidx = CmdIdx::MmcCmd(MmcCmd::APP_CMD); // 55
            config.cmdarg = 0;
            config.resptype = MmcResp::R1;
            inner.send(&mut config).unwrap();
            Timer.mdelay(1);
            config.cmdidx = CmdIdx::SdCmd(SdCmd::APP_SEND_OP_COND); // 41
            config.cmdarg = 0xfe0000 & 0xff8000 | 0x40000000;
            config.resptype = MmcResp::R3;
            inner.send(&mut config).unwrap();
            Timer.mdelay(1);
            if config.resp[0] & 0x8000_0000u32 != 0 {
                break;
            }
        }
        inner.mmc.ocr = config.resp[0];
        debug!("STEP5");
        //println!!(" STEP5");
        // mmc_startup
        /* Put the Card in Identify Mode */
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::ALL_SEND_CID); // 2
        config.cmdarg = 0;
        config.resptype = MmcResp::R2;
        inner.send(&mut config).unwrap();
        inner.dumpinfo(&config);
        inner.mmc.cid.as_mut_slice().copy_from_slice(&config.resp);
        config.clear();
        debug!("STEP6");
        //println!!(" STEP6");
        /*
         * For MMC cards, set the Relative Address.
         * For SD cards, get the Relatvie Address.
         * This also puts the cards into Standby State
         */
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SET_RELATIVE_ADDR); // 3
        config.cmdarg = 0;
        config.resptype = MmcResp::R6;
        inner.send(&mut config).unwrap();
        inner.dumpinfo(&config);
        inner.mmc.rca = (config.resp[0] >> 16) & 0xFFFF;

        config.clear();
        debug!("STEP7");
        //println!!(" STEP7");
        /* Get the Card-Specific Data */
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SEND_CSD); // 9
        config.cmdarg = inner.mmc.rca << 16;
        config.resptype = MmcResp::R2;
        inner.send(&mut config).unwrap();
        debug!("STEP8");
        //println!!(" STEP8");
        inner.send_status().unwrap();
        debug!("STEP9");
        //println!!(" STEP9");
        inner.dumpinfo(&config);
        inner.mmc.csd.as_mut_slice().copy_from_slice(&config.resp);
        let frep = FREQ_BASE[(config.resp[0] & 0x7) as usize];
        let mult = TRANS_SPEED_BASE[((config.resp[0] >> 3) & 0x7) as usize];
        // inner.mmc.read_bl_len = 1 << ((inner.mmc.csd[1] >> 16) & 0xf);
        // inner.mmc.write_bl_len = inner.mmc.read_bl_len;
        inner.mmc.tran_speed = frep * mult;

        config.clear();
        debug!("STEP10");
        //println!!(" STEP10");
        /* Select the card, and put it into Transfer Mode */
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SELECT_CARD); // 7
        config.cmdarg = inner.mmc.rca << 16;
        config.resptype = MmcResp::R1B;
        inner.send(&mut config).unwrap();
        inner.dumpinfo(&config);
        debug!("STEP10.5");
        //println!!(" STEP10.5");
        inner.set_clock(25000000, self.ccmu_addr)?;
        debug!("STEP11");
        //println!!(" STEP11");
        /* change freq */
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::APP_CMD); // 55
        config.cmdarg = inner.mmc.rca << 16;
        config.resptype = MmcResp::R1;
        config.clear();
        inner.send(&mut config).unwrap();
        debug!("STEP12");
        //println!!(" STEP12");
        let buf: Aligned<A4, [u8; 512 * 4]> = Aligned([0u8; 512 * 4]);
        config.cmdidx = CmdIdx::SdCmd(SdCmd::APP_SEND_SCR); // 51
        config.data = Some(buf.as_ptr() as usize);
        config.resptype = MmcResp::R1;
        config.cmdarg = 0;
        config.blocks = 1;
        config.blocksize = 8;
        config.write = false;
        inner.send(&mut config).unwrap();
        debug!("STEP13");
        //println!!(" STEP13");
        inner.mmc.scr[0] = u32::from_be(unsafe { config.get_data::<u32>(0)? });
        inner.mmc.scr[1] = u32::from_be(unsafe { config.get_data::<u32>(1)? });

        if inner.mmc.scr[0] & 0x40000 != 0 {
            debug!("MMC_MODE_BIT 0x{:x}", inner.mmc.scr[0]);
        }
        for _ in 0..4 {
            config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SWITCH); // 6
            config.resptype = MmcResp::R1;
            config.cmdarg = 0xfffff1;
            config.blocks = 1;
            config.blocksize = 64;
            config.write = false;
            inner.send(&mut config).unwrap();
            if unsafe { config.get_data::<u32>(7)? } & SwitchStatus::HIGH_SPEED_BUSY.bits() == 0 {
                break;
            }
        }
        debug!("STEP14");
        // println!!(" STEP14");
        // if unsafe { config.get_data::<u32>(3)? } & SwitchStatus::HIGH_SPEED_BUSY.bits() == 0 {
        //     break;
        // }

        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SWITCH); // 6
        config.data = Some(buf.as_ptr() as usize);
        config.resptype = MmcResp::R1;
        config.cmdarg = 0x80fffff1u32;
        config.blocks = 1;
        config.blocksize = 64;
        config.write = false;
        inner.send(&mut config).unwrap();
        if unsafe { config.get_data::<u8>(16)? } & 0xf != 1 {
            return Err(SdError::SdcardBusy);
        }
        debug!("STEP15");
        inner.update_clock()?;
        //info!("init card over");
        Ok(())
    }

    pub fn read_block(&self, block_id: usize, buf: &mut [u8]) {
        assert!((buf.len() & (BLOCK_SIZE - 1)) == 0);
        let blkcnt = buf.len() / BLOCK_SIZE;
        // //info!("read block {} len = {}", block_id, blkcnt);
        if blkcnt == 0 || block_id + blkcnt >= MAX_BLOCK_ID {
            error!("invalid read at {} blkcnt = {}", block_id, blkcnt);
            return;
        }
        let mut config = MmcCmdConfig::new();
        config.cmdidx = if blkcnt == 1 {
            CmdIdx::MmcCmd(MmcCmd::READ_SINGLE_BLOCK)
        } else {
            CmdIdx::MmcCmd(MmcCmd::READ_MULTIPLE_BLOCK)
        };
        config.data = Some(buf.as_ptr() as usize);
        config.resptype = MmcResp::R1;
        config.cmdarg = block_id as u32;
        config.blocks = blkcnt as u32;
        config.blocksize = BLOCK_SIZE as u32;
        config.write = false;
        let mut inner = self.inner.lock();
        inner.send(&mut config).unwrap();
    }

    pub fn write_block(&self, block_id: usize, buf: &[u8]) {
        assert!((buf.len() & (BLOCK_SIZE - 1)) == 0);
        let blkcnt = buf.len() / BLOCK_SIZE;
        // //info!("write block {} len = {}", block_id, blkcnt);
        if blkcnt == 0 || block_id + blkcnt >= MAX_BLOCK_ID {
            error!("invalid write at {} blkcnt = {}", block_id, blkcnt);
            return;
        }
        let mut config = MmcCmdConfig::new();
        config.cmdidx = if blkcnt == 1 {
            CmdIdx::MmcCmd(MmcCmd::WRITE_SINGLE_BLOCK)
        } else {
            CmdIdx::MmcCmd(MmcCmd::WRITE_MULTIPLE_BLOCK)
        };
        config.data = Some(buf.as_ptr() as usize);
        config.resptype = MmcResp::R1;
        config.cmdarg = block_id as u32;
        config.blocks = blkcnt as u32;
        config.blocksize = BLOCK_SIZE as u32;
        config.write = true;
        let mut inner = self.inner.lock();
        inner.send(&mut config).unwrap();
        // if (blkcnt > 1) {
        Timer.mdelay(10);
        inner.send_status().unwrap();
        // }
    }

    /// Handle virtio blk intrupt.
    pub fn handle_irq(&self) {
        info!("------ sdcard interrupt happended! ----------");
        // let inner = self.inner.lock();
        error!(
            "Handle SDCard is not implemented !\n Int Reg = {:x}",
            read_reg::<u32>(SHMC0_BASE_ADDR, SHMC_RINT)
        );
    }

    // pub fn update_clock(&mut self) -> SdResult {
    //     let mut inner = self.inner.lock();
    //     inner.update_clock()
    // }

    // pub fn set_clock(&mut self, clock: u32) -> SdResult {
    //     let mut inner = self.inner.lock();
    //     inner.set_clock(clock)
    // }
}

struct MmcHostInner {
    // register
    shmc: Wapper<'static, ShmcRegs>,
    // ADMA desc
    queue: SdQueue<'static>,
    // mmc status
    mmc: MmcStatus,
    // shmc clock
    clk: Wapper<'static, MmcClk>,
}

impl MmcHostInner {
    pub fn new(ccmu_addr: PhysAddr, shmc_addr: PhysAddr) -> Self {
        Self {
            shmc: Wapper::from_raw(shmc_addr),
            queue: SdQueue::new(MAX_QUEUE_SIZE as u16).unwrap(),
            mmc: MmcStatus::default(),
            clk: Wapper::from_raw(ccmu_addr + ccmu::CCMU_SDMMC0_CLK_REG),
        }
    }

    pub fn send(&mut self, config: &mut MmcCmdConfig) -> SdResult {
        debug!("[CMD] SEND CMD {:?}", config.cmdidx);
        let use_dma = config.blocksize * config.blocks > 1;
        let mut cmdval = ShmcCmd::CMD_START;
        if config.cmdidx == CmdIdx::MmcCmd(MmcCmd::GO_IDLE_STATE) {
            cmdval |= ShmcCmd::INIT_CMD;
        }
        if config.resptype.contains(MmcResp::PRESENT) {
            cmdval |= ShmcCmd::HAS_RESP;
        }
        if config.resptype.contains(MmcResp::RESP_136) {
            cmdval |= ShmcCmd::LONG_RESP;
        }
        if config.resptype.contains(MmcResp::CRC) {
            cmdval |= ShmcCmd::RESP_CRC;
        }
        let data = config.data.is_some();
        if data {
            cmdval |= ShmcCmd::HAS_DATA | ShmcCmd::SYNC_TRANS;
            if config.write {
                cmdval |= ShmcCmd::DATA_WRITE;
            }
            if config.blocks > 1 {
                cmdval |= ShmcCmd::STOP_AUTO;
            }
            self.shmc.blksz.write(config.blocksize);
            self.shmc.bytecnt.write(config.blocks * config.blocksize);
        } else if config.cmdidx == CmdIdx::MmcCmd(MmcCmd::STOP_TRANSMISSION) && use_dma == false {
            cmdval |= ShmcCmd::STOP_ABORT;
            cmdval &= !ShmcCmd::SYNC_TRANS;
        }

        self.shmc.arg.write(config.cmdarg);

        debug!("send STEP0");
        if data {
            if use_dma {
                debug!("trans data use dma");
                self.shmc
                    .gctrl
                    .modify(|x| x & !GctrlReg::FIFO_AC_MOD.bits()); // fifo dma mode
                self.init_dma(&config)?;
                self.shmc.cmd.write(config.cmdidx.raw() | cmdval.bits());
            } else {
                warn!("trans data by polling");
                self.shmc.gctrl.modify(|x| x | GctrlReg::FIFO_AC_MOD.bits()); // fifo dma mode
                self.shmc.cmd.write(config.cmdidx.raw() | cmdval.bits());
                self.trans_by_cpu(&config)?;
            }
            debug!("GCTRL: 0x{:x}", self.shmc.gctrl.read());
            debug!("CMD: 0x{:x}", self.shmc.cmd.read());
            debug!("BtCnt: 0x{:x}", self.shmc.bytecnt.read());
        } else {
            debug!("CMDIDX: 0x{:x}", config.cmdidx.raw() | cmdval.bits());
            self.shmc.cmd.write(config.cmdidx.raw() | cmdval.bits());
        }
        let mint = self.shmc.mint.read();
        let idie = self.shmc.idie.read();
        info!(
            "send STEP1 intr = {} maskedINT = {:x} idie = {:x}",
            intr_get(),
            mint,
            idie
        );
        // loop {
        //     if false {
        //         break;
        //     }
        //     info!(
        //         "Raw Int Reg = {:x} M Int Reg = {:x}",
        //         read_reg::<u32>(SHMC0_BASE_ADDR, SHMC_RINT),
        //         read_reg::<u32>(SHMC0_BASE_ADDR, SHMC_MINT),
        //     );
        //     Timer.udelay(1);
        // }
        self.rint_polling(IntMask::COMMAND_DONE, true, DATA_TIMEOUT)
            .unwrap();
        info!("send STEP2");
        if data {
            if use_dma {
                let intmask = if config.write {
                    DmaIntStatus::WINT
                } else {
                    DmaIntStatus::RINT
                };
                self.idst_polling(intmask, true, DATA_TIMEOUT)?;
                fence(Ordering::SeqCst);
            }
            info!("send idst over");
            if config.blocks == 1 {
                self.rint_polling(IntMask::DATA_OVER, true, DATA_TIMEOUT)
                    .unwrap();
            } else if !config.write {
                self.rint_polling(IntMask::AUTO_COMMAND_DONE, true, DATA_TIMEOUT)
                    .unwrap();
            }
        }
        info!("send data done");
        if config.resptype.contains(MmcResp::BUSY) {
            self.status_polling(StatusReg::CARD_BUSY, false, 64 * DATA_TIMEOUT)
                .unwrap();
        }
        debug!("send STEP4");
        if config.resptype.contains(MmcResp::RESP_136) {
            config.resp[0] = self.shmc.resp3.read();
            config.resp[1] = self.shmc.resp2.read();
            config.resp[2] = self.shmc.resp1.read();
            config.resp[3] = self.shmc.resp0.read();
        } else {
            config.resp[0] = self.shmc.resp0.read();
        }
        debug!("send STEP5");
        // clear idst idie dmac, gctrl.dma_enbale
        let idst = self.shmc.idst.read();
        self.shmc.idst.write(idst);
        self.shmc.idie.write(0); // DMA Interrupt
        self.shmc.dmac.write(0);
        self.shmc
            .gctrl
            .modify(|x| x & (!GctrlReg::DMA_ENABLE.bits()));
	// TODO, AHB bus (rw data through CPU) OR DMA bus

        debug!("send SHMC_RINT:   0x{:x}", self.shmc.rint.read());
        self.shmc.rint.write(IntMask::ALL.bits());
        Ok(())
    }

    pub fn send_status(&mut self) -> SdResult {
        let mut config = MmcCmdConfig::new();
        config.cmdidx = CmdIdx::MmcCmd(MmcCmd::SEND_STATUS);
        config.resptype = MmcResp::R1;
        config.cmdarg = self.mmc.rca << 16;
        let retries = 1000;
        static MMC_STATUS_RDY_FOR_DATA: u32 = 1 << 8;
        for _ in 0..retries {
            self.send(&mut config)?;
            if (config.resp[0] & MMC_STATUS_RDY_FOR_DATA) != 0 {
                return Ok(());
            }
            Timer.mdelay(1);
            if (config.resp[0] & StatusReg::STATUS_ERR_MASK.bits()) != 0 {
                return Err(SdError::MMCStatusError);
            }
        }
        Err(SdError::SdcardBusy)
    }

    pub fn init_clock_primary(&mut self) {
        debug!("C1");
        self.clk.shmc_bgr.write(ShmcBgrReg::SHMC0_RST.bits());
        Timer.mdelay(1); // ???
        debug!("C2");
        self.clk
            .shmc_bgr
            .modify(|x| x | ShmcBgrReg::SHMC0_GATING.bits());
        debug!("C3");
        let factor_m = 14;
        self.clk
            .sdmmc0
            .write(MmcClkReg::CLK_GATING.bits() | MmcClkReg::N_4.bits() | factor_m);
        debug!("C4");
    }

    fn register_polling(
        register: &mut Reg,
        done_flag: u32,
        set: bool,
        error_flag: Option<u32>,
        error_ret: Option<SdError>,
        timeout_ms: usize,
        timeout_ret: SdError,
    ) -> SdResult {
        let timeout = Timer.get_us() + timeout_ms as u64;
        let error_flag = error_flag.unwrap_or(0);
        loop {
            let reg = register.read();
            if (reg & error_flag) != 0 {
                error!(
                    "QAQ !!! reg & error_flag = 0x{:x} 0x{:x}",
                    reg & error_flag,
                    reg
                );
                return Err(error_ret.unwrap());
            }
            if ((reg & done_flag) != 0) == set {
                break;
            }
            if Timer.get_us() > timeout {
                return Err(timeout_ret);
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    pub fn rint_polling(&mut self, done_flag: IntMask, set: bool, timeout_ms: usize) -> SdResult {
        let rst = Self::register_polling(
            &mut self.shmc.rint,
            done_flag.bits(),
            set,
            Some(IntMask::DEFAULT_ERROR.bits()),
            Some(SdError::RintError),
            timeout_ms,
            SdError::RintTimeout,
        );
        if let Err(SdError::RintError) = rst {
            self.reset();
        }
        rst
    }

    pub fn idst_polling(
        &mut self,
        done_flag: DmaIntStatus,
        set: bool,
        timeout_ms: usize,
    ) -> SdResult {
        let rst = Self::register_polling(
            &mut self.shmc.idst,
            done_flag.bits(),
            set,
            Some(DmaIntStatus::DEFAULT_ERROR.bits()),
            Some(SdError::DmaError),
            timeout_ms,
            SdError::DmaTimeout,
        );
        if let Err(SdError::DmaError) = rst {
            self.reset();
        }
        rst
    }

    pub fn status_polling(
        &mut self,
        done_flag: StatusReg,
        set: bool,
        timeout_ms: usize,
    ) -> SdResult {
        Self::register_polling(
            &mut self.shmc.status,
            done_flag.bits(),
            set,
            None,
            None,
            timeout_ms,
            SdError::MMCSendTimeout,
        )
    }

    pub fn reset(&mut self) {
        self.shmc.gctrl.write(GctrlReg::RESET_ALL.bits());
        polling_reg_clear!(self.shmc.gctrl, GctrlReg::RESET_ALL);
	self.shmc.gctrl.modify(|x| x | GctrlReg::GINT_ENABLE.bits());
        // self.update_clock().unwrap();
        self.shmc.rint.write(IntMask::ALL.bits());
    }

    pub fn init_dma(&mut self, config: &MmcCmdConfig) -> SdResult {
        debug!("DMA STEP0");
        let byte_cnt = config.blocks * config.blocksize;
        let des_phys_addr = self.queue.add(config.data.unwrap(), byte_cnt as usize)?;
        debug!("DMA STEP1");
        // TODO: async ?
        self.shmc
            .gctrl
            .modify(|x| x | GctrlReg::DMA_ENABLE.bits() | GctrlReg::DMA_RST.bits());
        debug!("DMA STEP2");
        polling_reg_clear!(self.shmc.gctrl, GctrlReg::DMA_RST);
        self.shmc.dmac.write(DmaCtrlReg::DMA_RST.bits());
        polling_reg_clear!(self.shmc.dmac, DmaCtrlReg::DMA_RST);
        debug!("DMA STEP3");
        self.shmc
            .dmac
            .write(DmaCtrlReg::FIXED_BURST.bits() | DmaCtrlReg::DMA_ENABLE.bits());
        debug!("DMA STEP4");
        // self.shmc.idie.modify(|x| {
        //     x | if config.write {
        //         DmaIntMaskReg::WINT_ENABLE.bits()
        //     } else {
        //         DmaIntMaskReg::RINT_ENABLE.bits()
        //     }
        // });
        self.shmc.idie.write(0xFFFF); //DMA Interrupt
        debug!("DMA STEP5 {:x}", des_phys_addr);
        if (des_phys_addr & 0x3) != 0 {
            error!("DES should be 4 bytes aligned!");
            return Err(SdError::InvalidParam);
        }
        self.shmc.dlba.write((des_phys_addr as u32) >> 2); // 4bytes alighed
        self.shmc.ftrglevel.write(FifoTriLevel::DEFAULT.bits());
        debug!("DMA STEP6");
        Ok(())
    }

    pub fn trans_by_cpu(&mut self, config: &MmcCmdConfig) -> SdResult<usize> {
        debug!("CPU STEP0");
        let byte_cnt = config.blocks * config.blocksize;
        for idx in 0..(byte_cnt >> 2) {
            self.status_polling(StatusReg::FIFO_EMPTY, false, 0xFFFFFF)?;
            if config.write {
                let v: u32 = unsafe { config.get_data::<u32>(idx as usize).unwrap() };
                self.shmc.fifo.write(v);
            } else {
                let v: u32 = self.shmc.fifo.read();
                unsafe { config.set_data(v, idx as usize).unwrap() };
            }
        }
        debug!("CPU STEP4");
        Ok(byte_cnt as usize)
    }

    pub fn update_clock(&mut self) -> SdResult {
        self.shmc.clkcr.modify(|x| x | 1 << 31);
        self.shmc.cmd.write((1 << 31) | (1 << 21) | (1 << 13));
        polling_reg_clear!(self.shmc.cmd, ShmcCmd::CMD_START);
        // if shmc.cmd.read().bits() & 0x8000_0000u32 != 0 {
        //     return Err(SdError::UpdateClockFail);
        // }
        self.shmc.clkcr.modify(|x| x & !(1 << 31));
        self.shmc.rint.write(0xFFFFFFFFu32);
        Ok(())
    }

    // pub fn _get_clock(&self, ccmu_addr: usize) -> u32 {
    //     let rval = self.clk.sdmmc0.read();
    //     let m = rval & 0xf;
    //     let n = (rval >> 8) & 0x3;
    //     let src = (rval >> 24) & 0x3;
    //     let sclk_hz = match src {
    //         0 => 24000000,
    //         2 => get_pll_periph0(ccmu_addr) * 2 * 1000000, /* use 2x pll6 */
    //         _ => {
    //             return 0;
    //         }
    //     };
    //     sclk_hz / (1 << n) / (m + 1)
    // }

    pub fn set_clock(&mut self, clock: u32, ccmu_addr: PhysAddr) -> SdResult {
        /* disable card clock */
        self.shmc.clkcr.modify(|x| x & !(1 << 16));
        /* update clock */
        self.update_clock()?;
        /* disable mclk */
        self.clk.sdmmc0.write(0);
        self.shmc.ntsr.modify(|x| x | (1 << 31));
        let (src, sclk_hz): (u32, u32) = if clock <= 4000000 {
            (0, 24000000)
        } else {
            (2, get_pll_periph0(ccmu_addr) * 2 * 1000000)
        };
        let mut div = (2 * sclk_hz + clock) / (2 * clock);
        if div == 0 {
            div = 1;
        }
        let (m, n) = match div {
            x if x > 128 => (1, 0),
            x if x > 64 => (div >> 3, 3),
            x if x > 32 => (div >> 2, 2),
            x if x > 16 => (div >> 1, 1),
            _ => (div >> 0, 0),
        };

        self.clk.sdmmc0.write((src << 24) | (n << 8) | (m - 1));
        /* re-enable mclk */
        self.clk.sdmmc0.modify(|x| x | (1u32 << 31));
        self.shmc.clkcr.modify(|x| x & !(0xff));
        /* update clock */
        self.update_clock()?;

        /* config delay */
        // let odly = 0;
        let sdly = 0;
        // let mut rval = self.shmc.drv_dl.read();
        // rval |= ((odly & 0x1) << 16) | ((odly & 0x1) << 17);
        // clk_gating_wrapper(|| self.shmc.drv_dl.modify())
        // write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) & !(1 << 31));
        // self.shmc.drv_dl.write(|w| w.bits(rval));
        // write_reg(MCLKBASE, 0, read_reg::<u32>(MCLKBASE, 0) | (1 << 31));

        self.shmc
            .ntsr
            .modify(|x| x & !(0x3 << 4) | ((sdly & 0x30) << 4));
        /* Re-enable card clock */
        self.shmc.clkcr.modify(|x| x | (0x1 << 16));
        /* update clock */
        self.update_clock()
    }

    pub fn init_clock_and_intr(&mut self) {
        /* enable 2xclk mode, and use default input phase */
        self.shmc.ntsr.modify(|x| x | (1 << 31));
        /* use 90 degree output phase */
        let drv_dl = self.shmc.drv_dl.read();
        // self.clk.clk_gating_wrapper(|| );
        self.clk
            .sdmmc0
            .modify(|x| x & !MmcClkReg::CLK_GATING.bits());
        self.shmc.drv_dl.write(drv_dl);
        self.clk.sdmmc0.modify(|x| x | MmcClkReg::CLK_GATING.bits());
        // self.shmc
        //     .imask
        //     .write((IntMask::SDIO_INTERRUPT | IntMask::DEFAULT_MASK).bits());
        //self.shmc.imask.write(0xFFFFFFFF);

        self.shmc
            .clkcr
            .modify(|x| x | ClkCtrlReg::CLK_ENABLE.bits() | ClkCtrlReg::MASK_DATA0.bits()); // enable clk
        self.shmc.cmd.write(
            ShmcCmd::CMD_START.bits() | ShmcCmd::SYNC_TRANS.bits() | ShmcCmd::CLK_CHANGE.bits(),
        ); // notify clk change
        self.shmc
            .clkcr
            .modify(|x| x & !ClkCtrlReg::MASK_DATA0.bits()); // unmask data0
    }

    // pub fn config_ios(&mut self) {

    // }

    // static void mmc_set_ios(struct mmc *mmc)
    // {
    //     struct sunxi_mmc_host *mmchost = (struct sunxi_mmc_host *)mmc->priv;

    //     mmcdbg("mmc %d ios: bus: %u, clock: %u\n", mmchost->mmc_no,
    //         mmc->bus_width, mmc->clock);

    //     if (mmc->clock && mmc_config_clock(mmc, mmc->clock)) {
    //         mmcinfo("[mmc]: "
    //             "*** update clock failed\n");
    //         mmchost->fatal_err = 1;
    //         return;
    //     }
    //     /* Change bus width */
    //     if (mmc->bus_width == 8)
    //         writel(2, &mmchost->reg->width);
    //     else if (mmc->bus_width == 4)
    //         writel(1, &mmchost->reg->width);
    //     else
    //         writel(0, &mmchost->reg->width);

    //     /* set ddr mode */
    //     if (mmc->speed_mode == HSDDR52_DDR50) {
    //         mmc_ddr_mode_onoff(mmc, 1);
    //         mmc_hs400_mode_onoff(mmc, 0);
    //     } else if (mmc->speed_mode == HS400) {
    //         mmc_ddr_mode_onoff(mmc, 0);
    //         mmc_hs400_mode_onoff(mmc, 1);
    //     } else {
    //         mmc_ddr_mode_onoff(mmc, 0);
    //         mmc_hs400_mode_onoff(mmc, 0);
    //     }
    // }

    pub fn dumpinfo(&self, config: &MmcCmdConfig) {
        debug!("SHMC STATUS: 0x{:x}", self.shmc.status.read());
        debug!("SHMC CMD:    0x{:x}", self.shmc.cmd.read());
        debug!(
            "RESPONSES:   [0x{:x}, 0x{:x}, 0x{:x}, 0x{:x}]",
            config.resp[0], config.resp[1], config.resp[2], config.resp[3]
        );
    }
}

// cmd config
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum MmcCmd {
    GO_IDLE_STATE = 0,
    SEND_OP_COND = 1,
    ALL_SEND_CID = 2,
    SET_RELATIVE_ADDR = 3,
    SET_DSR = 4,
    SWITCH = 6,
    SELECT_CARD = 7,
    SEND_EXT_CSD = 8,
    SEND_CSD = 9,
    SEND_CID = 10,
    STOP_TRANSMISSION = 12,
    SEND_STATUS = 13,
    SET_BLOCKLEN = 16,
    READ_SINGLE_BLOCK = 17,
    READ_MULTIPLE_BLOCK = 18,
    WRITE_SINGLE_BLOCK = 24,
    WRITE_MULTIPLE_BLOCK = 25,
    ERASE_GROUP_START = 35,
    ERASE_GROUP_END = 36,
    ERASE = 38,
    APP_CMD = 55,
    SPI_READ_OCR = 58,
    SPI_CRC_ON_OFF = 59,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum SdCmd {
    SEND_RELATIVE_ADDR = 3,
    SWITCH_FUNC = 6, // APP_SET_BUS_WIDTH = 6,
    SEND_IF_COND = 8,
    ERASE_WR_BLK_START = 32,
    ERASE_WR_BLK_END = 33,
    APP_SEND_OP_COND = 41,
    APP_SEND_SCR = 51,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CmdIdx {
    MmcCmd(MmcCmd),
    SdCmd(SdCmd),
}

// impl core::convert::Into<u32> for CmdIdx {
//     fn into(self) -> u32 {
//         match self {
//             Self::MmcCmd(cmd) => cmd as u32,
//             Self::SdCmd(cmd) => cmd as u32,
//         }
//     }
// }

impl CmdIdx {
    pub fn raw(&self) -> u32 {
        match self {
            Self::MmcCmd(cmd) => *cmd as u32,
            Self::SdCmd(cmd) => *cmd as u32,
        }
    }
}

bitflags::bitflags! {
    pub struct MmcResp : u32 {
        const PRESENT = 1 << 0;
        const RESP_136     = 1 << 1;
        const CRC     = 1 << 2;
        const BUSY    = 1 << 3;
        const OPCODE  = 1 << 4;
        const NONE    = 0;

        const R1      =
            MmcResp::PRESENT.bits |
            MmcResp::CRC.bits |
            MmcResp::OPCODE.bits;
        const R1B     =
            MmcResp::PRESENT.bits |
            MmcResp::CRC.bits |
            MmcResp::OPCODE.bits |
            MmcResp::BUSY.bits;
        const R2      =
            MmcResp::PRESENT.bits |
            MmcResp::RESP_136.bits |
            MmcResp::CRC.bits;
        const R3      = MmcResp::PRESENT.bits;
        const R6      =
            MmcResp::PRESENT.bits |
            MmcResp::CRC.bits |
            MmcResp::OPCODE.bits;
        const R7      =
            MmcResp::PRESENT.bits |
            MmcResp::CRC.bits |
            MmcResp::OPCODE.bits;
    }
}

pub struct MmcCmdConfig {
    // cmd args
    pub cmdidx: CmdIdx,
    pub cmdarg: u32,
    pub resptype: MmcResp,
    // data
    pub data: Option<VirtAddr>,
    pub write: bool,
    pub blocks: u32,
    pub blocksize: u32,
    // resp
    pub resp: [u32; 4],
}

impl MmcCmdConfig {
    pub fn new() -> Self {
        Self {
            cmdidx: CmdIdx::MmcCmd(MmcCmd::GO_IDLE_STATE),
            cmdarg: 0,
            resptype: MmcResp::NONE,
            data: None,
            write: false,
            blocks: 0,
            blocksize: 0,
            resp: [0; 4],
        }
    }

    pub fn clear(&mut self) {
        self.resp.iter_mut().for_each(|x| *x = 0);
        if let Some(ptr) = self.data {
            let byte_cnt = self.blocks * self.blocksize;
            // TODO: remove this unsafe
            let buf: &mut [usize] =
                unsafe { core::slice::from_raw_parts_mut(ptr as _, byte_cnt as _) };
            buf.fill(0);
        }
    }

    pub unsafe fn get_data<T: Sized>(&self, offset: usize) -> SdResult<T> {
        if let Some(ptr) = self.data {
            let byte_cnt = self.blocks * self.blocksize;
            if offset * core::mem::size_of::<T>() < byte_cnt as usize {
                return Ok((ptr as *const T).add(offset).read());
            }
        }
        Err(SdError::InvalidParam)
    }

    pub unsafe fn set_data<T: Sized>(&self, value: T, offset: usize) -> SdResult {
        if let Some(ptr) = self.data {
            let byte_cnt = self.blocks * self.blocksize;
            if offset * core::mem::size_of::<T>() < byte_cnt as usize {
                (ptr as *mut T).add(offset).write(value);
                return Ok(());
            }
        }
        Err(SdError::InvalidParam)
    }

    pub fn _dump(&self) {
        debug!("Count:0x{:x}", self.blocks * self.blocksize * 4);
        if let Some(ptr) = self.data {
            let byte_cnt = self.blocks * self.blocksize;
            // TODO: remove this unsafe
            let buf: &mut [usize] =
                unsafe { core::slice::from_raw_parts_mut(ptr as _, byte_cnt as _) };
            for i in buf.iter() {
                debug!("0x{:<02x}, ", i);
            }
        }
    }
}

#[derive(Default)]
pub struct MmcStatus {
    pub ocr: u32,
    pub cid: [u32; 4],
    pub csd: [u32; 4],
    pub rca: u32,
    pub scr: [u32; 2],
    pub read_bl_len: u32,
    pub write_bl_len: u32,
    pub tran_speed: u32,
}

// struct MmcStatus {
//     char name[32];
//     unsigned voltages;
//     unsigned version;
//     unsigned has_init;
//     unsigned control_num;
//     unsigned f_min;
//     unsigned f_max;
//     unsigned f_max_ddr;
//     int high_capacity;
//     unsigned bus_width;
//     unsigned clock;
//     unsigned card_caps;
//     unsigned host_caps;
//     unsigned ocr;
//     unsigned scr[2];
//     unsigned csd[4];
//     unsigned cid[4];
//     unsigned rca;/*unsigned short rca;*/
//     unsigned part_config;/*char part_config;*/
//     unsigned part_num;/*char part_num;*/
//     unsigned tran_speed;
//     unsigned read_bl_len;
//     unsigned write_bl_len;
//     unsigned erase_grp_size;
//     unsigned long long capacity;
//     int (*send_cmd) (struct mmc *mmc,
//             struct mmc_cmd *cmd, struct mmc_data *data);
//     void (*set_ios) (struct mmc *mmc);
//     int (*init) (struct mmc *mmc);
//     int (*update_phase) (struct mmc *mmc);
//     struct tune_sdly tune_sdly;
//     unsigned b_max;
//     unsigned lba;/* number of blocks */
//     unsigned blksz;/* block size */
//     char revision[8 + 8];/*char revision[8+1];*/        /* CID:  PRV */
//     uint speed_mode;
// }
