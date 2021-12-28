use super::*;
use core::slice;
use core::sync::atomic::{fence, Ordering};

pub struct DMA {
    paddr: u32,
    pages: u32,
}

impl DMA {
    pub fn new(pages: usize) -> SdResult<Self> {
        let paddr = unsafe { sdcard_dma_alloc(pages) };
        if paddr == 0 {
            return Err(SdError::DmaError);
        }
        Ok(DMA {
            paddr: paddr as u32,
            pages: pages as u32,
        })
    }

    pub fn paddr(&self) -> usize {
        self.paddr as usize
    }

    pub fn vaddr(&self) -> usize {
        phys_to_virt(self.paddr as usize)
    }

    /// Page frame number
    pub fn pfn(&self) -> u32 {
        self.paddr >> 12
    }

    /// Convert to a buffer
    pub unsafe fn as_buf(&self) -> &'static mut [u8] {
        core::slice::from_raw_parts_mut(self.vaddr() as _, PAGE_SIZE * self.pages as usize)
    }
}

impl Drop for DMA {
    fn drop(&mut self) {
        let err = unsafe { sdcard_dma_dealloc(self.paddr as usize, self.pages as usize) };
        assert_eq!(err, 0, "failed to deallocate DMA");
    }
}

pub const DES_NUM: usize = 64;
pub const DES_SIZE: usize = 128;
pub const SDXC_DES_NUM_SHIFT: usize = 12;
pub const SDXC_DES_BUFFER_MAX_LEN: usize = 1 << SDXC_DES_NUM_SHIFT;

#[repr(C, align(128))]
#[derive(Debug)]
pub struct Descriptor {
    des0: u32,
    des1: u32,
    des2: u32,
    des3: u32,
}

bitflags::bitflags! {
    /* section 3.2.6.63 : SMHC Bus Gating Reset Register */
    pub struct DesFlags : u32 {
        const DISABLE_COMP_INT = 1 << 1;
        const LAST = 1 << 2;
        const FIRST = 1 << 3;
        const CHAIN = 1 << 4;
        const END_OF_RING = 1 << 5; // ??
        const ERROR = 1 << 30;
        const HOLD = 1 << 31;
    }
}

impl Descriptor {
    pub fn set_flags(&mut self, flags: DesFlags) {
        self.des0 = flags.bits();
    }

    pub fn set_buf_size(&mut self, size: u32) {
        self.des1 = size & (SDXC_DES_BUFFER_MAX_LEN - 1) as u32;
    }

    pub fn set_buf(&mut self, buf_vaddr: VirtAddr) {
        self.des2 = virt_to_phys(buf_vaddr) as u32;
    }

    pub fn set_next(&mut self, next_vaddr: VirtAddr) {
        self.des3 = virt_to_phys(next_vaddr) as u32;
    }

    pub fn as_ptr(&self) -> *const Self {
        self as _
    }
}

#[repr(C)]
pub struct SdQueue<'a> {
    /// DMA guard
    dma: DMA,
    /// Descriptor table
    desc: &'a mut [Descriptor],
    /// The size of queue
    queue_size: u16,
    // Waker to notify that there are more available descriptors
    // waker: Option<Waker>,
}

pub const MAX_QUEUE_SIZE: usize = 32 * 2; // 2 pages

impl SdQueue<'_> {
    /// Create a new SdQueue.
    pub fn new(queue_size: u16) -> SdResult<Self> {
        if !queue_size.is_power_of_two() || queue_size > MAX_QUEUE_SIZE as u16 {
            return Err(SdError::InvalidParam);
        }
        const_assert_eq!(core::mem::size_of::<Descriptor>(), DES_SIZE);
        let dma = DMA::new(queue_size as usize * core::mem::size_of::<Descriptor>() / PAGE_SIZE)?;
        let desc = unsafe {
            slice::from_raw_parts_mut(dma.vaddr() as *mut Descriptor, queue_size as usize)
        };
        Ok(SdQueue {
            dma,
            desc,
            queue_size,
        })
    }

    /// Add buffers to the sdqueue
    pub fn add(&mut self, buf: VirtAddr, buf_size: usize) -> SdResult<PhysAddr> {
        if buf_size > self.available_size() || buf & 0x3 != 0 {
            error!(
                "[DMA] buf pointer error, buf = 0x{:x?}, buf_size = 0x{:x?}",
                buf, buf_size
            );
            return Err(SdError::InvalidParam);
        }
        let remain = buf_size & (SDXC_DES_BUFFER_MAX_LEN - 1);
        let des_num = (buf_size + SDXC_DES_BUFFER_MAX_LEN - 1) >> SDXC_DES_NUM_SHIFT;
        // allocate descriptors from free list
        for i in 0..des_num {
            let desc = &mut self.desc[i];
            let mut flags: DesFlags = DesFlags::CHAIN | DesFlags::HOLD;
            desc.set_buf((buf + i * SDXC_DES_BUFFER_MAX_LEN) >> 2); // 4 bytes alighed
            if i == des_num - 1 {
                flags |= DesFlags::LAST | DesFlags::END_OF_RING;
                desc.set_buf_size(remain as u32);
                desc.set_next(0);
            } else {
                // flags |= DesFlags::DISABLE_COMP_INT;
                desc.set_buf_size(SDXC_DES_BUFFER_MAX_LEN as u32);
                desc.set_next((unsafe { desc.as_ptr().add(1) } as VirtAddr) >> 2);
            }
            desc.set_flags(flags);
        }
        self.desc[0].des0 |= DesFlags::FIRST.bits();
        // write barrier
        fence(Ordering::SeqCst);
        Ok(virt_to_phys(self.desc.as_ptr() as VirtAddr))
    }

    ///
    pub fn available_size(&self) -> usize {
        self.queue_size as usize * SDXC_DES_BUFFER_MAX_LEN
    }
}
