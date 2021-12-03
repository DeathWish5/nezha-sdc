use core::marker;
use core::marker::PhantomData;
use core::ops::Deref;

pub trait Readable {}
pub trait Writable {}
pub trait ResetValue {
    type Type;
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

pub struct R<U, T> {
    pub(crate) bits: U,
    _reg: marker::PhantomData<T>,
}

impl<U, T> R<U, T>
where
    U: Copy,
{
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

pub struct W<U, REG> {
    pub(crate) bits: U,
    _reg: marker::PhantomData<REG>,
}

impl<U, REG> W<U, REG> {
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: U) -> &mut Self {
        self.bits = bits;
        self
    }
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
        impl ResetValue for Register {
            type Type = u32;
            fn reset_value() -> Self::Type {
                0
            }
        }
    }
}

pub struct Peripherals {
    pub shmc: SHMC,
}

impl Peripherals {
    pub unsafe fn steal() -> Self {
        Peripherals {
            shmc: SHMC {
                _marker: PhantomData,
            },
        }
    }
}