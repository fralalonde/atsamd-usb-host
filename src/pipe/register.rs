//! Generic register access.

use core::marker::PhantomData;
use vcell::VolatileCell;

pub trait Readable {}
pub trait Writable {}

pub struct Register<T, Reg> {
    value: VolatileCell<T>,
    _marker: PhantomData<Reg>,
}

impl<T, Reg> From<T> for Register<T, Reg> {
    fn from(v: T) -> Self {
        Self {
            value: VolatileCell::new(v),
            _marker: PhantomData,
        }
    }
}

impl<T, Reg> Register<T, Reg>
where
    Self: Readable,
    T: Copy,
{
    pub fn read(&self) -> R<T, Self> {
        R {
            bits: self.value.get(),
            _marker: PhantomData,
        }
    }
}

impl<T, Reg> Register<T, Reg>
where
    Self: Readable + Writable,
    T: Copy,
{
    pub fn write<F>(&self, f: F)
    where
        F: FnOnce(&mut W<T, Self>) -> &mut W<T, Self>,
    {
        let mut w = W {
            bits: self.value.get(),
            _marker: PhantomData,
        };
        self.value.set(f(&mut w).bits);
    }

    pub fn modify<F>(&self, f: F)
    where
        F: FnOnce(T, &mut W<T, Self>) -> &mut W<T, Self>,
    {
        let mut w = W {
            bits: self.value.get(),
            _marker: PhantomData,
        };
        self.value.set(f(w.bits, &mut w).bits);
    }
}

pub struct R<T, Reg> {
    pub bits: T,
    _marker: PhantomData<Reg>,
}

impl<T, Reg> R<T, Reg>
where
    T: Copy,
{
    pub fn new(bits: T) -> Self {
        Self {
            bits,
            _marker: PhantomData,
        }
    }

    pub fn bits(&self) -> T {
        self.bits
    }
}

pub struct W<T, Reg> {
    pub bits: T,
    _marker: PhantomData<Reg>,
}

impl<T, Reg> W<T, Reg> {
    pub unsafe fn bits(&mut self, bits: T) -> &mut Self {
        self.bits = bits;
        self
    }
}
