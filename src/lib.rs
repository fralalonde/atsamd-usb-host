//! USB Host driver implementation for SAMD* series chips.
//! USB Host driver implementation for SAMD* series chips.

#![no_std]

mod pipe;
mod host;

#[macro_use]
extern crate defmt;

use core::mem;

pub use host::*;

