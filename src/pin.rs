// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use super::GpioPin;
use alloc::boxed::Box;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use zephyr::raw::ZR_GPIO_OUTPUT;

pub struct GlobalPin {
    instance: AtomicPtr<Pin>,
    is_initialized: AtomicBool,
}

impl GlobalPin {
    pub const fn new() -> Self {
        GlobalPin {
            instance: AtomicPtr::new(core::ptr::null_mut()),
            is_initialized: AtomicBool::new(false),
        }
    }

    pub fn init(&self, pin: Pin) {
        if self.is_initialized.load(Ordering::Acquire) {
            panic!("Pin already init.");
        }

        let pin_ptr = Box::into_raw(Box::new(pin));

        unsafe {
            self.instance.store(pin_ptr, Ordering::Release);
            self.is_initialized.store(true, Ordering::Release);
        }
    }

    #[inline(always)]
    pub fn get(&self) -> &'static Pin {
        if !self.is_initialized.load(Ordering::Acquire) {
            panic!("Pin not init.");
        }
        unsafe { &*self.instance.load(Ordering::Relaxed) }
    }
}

pub struct Pin {
    _private: (),
    gpio: UnsafeCell<GpioPin>,
}

unsafe impl Send for Pin {}
unsafe impl Sync for Pin {}

impl Pin {
    pub fn new(mut pin: GpioPin) -> Self {
        if !pin.is_ready() {
            panic!("Pin not ready.");
        }

        pin.configure(ZR_GPIO_OUTPUT);

        Pin {
            _private: (),
            gpio: UnsafeCell::new(pin),
        }
    }

    #[inline(always)]
    pub fn set(&self, value: bool) {
        unsafe {
            (*self.gpio.get()).set(value);
        }
    }

    #[inline(always)]
    pub fn toggle(&self) {
        unsafe {
            (*self.gpio.get()).toggle_pin();
        }
    }
}
