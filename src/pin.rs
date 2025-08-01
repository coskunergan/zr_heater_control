// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use super::GpioPin;
use alloc::boxed::Box;
use core::cell::UnsafeCell;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicPtr,AtomicBool, Ordering};
use zephyr::raw::ZR_GPIO_OUTPUT;

static mut PIN_INSTANCE: AtomicPtr<Pin> = AtomicPtr::new(core::ptr::null_mut());
static IS_INITIALIZED: AtomicBool = AtomicBool::new(false);
pub struct Pin {
    _private: (),
    gpio: UnsafeCell<GpioPin>,
}

impl Pin {
    pub fn init(mut pin: GpioPin) {
        if IS_INITIALIZED.load(Ordering::Acquire) {
            panic!("Pin zaten başlatıldı.");
        }

        if !pin.is_ready() {
            panic!("Pin aygıtı hazır değil.");
        }
        
        pin.configure(ZR_GPIO_OUTPUT);

        let pin_instance = Pin {
            _private: (),
            gpio: UnsafeCell::new(pin),
        };

        let pin_ptr = Box::into_raw(Box::new(pin_instance));

        unsafe {
            PIN_INSTANCE.store(pin_ptr, Ordering::Release);
            IS_INITIALIZED.store(true, Ordering::Release);
        }
    }

    #[inline(always)]
    pub fn get() -> &'static Self {
        if !IS_INITIALIZED.load(Ordering::Acquire) {
            panic!("Pin başlatılmadı. `Pin::init()` çağrılmalı.");
        }
        
        unsafe {
            let pin_ptr = PIN_INSTANCE.load(Ordering::Relaxed);
            &*pin_ptr
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