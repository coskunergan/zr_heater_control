// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use alloc::vec;
use core::ffi::{c_char, c_int, c_void};
use core::mem;
use core::ptr;
use core::slice;
use zephyr::raw::{device, device_get_binding, eeprom_read, eeprom_write};

type Offt = i32;

static mut EEPROM_DEVICE: *const device = ptr::null();

pub struct EepromInt {
    _private: (),
}

impl EepromInt {
    pub fn new() -> Self {
        unsafe {
            EEPROM_DEVICE = device_get_binding(b"eeprom1\0".as_ptr() as *const c_char);
            //EEPROM_DEVICE = device_get_binding(b"eeprom@8080000\0".as_ptr() as *const c_char);
            if EEPROM_DEVICE.is_null() {
                panic!("Failed to initialize EEPROM.");
            }
        }
        EepromInt { _private: () }
    }

    pub fn read<T: Copy>(&self, offset: Offt) -> Result<T, c_int> {
        unsafe {
            if EEPROM_DEVICE.is_null() {
                return Err(-1);
            }

            let mut buffer = vec![0u8; mem::size_of::<T>()];
            let result = eeprom_read(
                EEPROM_DEVICE,
                offset,
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len(),
            );

            if result == 0 {
                // Türün boyutuna uygun baytları oku ve T türüne dönüştür
                let value = ptr::read(buffer.as_ptr() as *const T);
                Ok(value)
            } else {
                Err(result)
            }
        }
    }

    pub fn write<T: Copy>(&self, offset: Offt, data: &T) -> Result<(), c_int> {
        unsafe {
            if EEPROM_DEVICE.is_null() {
                return Err(-1);
            }

            let bytes = slice::from_raw_parts(data as *const T as *const u8, mem::size_of::<T>());
            let result = eeprom_write(
                EEPROM_DEVICE,
                offset,
                bytes.as_ptr() as *const c_void,
                bytes.len(),
            );

            if result == 0 {
                Ok(())
            } else {
                Err(result)
            }
        }
    }
}
