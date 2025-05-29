// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use core::ptr;
use zephyr::raw::{device, device_get_binding, eeprom_read, eeprom_write};
use core::ffi::{c_char, c_int, c_void};

type off_t = i32;

static mut EEPROM_DEVICE: *const device = ptr::null();

pub struct EepromInt {
    _private: (),
}

impl EepromInt {
    pub fn new() -> Self {
        unsafe {
            EEPROM_DEVICE = device_get_binding(b"eeprom1\0".as_ptr() as *const c_char);
            if EEPROM_DEVICE.is_null() {                
                panic!("Failed to initialize EEPROM.");
            }
        }
        EepromInt { _private: () }
    }

    pub fn read(&self, offset: off_t, data: &mut [u8]) -> Result<(), c_int> {
        unsafe {
            if EEPROM_DEVICE.is_null() {
                return Err(-1);
            }

            let result = eeprom_read(
                EEPROM_DEVICE,
                offset,
                data.as_mut_ptr() as *mut c_void,
                data.len(),
            );

            if result == 0 {
                Ok(())
            } else {
                Err(result)
            }
        }
    }

    pub fn write(&self, offset: off_t, data: &[u8]) -> Result<(), c_int> {
        unsafe {
            if EEPROM_DEVICE.is_null() {
                return Err(-1);
            }

            let result = eeprom_write(
                EEPROM_DEVICE,
                offset,
                data.as_ptr() as *const c_void,
                data.len(),
            );

            if result == 0 {
                Ok(())
            } else {
                Err(result)
            }
        }
    }
}
