// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

extern "C" {
    fn pin_init() -> i32;
    fn pin_set(value: bool);
    fn pin_toggle();
}

pub struct Pin {
    _private: (),
}

impl Pin {
    pub fn new() -> Self {
        let ret = unsafe { pin_init() };
        if ret != 0 {
            panic!("Failed to initialize PIN: error {}", ret);
        }
        Pin { _private: () } 
    }

    pub fn set(&self, value: bool) {
        unsafe { pin_set(value) };        
    }

    pub fn toggle(&self) {
        unsafe { pin_toggle() };        
    }    
}
