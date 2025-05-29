// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

extern "C" {
    fn iwdt_init(timeout: u32) -> i32;
    fn iwdt_reload() -> i32;
}

pub struct Iwdt {
    _private: (),
}

impl Iwdt {
    pub fn new(timeout: u32) -> Self {
        let ret = unsafe { iwdt_init(timeout) };
        if ret != 0 {
            panic!("Failed to initialize IWDT: error {}", ret);
        }

        Iwdt { _private: () }
    }

    pub fn reload(&self) {
        let ret = unsafe { iwdt_reload() };
        if ret != 0 {
            panic!("Failed to write to IWDT: error {}", ret);
        }
    }
}
