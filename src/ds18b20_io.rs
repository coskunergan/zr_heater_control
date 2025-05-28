// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

extern "C" {
    fn ds18b20_init() -> i32;
    fn ds18b20_read_q15(data: &mut i32) -> i32;
}

pub struct Ds18b20 {
    _private: (),
}

impl Ds18b20 {
    pub fn new() -> Self {
        unsafe { ds18b20_init(); };        
        Ds18b20 { _private: () }
    }

    pub fn read(&self, data: &mut i32)  -> i32{
        unsafe { ds18b20_read_q15(data) }
    }
}
