// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use embassy_time::{Duration, Timer};

use alloc::boxed::Box;
use zephyr::{
    raw::ZR_GPIO_INPUT,
};

use super::{GpioPin};
use log::warn;

pub struct Encoder {
    pin_a: GpioPin,
    pin_b: GpioPin,
    callback: Box<dyn Fn(bool) + Send + Sync + 'static>,
    debounce: Duration,
}

impl Encoder {
    pub fn new(
        pin_a: GpioPin,
        pin_b: GpioPin,
        callback: Box<dyn Fn(bool) + Send + Sync + 'static>,
        debounce: Duration,
    ) -> Self {
        Self {
            pin_a,
            pin_b,
            callback,
            debounce,
        }
    }

    pub async fn work(&mut self) {

        if !self.pin_a.is_ready() || !self.pin_b.is_ready() {
            warn!("Encoder pins is not ready");
            loop {}
        }

        self.pin_a.configure(ZR_GPIO_INPUT);
        self.pin_b.configure(ZR_GPIO_INPUT);

        loop {
            let mut state = 0;

            unsafe { self.pin_a.wait_for_low().await };

            if self.pin_b.get() == false {
                state += 1;
            } else {
                state -= 1;
            }

            Timer::after(self.debounce).await;

            unsafe { self.pin_a.wait_for_high().await };

            if self.pin_b.get() == true {
                state += 1;
            } else {
                state -= 1;
            }

            Timer::after(self.debounce).await;

            match state {
                2 => (self.callback)(true),
                -2 => (self.callback)(false),
                _ => {
                    warn!("Unexpected state value: {}", state)
                }
            };
        }
    }
}

#[macro_export]
macro_rules! declare_encoders {
    ($spawner:expr, [ $( ($pin_a:expr,$pin_b:expr, $closure:expr, $debounce:expr) ),* ]) => {
        {
            const ENC_COUNT: usize = 0 $( + { let _ = ($debounce); 1 } )*;
            log::info!("Declared Encoder count: {}", ENC_COUNT);

            #[embassy_executor::task(pool_size = ENC_COUNT)]
            async fn encoder_task(mut encoder: crate::encoder::Encoder) {
                encoder.work().await;
            }

            $(
                use alloc::boxed::Box;
                let pin_a = $pin_a;
                let pin_b = $pin_b;
                let debounce = $debounce;
                let encoder = $crate::encoder::Encoder::new(pin_a, pin_b, Box::new($closure), debounce);
                match $spawner.spawn(encoder_task(encoder)) {
                    Ok(_) => log::info!("Encoder task started."),
                    Err(e) => log::error!("Encoder task failure: {:?}", e),
                }
            )*
        }
    };
}
