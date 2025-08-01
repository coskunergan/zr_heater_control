// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use embassy_time::{Duration, Timer};

use alloc::boxed::Box;
use zephyr::raw::ZR_GPIO_INPUT;

use super::GpioPin;
use log::warn;

pub struct Button {
    pin: GpioPin,
    callback: Box<dyn Fn() + Send + Sync + 'static>,
    debounce: Duration,
}

impl Button {
    pub fn new(
        pin: GpioPin,
        callback: Box<dyn Fn() + Send + Sync + 'static>,
        debounce: Duration,
    ) -> Self {
        Self {
            pin,
            callback,
            debounce,
        }
    }
    #[allow(dead_code)]
    pub fn set_callback(&mut self, cb: Box<dyn Fn() + Send + Sync + 'static>) {
        self.callback = cb;
    }
    #[allow(dead_code)]
    pub fn trigger_callback(&self) {
        (self.callback)();
    }

    pub async fn work(&mut self) {
        if !self.pin.is_ready() {
            warn!("Button pin is not ready");
            loop {}
        }

        self.pin.configure(ZR_GPIO_INPUT);

        loop {
            unsafe { self.pin.wait_for_high().await };

            (self.callback)();

            Timer::after(self.debounce).await;

            unsafe { self.pin.wait_for_low().await };

            Timer::after(self.debounce).await;
        }
    }
}

#[macro_export]
macro_rules! declare_buttons {
    ($spawner:expr, [ $( ($pin:expr, $closure:expr, $debounce:expr) ),* ]) => {
        {
            const BUTTON_COUNT: usize = 0 $( + { let _ = ($debounce); 1 } )*;
            log::info!("Declared button count: {}", BUTTON_COUNT);

            #[embassy_executor::task(pool_size = BUTTON_COUNT)]
            async fn button_task(mut button: crate::button::Button) {
                button.work().await;
            }

            $(
                use alloc::boxed::Box;
                let pin = $pin;
                let debounce = $debounce;
                let button = $crate::button::Button::new(pin, Box::new($closure), debounce);
                match $spawner.spawn(button_task(button)) {
                    Ok(_) => log::info!("Button task started."),
                    Err(e) => log::error!("Button task failure: {:?}", e),
                }
            )*
        }
    };
}
