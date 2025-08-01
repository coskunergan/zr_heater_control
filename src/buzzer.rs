// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use alloc::boxed::Box;
use alloc::sync::Arc;
use zephyr::raw::ZR_GPIO_OUTPUT;

use super::GpioPin;
use log::warn;

pub struct Buzzer {
    pin: GpioPin,
    callback: Box<dyn Fn() + Send + Sync + 'static>,
}

impl Buzzer {
    pub fn new(pin: GpioPin, callback: Box<dyn Fn() + Send + Sync + 'static>) -> Self {
        Self { pin, callback }
    }

    #[allow(dead_code)]
    pub fn set_callback(&mut self, cb: Box<dyn Fn() + Send + Sync + 'static>) {
        self.callback = cb;
    }

    #[allow(dead_code)]
    pub fn trigger_callback(&self) {
        (self.callback)();
    }

    pub async fn trigger(&mut self, active_duration: Duration) {
        if !self.pin.is_ready() {
            warn!("Buzzer pin is not ready");
            return;
        }

        self.pin.configure(ZR_GPIO_OUTPUT);

        unsafe { self.pin.set(true) };

        (self.callback)();

        Timer::after(active_duration).await;

        unsafe { self.pin.set(false) };
    }
}


#[derive(Clone)]
pub struct BuzzerHandle {
    signal: Arc<Signal<NoopRawMutex, Duration>>,
    buzzer_mutex: Arc<Mutex<NoopRawMutex, Buzzer>>,
}

impl BuzzerHandle {
    pub fn new(
        signal: Arc<Signal<NoopRawMutex, Duration>>,
        buzzer_mutex: Arc<Mutex<NoopRawMutex, Buzzer>>,
    ) -> Self {
        Self {
            signal,
            buzzer_mutex,
        }
    }
    pub fn trigger(&self, duration: Duration) {
        self.signal.signal(duration);
    }
}

#[macro_export]
macro_rules! declare_buzzer {
    ($spawner:expr, $pin:expr, $closure:expr) => {{
        use alloc::boxed::Box;
        use alloc::sync::Arc;
        use embassy_sync::blocking_mutex::raw::NoopRawMutex;
        use embassy_sync::mutex::Mutex;
        use embassy_sync::signal::Signal;
        use embassy_time::Duration;

        #[embassy_executor::task]
        async fn buzzer_task(
            signal: Arc<Signal<NoopRawMutex, Duration>>,
            buzzer_mutex: Arc<Mutex<NoopRawMutex, crate::buzzer::Buzzer>>,
        ) {
            loop {
                let duration = signal.wait().await;
                let mut buzzer = buzzer_mutex.lock().await;
                buzzer.trigger(duration).await;
            }
        }

        let pin = $pin;
        let buzzer = crate::buzzer::Buzzer::new(pin, Box::new($closure));
        let buzzer_mutex_arc = Arc::new(Mutex::new(buzzer));
        let signal_arc = Arc::new(Signal::new());

        match $spawner.spawn(buzzer_task(signal_arc.clone(), buzzer_mutex_arc.clone())) {
            Ok(_) => log::info!("Buzzer task started."),
            Err(e) => log::error!("Buzzer task failure: {:?}", e),
        }

        crate::buzzer::BuzzerHandle::new(signal_arc, buzzer_mutex_arc)
    }};
}
