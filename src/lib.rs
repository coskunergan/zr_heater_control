// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#![no_std]

extern crate alloc;

use alloc::format;
use embassy_time::{Duration, Timer};

#[cfg(feature = "executor-thread")]
use embassy_executor::Executor;

#[cfg(feature = "executor-zephyr")]
use zephyr::embassy::Executor;

use core::cell::OnceCell;
use critical_section::Mutex as CriticalMutex;
use embassy_executor::Spawner;
use static_cell::StaticCell;
use zephyr::sync::Mutex;

use adc_io::Adc;
use dac_io::Dac;
use display_io::Display;
use sogi_pll::sogi_pll::{
    fast_sin, pi_transfer, q15_to_float, spll_update, SogiPllState, N_SAMPLE, PHASE_OFFSET,
    Q15_SCALE,
};

mod adc_io;
mod dac_io;
mod display_io;
mod sogi_pll;
mod usage;

static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: CriticalMutex<OnceCell<&'static Mutex<SogiPllState>>> =
    CriticalMutex::new(OnceCell::new());
static DAC_STATE: StaticCell<Mutex<Dac>> = StaticCell::new();
static DAC_STATE_REF: CriticalMutex<OnceCell<&'static Mutex<Dac>>> =
    CriticalMutex::new(OnceCell::new());

fn adc_callback(idx: usize, value: i16) {
    if idx == 0 {
        critical_section::with(|cs| {
            if let Some(state_ref) = SOGI_STATE_REF.borrow(cs).get() {
                let mut state = state_ref.lock().unwrap();
                state.duration_ns = usage::measure_function_duration_ns(|| {
                    spll_update(value as i32 * Q15_SCALE as i32, &mut state)
                }) as u32;

                if let Some(dac_ref) = DAC_STATE_REF.borrow(cs).get() {
                    let dac = dac_ref.lock().unwrap();
                    let phase = sogi_pll::sogi_pll::q15_add(state.cur_phase, PHASE_OFFSET);
                    let sin_value: i32 = fast_sin(phase);
                    let amplitude = {
                        let amp = q15_to_float(sogi_pll::sogi_pll::q15_sub(
                            state.auto_offset_max,
                            state.auto_offset_min,
                        )) * 0.5;
                        (amp * 2048.0).min(2048.0)
                    };
                    let dac_value =
                        (q15_to_float(sin_value) * amplitude + 2048.0).clamp(0.0, 4095.0) as u32;
                    dac.write(dac_value);
                }
            }
        });
    }
}

#[embassy_executor::task]
async fn display_task(_spawner: Spawner) {
    let display = Display::new();
    let mut cur_phase: i32 = 0;
    let mut omega: i32 = 0;
    let mut auto_offset_min: i32 = 0;
    let mut auto_offset_max: i32 = 0;
    let mut sogi_s1: i32 = 0;
    let mut sogi_s2: i32 = 0;
    let mut last_error: i32 = 0;
    let mut duration_ns: u32 = 0;
    let mut lock: bool = false;
    let mut freq: u8 = 0;
    let mut theta: u16 = 0;

    display.set_backlight(1);

    let mut my_pid = sogi_pll::sogi_pll::PidState {
        i_sum: 0,
        sat_err: 0,
        kp: 75 * 3276, // 7.5
        ki: 15 * 327, // 0.15
        kc: 10 * 327, // 0.1
        i_min: 18 * 32768,
        i_max: 45 * 32768,
    };

    loop {
        critical_section::with(|cs| {
            let state_ref = SOGI_STATE_REF.borrow(cs).get().unwrap();
            let state = state_ref.lock().unwrap();
            cur_phase = state.cur_phase;
            omega = state.omega;
            auto_offset_min = state.auto_offset_min;
            auto_offset_max = state.auto_offset_max;
            sogi_s1 = state.sogi_s1;
            sogi_s2 = state.sogi_s2;
            last_error = state.last_error;
            duration_ns = state.duration_ns;
            lock = state.get_lock();
            freq = state.get_freq();
            theta = state.get_theta();
        });

        display.clear();
        let msg = format!(
            "sPLL:{}   F:{:<02}Hz    D:{:.2} uS",
            lock as u8,
            freq,
            (duration_ns as f32 / 1e3)
        );
        display.write(msg.as_bytes());

        log::info!(
                ">OFFSET_MAX:{:.3}, OFFSET_MIN:{:.3}, OMEGA:{:.3}, THETA:{:.3}, S1:{:.3}, S2:{:.3}, ERR:{:.3}, FREQ:{:.3}, D_TIME:{}\n\0",
                auto_offset_min,
                auto_offset_max,
                omega,
                theta,
                sogi_s1,
                sogi_s2,
                last_error,
                freq,
                (duration_ns as f32 / 1e3)
            );

        let _ = Timer::after(Duration::from_millis(100)).await;

        let mut res = pi_transfer(0, &mut my_pid);
    }
}

static EXECUTOR_MAIN: StaticCell<Executor> = StaticCell::new();

#[no_mangle]
extern "C" fn rust_main() {
    let _ = usage::set_logger();

    let sogi_state = SOGI_STATE.init(Mutex::new(SogiPllState::new()));
    critical_section::with(|cs| {
        SOGI_STATE_REF.borrow(cs).set(sogi_state).unwrap();
    });

    let dac_state = DAC_STATE.init(Mutex::new(Dac::new()));
    critical_section::with(|cs| {
        DAC_STATE_REF.borrow(cs).set(dac_state).unwrap();
    });

    let mut adc = Adc::new();
    adc.read_async_isr(
        Duration::from_micros((1e6 / sogi_pll::sogi_pll::SAMPLE_FREQ) as u64).into(),
        Some(adc_callback),
    );

    let executor = EXECUTOR_MAIN.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(display_task(spawner)).unwrap();
    })
}
