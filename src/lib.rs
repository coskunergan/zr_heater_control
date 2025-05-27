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

use zephyr::{
    device::gpio::{GpioPin, GpioToken},
    sync::{Arc, Mutex},
};

use core::{
    sync::atomic::AtomicBool, sync::atomic::AtomicI32, sync::atomic::AtomicU16,
    sync::atomic::Ordering,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
// use zephyr::device::{self, i2c};
// use zephyr::raw;

use adc_io::Adc;
use dac_io::Dac;
use display_io::Display;
use ds18b20_io::Ds18b20;
use sogi_pll::sogi_pll::{
    fast_sin, pi_transfer, spll_update, SogiPllState, PHASE_OFFSET, Q15_SCALE,
};

mod adc_io;
mod button;
mod dac_io;
mod display_io;
mod ds18b20_io;
mod encoder;
mod sogi_pll;
mod usage;

const PULSE_OFF_DEGREE: u16 = 170;

static EXECUTOR_MAIN: StaticCell<Executor> = StaticCell::new();

static SOGI_STATE: StaticCell<Mutex<SogiPllState>> = StaticCell::new();
static SOGI_STATE_REF: CriticalMutex<OnceCell<&'static Mutex<SogiPllState>>> =
    CriticalMutex::new(OnceCell::new());
static DAC_STATE: StaticCell<Mutex<Dac>> = StaticCell::new();
static DAC_STATE_REF: CriticalMutex<OnceCell<&'static Mutex<Dac>>> =
    CriticalMutex::new(OnceCell::new());

pub static BUTTON_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static ENCODER_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static DISPLAY_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static SENSOR_VALUE: AtomicI32 = AtomicI32::new(0);
static ENCODER_COUNT: AtomicU16 = AtomicU16::new(0);
static SET_THETA: AtomicU16 = AtomicU16::new(0);

fn adc_callback(idx: usize, value: i16) {
    if idx == 0 {
        critical_section::with(|cs| {
            if let Some(state_ref) = SOGI_STATE_REF.borrow(cs).get() {
                let mut state = state_ref.lock().unwrap();
                state.duration_ns = usage::measure_function_duration_ns(|| {
                    spll_update(value as i32 * Q15_SCALE as i32, &mut state);
                    let theta: u16 = state.get_theta() % (180 as u16);
                    let set_theta: u16 = SET_THETA.load(Ordering::SeqCst);
                    if !state.get_lock() {
                        // pin LOW
                    } else if theta > set_theta {
                        // pin HIGH
                    } else if theta > PULSE_OFF_DEGREE || theta < set_theta {
                        // pin LOW
                    }
                    if let Some(dac_ref) = DAC_STATE_REF.borrow(cs).get() {
                        let dac = dac_ref.lock().unwrap();
                        dac.write((theta * (4096 / 180)) as u32);
                    }
                }) as u32;
            }
        });
    }
}

#[embassy_executor::task]
async fn display_task(spawner: Spawner) {
    let _ = spawner;
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
            "Val:{:<03}   F:{:<02}Hz    D:{:.2} uS",
            ENCODER_COUNT.load(Ordering::SeqCst),
            freq,
            (duration_ns as f32 / 1e3)
        );
        display.write(msg.as_bytes());

        //log::info!(
        zephyr::printk!(
                ">SENSOR:{}, ENC:{}, OFFSET_MAX:{:.3}, OFFSET_MIN:{:.3}, OMEGA:{:.3}, THETA:{:.3}, S1:{:.3}, S2:{:.3}, ERR:{:.3}, FREQ:{:.3}, D_TIME:{}\n\0",
                SENSOR_VALUE.load(Ordering::SeqCst),
                ENCODER_COUNT.load(Ordering::SeqCst),
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

        let _ = Timer::after(Duration::from_millis(30)).await;

        DISPLAY_SIGNAL.wait().await;
    }
}

#[embassy_executor::task]
async fn sensor_task(spawner: Spawner) {
    let _ = spawner;
    let sensor = Ds18b20::new();
    let mut data: i32 = 0;
    let mut data_mem: i32 = i32::MAX;
    loop {
        let _ = Timer::after(Duration::from_millis(2000)).await;
        if sensor.read(&mut data) == 0 {
            log::info!("Sensor Value: {}\n\0", data);
            SENSOR_VALUE.store(data, Ordering::Release);
        } else {
            log::info!("Sensor read ERROR!\n\0");
        }
        // if data_mem != data
        {
            data_mem = data;
            DISPLAY_SIGNAL.signal(true);
        }
    }
}

#[embassy_executor::task]
async fn button_task(spawner: Spawner) {
    let gpio_token = Arc::new(Mutex::new(unsafe { GpioToken::get_instance().unwrap() }));
    let button = zephyr::devicetree::labels::button::get_instance().unwrap();

    declare_buttons!(
        spawner,
        gpio_token,
        [(
            button,
            || {
                zephyr::printk!("Button Pressed!\n");
                BUTTON_SIGNAL.signal(true);
            },
            Duration::from_millis(100)
        )]
    );

    let encoder_a = zephyr::devicetree::labels::encoder_a::get_instance().unwrap();
    let encoder_b = zephyr::devicetree::labels::encoder_b::get_instance().unwrap();

    declare_encoders!(
        spawner,
        gpio_token,
        [(
            encoder_a,
            encoder_b,
            |clockwise| {
                let mut value = ENCODER_COUNT.load(Ordering::SeqCst);
                if clockwise {
                    value += 1;
                } else {
                    value -= 1;
                }
                ENCODER_COUNT.store(value, Ordering::Release);
                ENCODER_SIGNAL.signal(clockwise);
                DISPLAY_SIGNAL.signal(true);
            },
            Duration::from_millis(1)
        )]
    );
}

#[embassy_executor::task]
async fn control_task(spawner: Spawner) {
    let _ = spawner;
    let mut pid = sogi_pll::sogi_pll::PidState {
        i_sum: 0,
        sat_err: 0,
        kp: 75 * 3276, // 7.5
        ki: 15 * 327,  // 0.15
        kc: 10 * 327,  // 0.1
        i_min: 0 * 32768,
        i_max: PULSE_OFF_DEGREE as i32 * 32768,
    };

    loop {
        let _ = Timer::after(Duration::from_millis(500)).await;

        let set_value: u16 = ENCODER_COUNT.load(Ordering::SeqCst) / 10;
        let measure_value: i32 = SENSOR_VALUE.load(Ordering::SeqCst);
        let set_degree = pi_transfer(measure_value - set_value as i32, &mut pid) as u16;

        SET_THETA.store(set_degree, Ordering::Release);
    }
}

#[no_mangle]
extern "C" fn rust_main() {
    let _ = usage::set_logger();

    // let  i2c_dev = zephyr::devicetree::aliases::eeprom_0::get_instance().unwrap();
    // let get_serial: [u8; 2] = [0x36, 0x82];
    // let mut id: [u8; 9] = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    // i2c_dev.write_read(&get_serial, &mut id);

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
        spawner.spawn(sensor_task(spawner)).unwrap();
        spawner.spawn(button_task(spawner)).unwrap();
        spawner.spawn(control_task(spawner)).unwrap();
    })
}
