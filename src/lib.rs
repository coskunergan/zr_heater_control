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

use core::{sync::atomic::AtomicBool, sync::atomic::AtomicI32, sync::atomic::Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use adc_io::Adc;
use dac_io::Dac;
use display_io::Display;
use ds18b20_io::Ds18b20;
use eeprom_int::EepromInt;
use iwdt_io::Iwdt;
use pin_io::Pin;
use sogi_pll::sogi_pll::{pi_transfer, q15_div, spll_update, SogiPllState, Q15_SCALE};

mod adc_io;
mod button;
mod dac_io;
mod display_io;
mod ds18b20_io;
mod eeprom_int;
mod encoder;
mod iwdt_io;
mod pin_io;
mod sogi_pll;
mod usage;

const MAX_PULSE_DEGREE: i32 = 177 * 32768;
const ENCODER_STEP: i32 = (0.1 * 32768.0) as i32;
const ENCODER_MAX: i32 = (45.0 * 32768.0) as i32;
const ENCODER_MIN: i32 = (10.0 * 32768.0) as i32;
const BL_TIMEOUT_SEC: i32 = 5;
const VERSION_MSG: &str = "\r Coskun ERGAN \nVersion: 2.0  \r";

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
static MEASURE_VALUE: AtomicI32 = AtomicI32::new(0);
static ENCODER_COUNT: AtomicI32 = AtomicI32::new(0);
static SET_THETA: AtomicI32 = AtomicI32::new(0);
static BL_TIMEOUT: AtomicI32 = AtomicI32::new(0);
static SET_MODE: AtomicBool = AtomicBool::new(true);

static mut GLOBAL_PIN: Option<Pin> = None;

pub unsafe fn init_pin() {
    GLOBAL_PIN = Some(Pin::new());
}

pub unsafe fn get_pin() -> &'static Pin {
    match &GLOBAL_PIN {
        Some(pin) => pin,
        None => panic!("Pin is not initialized!"),
    }
}

fn adc_callback(idx: usize, value: i16) {
    if idx == 0 {
        critical_section::with(|cs| {
            if let Some(state_ref) = SOGI_STATE_REF.borrow(cs).get() {
                let mut state = state_ref.lock().unwrap();
                state.duration_ns = usage::measure_function_duration_ns(|| {
                    spll_update(value as i32 * Q15_SCALE as i32, &mut state);
                    let theta: i32 = state.get_half_theta();
                    let set_theta: i32 = SET_THETA.load(Ordering::Relaxed);
                    {
                        unsafe {
                            let pin = get_pin();
                            if !state.get_lock() {
                                // pin LOW
                                pin.set(false);
                            } else if theta > set_theta {
                                // pin HIGH
                                pin.set(true);
                            } else if theta > MAX_PULSE_DEGREE || theta < set_theta {
                                // pin LOW
                                pin.set(false);
                            }
                        }
                    }
                    if let Some(dac_ref) = DAC_STATE_REF.borrow(cs).get() {
                        let dac = dac_ref.lock().unwrap();
                        let dac_value = (((theta / 32768) * 4096) / 180).clamp(0, 4095) as u32;
                        dac.write(dac_value);
                    }
                }) as u32;
            }
        });
    }
}

#[embassy_executor::task]
async fn display_task(spawner: Spawner) {
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
    let mut theta: i32 = 0;

    let gpio_token = Arc::new(Mutex::new(unsafe { GpioToken::get_instance().unwrap() }));
    let button = zephyr::devicetree::labels::button::get_instance().unwrap();

    declare_buttons!(
        spawner,
        gpio_token,
        [(
            button,
            || {
                zephyr::printk!("Button Pressed!\n");
                BL_TIMEOUT.store(BL_TIMEOUT_SEC * 2, Ordering::Relaxed);
                SET_MODE.store(!SET_MODE.load(Ordering::Relaxed), Ordering::Relaxed);
                DISPLAY_SIGNAL.signal(true);
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
                if SET_MODE.load(Ordering::Relaxed) {
                    let mut value = ENCODER_COUNT.load(Ordering::Relaxed);
                    if clockwise && value < ENCODER_MAX {
                        value += ENCODER_STEP;
                    } else if value > ENCODER_MIN {
                        value -= ENCODER_STEP;
                    }
                    ENCODER_COUNT.store(value, Ordering::Relaxed);
                    ENCODER_SIGNAL.signal(clockwise);
                    DISPLAY_SIGNAL.signal(true);
                }
                BL_TIMEOUT.store(BL_TIMEOUT_SEC * 2, Ordering::Relaxed);
            },
            Duration::from_millis(1)
        )]
    );

    display.set_backlight(1);
    display.write(VERSION_MSG.as_bytes());
    let _ = Timer::after(Duration::from_millis(2000)).await;

    loop {
        //---------------------------------------
        let mut timeout = BL_TIMEOUT.load(Ordering::Relaxed);
        if timeout > 0 {
            timeout -= 1;
            BL_TIMEOUT.store(timeout, Ordering::Relaxed);
            if timeout == 0 {
                SET_MODE.store(false, Ordering::Relaxed);
                display.set_backlight(0);
            } else {
                display.set_backlight(1);
            }
        }
        //---------------------------------------
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
            theta = state.get_half_theta();
        });

        {
            let mode = SET_MODE.load(Ordering::Relaxed);
            display.clear();
            let encoder = ENCODER_COUNT.load(Ordering::Relaxed) / ENCODER_STEP;
            let msg = format!(
                //"ISI: {:04.1}  %{:02}  SET:{}{:02}.{:1}{} {}{:02}",
                "\rISI: {:04.1} %{:02}\nSET:{}{:02}.{:1}{}{}{:02} ", //(convert-2)
                MEASURE_VALUE.load(Ordering::Relaxed) as f32 / 32768.0,
                (100 - (q15_div(
                    100 * 32768,
                    q15_div(MAX_PULSE_DEGREE, SET_THETA.load(Ordering::Relaxed))
                )) / 32768)
                    .clamp(0, 99),
                if mode { '>' } else { ' ' },
                encoder / 10,
                encoder % 10,
                if mode { '<' } else { ' ' },
                if lock { 'L' } else { 'x' },
                freq
            );
            display.write(msg.as_bytes());
        }

        //log::info!(
        zephyr::printk!(
                ">SENSOR:{}, ENC:{}, OFFSET_MAX:{:.3}, OFFSET_MIN:{:.3}, OMEGA:{:.3}, THETA:{:.3}, S1:{:.3}, S2:{:.3}, ERR:{:.3}, FREQ:{:.3}, D_TIME:{}\n\0",
                MEASURE_VALUE.load(Ordering::Relaxed),
                ENCODER_COUNT.load(Ordering::Relaxed),
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

        let _ = Timer::after(Duration::from_millis(50)).await;

        DISPLAY_SIGNAL.wait().await;
    }
}

#[embassy_executor::task]
async fn control_task(spawner: Spawner, iwdt: Iwdt) {
    let _ = spawner;
    let mut pid = sogi_pll::sogi_pll::PidState {
        i_sum: 0,
        sat_err: 0,
        kp: 75 * 3276, // 7.5
        ki: 15 * 327,  // 0.15
        kc: 10 * 327,  // 0.1
        i_min: 0,
        i_max: MAX_PULSE_DEGREE,
    };
    let sensor = Ds18b20::new();
    let mut measure_value: i32 = 0;

    let eeprom = EepromInt::new();

    // let test_value = 35 *32768;
    // match eeprom.write(0, &test_value) {
    //     Ok(()) => log::info!("Eeprom data was write succesfly {:?}", test_value),
    //     Err(e) => log::info!("Eeprom data write failure! error: {}", e),
    // };

    let mut eeprom_value: i32 = match eeprom.read(0) {
        Ok(value) => value,
        Err(_) => 0,
    };

    if eeprom_value < ENCODER_MIN || eeprom_value > ENCODER_MAX {
        eeprom_value = (ENCODER_MAX - ENCODER_MIN) / 2; // default
    }

    ENCODER_COUNT.store(eeprom_value, Ordering::Relaxed);

    loop {
        //---------------------------------------------
        iwdt.reload();
        //---------------------------------------------
        let _ = Timer::after(Duration::from_millis(500)).await;
        //---------------------------------------------
        let set_value: i32 = ENCODER_COUNT.load(Ordering::Relaxed);
        //---------------------------------------------
        sensor.read(&mut measure_value);
        log::info!("Sensor Value: {}\n\0", (measure_value as f32 / 32768.0));
        MEASURE_VALUE.store(measure_value, Ordering::Relaxed);
        //---------------------------------------------
         let set_degree: i32 = pi_transfer(measure_value - set_value as i32, &mut pid);
        //---------------------------------------------
        //let set_degree = 177 * 32768; // test
        SET_THETA.store(set_degree, Ordering::Release);

        DISPLAY_SIGNAL.signal(true);
        //---------------------------------------------
        eeprom_value = match eeprom.read(0) {
            Ok(value) => value,
            Err(_) => 0,
        };
        if eeprom_value != set_value {
            match eeprom.write(0, &set_value) {
                Ok(()) => log::info!("Eeprom data was write succesfly {:?}", set_value / 3276),
                Err(e) => log::info!("Eeprom data write failure! error: {}", e),
            };
        }
        //---------------------------------------------
    }
}

#[no_mangle]
extern "C" fn rust_main() {
    let iwdt = Iwdt::new(8000);

    let _ = usage::set_logger();

    log::info!("Restart!!!\r\n");

    unsafe {
        init_pin();
    }

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
        spawner.spawn(control_task(spawner, iwdt)).unwrap();
    })
}
