// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

extern crate alloc;
use alloc::format;
use core::f32::consts;

pub mod sogi_pll {
    use alloc::format;
    use core::f32::consts;
    pub const Q15_SHIFT: i32 = 15;
    pub const Q15_SCALE: f32 = (1 << Q15_SHIFT) as f32;
    pub const TWO_PI: f32 = consts::PI * 2.0;
    pub const SAMPLE_FREQ: f32 = 2500.0;
    pub const TARGET_FREQ: f32 = 50.0;
    pub const TARGET_FREQ_RANGE: f32 = 15.0;
    pub const PI_Q15: i32 = (consts::PI * Q15_SCALE) as i32;
    pub const TAU_Q15: i32 = (TWO_PI * Q15_SCALE) as i32;
    pub const T_Q15: i32 = ((1.0 / SAMPLE_FREQ as f32) * Q15_SCALE) as i32;
    pub const INPUT_MAX: i32 = 4095;
    pub const INPUT_MIN: i32 = 0;
    pub const T: f32 = 1.0 / SAMPLE_FREQ as f32;
    pub const N_SAMPLE: usize = (1.0 / T / TARGET_FREQ) as usize;
    pub const PID_KP_FLOAT: f32 = 150.0;
    pub const PID_KI_FLOAT: f32 = 15.0;
    pub const PID_KC_FLOAT: f32 = 10.0;
    pub const OFFSET_STEP_COEFF: i32 = (1e4) as i32;
    pub const SOGI_K_FLOAT: f32 = 1.4142135623730951;
    pub const ANGLE_TO_RAD_SCALE: f32 = 360.0 / TWO_PI;
    pub const RAD_TO_ANGLE_SCALE: f32 = TWO_PI / 360.0;
    pub const FREQ_LIMIT: f32 = TARGET_FREQ + TARGET_FREQ_RANGE;
    pub const ANGULAR_FREQ: f32 = FREQ_LIMIT * TWO_PI;
    pub const PID_I_MAX: i32 = (ANGULAR_FREQ * Q15_SCALE) as i32;
    pub const PID_I_MIN: i32 = (-ANGULAR_FREQ * Q15_SCALE) as i32;
    pub const PID_KP: i32 = (PID_KP_FLOAT * Q15_SCALE) as i32;
    pub const PID_KI: i32 = (PID_KI_FLOAT * Q15_SCALE) as i32;
    pub const PID_KC: i32 = (PID_KC_FLOAT * Q15_SCALE) as i32;
    pub const INITIAL_OMEGA: i32 = (TARGET_FREQ * TWO_PI * Q15_SCALE) as i32;
    pub const OFFSET_STEP: i32 = (((INPUT_MAX - INPUT_MIN) * Q15_SCALE as i32) / OFFSET_STEP_COEFF) as i32;
    pub const SOGI_K: i32 = (SOGI_K_FLOAT * Q15_SCALE) as i32;
    pub const ANGLE_TO_RAD: i32 = (ANGLE_TO_RAD_SCALE * Q15_SCALE) as i32;
    pub const RAD_TO_ANGLE: i32 = (RAD_TO_ANGLE_SCALE * Q15_SCALE) as i32;
    pub const HALF_SCALE: i32 = (0.5 * Q15_SCALE) as i32;
    pub const DEFAULT_DENOM: i32 = (1.0 * Q15_SCALE) as i32;
    pub const THETA_SCALE: i32 = (360.0 / (TWO_PI * 180.0) * Q15_SCALE) as i32;
    pub const PHASE_OFFSET: i32 = ((consts::PI / 2.0) * Q15_SCALE) as i32;

    pub fn q15_to_float(value: i32) -> f32 {
        value as f32 / Q15_SCALE
    }

    pub fn q15_mul(a: i32, b: i32) -> i32 {
        (((a as i64) * (b as i64)) >> Q15_SHIFT) as i32
    }

    pub fn q15_add(a: i32, b: i32) -> i32 {
        a.saturating_add(b)
    }

    pub fn q15_sub(a: i32, b: i32) -> i32 {
        a.saturating_sub(b)
    }

    pub fn q15_div(a: i32, b: i32) -> i32 {
        if b == 0 {
            return 0;
        }
        (((a as i64) << Q15_SHIFT) / b as i64) as i32
    }

    #[derive(Clone, Copy)]
    pub struct PidState {
        pub i_sum: i32,
        pub sat_err: i32,
        pub kp: i32,
        pub ki: i32,
        pub kc: i32,
        pub i_min: i32,
        pub i_max: i32,
    }

    #[derive(Clone, Copy)]
    pub struct SogiPllState {
        pub pid: PidState,
        pub launch_loop: bool,
        pub sample_index: u16,
        pub omega: i32,
        pub cur_phase: i32,
        pub auto_offset_min: i32,
        pub auto_offset_max: i32,
        pub sogi_s1: i32,
        pub sogi_s2: i32,
        pub last_error: i32,
        pub duration_ns: u32,
    }

    impl SogiPllState {
        pub const fn new() -> Self {
            SogiPllState {
                pid: PidState {
                    i_sum: 0,
                    sat_err: 0,
                    kp: PID_KP,
                    ki: PID_KI,
                    kc: PID_KC,
                    i_min: PID_I_MIN,
                    i_max: PID_I_MAX,
                },
                launch_loop: false,
                sample_index: 0,
                omega: INITIAL_OMEGA,
                cur_phase: 0,
                auto_offset_min: INPUT_MAX * Q15_SCALE as i32,
                auto_offset_max: INPUT_MIN * Q15_SCALE as i32,
                sogi_s1: 0,
                sogi_s2: 0,
                last_error: 0,
                duration_ns: 0,
            }
        }

        pub fn reset(&mut self) {
            self.pid.i_sum = 0;
            self.pid.sat_err = 0;
            self.launch_loop = false;
            self.sample_index = 0;
            self.omega = INITIAL_OMEGA;
            self.cur_phase = 0;
            self.auto_offset_min = INPUT_MAX * Q15_SCALE as i32;
            self.auto_offset_max = INPUT_MIN * Q15_SCALE as i32;
            self.sogi_s1 = 0;
            self.sogi_s2 = 0;
            self.last_error = 0;
        }

        pub fn is_lock(&self, th: i32) -> bool {
            self.launch_loop && self.last_error.abs() < th
        }

        pub fn get_freq(&self) -> u8 {
            (q15_div(self.omega, TAU_Q15) / Q15_SCALE as i32).clamp((TARGET_FREQ - TARGET_FREQ_RANGE)as i32, (TARGET_FREQ + TARGET_FREQ_RANGE) as i32) as u8
        }

        pub fn get_theta(&self) -> u16 {
            (q15_mul(self.cur_phase, THETA_SCALE) / Q15_SCALE as i32) as u16
        }

        pub fn get_lock(&self) -> bool {
            self.is_lock((30e-3 * Q15_SCALE) as i32)
        }
    }

    pub fn fast_sin(theta: i32) -> i32 {
        const SIN_COEFF: i32 = ((consts::PI / 4.0) * Q15_SCALE) as i32;
        const SIN_CUBIC_COEFF: i32 = ((1.0 / 6.0) * Q15_SCALE) as i32;

        let mut theta = theta % TAU_Q15;
        if theta < 0 {
            theta += TAU_Q15;
        }
        let mut sign = 1;
        let mut x = theta;

        if theta >= PI_Q15 / 2 && theta < PI_Q15 {
            x = PI_Q15 - theta;
        } else if theta >= PI_Q15 && theta < PI_Q15 + PI_Q15 / 2 {
            x = theta - PI_Q15;
            sign = -1;
        } else if theta >= PI_Q15 + PI_Q15 / 2 {
            x = TAU_Q15 - theta;
            sign = -1;
        }

        let x_scaled = q15_mul(x, SIN_COEFF);
        let x2 = q15_mul(x_scaled, x_scaled);
        let x3 = q15_mul(x2, x_scaled);
        let sin_x = q15_sub(x_scaled, q15_mul(x3, SIN_CUBIC_COEFF));

        q15_mul(sin_x, sign * Q15_SCALE as i32)
    }

    pub fn fast_cos(theta: i32) -> i32 {
        fast_sin(q15_add(theta, PI_Q15 / 2))
    }

    pub fn pi_transfer(e: i32, pid: &mut PidState) -> i32 {
        let sat = q15_add(q15_mul(pid.kp, e), pid.i_sum);
        let out = if sat > pid.i_max {
            pid.i_max
        } else if sat < pid.i_min {
            pid.i_min
        } else {
            sat
        };
        pid.sat_err = q15_sub(out, sat);
        pid.i_sum = q15_add(
            pid.i_sum,
            q15_add(q15_mul(pid.ki, e), q15_mul(pid.kc, pid.sat_err)),
        );
        if pid.i_sum > pid.i_max {
            pid.i_sum = pid.i_max;
        } else if pid.i_sum < pid.i_min {
            pid.i_sum = pid.i_min;
        }
        out
    }

    pub fn spll_update(val: i32, state: &mut SogiPllState) {
        let v_org = {
            state.auto_offset_max = q15_sub(state.auto_offset_max, OFFSET_STEP);
            state.auto_offset_min = q15_add(state.auto_offset_min, OFFSET_STEP);
            if val > state.auto_offset_max {
                state.auto_offset_max = val;
            }
            if val < state.auto_offset_min {
                state.auto_offset_min = val;
            }
            let mid = q15_mul(
                q15_add(state.auto_offset_min, state.auto_offset_max),
                HALF_SCALE,
            );
            q15_sub(val, mid)
        };

        state.sample_index = state.sample_index.wrapping_add(1);

        if state.sample_index < N_SAMPLE as u16 {
            state.launch_loop = false;
            return;
        } else {
            state.launch_loop = true;
        }

        let denom = q15_sub(state.auto_offset_max, state.auto_offset_min).max(DEFAULT_DENOM);
        let v = q15_div(v_org, denom);

        let sogi_u = q15_mul(
            q15_sub(q15_mul(SOGI_K, q15_sub(v, state.sogi_s1)), state.sogi_s2),
            INITIAL_OMEGA,
        );
        state.sogi_s1 = q15_add(state.sogi_s1, q15_mul(T_Q15, sogi_u));
        state.sogi_s2 = q15_add(
            state.sogi_s2,
            q15_mul(T_Q15, q15_mul(INITIAL_OMEGA, state.sogi_s1)),
        );

        let ua = state.sogi_s1;
        let ub = state.sogi_s2;
        let theta = q15_mul(state.cur_phase, ANGLE_TO_RAD);
        let st = fast_sin(q15_mul(theta, RAD_TO_ANGLE));
        let ct = fast_cos(q15_mul(theta, RAD_TO_ANGLE));
        let uq = q15_sub(q15_mul(ct, ub), q15_mul(st, ua));

        let e = q15_sub(0, uq);
        let u = pi_transfer(e, &mut state.pid);

        let mut i = q15_add(state.cur_phase, q15_mul(T_Q15, u));
        if i > TAU_Q15 {
            i = q15_sub(i, TAU_Q15);
        } else if i < -TAU_Q15 {
            i = q15_add(i, TAU_Q15);
        }

        state.omega = u;
        state.cur_phase = i;
        state.last_error = e;
    }
}