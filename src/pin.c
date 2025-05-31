// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

static struct gpio_dt_spec pin_dt;

int pin_init(/*const struct gpio_dt_spec pin*/)
{
    //const struct device *dev = device_get_binding(label);
    struct gpio_dt_spec pin = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(pulse_pin), gpios,
                                    {
                                        0
                                    });
    // pin_dt.port = port;
    // pin_dt.pin = 5;
    // pin_dt.dt_flags = GPIO_ACTIVE_HIGH;
    pin_dt = pin;                                    
    if(!gpio_is_ready_dt(&pin_dt))
    {
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&pin_dt, GPIO_OUTPUT_ACTIVE);
}

void pin_set(bool value)
{
    gpio_pin_set_dt(&pin_dt, value);
}

void pin_toggle()
{
    gpio_pin_toggle_dt(&pin_dt);
}