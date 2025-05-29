// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/watchdog.h>

const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
//const struct device *const wdt = DEVICE_DT_GET_OR_NULL(DT_ALIAS(watchdog0));
int wdt_channel_id;

int iwdt_init(uint32_t timeout)
{
    if(!device_is_ready(wdt))
    {
        return -ENODEV;
    }

    struct wdt_timeout_cfg wdt_config;
    wdt_config.flags = WDT_FLAG_RESET_SOC;
    wdt_config.window.min = 0U;
    wdt_config.window.max = timeout;

    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if(wdt_channel_id == -ENOTSUP)
    {         
        /* IWDG driver for STM32 doesn't support callback */       
        wdt_config.callback = NULL;
        wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    }
    if(wdt_channel_id < 0)
    {        
        return -ENOTSUP;
    }
    int err = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if(err < 0)
    {
        printk("Watchdog setup error\n");
        return err;
    }

    return 0;
}

int iwdt_reload()
{
    return wdt_feed(wdt, wdt_channel_id);
}