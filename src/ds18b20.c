// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ds18b20, LOG_LEVEL_DBG);

//static const struct device *const dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(maxim_ds18b20));
static const struct device *const dev = DEVICE_DT_GET_OR_NULL(ds18b20);

int ds18b20_init()
{
    if(!device_is_ready(dev))
    {
        LOG_ERR("ds18b20 device is not ready.");
        return -ENODEV;
    }
    return 0;
}

int ds18b20_read(int32_t *value)
{    
    if(dev)
    {
        struct sensor_value temp;
        int rc = sensor_sample_fetch(dev);
        if(rc == 0)
        {
            sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            *value = temp.val2;
        }
        return rc;
    }
    return ENODEV;
}

