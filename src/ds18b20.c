// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

static const struct device *const dev = DEVICE_DT_GET_ANY(maxim_ds18b20);
static struct sensor_value temp;
static int32_t measure;
static int status = -ENODEV;

LOG_MODULE_REGISTER(ds18b20, LOG_LEVEL_DBG);

int32_t min_max_filter(int32_t val)
{
    static int32_t measure_value_arr[5] = {-99, 99, -99, 99, -99};
    static uint8_t index = 0;
    uint8_t min_index = 0;
    uint8_t max_index = 0;
    uint8_t catch_index = 0x1F;
    uint8_t i, j;
    int32_t temp_min;
    int32_t temp_max;
    measure_value_arr[index] = val;
    index++;
    index %= 5;
    for(j = 0; j < 2; j++)
    {
        //------- find min -------
        for(i = 0; i < 5; i++)
        {
            if(catch_index & (1U << i))
            {
                break;
            }
        }
        temp_min = measure_value_arr[i];
        for(i = 0; i < 5; i++)
        {
            if((catch_index & (1U << i)) && (measure_value_arr[i] < temp_min))
            {
                temp_min = measure_value_arr[i];
                min_index = i;
            }
        }
        catch_index &= ~(1U << min_index);
        //------- find max -------
        for(i = 0; i < 5; i++)
        {
            if(catch_index & (1U << i))
            {
                break;
            }
        }
        temp_max = measure_value_arr[i];
        for(i = 0; i < 5; i++)
        {
            if((catch_index & (1U << i)) && (measure_value_arr[i] > temp_max))
            {
                temp_max = measure_value_arr[i];
                max_index = i;
            }
        }
        catch_index &= ~(1U << max_index);
    }
    for(i = 0; i < 5; i++)
    {
        catch_index >>= 1U;
        if(catch_index == 0)
        {
            break;
        }
    }
    return measure_value_arr[i];
}

void sensor_work_handler(struct k_work *work)
{
    if(dev)
    {
        int rc = sensor_sample_fetch(dev);
        if(rc == 0)
        {
            sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            measure = min_max_filter(((uint64_t)temp.val1 * 32768) + (((uint64_t)temp.val2 * 32768) / 1000000));
        }
        else if(measure < (999 * 3276))
        {
            measure += 32768;
            measure = min_max_filter(measure);
        }
        status = rc;
    }
}

K_WORK_DEFINE(sensor_work, sensor_work_handler);

void sensor_timer_handler(struct k_timer *timer)
{
    k_work_submit(&sensor_work);
}

K_TIMER_DEFINE(sensor_timer, sensor_timer_handler, NULL);

int ds18b20_init()
{
    if(!device_is_ready(dev))
    {
        LOG_ERR("ds18b20 device is not ready.");
        return -ENODEV;
    }
    k_timer_start(&sensor_timer, K_SECONDS(1), K_SECONDS(1));
    return 0;
}

int ds18b20_read_q15(int32_t *value)
{
    *value = measure;
    return status;
}

