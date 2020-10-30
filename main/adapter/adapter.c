/*
 * Copyright (c) 2019-2020, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <xtensa/hal.h>
#include <esp_timer.h>
#include "sdkconfig.h"
#include "../zephyr/types.h"
#include "../util.h"
#include "config.h"
#include "adapter.h"
#include "dc.h"

static buffer_init_t buffer_init_func[WIRED_MAX] = {
    dc_init_buffer, /* DC */
};

struct generic_fb fb_input;
struct wired_adapter wired_adapter = {0};

static void adapter_fb_stop_cb(void* arg) {
    uint8_t dev_id = (uint8_t)(uintptr_t)arg;

    /* Send 1 byte, system that require callback stop shall look for that */
    xRingbufferSend(wired_adapter.input_q_hdl, &dev_id, 1, 0);
}

void adapter_init_buffer(uint8_t wired_id) {
    if (wired_adapter.system_id != WIRED_NONE && buffer_init_func[wired_adapter.system_id]) {
        buffer_init_func[wired_adapter.system_id](config.out_cfg[wired_id].dev_mode, &wired_adapter.data[wired_id]);
    }
}

void adapter_fb_stop_timer_start(uint8_t dev_id, uint64_t dur_us) {
    if (wired_adapter.data[dev_id].fb_timer_hdl == NULL) {
        const esp_timer_create_args_t fb_timer_args = {
            .callback = &adapter_fb_stop_cb,
            .arg = (void*)(uintptr_t)dev_id,
            .name = "fb_timer"
        };
        esp_timer_create(&fb_timer_args, (esp_timer_handle_t *)&wired_adapter.data[dev_id].fb_timer_hdl);
        esp_timer_start_once(wired_adapter.data[dev_id].fb_timer_hdl, dur_us);
    }
}

void adapter_fb_stop_timer_stop(uint8_t dev_id) {
    esp_timer_delete(wired_adapter.data[dev_id].fb_timer_hdl);
    wired_adapter.data[dev_id].fb_timer_hdl = NULL;
}


void IRAM_ATTR adapter_q_fb(uint8_t *data, uint32_t len) {
    UBaseType_t ret;
    ret = xRingbufferSendFromISR(wired_adapter.input_q_hdl, data, len, NULL);
    if (ret != pdTRUE) {
        ets_printf("# %s input_q full!\n", __FUNCTION__);
    }
}

void adapter_init(void) {
    wired_adapter.system_id = WIRED_NONE;

    wired_adapter.input_q_hdl = xRingbufferCreate(64, RINGBUF_TYPE_NOSPLIT);
    if (wired_adapter.input_q_hdl == NULL) {
        printf("# %s: Failed to create ring buffer\n", __FUNCTION__);
    }
}
