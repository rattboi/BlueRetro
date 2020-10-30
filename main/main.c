/*
 * Copyright (c) 2019-2020, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adapter/adapter.h"
#include "adapter/config.h"
#include "wired/maple.h"

static void wired_init_task(void *arg) {
    adapter_init();

    adapter_init_buffer(0);

    maple_init();

    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreatePinnedToCore(wired_init_task, "wired_init_task", 2048, NULL, 10, NULL, 1);
}
