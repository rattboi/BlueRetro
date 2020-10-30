/*
 * Copyright (c) 2019-2020, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "../zephyr/types.h"
#include "../util.h"
#include "dc.h"

enum {
    DC_Z = 0,
    DC_Y,
    DC_X,
    DC_D,
    DC_RD_UP,
    DC_RD_DOWN,
    DC_RD_LEFT,
    DC_RD_RIGHT,
    DC_C,
    DC_B,
    DC_A,
    DC_START,
    DC_LD_UP,
    DC_LD_DOWN,
    DC_LD_LEFT,
    DC_LD_RIGHT,
};

const uint8_t dc_axes_idx[ADAPTER_MAX_AXES] =
{
/*  AXIS_LX, AXIS_LY, AXIS_RX, AXIS_RY, TRIG_L, TRIG_R  */
    7,       6,       5,       4,       0,      1
};

const struct ctrl_meta dc_btns_meta =
{
    .polarity = 1,
};

const struct ctrl_meta dc_axes_meta[ADAPTER_MAX_AXES] =
{
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0x80},
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0x80, .polarity = 1},
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0x80},
    {.size_min = -128, .size_max = 127, .neutral = 0x80, .abs_max = 0x80, .polarity = 1},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0xFF},
    {.size_min = 0, .size_max = 255, .neutral = 0x00, .abs_max = 0xFF},
};

struct dc_map {
    union {
        struct {
            uint16_t reserved;
            uint16_t buttons;
        };
        uint8_t axes[8];
    };
} __packed;

const uint32_t dc_mask[4] = {0x333FFFFF, 0x00000000, 0x00000000, 0x00000000};
const uint32_t dc_desc[4] = {0x110000FF, 0x00000000, 0x00000000, 0x00000000};

const uint32_t dc_btns_mask[32] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    BIT(DC_LD_LEFT), BIT(DC_LD_RIGHT), BIT(DC_LD_DOWN), BIT(DC_LD_UP),
    BIT(DC_RD_LEFT), BIT(DC_RD_RIGHT), BIT(DC_RD_DOWN), BIT(DC_RD_UP),
    BIT(DC_X), BIT(DC_B), BIT(DC_A), BIT(DC_Y),
    BIT(DC_START), BIT(DC_D), 0, 0,
    0, BIT(DC_Z), 0, 0,
    0, BIT(DC_C), 0, 0,
};

void dc_init_buffer(int32_t dev_mode, struct wired_data *wired_data) {
    struct dc_map *map = (struct dc_map *)wired_data->output;

    map->buttons = 0xFFFF;
    for (uint32_t i = 0; i < ADAPTER_MAX_AXES; i++) {
        map->axes[dc_axes_idx[i]] = dc_axes_meta[i].neutral;
    }
}

void dc_meta_init(int32_t dev_mode, struct generic_ctrl *ctrl_data) {
    memset((void *)ctrl_data, 0, sizeof(*ctrl_data)*4);

    for (uint32_t i = 0; i < WIRED_MAX_DEV; i++) {
        for (uint32_t j = 0; j < ADAPTER_MAX_AXES; j++) {
            ctrl_data[i].mask = dc_mask;
            ctrl_data[i].desc = dc_desc;
            ctrl_data[i].axes[j].meta = &dc_axes_meta[j];
        }
    }
}
