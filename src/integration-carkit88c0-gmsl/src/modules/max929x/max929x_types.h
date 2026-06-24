/*
 * Copyright (c) 2020-2026, Infineon Technologies AG.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAX929X_TYPES_H
#define MAX929X_TYPES_H

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>

#define MAX929X_DELAY    0x00

struct max929x_reg
{
    u16 slave_addr;
    u16 reg;
    u16 val;
};

struct max929x
{
    struct i2c_client *i2c_client;
    struct regmap *regmap;
    struct gpio_desc *pwdn_gpio;
    bool streaming_en;
    u32 deser_index;
};

extern struct max929x *extenal_priv[4];

#endif
