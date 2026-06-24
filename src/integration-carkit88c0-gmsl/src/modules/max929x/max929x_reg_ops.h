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

#ifndef MAX929X_REG_OPS_H
#define MAX929X_REG_OPS_H

#include "max929x_types.h"

#include <linux/delay.h>

static int max929x_write_reg(struct max929x *priv, u8 slave_addr, u16 reg, u8 val)
{
    struct i2c_client *i2c_client = priv->i2c_client;
    int err;

    i2c_client->addr = slave_addr;

    err = regmap_write(priv->regmap, reg, val);
    if (err)
    {
        dev_dbg(&i2c_client->dev, "%s:slave_addr 0x%x i2c write failed, 0x%x = %x\n", __func__, slave_addr, reg, val);
        return err;
    }

    return 0;
}

static int max929x_read_reg(struct max929x *priv, u8 slave_addr, u16 reg, u8 *val)
{
    struct i2c_client *i2c_client = priv->i2c_client;
    int err;
    u32 reg_val;

    i2c_client->addr = slave_addr;

    err = regmap_read(priv->regmap, reg, &reg_val);
    if (err)
    {
        dev_dbg(&i2c_client->dev, "%s:i2c read failed, 0x%x\n", __func__, reg);
        return err;
    }

    *val = reg_val & 0xFF;
    return 0;
}

static int max929x_write_reg_list(struct max929x *priv, struct max929x_reg *table, int size)
{
    int err = 0, i;
    u16 reg;
    u8 slave_addr;
    u8 val, retry;

    for (i = 0; i < size; i++)
    {
        slave_addr = table[i].slave_addr;
        if (slave_addr == MAX929X_DELAY)
        {
            msleep(table[i].val);
            continue;
        }

        slave_addr = slave_addr >> 1;
        reg        = table[i].reg;
        val        = table[i].val;
        retry      = 3;
        /*retry 3 times*/
        while (retry)
        {
            err = max929x_write_reg(priv, slave_addr, reg, val);
            if (!err)
                break;
            usleep_range(1000, 1010);
            retry--;
        }
        if (!retry)
            return EBUSY;
    }
    return 0;
}

#endif
