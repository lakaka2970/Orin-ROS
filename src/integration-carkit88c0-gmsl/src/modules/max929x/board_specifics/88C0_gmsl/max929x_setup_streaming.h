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

#ifndef MAX929X_SETUP_STREAMING_H
#define MAX929X_SETUP_STREAMING_H

#include "max929x_config.h"
#include "max929x_reg_ops.h"
#include "max929x_types.h"

static int max929x_setup_streaming(u32 deser_index)
{
    struct max929x *priv = extenal_priv[deser_index];
    u8 err;
    u8 reg_value;
    u8 link_a_locked;
    u8 link_b_locked;
    struct device dev = priv->i2c_client->dev;

    // Check LINK A LOCK
    // Select LINK A
    max929x_write_reg(priv, 0x90 >> 1, 0x10, 0x21);
    msleep(0x78);  // 120 msec delay
    max929x_read_reg(priv, 0x90 >> 1, 0x13, &reg_value);
    link_a_locked = (reg_value >> 3) & 0x1;

    // Check LINK B LOCK
    // Select LINK B
    max929x_write_reg(priv, 0x90 >> 1, 0x10, 0x22);
    msleep(0x78);  // 120 msec delay
    max929x_read_reg(priv, 0x90 >> 1, 0x13, &reg_value);
    link_b_locked = (reg_value >> 3) & 0x1;

    if (link_a_locked && link_b_locked)
    {
        dev_info(&dev, "%s: Splitter Mode detected\n", __func__);

        // Reset LINK B Serializer if already on remapped address
        dev_dbg(&dev, "%s: Check if LINK B serializer is on remapped address 0x62\n", __func__);
        max929x_write_reg(priv, 0x90 >> 1, 0x10, 0x22);
        msleep(0x78);  // 120 msec delay

        // Try to read from 0xC4 address to check if serializer is there
        err = max929x_read_reg(priv, 0xC4 >> 1, 0x00, &reg_value);
        if (!err)
        {
            dev_dbg(&dev, "%s: LINK B serializer found on 0x62, resetting\n", __func__);
            max929x_write_reg(priv, 0xC4 >> 1, 0x10, 0x80);
            msleep(50);
        }

        err = max929x_write_reg_list(priv, max9296_SPLITTER_MODE_Dser_Ser_init,
                                     sizeof(max9296_SPLITTER_MODE_Dser_Ser_init) /
                                         sizeof(struct max929x_reg));
    }
    else if (link_a_locked || link_b_locked)
    {
        dev_info(&dev, "%s: Single-Link (Link %c) detected\n", __func__, link_a_locked ? 'A' : 'B');
        err = max929x_write_reg_list(priv, max9296_SINGLE_LINK_Dser_Ser_init,
                                     sizeof(max9296_SINGLE_LINK_Dser_Ser_init) /
                                         sizeof(struct max929x_reg));
    }
    else
    {
        dev_err(&dev, "%s: No LINK LOCK set\n", __func__);
        return -EINVAL;
    }

    if (err)
    {
        dev_err(&dev, "%s: max929x_write_reg_list failed\n", __func__);
        return err;
    }

    priv->streaming_en = true;

#ifdef MAX929X_FRAME_SYNC
    err = max929x_write_reg_list(priv, max9296_enable_trigger,
                                 sizeof(max9296_enable_trigger) / sizeof(struct max929x_reg));
#endif
    return 0;
}

#endif