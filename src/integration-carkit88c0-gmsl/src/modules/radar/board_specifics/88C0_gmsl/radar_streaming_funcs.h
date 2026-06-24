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

#ifndef RADAR_STREAMING_FUNCS_H
#define RADAR_STREAMING_FUNCS_H

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

static int radar_start_streaming(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev                = s_data->dev;

    dev_info(dev, "radar_start_streaming was called!\n");
    return 0;
}

static int radar_stop_streaming(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev                = s_data->dev;

    dev_info(dev, "radar_stop_streaming was called!\n");
    return 0;
}

#endif