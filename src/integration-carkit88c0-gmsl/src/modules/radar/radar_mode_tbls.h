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

#ifndef RADAR_MODE_TBLS_H
#define RADAR_MODE_TBLS_H

#include <media/camera_common.h>

enum
{
    RADAR_MODE_1024x256 = 0,
    RADAR_MODE_2048x256,
    RADAR_MODE_4096x256,
    RADAR_MODE_8192x256,
    RADAR_MODE_16384x256,
    RADAR_MODE_1024x512,
    RADAR_MODE_2048x512,
    RADAR_MODE_4096x512,
    RADAR_MODE_8192x512,
    RADAR_MODE_16384x512,
    RADAR_MODE_1024x640,
    RADAR_MODE_2048x640,
    RADAR_MODE_4096x640,
    RADAR_MODE_8192x640,
    RADAR_MODE_16384x640,
    RADAR_MODE_1024x1024,
    RADAR_MODE_2048x1024,
    RADAR_MODE_4096x1024,
    RADAR_MODE_8192x1024,
    RADAR_MODE_16384x1024,
};

static const int radar_30fps[] = {
    30,
};

static const struct camera_common_frmfmt radar_frmfmt[] = {
    {{1024, 256}, radar_30fps, 1, 0, RADAR_MODE_1024x256},
    {{2048, 256}, radar_30fps, 1, 0, RADAR_MODE_2048x256},
    {{4096, 256}, radar_30fps, 1, 0, RADAR_MODE_4096x256},
    {{8192, 256}, radar_30fps, 1, 0, RADAR_MODE_8192x256},
    {{16384, 256}, radar_30fps, 1, 0, RADAR_MODE_16384x256},
    {{1024, 512}, radar_30fps, 1, 0, RADAR_MODE_1024x512},
    {{2048, 512}, radar_30fps, 1, 0, RADAR_MODE_2048x512},
    {{4096, 512}, radar_30fps, 1, 0, RADAR_MODE_4096x512},
    {{8192, 512}, radar_30fps, 1, 0, RADAR_MODE_8192x512},
    {{16384, 512}, radar_30fps, 1, 0, RADAR_MODE_16384x512},
    {{1024, 640}, radar_30fps, 1, 0, RADAR_MODE_1024x640},
    {{2048, 640}, radar_30fps, 1, 0, RADAR_MODE_2048x640},
    {{4096, 640}, radar_30fps, 1, 0, RADAR_MODE_4096x640},
    {{8192, 640}, radar_30fps, 1, 0, RADAR_MODE_8192x640},
    {{16384, 640}, radar_30fps, 1, 0, RADAR_MODE_16384x640},
    {{1024, 1024}, radar_30fps, 1, 0, RADAR_MODE_1024x1024},
    {{2048, 1024}, radar_30fps, 1, 0, RADAR_MODE_2048x1024},
    {{4096, 1024}, radar_30fps, 1, 0, RADAR_MODE_4096x1024},
    {{8192, 1024}, radar_30fps, 1, 0, RADAR_MODE_8192x1024},
    {{16384, 1024}, radar_30fps, 1, 0, RADAR_MODE_16384x1024},
};

#endif
