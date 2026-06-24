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

#include "radar_mode_tbls.h"
#include "radar_streaming_funcs.h"

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
// Linux内核模块基础头文件
#include <linux/module.h>
// 设备树解析相关头文件
#include <linux/of.h>
#include <linux/of_device.h>

static const struct of_device_id radar_of_match[] = {
    {.compatible = "infineon,radar_ctrx"},
    {},
};
// 将匹配表注册到内核，用于设备树匹配
MODULE_DEVICE_TABLE(of, radar_of_match);

struct radar
{
    struct tegracam_device *tc_dev;
};
// 定义传感器寄存器映射配置，指定寄存器位宽和数值位宽
static const struct regmap_config sensor_regmap_config = {
    .reg_bits = 16,  // 寄存器地址位宽：16位
    .val_bits = 8,   // 寄存器数值位宽：8位
};

static int radar_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
    return 0;
}

static struct tegracam_ctrl_ops radar_ctrl_ops = {
    .numctrls       = 0,
    .set_group_hold = radar_set_group_hold,
};

static inline int radar_read_reg(struct camera_common_data *s_data, u16 addr, u8 *val)
{
    return 0;
}

static int radar_write_reg(struct camera_common_data *s_data, u16 addr, u8 val)
{
    return 0;
}

static int radar_power_on(struct camera_common_data *s_data)
{
    return 0;
}

static int radar_power_off(struct camera_common_data *s_data)
{
    return 0;
}

static int radar_power_get(struct tegracam_device *tc_dev)
{
    return 0;
}

static int radar_power_put(struct tegracam_device *tc_dev)
{
    return 0;
}

static struct camera_common_pdata *radar_parse_dt(struct tegracam_device *tc_dev)
{
    struct device *dev = tc_dev->dev;
    return devm_kzalloc(dev, sizeof(struct camera_common_pdata), GFP_KERNEL);
}
/**
 * @brief 设置雷达工作模式（分辨率、帧率等）
 * @param tc_dev Tegra相机设备句柄
 * @return 0表示成功，负数表示错误
 */
static int radar_set_mode(struct tegracam_device *tc_dev)
{
    struct camera_common_data *s_data = tc_dev->s_data;
    struct device *dev                = s_data->dev;

    const struct sensor_mode_properties *mode = NULL;
    int idx                                   = s_data->mode_prop_idx;

    dev_info(dev, "radar_set_mode ModeIdx = %d\n", s_data->mode_prop_idx);

    if (s_data->mode_prop_idx < 0)
    {
        dev_err(dev, "Invalid mode index = %d\n", s_data->mode_prop_idx);
        return -EINVAL;
    }

    // Debug Infos
    mode = &s_data->sensor_props.sensor_modes[idx];
    dev_info(dev, "radar mipi_clock = %llu\n", mode->signal_properties.mipi_clock.val);
    dev_info(dev, "radar pixel_clock = %llu\n", mode->signal_properties.pixel_clock.val);
    dev_info(dev, "radar width = %d\n", mode->image_properties.width);
    dev_info(dev, "radar height = %d\n", mode->image_properties.height);
    dev_info(dev, "radar line_length = %d\n", mode->image_properties.line_length);

    return 0;
}

// 定义相机公共传感器操作集，关联雷达所有核心操作函数
static struct camera_common_sensor_ops radar_common_ops = {
    // number of formats and format table
    .numfrmfmts   = ARRAY_SIZE(radar_frmfmt),
    .frmfmt_table = radar_frmfmt,

    // register read
    .read_reg  = radar_read_reg,
    .write_reg = radar_write_reg,

    // power control
    .power_on  = radar_power_on,
    .power_off = radar_power_off,
    .power_get = radar_power_get,
    .power_put = radar_power_put,

    // parse device tree
    .parse_dt = radar_parse_dt,

    // Sensor streaming
    .set_mode        = radar_set_mode,
    .start_streaming = radar_start_streaming,
    .stop_streaming  = radar_stop_streaming,
};

static int radar_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    return 0;
}

static const struct v4l2_subdev_internal_ops radar_subdev_internal_ops = {
    .open = radar_open,
};
/**
 * @brief I2C设备探测函数（设备匹配成功后执行）
 * @param client I2C设备客户端句柄
 * @param id I2C设备ID表项
 * @return 0表示成功，负数表示错误
 */
static int radar_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    struct device *dev       = &client->dev;
    struct device_node *node = client->dev.of_node;
    struct tegracam_device *tc_dev;
    struct radar *priv;
    int err;
    const struct of_device_id *match;

    match = of_match_device(radar_of_match, dev);
    if (!match)
    {
        dev_err(dev, "No device match found\n");
        return -ENODEV;
    }

    if (!IS_ENABLED(CONFIG_OF) || !node)
        return -EINVAL;
    // 分配Tegra相机设备结构体内存（内核态，自动释放）
    tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
    if (!tc_dev)
        return -ENOMEM;
    // 初始化Tegra相机设备结构体核心字段
    tc_dev->client = client;
    tc_dev->dev    = dev;
    strncpy(tc_dev->name, "radar", sizeof(tc_dev->name));
    tc_dev->dev_regmap_config   = &sensor_regmap_config;
    tc_dev->sensor_ops          = &radar_common_ops;
    tc_dev->v4l2sd_internal_ops = &radar_subdev_internal_ops;
    tc_dev->tcctrl_ops          = &radar_ctrl_ops;
    // 注册Tegra相机设备到内核
    err = tegracam_device_register(tc_dev);
    if (err)
    {
        dev_err(dev, "tegra camera driver registration failed\n");
        return err;
    }
    // 分配雷达私有数据结构体内存（内核态，自动释放）
	priv = devm_kzalloc(dev, sizeof(struct radar), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;
    priv->tc_dev = tc_dev;
    // 将私有数据设置到Tegra相机设备中，便于后续获取
    tegracam_set_privdata(tc_dev, (void *)priv);
    // 注册V4L2子设备（true表示注册为视频设备）
    err = tegracam_v4l2subdev_register(tc_dev, true);
    if (err)
    {
        dev_err(dev, "tegra camera subdev registration failed\n");
        return err;
    }

    dev_info(dev, "Detected Infineon Radar device\n");
    return 0;
}
/**
 * @brief I2C设备移除函数（设备卸载时执行）
 * @param client I2C设备客户端句柄
 * @return 0表示成功，负数表示错误
 */
static int radar_remove(struct i2c_client *client)
{
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct radar *priv                = (struct radar *)s_data->priv;
    // 注销V4L2子设备 Tegra相机设备
    tegracam_v4l2subdev_unregister(priv->tc_dev);
    tegracam_device_unregister(priv->tc_dev);

    return 0;
}

static const struct i2c_device_id radar_id[] = {
    {"radar", 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, radar_id);
// 定义I2C驱动结构体，关联核心回调函数
static struct i2c_driver radar_i2c_driver = {
    .driver = {
        .name           = "Radar",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(radar_of_match),
    },
    .probe    = radar_probe,
    .remove   = radar_remove,
    .id_table = radar_id,
};
// 注册I2C驱动到内核（宏封装，简化驱动注册流程）
module_i2c_driver(radar_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for CTRX Radar");
MODULE_AUTHOR("Infineon Technologies AG");
MODULE_LICENSE("GPL v2");
