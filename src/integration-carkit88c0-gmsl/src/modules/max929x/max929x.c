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

#include "max929x_types.h"
#include "max929x_setup_streaming.h"

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#define MAIN_GPIO_BASE      (348)
#define GPIO_PORT_P_OFFSET  (92)
#define GPIO_PORT_AC_OFFSET (138)
#define MMIC_A_RESETN       ((GPIO_PORT_P_OFFSET + 1) + MAIN_GPIO_BASE)
#define MMIC_A_DMUX1        ((GPIO_PORT_AC_OFFSET + 1) + MAIN_GPIO_BASE)
#define MMIC_B_RESETN       ((GPIO_PORT_AC_OFFSET + 3) + MAIN_GPIO_BASE)

struct max929x *extenal_priv[4];

static struct regmap_config max929x_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
};

static int max929x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    struct device dev = client->dev;
    struct device_node *np;
    struct max929x *priv;
    int ret = 0;

    dev_info(&dev, "%s: enter\n", __func__);

    priv             = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
    priv->i2c_client = client;
    priv->regmap     = devm_regmap_init_i2c(priv->i2c_client, &max929x_regmap_config);
    if (IS_ERR(priv->regmap))
    {
        dev_err(&client->dev,
                "regmap init failed: %ld\n", PTR_ERR(priv->regmap));
        return -ENODEV;
    }

    np = dev.of_node;
    if (!np)
        return -EINVAL;

    // Reset the deserializer
    priv->pwdn_gpio = devm_gpiod_get(&client->dev, "pwdn", GPIOD_OUT_HIGH);
    if (!IS_ERR(priv->pwdn_gpio))
    {
        gpiod_set_value(priv->pwdn_gpio, false);
        msleep(200);
        gpiod_set_value(priv->pwdn_gpio, true);
        msleep(200);
    }
    else
    {
        dev_err(&client->dev, "pwdn-gpios not in DT\n");
    }

    msleep(200);

    priv->deser_index = 0;
    device_property_read_u32(&client->dev, "deser-index", &priv->deser_index);
    priv->streaming_en              = false;
    extenal_priv[priv->deser_index] = priv;

    max929x_setup_streaming(priv->deser_index);

    ret = gpio_request(MMIC_A_RESETN, "MMIC_A_RESETN");
    if (ret)
    {
        pr_err("Failed to request GPIO %d\n", MMIC_A_RESETN);
        return ret;
    }

    ret = gpio_request(MMIC_A_DMUX1, "MMIC_A_DMUX1");
    if (ret)
    {
        pr_err("Failed to request GPIO %d\n", MMIC_A_DMUX1);
        return ret;
    }

    ret = gpio_request(MMIC_B_RESETN, "MMIC_B_RESETN");
    if (ret)
    {
        pr_err("Failed to request GPIO %d\n", MMIC_B_RESETN);
        return ret;
    }

    gpio_direction_output(MMIC_A_RESETN, 1);
    gpio_direction_output(MMIC_A_DMUX1, 1);
    gpio_direction_output(MMIC_B_RESETN, 1);
    msleep(100);
    gpio_set_value(MMIC_A_RESETN, 0);
    gpio_set_value(MMIC_A_DMUX1, 0);
    gpio_set_value(MMIC_B_RESETN, 0);
    gpio_free(MMIC_A_RESETN);
    gpio_free(MMIC_A_DMUX1);
    gpio_free(MMIC_B_RESETN);

    dev_info(&dev, "%s: success\n", __func__);

    return 0;
}

static int max929x_remove(struct i2c_client *client)
{
    struct device dev = client->dev;

    dev_info(&dev, "%s: \n", __func__);

    return 0;
}

static const struct i2c_device_id max929x_id[] = {
    {"max929x", 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, max929x_id);

const struct of_device_id max929x_of_match[] = {
    {
        .compatible = "nvidia,max929x",
    },
    {},
};
MODULE_DEVICE_TABLE(of, max929x_of_match);

static struct i2c_driver max929x_i2c_driver = {
    .driver = {
        .owner          = THIS_MODULE,
        .name           = "max929x",
        .of_match_table = of_match_ptr(max929x_of_match),
    },
    .probe    = max929x_probe,
    .remove   = max929x_remove,
    .id_table = max929x_id,
};

module_i2c_driver(max929x_i2c_driver);

MODULE_DESCRIPTION("IO Expander driver max929x");
MODULE_AUTHOR("Infineon Technologies AG");
MODULE_LICENSE("GPL v2");
