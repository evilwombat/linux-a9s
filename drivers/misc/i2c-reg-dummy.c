/*
 * Copyright (c) 2016  evilwombat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/of.h>

static const struct i2c_device_id i2c_reg_dummy_id[] = {
	{ "i2c-reg-dummy" },
	{ }
};

MODULE_DEVICE_TABLE(i2c, i2c_reg_dummy_id);

static const struct of_device_id of_i2c_reg_dummy_match[] = {
	{ .compatible = "linux,i2c-reg-dummy", },
	{},
};

static int reg_dummy_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct i2c_adapter *adapter;
	const unsigned char *data;
	int len = 0, i;

	if (!np)
		return -ENOSYS;

	adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	data = of_get_property(np, "linux,reg-data", &len);

	if (!data || len <= 0)
		return -EINVAL;

	for (i = 0; i < len; i+= 2)
		i2c_smbus_write_byte_data(client, data[i], data[i+1]);

	return 0;
}

static int reg_dummy_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver reg_dummy_driver = {
	.driver   = {
		.name    = "i2c-reg-dummy",
		.owner   = THIS_MODULE,
		.of_match_table = of_match_ptr(of_i2c_reg_dummy_match),
	},
	.probe    = reg_dummy_probe,
	.remove   = reg_dummy_remove,
	.id_table = i2c_reg_dummy_id,
};

module_i2c_driver(reg_dummy_driver);

MODULE_AUTHOR("evilwombat");
MODULE_DESCRIPTION("I2C dummy register driver");
MODULE_LICENSE("GPL v2");
