/*******************************************************************************
*
* File	    : lis3dh.c
* Author	: Lorenzo Chianura
* Version	: 0.1
* Date		: 16/May/2019
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
*******************************************************************************/

#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "lis3dh.h"

#define	LIS3DH_GACC_MAX 		16000 
#define LIS3DH_SENS_2G 			1	/*mg/LSB*/
#define LIS3DH_SENS_4G 			2	/*mg/LSB*/
#define LIS3DH_SENS_8G 			4	/*mg/LSB*/
#define LIS3DH_SENS_16G 		12	/*mg/LSB*/
#define	HIGH_RESOLUTION 		0x08
#define	LIS3DH_AXISDATA_REG 	0x28
#define WHOAMI_LIS3DH_ACC 		0x33

#define LIS3DH_POWON			0x47
#define WHO_AM_I 				0x0F

#define	LIS3DH_TEMPERATURE_CONFIG_REG 0x1F
#define	LIS3DH_CONTROL_REG1 0x20
#define	LIS3DH_CONTROL_REG2 0x21
#define	LIS3DH_CONTROL_REG3 0x22
#define	LIS3DH_CONTROL_REG4 0x23
#define	LIS3DH_CONTROL_REG5 0x24
#define	LIS3DH_CONTROL_REG6 0x25
#define	LIS3DH_FIFO_CONTROL_REG 0x2E
#define	LIS3DH_INT_CFG1 0x30
#define	LIS3DH_INT_SRC1 0x31
#define	LIS3DH_INT_THS1 0x32
#define	LIS3DH_INT_DUR1 0x33
#define	LIS3DH_INT_CFG2 0x34
#define	LIS3DH_INT_SRC2 0x35
#define	LIS3DH_INT_THS2 0x36
#define	LIS3DH_INT_DUR2 0x37

#define HI_RES_EN	1
#define LIS3DH_ACC_PM_OFF 0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES 0x07
#define LIS3DH_PMMODE_MASK 0x08

struct lxaccell_lis3dh_data {

	struct i2c_client *client;
	struct lxaccell_platform_data *pdata;
	struct mutex lock;

	int hw_initialized;
	int hw_working;

	atomic_t enabled;
	u8 sensitivity;
	u8 register_area[SAVE];
	int irq;
	struct fasync_struct *async_queue;
};

struct lxaccell_lis3dh_data *lis3dh_acc_misc_data;
struct i2c_client *lis3dh_i2c_client;

static int lis3dh_acc_i2c_read(struct lxaccell_lis3dh_data *lxaccell_data, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = lxaccell_data->client->addr,
			.flags = lxaccell_data->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = lxaccell_data->client->addr,
			.flags = (lxaccell_data->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(lxaccell_data->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&lxaccell_data->client->dev, "read transfer error\n");
		err = -EIO;
	} else
		err = 0;

	return err;
}

static int lis3dh_acc_i2c_write(struct lxaccell_lis3dh_data *lxaccell_data, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = lxaccell_data->client->addr,
			.flags = lxaccell_data->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(lxaccell_data->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&lxaccell_data->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_hw_init(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err = -1;
	u8 buf[7];

	printk("%s: device initialization started\n", LIS3DH_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	
	err = lis3dh_acc_i2c_read(lxaccell_data, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		lxaccell_data->hw_working = 1;

	buf[0] = LIS3DH_CONTROL_REG1;
	buf[1] = lxaccell_data->register_area[RES_CONTROL_REG1];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto hw_init_error;

	buf[0] = LIS3DH_TEMPERATURE_CONFIG_REG;
	buf[1] = lxaccell_data->register_area[RES_TEMPERATURE_CONFIG_REG];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto hw_init_error;
	
	buf[0] = LIS3DH_FIFO_CONTROL_REG;
	buf[1] = lxaccell_data->register_area[RES_FIFO_CONTROL_REG];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto hw_init_error;

	buf[0] = (I2C_AUTO_INCREMENT | LIS3DH_INT_THS1);
	buf[1] = lxaccell_data->register_area[RES_INT_THS1];
	buf[2] = lxaccell_data->register_area[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 2);
	if (err < 0)
		goto hw_init_error;

	buf[0] = LIS3DH_INT_CFG1;
	buf[1] = lxaccell_data->register_area[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto hw_init_error;

	buf[0] = (I2C_AUTO_INCREMENT | LIS3DH_INT_THS2);
	buf[1] = lxaccell_data->register_area[RES_INT_THS2];
	buf[2] = lxaccell_data->register_area[RES_INT_DUR2];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 2);
	if (err < 0)
		goto hw_init_error;

	buf[0] = LIS3DH_INT_CFG2;
	buf[1] = lxaccell_data->register_area[RES_INT_CFG2];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto hw_init_error;

	buf[0] = (I2C_AUTO_INCREMENT | LIS3DH_CONTROL_REG2);
	buf[1] = lxaccell_data->register_area[RES_CONTROL_REG2];
	buf[2] = lxaccell_data->register_area[RES_CONTROL_REG3];
	buf[3] = lxaccell_data->register_area[RES_CONTROL_REG4];
	buf[4] = lxaccell_data->register_area[RES_CONTROL_REG5];
	buf[5] = lxaccell_data->register_area[RES_CONTROL_REG6];

	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 5);
	if (err < 0)
		goto hw_init_error;

	lxaccell_data->hw_initialized = 1;

	printk("%s: device initialization done\n", LIS3DH_ACC_DEV_NAME);
	return 0;

error_firstread:
	lxaccell_data->hw_working = 0;
	dev_warn(&lxaccell_data->client->dev, "hw reading error\n");

hw_init_error:
	lxaccell_data->hw_initialized = 0;
	dev_err(&lxaccell_data->client->dev, "device initialization error 0x%x,0x%x: %d\n", buf[0], buf[1], err);

	return err;
}

static void lis3dh_acc_device_power_off(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err;
	u8 buf[2] = { LIS3DH_CONTROL_REG1, LIS3DH_ACC_PM_OFF };
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);

	if (err < 0)
		dev_err(&lxaccell_data->client->dev, "power off failed: %d\n", err);

	if (lxaccell_data->hw_initialized) {
		if (lxaccell_data->irq != 0)
			disable_irq_nosync(lxaccell_data->irq);

		lxaccell_data->hw_initialized = 0;
	}
}

static int lis3dh_acc_device_power_on(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err = -1;

	if (!lxaccell_data->hw_initialized) {
		err = lis3dh_acc_hw_init(lxaccell_data);

		if (lxaccell_data->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(lxaccell_data);
			return err;
		}
	}

	return 0;
}

int lis3dh_acc_update_g_range(struct lxaccell_lis3dh_data *lxaccell_data, u8 new_g_range)
{
	int err;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
	case LIS3DH_ACC_G_2G:
		sensitivity = LIS3DH_SENS_2G;
		break;
	case LIS3DH_ACC_G_4G:
		sensitivity = LIS3DH_SENS_4G;
		break;
	case LIS3DH_ACC_G_8G:
		sensitivity = LIS3DH_SENS_8G;
		break;
	case LIS3DH_ACC_G_16G:
		sensitivity = LIS3DH_SENS_16G;
		break;
	default:
		dev_err(&lxaccell_data->client->dev, "invalid g range requested: %u\n",
			new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&lxaccell_data->enabled)) {
		// Set configuration register 4, which contains g range setting
		buf[0] = LIS3DH_CONTROL_REG4;
		err = lis3dh_acc_i2c_read(lxaccell_data, buf, 1);
		if (err < 0)
			goto error;

		init_val = buf[0];
		lxaccell_data->register_area[RES_CONTROL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = LIS3DH_CONTROL_REG4;

		err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
		if (err < 0)
			goto error;

		lxaccell_data->register_area[RES_CONTROL_REG4] = updated_val;
		lxaccell_data->sensitivity = sensitivity;
	}

	return 0;

error:
	dev_err(&lxaccell_data->client->dev, "update g range failed 0x%x,0x%x: %d\n", buf[0], buf[1], err);

	return err;
}

int lis3dh_acc_update_odr(struct lxaccell_lis3dh_data *lxaccell_data, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];
	
	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  LIS3DH_OUTPUT_RATE cannot support the current poll interval, we stop searching */
	for (i = ARRAY_SIZE(lxaccell_data_rate) - 1; i >= 0; i--) {
		if (lxaccell_data_rate[i].lxaccell_upper_ms <= poll_interval_ms)
			break;
	}

	config[1] = lxaccell_data_rate[i].mask;
	config[1] |= LIS3DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&lxaccell_data->enabled)) {
		config[0] = LIS3DH_CONTROL_REG1;
		err = lis3dh_acc_i2c_write(lxaccell_data, config, 1);
		if (err < 0)
			goto error;
		lxaccell_data->register_area[RES_CONTROL_REG1] = config[1];
	}

	return 0;

error:
	dev_err(&lxaccell_data->client->dev, "update odr failed 0x%x,0x%x: %d\n",
		config[0], config[1], err);

	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lxaccell_lis3dh_data *lxaccell_data, int *xyz)
{
	int err = -1;

	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];

	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
	acc_data[0] = (I2C_AUTO_INCREMENT | LIS3DH_AXISDATA_REG);


	err = lis3dh_acc_i2c_read(lxaccell_data, acc_data, 6);
	if (err < 0) {
		printk("%s I2C read error %d\n", LIS3DH_ACC_I2C_NAME, err);
		return err;
	}

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);
	hw_d[0] = hw_d[0] * lxaccell_data->sensitivity;
	hw_d[1] = hw_d[1] * lxaccell_data->sensitivity;
	hw_d[2] = hw_d[2] * lxaccell_data->sensitivity;
	xyz[0] = ((lxaccell_data->pdata->negate_x) ? (-hw_d[lxaccell_data->pdata->axis_map_x])
		  : (hw_d[lxaccell_data->pdata->axis_map_x]));
	xyz[1] = ((lxaccell_data->pdata->negate_y) ? (-hw_d[lxaccell_data->pdata->axis_map_y])
		  : (hw_d[lxaccell_data->pdata->axis_map_y]));
	xyz[2] = ((lxaccell_data->pdata->negate_z) ? (-hw_d[lxaccell_data->pdata->axis_map_z])
		  : (hw_d[lxaccell_data->pdata->axis_map_z]));

	return err;
}

static int lis3dh_acc_enable(struct lxaccell_lis3dh_data *lxaccell_data)
{
    int err;

    if (!atomic_cmpxchg(&lxaccell_data->enabled, 0, 1)) {
        err = lis3dh_acc_device_power_on(lxaccell_data);
        if (err < 0) {
            atomic_set(&lxaccell_data->enabled, 0);

            return err;
        }
        lis3dh_acc_update_odr(lxaccell_data, lxaccell_data->pdata->poll_interval);
        enable_irq(lxaccell_data->irq);
    }

    return 0;
}

static int lis3dh_acc_disable(struct lxaccell_lis3dh_data *lxaccell_data)
{
    if (atomic_cmpxchg(&lxaccell_data->enabled, 1, 0)) {
        lis3dh_acc_device_power_off(lxaccell_data);
        disable_irq_nosync(lxaccell_data->irq);
    }

    return 0;
}

static int lis3dh_open_misc_dev(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		printk("%s: error open %d\n", LIS3DH_ACC_DEV_NAME, err);
		return err;
	}

	lis3dh_acc_enable(lis3dh_acc_misc_data);
	file->private_data = lis3dh_acc_misc_data;
	
	return 0;
}

static ssize_t lis3dh_read_misc_dev(struct file *filp, char __user *buf, size_t len, loff_t *off)
{   
	int xyz[3] = { 0 };

	struct lxaccell_lis3dh_data *lxaccell_data = filp->private_data;
    
    if (lxaccell_data && lxaccell_data->client) {

		if (lis3dh_acc_get_acceleration_data(lxaccell_data, xyz) >= 0)
        	len = snprintf(buf, len, "%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
		return len;

    } else {
		printk("Unexpected error %p:%p:%p\n",
			lxaccell_data, lxaccell_data->client, lxaccell_data->client->adapter);
		return -EIO;
    }
}

static int lis3dh_misc_dev_fasync(int fd, struct file *file, int mode)
{
    struct lxaccell_lis3dh_data *lxaccell_data = file->private_data;
    return fasync_helper(fd, file, mode, &lxaccell_data->async_queue);
}

static int lis3dh_release_misc_dev(struct inode *inode, struct file *file)
{
    lis3dh_misc_dev_fasync(-1, file, 0);
    return 0;
}

static const struct file_operations lis3dh_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = lis3dh_open_misc_dev,
	.read = lis3dh_read_misc_dev,
	.release = lis3dh_release_misc_dev,
	.fasync = lis3dh_misc_dev_fasync
};

static struct miscdevice lis3dh_acc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = LIS3DH_ACC_DEV_NAME,
	.fops = &lis3dh_acc_misc_fops,
};

void init_regs(struct lxaccell_lis3dh_data *lxaccell_data)
{
	memset(lxaccell_data->register_area, 0, ARRAY_SIZE(lxaccell_data->register_area));
	lxaccell_data->register_area[RES_CONTROL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	lxaccell_data->register_area[RES_CONTROL_REG2] = 0x00;
	lxaccell_data->register_area[RES_CONTROL_REG3] = CONTROL_REG3_I1_AOI1;
	lxaccell_data->register_area[RES_CONTROL_REG4] = 0x00;
	lxaccell_data->register_area[RES_CONTROL_REG5] = 0x00;
	lxaccell_data->register_area[RES_CONTROL_REG6] = 0x00;
	lxaccell_data->register_area[RES_TEMPERATURE_CONFIG_REG] = 0x00;
	lxaccell_data->register_area[RES_FIFO_CONTROL_REG] = 0x00;
	lxaccell_data->register_area[RES_INT_CFG1] = 0x3F;
	lxaccell_data->register_area[RES_INT_THS1] = 0x04;//defualt 100
	lxaccell_data->register_area[RES_INT_DUR1] = 0x30;
	lxaccell_data->register_area[RES_INT_CFG2] = 0x00;
	lxaccell_data->register_area[RES_INT_THS2] = 0x00;
	lxaccell_data->register_area[RES_INT_DUR2] = 0x00;
	lxaccell_data->register_area[RES_TT_CFG] = 0x00;
	lxaccell_data->register_area[RES_TT_THS] = 0x00;
	lxaccell_data->register_area[RES_TT_LIM] = 0x00;
	lxaccell_data->register_area[RES_TT_TLAT] = 0x00;
	lxaccell_data->register_area[RES_TT_TW] = 0x00;
	lxaccell_data->pdata->g_range = 0;
}

int lis3dh_acc_power_on(struct i2c_client *client)
{
	struct lxaccell_lis3dh_data *lxaccell_data = i2c_get_clientdata(client);
    int status;
    u8 enable_power[] = {
        LIS3DH_CONTROL_REG1,
        LIS3DH_POWON
    };

    status = lis3dh_acc_i2c_write(lxaccell_data, enable_power, 2);
    return (status >= 0);
}

void init_overlay_node(struct i2c_client *client, struct lxaccell_lis3dh_data *lxaccell_data)
{ 
	struct device_node *np;
	
	np = client->dev.of_node;
	if (!np) { 
		printk("%s: missing device node, is the overlay loaded?\n", LIS3DH_ACC_DEV_NAME);
		return;
	}

	lxaccell_data->pdata->poll_interval = 200;
	lxaccell_data->pdata->min_interval = 5;
	lxaccell_data->pdata->g_range = 0;
	lxaccell_data->pdata->negate_x = 1;
	lxaccell_data->pdata->negate_y = 1;
	lxaccell_data->pdata->gpio_int1 = 0;
	lxaccell_data->pdata->gpio_int2 = 0;
	lxaccell_data->pdata->axis_map_x = 0;
	lxaccell_data->pdata->axis_map_y = 1;
	lxaccell_data->pdata->axis_map_z = 2;
}

static int lis3dh_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lxaccell_lis3dh_data *lxaccell_data;
	int err = -1;
	int tempvalue;

	printk("%s: probe start.\n", LIS3DH_ACC_DEV_NAME);

	lxaccell_data = kzalloc(sizeof(struct lxaccell_lis3dh_data), GFP_KERNEL);
	if (!lxaccell_data)
		return -ENOMEM;

	mutex_init(&lxaccell_data->lock);
	mutex_lock(&lxaccell_data->lock);

	lxaccell_data->client = client;
	lis3dh_i2c_client = client;
	i2c_set_clientdata(client, lxaccell_data);

	// configure GPIO_48
	gpio_request(GPIO_INTERRUPT1, "GPIO_INTERRUPT1");
	gpio_direction_input(GPIO_INTERRUPT1);
	lxaccell_data->irq = gpio_to_irq(GPIO_INTERRUPT1);
	
	lxaccell_data->pdata = kzalloc(sizeof(*lxaccell_data->pdata), GFP_KERNEL);
	if (!lxaccell_data->pdata) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (client->dev.platform_data)
		lxaccell_data->pdata = client->dev.platform_data;

	init_overlay_node(lis3dh_i2c_client, lxaccell_data);
	
	err = lis3dh_acc_device_power_on(lxaccell_data);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_free_pdata;
	}

	if (i2c_smbus_read_byte(client) < 0) {
		pr_err("i2c_smbus_read_byte error!!\n");
		goto err_mutexunlock;
	} else
		printk("%s: device detected.\n", LIS3DH_ACC_DEV_NAME);
	
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	if ((tempvalue & 0x00FF) == WHOAMI_LIS3DH_ACC)
		printk("%s: I2C driver registered\n", LIS3DH_ACC_DEV_NAME);
	else {
		lxaccell_data->client = NULL;
		pr_info("I2C driver not registered, device unknown 0x%x\n", tempvalue);
		goto err_power_off;
	}

	i2c_set_clientdata(client, lxaccell_data);
	
	atomic_set(&lxaccell_data->enabled, 1);
	
	// set g range
	err = lis3dh_acc_update_g_range(lxaccell_data, lxaccell_data->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	// set poll interval
	err = lis3dh_acc_update_odr(lxaccell_data, lxaccell_data->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}

	lis3dh_acc_misc_data = lxaccell_data;

	err = misc_register(&lis3dh_acc_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "misc %s register failed\n", LIS3DH_ACC_DEV_NAME);
		goto err_power_off;
	}
	
	mutex_unlock(&lxaccell_data->lock);

	printk("%s: probe success.\n", LIS3DH_ACC_DEV_NAME);

	return 0;

err_power_off:
	lis3dh_acc_device_power_off(lxaccell_data);

err_free_pdata:
	kfree(lxaccell_data->pdata);
	
err_mutexunlock:
    mutex_unlock(&lxaccell_data->lock);

	pr_err("%s: driver init failed\n", LIS3DH_ACC_DEV_NAME);

	return err;
}

static int lis3dh_acc_remove(struct i2c_client *client)
{
	struct lxaccell_lis3dh_data *lxaccell_data = i2c_get_clientdata(client);

	if (lxaccell_data != NULL) {
		lis3dh_acc_disable(lxaccell_data);
		misc_deregister(&lis3dh_acc_misc_device);
		lis3dh_acc_device_power_off(lxaccell_data);

		kfree(lxaccell_data->pdata);
		kfree(lxaccell_data);
	}
	printk("%s: removed.\n", LIS3DH_ACC_DEV_NAME);

	return 0;
}

static const struct i2c_device_id lis3dh_acc_id[] = { 
	{ LIS3DH_ACC_DEV_NAME, 0 }, 
	{}, 
};
MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);

static const struct of_device_id lis3dh_of_match[] = {
	{.compatible = DEVICE_INFO,},
	{},
};
MODULE_DEVICE_TABLE(of, lis3dh_of_match);

static struct i2c_driver lis3dh_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DH_ACC_I2C_NAME,
		.of_match_table = lis3dh_of_match,
	},
	.probe = lis3dh_acc_probe,
	.remove = lis3dh_acc_remove,
	.id_table = lis3dh_acc_id
};

module_i2c_driver(lis3dh_driver);

MODULE_DESCRIPTION("LXGROUP ST LIS3DH i2c driver");
MODULE_AUTHOR("Lorenzo Chianura");
MODULE_LICENSE("GPL");
