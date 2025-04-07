/*
 *  PCA953x 4/8/16/24/40 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>
#include <linux/reset.h>
#include <linux/slab.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif

#define PCA953X_INPUT		0
#define PCA953X_OUTPUT		1
#define PCA953X_INVERT		2
#define PCA953X_DIRECTION	3

#define REG_ADDR_AI		0x80

#define PCA957X_IN		0
#define PCA957X_INVRT		1
#define PCA957X_BKEN		2
#define PCA957X_PUPD		3
#define PCA957X_CFG		4
#define PCA957X_OUT		5
#define PCA957X_MSK		6
#define PCA957X_INTS		7

#define PCA_GPIO_MASK		0x00FF
#define PCA_INT			0x0100
#define PCA953X_TYPE		0x1000
#define PCA957X_TYPE		0x2000

static const struct i2c_device_id pca953x_id[] = {
	{ "pca9505", 40 | PCA953X_TYPE | PCA_INT, },
	{ "pca9534", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9535", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9536", 4  | PCA953X_TYPE, },
	{ "pca9537", 4  | PCA953X_TYPE | PCA_INT, },
	{ "pca9538", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9539", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9554", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9555", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9556", 8  | PCA953X_TYPE, },
	{ "pca9557", 8  | PCA953X_TYPE, },
	{ "pca9574", 8  | PCA957X_TYPE | PCA_INT, },
	{ "pca9575", 16 | PCA957X_TYPE | PCA_INT, },
	{ "pca9698", 40 | PCA953X_TYPE, },

	{ "max7310", 8  | PCA953X_TYPE, },
	{ "max7312", 16 | PCA953X_TYPE | PCA_INT, },
	{ "max7313", 16 | PCA953X_TYPE | PCA_INT, },
	{ "max7315", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca6107", 8  | PCA953X_TYPE | PCA_INT, },
	{ "tca6408", 8  | PCA953X_TYPE | PCA_INT, },
	{ "tca6416", 16 | PCA953X_TYPE | PCA_INT, },
	{ "tca6424", 24 | PCA953X_TYPE | PCA_INT, },
	{ "xra1202", 8  | PCA953X_TYPE },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca953x_id);

#define MAX_BANK 5
#define BANK_SZ 8

#define NBANK(chip) (chip->gpio_chip.ngpio / BANK_SZ)

struct pca953x_chip {
        unsigned gpio_start;
        u8 reg_output[MAX_BANK];
        u8 reg_direction[MAX_BANK];
        struct mutex i2c_lock;

#ifdef CONFIG_GPIO_PCA953X_IRQ
        struct mutex irq_lock;
        u8 irq_mask[MAX_BANK];
        u8 irq_stat[MAX_BANK];
        u8 irq_trig_raise[MAX_BANK];
        u8 irq_trig_fall[MAX_BANK];
#endif

        struct i2c_client *client;
        struct gpio_chip gpio_chip;
        const char *const *names;
        int	chip_type;
};

struct pca953x_chip *my_chip;
static int Direction;
static int Output;
static int Invert_Output;

static inline struct pca953x_chip *to_pca(struct gpio_chip *gc)
{
        return container_of(gc, struct pca953x_chip, gpio_chip);
}

static int pca953x_write_single(struct pca953x_chip *chip, int reg, u32 val,
                int off)
{
        int ret = 0;
        int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);
        int offset = off / BANK_SZ;

        ret = i2c_smbus_write_byte_data(chip->client,
                        (reg << bank_shift) + offset, val);

        if (ret < 0) {
                dev_err(&chip->client->dev, "failed writing register\n");
                return ret;
        }

        return 0;
}


static void pca953x_gpio_set_value(struct gpio_chip *gc, unsigned int off, int val)
{
        struct pca953x_chip *chip = to_pca(gc);
        u8 reg_val;
        int ret, offset = 0;


        mutex_lock(&chip->i2c_lock);
        if (val)
                reg_val = chip->reg_output[off / BANK_SZ]
                        | (1u << (off % BANK_SZ));
        else
                reg_val = chip->reg_output[off / BANK_SZ]
                        & ~(1u << (off % BANK_SZ));

        switch (chip->chip_type) {
                case PCA953X_TYPE:
                        offset = PCA953X_OUTPUT;
                        break;
                case PCA957X_TYPE:
                        offset = PCA957X_OUT;
                        break;
        }
        ret = pca953x_write_single(chip, offset, reg_val, off);
        if (ret)
                goto exit;

        chip->reg_output[off / BANK_SZ] = reg_val;
exit:
        mutex_unlock(&chip->i2c_lock);
}


static int pca953x_read_regs(struct pca953x_chip *chip, int reg, u8 *val)
{
        int ret;

        if (chip->gpio_chip.ngpio <= 8) {
                ret = i2c_smbus_read_byte_data(chip->client, reg);
                *val = ret;
        } else if (chip->gpio_chip.ngpio >= 24) {
                int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);

                ret = i2c_smbus_read_i2c_block_data(chip->client,
                                (reg << bank_shift) | REG_ADDR_AI,
                                NBANK(chip), val);
        } else {
                ret = i2c_smbus_read_word_data(chip->client, reg << 1);
                val[0] = (u16)ret & 0xFF;
                val[1] = (u16)ret >> 8;
        }
        if (ret < 0) {
                dev_err(&chip->client->dev, "failed reading register\n");
                return ret;
        }

        return 0;
}

static int pca953x_write_regs(struct pca953x_chip *chip, int reg, u8 *val)
{
        int ret = 0;

        if (chip->gpio_chip.ngpio <= 8)
                ret = i2c_smbus_write_byte_data(chip->client, reg, *val);
        else if (chip->gpio_chip.ngpio >= 24) {
                int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);
                ret = i2c_smbus_write_i2c_block_data(chip->client,
                                (reg << bank_shift) | REG_ADDR_AI,
                                NBANK(chip), val);
        } else {
                switch (chip->chip_type) {
                        case PCA953X_TYPE:
                                ret = i2c_smbus_write_word_data(chip->client,
                                                reg << 1, (u16) *val);
                                break;
                        case PCA957X_TYPE:
                                ret = i2c_smbus_write_byte_data(chip->client, reg << 1,
                                                val[0]);
                                if (ret < 0)
                                        break;
                                ret = i2c_smbus_write_byte_data(chip->client,
                                                (reg << 1) + 1,
                                                val[1]);
                                break;
                }
        }

        if (ret < 0) {
                dev_err(&chip->client->dev, "failed writing register\n");
                return ret;
        }

        return 0;
}

static ssize_t Direction_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
        return sprintf(buf, "%d\n", Direction);
}

/*
 *This function will take the direction value and store it to the Direction register of IOMUX.
 *This Direction Setting will be applied to all Ports(0-4) of the IOMUX.
 */
static ssize_t Direction_store(struct kobject *kobj, struct kobj_attribute *attr,char *buf, size_t count)
{
        u8 val[MAX_BANK];
        u8 regis_output[MAX_BANK] = {0};
        int ret = -1;
        sscanf(buf, "%du", &Direction);
        memset(val, Direction, NBANK(my_chip));


        ret = pca953x_write_regs(my_chip, PCA953X_DIRECTION, val);

        ret = pca953x_read_regs(my_chip, PCA953X_DIRECTION, regis_output);
        //printk("Direction register readings:%d,%d,%d,%d,%d\n",regis_output[0],regis_output[1],regis_output[2],regis_output[3],regis_output[4]);
        return count;
}


static ssize_t Output_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
        return sprintf(buf, "%d\n", Output);
}

/*
 *This function will set(1) the Output pin given in the argument.
 */
static ssize_t Output_store(struct kobject *kobj, struct kobj_attribute *attr,char *buf, size_t count)
{
        u8 val[MAX_BANK];
        u8 regis_output[MAX_BANK] = {0};
        int ret = -1;
        sscanf(buf, "%du", &Output);
        memset(val, Output, NBANK(my_chip));

        pca953x_gpio_set_value(&my_chip->gpio_chip, Output,1);

	ret = pca953x_read_regs(my_chip, PCA953X_OUTPUT, regis_output);
        //printk("Output Config register readings:%d,%d,%d,%d,%d\n",regis_output[0],regis_output[1],regis_output[2],regis_output[3],regis_output[4]);
        return count;
}

static ssize_t Invert_Output_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
        return sprintf(buf, "%d\n", Invert_Output);
}

/*This function will reset(0) the Output pin given in the argument.
 *
 */
static ssize_t Invert_Output_store(struct kobject *kobj, struct kobj_attribute *attr,char *buf, size_t count)
{
        u8 val[MAX_BANK];
        u8 regis_output[MAX_BANK] = {0};
        int ret = -1;
        sscanf(buf, "%du", &Invert_Output);
        memset(val, Invert_Output, NBANK(my_chip));

        pca953x_gpio_set_value(&my_chip->gpio_chip, Invert_Output,0);

        ret = pca953x_read_regs(my_chip, PCA953X_OUTPUT, regis_output);
        //printk("Output Config register readings:%d,%d,%d,%d,%d\n",regis_output[0],regis_output[1],regis_output[2],regis_output[3],regis_output[4]);
        return count;
}

static struct kobj_attribute Direction_attribute =__ATTR(Direction, 0660, Direction_show,Direction_store);
static struct kobj_attribute Output_attribute =__ATTR(Output, 0660, Output_show,Output_store);
static struct kobj_attribute Invert_Output_attribute =__ATTR(Invert_Output, 0660, Invert_Output_show, Invert_Output_store);