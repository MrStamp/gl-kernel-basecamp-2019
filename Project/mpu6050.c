#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>

#include "mpu6050-regs.h"

#define ACCURACY 1000

static unsigned int interrupt_pin = GPIO_INTERRUPT;
static unsigned int irqNumber;
static irq_handler_t irqHandler(unsigned int irq, void *dev_id, struct pt_regs *regs);

struct mpu6050_data {
	struct i2c_client *drv_client;
	int accel_values[3];
	int old_accel_values[3];
	int steps;
};

static struct mpu6050_data g_mpu6050_data;

static int mpu6050_read_data(void)
{
	int sum = 0;
	struct i2c_client *drv_client = g_mpu6050_data.drv_client;
	if (drv_client == 0)
		return -ENODEV;
	/* save old accel values to predict false movement*/
	g_mpu6050_data.old_accel_values[0] = g_mpu6050_data.accel_values[0];
	g_mpu6050_data.old_accel_values[1] = g_mpu6050_data.accel_values[1];
	g_mpu6050_data.old_accel_values[2] = g_mpu6050_data.accel_values[2];
	/* accel */
	g_mpu6050_data.accel_values[0] = (s16)((u16)i2c_smbus_read_word_swapped(drv_client, REG_ACCEL_XOUT_H));
	g_mpu6050_data.accel_values[1] = (s16)((u16)i2c_smbus_read_word_swapped(drv_client, REG_ACCEL_YOUT_H));
	g_mpu6050_data.accel_values[2] = (s16)((u16)i2c_smbus_read_word_swapped(drv_client, REG_ACCEL_ZOUT_H));
	sum = g_mpu6050_data.accel_values[0] + g_mpu6050_data.accel_values[1] + g_mpu6050_data.accel_values[2];

	if (sum <= 0) {
	if ((g_mpu6050_data.accel_values[0]-g_mpu6050_data.old_accel_values[0]) > ACCURACY || (g_mpu6050_data.accel_values[1]-g_mpu6050_data.old_accel_values[1]) > ACCURACY || (g_mpu6050_data.accel_values[2]-g_mpu6050_data.old_accel_values[2]) > ACCURACY)
	g_mpu6050_data.steps += 1;
	}

	dev_info(&drv_client->dev, "sensor data read:\n");
	dev_info(&drv_client->dev, "ACCEL[X,Y,Z] = [%d, %d, %d]\n",
		g_mpu6050_data.accel_values[0],
		g_mpu6050_data.accel_values[1],
		g_mpu6050_data.accel_values[2]);
	dev_info(&drv_client->dev, "STEPS = %d\n", g_mpu6050_data.steps/2);
	return 0;
}

static int mpu6050_probe(struct i2c_client *drv_client,
			 const struct i2c_device_id *id)
{
	int ret;

	dev_info(&drv_client->dev,
		"i2c client address is 0x%X\n", drv_client->addr);

	/* Read who_am_i register */
	ret = i2c_smbus_read_byte_data(drv_client, REG_WHO_AM_I);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&drv_client->dev,
			"i2c_smbus_read_byte_data() failed with error: %d\n",
			ret);
		return ret;
	}
	if (ret != MPU6050_WHO_AM_I) {
		dev_err(&drv_client->dev,
			"wrong i2c device found: expected 0x%X, found 0x%X\n",
			MPU6050_WHO_AM_I, ret);
		return -1;
	}
	dev_info(&drv_client->dev,
		"i2c mpu6050 device found, WHO_AM_I register value = 0x%X\n",
		ret);

	/* Setup the device */
	i2c_smbus_write_byte_data(drv_client, REG_CONFIG, 0);
	i2c_smbus_write_byte_data(drv_client, REG_ACCEL_CONFIG, 0);
	i2c_smbus_write_byte_data(drv_client, REG_FIFO_EN, 0);
	i2c_smbus_write_byte_data(drv_client, REG_INT_PIN_CFG, 0);
	i2c_smbus_write_byte_data(drv_client, REG_INT_ENABLE, 0);
	i2c_smbus_write_byte_data(drv_client, REG_USER_CTRL, 0);
	i2c_smbus_write_byte_data(drv_client, REG_PWR_MGMT_1, 0);
	i2c_smbus_write_byte_data(drv_client, REG_PWR_MGMT_2, 0);
	i2c_smbus_write_byte_data(drv_client, MOT_THR, 0x14);
	i2c_smbus_write_byte_data(drv_client, MOT_DUR, 0x01);
	i2c_smbus_write_byte_data(drv_client, MOT_DETECT_CTRL, 0x15);

	g_mpu6050_data.drv_client = drv_client;

	dev_info(&drv_client->dev, "i2c driver probed\n");
	return 0;
}

static int mpu6050_remove(struct i2c_client *drv_client)
{
	i2c_smbus_write_byte_data(drv_client, REG_INT_PIN_CFG, 0);
	i2c_smbus_write_byte_data(drv_client, REG_ACCEL_CONFIG, 0);
	i2c_smbus_write_byte_data(drv_client, MOT_THR, 0);
	i2c_smbus_write_byte_data(drv_client, MOT_DUR, 0);
	i2c_smbus_write_byte_data(drv_client, MOT_DETECT_CTRL, 0);
	i2c_smbus_write_byte_data(drv_client, REG_INT_ENABLE, 0);
	g_mpu6050_data.drv_client = 0;
	dev_info(&drv_client->dev, "i2c driver removed\n");
	return 0;
}

static const struct i2c_device_id mpu6050_idtable[] = {
	{ "mpu6050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_idtable);

static struct i2c_driver mpu6050_i2c_driver = {
	.driver = {
		.name = "gl_mpu6050",
	},

	.probe = mpu6050_probe,
	.remove = mpu6050_remove,
	.id_table = mpu6050_idtable,
};

static irq_handler_t irqHandler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	mpu6050_read_data();
	printk("Device moved! Interrupt is called\n");
	return (irq_handler_t)IRQ_HANDLED;
}

static ssize_t accel_x_show(struct class *class,
			    struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.accel_values[0]);
	return strlen(buf);
}

static ssize_t accel_y_show(struct class *class,
			    struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.accel_values[1]);
	return strlen(buf);
}

static ssize_t accel_z_show(struct class *class,
			    struct class_attribute *attr, char *buf)
{
	mpu6050_read_data();

	sprintf(buf, "%d\n", g_mpu6050_data.accel_values[2]);
	return strlen(buf);
}

static ssize_t steps_show(struct class *class,
			 struct class_attribute *attr, char *buf)
{

	sprintf(buf, "%d\n", g_mpu6050_data.steps/2);
	return strlen(buf);
}

static struct class_attribute class_attr_accel_x = __ATTR(accel_x, 0444, &accel_x_show, NULL);
static struct class_attribute class_attr_accel_y = __ATTR(accel_y, 0444, &accel_y_show, NULL);
static struct class_attribute class_attr_accel_z = __ATTR(accel_z, 0444, &accel_z_show, NULL);
static struct class_attribute class_attr_steps = __ATTR(steps, 0444, &steps_show, NULL);

static struct class *attr_class;

static int mpu6050_init(void)
{
	int ret;

	/* Create i2c driver */
	ret = i2c_add_driver(&mpu6050_i2c_driver);
	if (ret) {
		pr_err("mpu6050: failed to add new i2c driver: %d\n", ret);
		return ret;
	}
	pr_info("mpu6050: i2c driver created\n");

	gpio_request(interrupt_pin, "fancy label");
	gpio_direction_input(interrupt_pin);
	gpio_set_debounce(interrupt_pin, 50);
	gpio_export(interrupt_pin, false);

	irqNumber = gpio_to_irq(interrupt_pin);

	ret = request_irq(irqNumber, (irq_handler_t) irqHandler, IRQF_TRIGGER_RISING, "mpu6050Handler", NULL);
	if (ret) {
		printk(KERN_ERR "mpu6050: Failed to request irq\n");
		return ret;
	}
	/* Create class */
	attr_class = class_create(THIS_MODULE, "mpu6050");
	if (IS_ERR(attr_class)) {
		ret = PTR_ERR(attr_class);
		pr_err("mpu6050: failed to create sysfs class: %d\n", ret);
		return ret;
	}
	pr_info("mpu6050: sysfs class created\n");

	/* Create accel_x */
	ret = class_create_file(attr_class, &class_attr_accel_x);
	if (ret) {
		pr_err("mpu6050: failed to create sysfs class attribute accel_x: %d\n", ret);
		return ret;
	}

	/* Create accel_y */
	ret = class_create_file(attr_class, &class_attr_accel_y);
	if (ret) {
		pr_err("mpu6050: failed to create sysfs class attribute accel_y: %d\n", ret);
		return ret;
	}

	/* Create accel_z */
	ret = class_create_file(attr_class, &class_attr_accel_z);
	if (ret) {
		pr_err("mpu6050: failed to create sysfs class attribute accel_z: %d\n", ret);
		return ret;
	}
	/* Create steps */
	ret = class_create_file(attr_class, &class_attr_steps);
	if (ret) {
		pr_err("mpu6050: failed to create sysfs class attribute steps: %d\n", ret);
		return ret;
	}
	pr_info("mpu6050: sysfs class attributes created\n");
	pr_info("mpu6050: module loaded\n");
	return 0;
}

static void mpu6050_exit(void)
{
	if (attr_class) {
		class_remove_file(attr_class, &class_attr_accel_x);
		class_remove_file(attr_class, &class_attr_accel_y);
		class_remove_file(attr_class, &class_attr_accel_z);
		class_remove_file(attr_class, &class_attr_steps);
		pr_info("mpu6050: sysfs class attributes removed\n");
		class_destroy(attr_class);
		pr_info("mpu6050: sysfs class destroyed\n");
	}
	free_irq(irqNumber, NULL);
	gpio_free(interrupt_pin);
	i2c_del_driver(&mpu6050_i2c_driver);
	pr_info("mpu6050: i2c driver deleted\n");
	pr_info("mpu6050: module exited\n");
}

module_init(mpu6050_init);
module_exit(mpu6050_exit);

MODULE_AUTHOR("Rubinshteyn Mark <mark.rubinshteyn@meta.ua>");
MODULE_DESCRIPTION("mpu6050 step");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");
