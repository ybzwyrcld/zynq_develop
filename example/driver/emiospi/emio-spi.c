/*
 * emio-spi.c - emio_spi-GPIO driver
 *
 * Copyright (c) 2012 Digilent. All right reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 * Modefied by Yuming Meng. 2018-03-09
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <asm/uaccess.h>

#define DRIVER_NAME "emio-spi"
#define SPI_DRIVER_NAME "emio-gpio-spi"
#define MAX_DEV_NUM 16
#define TRANSFER_BUF_SZ 512      /* 32 x 128 bit monochrome == 512 bytes */
//#define CONFIG_DEBUG 1


static dev_t gpio_spi_dev_id;
static unsigned int device_num;
static unsigned int cur_minor;
static unsigned int spi_drv_registered;
/* struct mutex minor_mutex; */
static struct class *gpio_spi_class;

struct gpio_spi_device {
	const char *name;
	/* R/W Mutex Lock */
	struct mutex mutex;
	uint8_t *transfer_buf;
	/* Pin Assignment */
	unsigned long iSCLK;
	unsigned long iSDIN;
	unsigned long iSDOUT;
	unsigned long iCS;
	/* SPI Info */
	uint32_t spi_id;
	/* platform device structures */
	struct platform_device *pdev;
	/* Char Device */
	struct cdev cdev;
	struct spi_device *spi;
	dev_t dev_id;
};

static int gpio_spi_open(struct inode *inode, struct file *fp)
{
	struct gpio_spi_device *dev;

	dev = container_of(inode->i_cdev, struct gpio_spi_device, cdev);
	fp->private_data = dev;

	return 0;
}

static int gpio_spi_close(struct inode *inode, struct file *fp)
{
	return 0;
}

static ssize_t gpio_spi_write(struct file *fp, const char __user *buffer, size_t length, loff_t *offset)
{
	ssize_t retval = 0;
	struct gpio_spi_device *dev;
	unsigned int minor_id;
	int cnt;
	int status;

	dev = fp->private_data;
	minor_id = MINOR(dev->dev_id);

	if (mutex_lock_interruptible(&dev->mutex)) {
		retval = -ERESTARTSYS;
		goto write_lock_err;
	}

	if (buffer == NULL) {
		dev_err(&dev->spi->dev, "%s: invalid buffer address: 0x%08lx\n", __func__, (__force unsigned long)buffer);
		retval = -EINVAL;
		goto quit_write;
	}

	if (length > TRANSFER_BUF_SZ)
		cnt = TRANSFER_BUF_SZ;
	else
		cnt = length;

	if (copy_from_user(dev->transfer_buf, buffer, cnt)) {
		dev_err(&dev->spi->dev, "%s: copy_from_user failed\n", __func__);
		retval = -EFAULT;
		goto quit_write;
	} else
		retval = cnt;

	status = spi_write(dev->spi, dev->transfer_buf, cnt);
	if (status) {
		dev_err(&dev->spi->dev, "%s: write failed\n", __func__);
		retval = -EFAULT;
		goto quit_write;
	}

quit_write:
	mutex_unlock(&dev->mutex);
write_lock_err:
	return retval;
}

static ssize_t gpio_spi_read(struct file *fp, char __user *buffer, size_t length, loff_t *offset)
{
	ssize_t retval = 0;
	struct gpio_spi_device *dev;
	unsigned int minor_id;
	int cnt;
	int status = 0;

	dev = fp->private_data;
	minor_id = MINOR(dev->dev_id);

	if (mutex_lock_interruptible(&dev->mutex)) {
		retval = -ERESTARTSYS;
		goto read_lock_err;
	}

	if (buffer == NULL) {
		dev_err(&dev->spi->dev, "%s: invalid buffer address: 0x%08lx\n", __func__, (__force unsigned long)buffer);
		retval = -EINVAL;
		goto quit_read;
	}

	if (length > TRANSFER_BUF_SZ)
		cnt = TRANSFER_BUF_SZ;
	else
		cnt = length;

	status = spi_read(dev->spi, dev->transfer_buf, cnt);
	if (status) {
		dev_err(&dev->spi->dev, "%s: read failed!!!\n", __func__);
		retval = -EFAULT;
		goto quit_read;
	}

	retval = copy_to_user((void __user *)buffer, dev->transfer_buf, cnt);
	if (!retval)
		retval = cnt; /* copy success, return amount in buffer */

quit_read:
	mutex_unlock(&dev->mutex);
read_lock_err:
	return retval;
}

static struct file_operations gpio_spi_cdev_fops = {
	.owner		= THIS_MODULE,
	.write		= gpio_spi_write,
	.read		= gpio_spi_read,
	.open		= gpio_spi_open,
	.release	= gpio_spi_close,
};

static int add_gpio_spi_device_to_bus(struct gpio_spi_device *dev)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	int status = 0;

	spi_master = spi_busnum_to_master(dev->spi_id);
	if (!spi_master) {
		dev_err(&dev->pdev->dev, "%s: spi_busnum_to_master(%d) returned NULL\n", __func__, dev->spi_id);
		return -ENOSYS;
	}

	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		dev_err(&dev->pdev->dev, "%s: spi_alloc_device() failed\n", __func__);
		return -ENOMEM;
	}

	spi_device->chip_select = 0;
	spi_device->max_speed_hz = 5000000;
	spi_device->mode = SPI_MODE_0;
	spi_device->bits_per_word = 8;
	spi_device->controller_data = (void *)dev->iCS;
	spi_device->dev.platform_data = dev;
	strlcpy(spi_device->modalias, SPI_DRIVER_NAME, sizeof(SPI_DRIVER_NAME));

	status = spi_add_device(spi_device);
	if (status < 0) {
		spi_dev_put(spi_device);
		dev_err(&dev->pdev->dev, "%s: spi_add_device() failed %d\n", __func__ , status);
		return status;
	}
	dev->spi = spi_device;

	put_device(&spi_master->dev);

	return status;
}

/**
 * gpio_spi_setup_cdev - Setup Char Device for gpio_spi device.
 * @dev: pointer to device tree node
 * @dev_id: pointer to device major and minor number
 * @spi: pointer to spi_device structure
 *
 * This function initializes char device for gpio_spi device, and add it into
 * kernel device structure. It returns 0, if the cdev is successfully
 * initialized, or a negative value if there is an error.
 */
static int gpio_spi_setup_cdev(struct gpio_spi_device *dev, dev_t *dev_id, struct spi_device *spi)
{
	int status = 0;
	struct device *device;

	cdev_init(&dev->cdev, &gpio_spi_cdev_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &gpio_spi_cdev_fops;
	dev->spi = spi;

	*dev_id = MKDEV(MAJOR(gpio_spi_dev_id), cur_minor++);
	status = cdev_add(&dev->cdev, *dev_id, 1);
	if (status < 0)
		return status;

	/* Add Device node in system */
	device = device_create(gpio_spi_class, NULL,
			       *dev_id, NULL,
			       "%s", dev->name);
	if (IS_ERR(device)) {
		status = PTR_ERR(device);
		dev_err(&spi->dev, "failed to create device node %s, err %d\n", dev->name, status);
		cdev_del(&dev->cdev);
	}

	return status;
}

static int gpio_spi_probe(struct spi_device *spi)
{
	int status = 0;
	struct gpio_spi_device *gpio_spi_dev;

	/* We rely on full duplex transfers, mostly to reduce
	 * per transfer overheads (by making few transfers).
	 */
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		status = -EINVAL;
		dev_err(&spi->dev, "SPI settings incorrect: %d\n", status);
		goto spi_err;
	}

	/* We must use SPI_MODE_0 */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	status = spi_setup(spi);
	if (status < 0) {
		dev_err(&spi->dev, "needs SPI mode %02x, %d KHz; %d\n",
			spi->mode, spi->max_speed_hz / 1000,
			status);
		goto spi_err;
	}

	/* Get gpio_spi_device structure */
	gpio_spi_dev = (struct gpio_spi_device *)spi->dev.platform_data;
	if (gpio_spi_dev == NULL) {
		dev_err(&spi->dev, "Cannot get gpio_spi_device.\n");
		status = -EINVAL;
		goto spi_platform_data_err;
	}

	pr_info(SPI_DRIVER_NAME " [%s] SPI Probing\n", gpio_spi_dev->name);

#ifdef CONFIG_DEBUG
	pr_info(SPI_DRIVER_NAME " [%s] spi_probe: setup char device\n", gpio_spi_dev->name);
#endif

	/* Setup char driver */
	status = gpio_spi_setup_cdev(gpio_spi_dev, &(gpio_spi_dev->dev_id), spi);
	if (status) {
		dev_err(&spi->dev, "spi_probe: Error adding %s device: %d\n", SPI_DRIVER_NAME, status);
		goto cdev_add_err;
	}

	/* Initialize Mutex */
	mutex_init(&gpio_spi_dev->mutex);

#ifdef CONFIG_DEBUG
	pr_info(SPI_DRIVER_NAME " [%s] spi_probe: initialize device\n", gpio_spi_dev->name);
#endif

	memset(gpio_spi_dev->transfer_buf, 0x00, TRANSFER_BUF_SZ);

	return status;

cdev_add_err:
	if (&gpio_spi_dev->cdev)
		cdev_del(&gpio_spi_dev->cdev);
spi_platform_data_err:
spi_err:
	return status;
}

static int gpio_spi_remove(struct spi_device *spi)
{
	int status;
	struct gpio_spi_device *dev;

	dev = (struct gpio_spi_device *)spi->dev.platform_data;

	if (dev == NULL) {
		dev_err(&spi->dev, "spi_remove: Error fetch gpio_spi_device struct\n");
		return -EINVAL;
	}

#ifdef CONFIG_DEBUG
	pr_info(SPI_DRIVER_NAME " [%s] spi_remove: Clearing Display\n", dev->name);
#endif

	/* Clear Buffer */
	memset(dev->transfer_buf, 0, TRANSFER_BUF_SZ);

	if (&dev->cdev) {
#ifdef CONFIG_DEBUG
		pr_info(SPI_DRIVER_NAME " [%s] spi_remove: Destroy Char Device\n", dev->name);
#endif
		device_destroy(gpio_spi_class, dev->dev_id);
		cdev_del(&dev->cdev);
	}

	cur_minor--;

	pr_info(SPI_DRIVER_NAME " [%s] spi_remove: Device Removed\n", dev->name);

	return status;
}

static struct spi_driver gpio_spi_driver = {
	.driver		= {
		.name	= SPI_DRIVER_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= gpio_spi_probe,
	.remove		= gpio_spi_remove,
};

static const struct of_device_id emio_spi_of_match[] = {
	{ .compatible = "dglnt,emio-spi", },
	{},
};
MODULE_DEVICE_TABLE(of, emio_spi_of_match);

static int emio_spi_of_probe(struct platform_device *pdev)
{
	struct gpio_spi_device *gpio_spi_dev;
	struct platform_device *gpio_spi_pdev;
	struct spi_gpio_platform_data *gpio_spi_pdata;

	struct device_node *np = pdev->dev.of_node;

	const u32 *tree_info;
	int status = 0;

	/* Alloc Space for platform device structure */
	gpio_spi_dev = kzalloc(sizeof(*gpio_spi_dev), GFP_KERNEL);
	if (!gpio_spi_dev) {
		status = -ENOMEM;
		goto dev_alloc_err;
	}

	/* Alloc Transfer Buffer for device */
	gpio_spi_dev->transfer_buf = kmalloc(TRANSFER_BUF_SZ, GFP_KERNEL);
	if (!gpio_spi_dev->transfer_buf) {
		status = -ENOMEM;
		dev_err(&pdev->dev, "Device Transfer data buffer allocation failed: %d\n", status);
		goto transfer_buf_alloc_err;
	}

	/* Get the GPIO Pins */
	gpio_spi_dev->iSCLK = of_get_named_gpio(np, "spi-sclk-gpio", 0);
	gpio_spi_dev->iSDIN = of_get_named_gpio(np, "spi-sdin-gpio", 0);
	gpio_spi_dev->iSDOUT = of_get_named_gpio(np, "spi-sdout-gpio", 0);
	status = of_get_named_gpio(np, "spi-cs-gpio", 0);
	gpio_spi_dev->iCS = (status < 0) ? SPI_GPIO_NO_CHIPSELECT : status;
#ifdef CONFIG_DEBUG
	pr_info(DRIVER_NAME " %s: iSCLK: 0x%lx\n", np->name, gpio_spi_dev->iSCLK);
	pr_info(DRIVER_NAME " %s: iSDIN: 0x%lx\n", np->name, gpio_spi_dev->iSDIN);
	pr_info(DRIVER_NAME " %s: iSDOUT: 0x%lx\n", np->name, gpio_spi_dev->iSDOUT);
	pr_info(DRIVER_NAME " %s: iCS : 0x%lx\n", np->name, gpio_spi_dev->iCS);
#endif

	/* Get SPI Related Params */
	tree_info = of_get_property(np, "spi-bus-num", NULL);
	if (tree_info) {
		gpio_spi_dev->spi_id = be32_to_cpup((tree_info));
#ifdef CONFIG_DEBUG
		pr_info(DRIVER_NAME " %s: BUS_ID\t%x\n", np->name, gpio_spi_dev->spi_id);
#endif
	}

	/* Alloc Space for platform data structure */
	gpio_spi_pdata = kzalloc(sizeof(*gpio_spi_pdata), GFP_KERNEL);
	if (!gpio_spi_pdata) {
		status = -ENOMEM;
		goto pdata_alloc_err;
	}

	/* Fill up Platform Data Structure */
	gpio_spi_pdata->sck = gpio_spi_dev->iSCLK;
	//gpio_spi_pdata->miso = SPI_GPIO_NO_MISO;
	gpio_spi_pdata->miso = gpio_spi_dev->iSDOUT;
	gpio_spi_pdata->mosi = gpio_spi_dev->iSDIN;
	gpio_spi_pdata->num_chipselect = 1;

	/* Alloc Space for platform data structure */
	gpio_spi_pdev = kzalloc(sizeof(*gpio_spi_pdev), GFP_KERNEL);
	if (!gpio_spi_pdev) {
		status = -ENOMEM;
		goto pdev_alloc_err;
	}

	/* Fill up Platform Device Structure */
	gpio_spi_pdev->name = "spi_gpio";
	gpio_spi_pdev->id = gpio_spi_dev->spi_id;
	gpio_spi_pdev->dev.platform_data = gpio_spi_pdata;
	gpio_spi_dev->pdev = gpio_spi_pdev;

	/* Register spi_gpio master */
	status = platform_device_register(gpio_spi_dev->pdev);
	if (status < 0) {
		dev_err(&pdev->dev, "platform_device_register failed: %d\n", status);
		goto pdev_reg_err;
	}

#ifdef CONFIG_DEBUG
	pr_info(DRIVER_NAME " %s: spi_gpio platform device registered.\n", np->name);
#endif
	gpio_spi_dev->name = np->name;

	/* Fill up Board Info for SPI device */
	status = add_gpio_spi_device_to_bus(gpio_spi_dev);
	if (status < 0) {
		dev_err(&pdev->dev, "add_gpio_spi_device_to_bus failed: %d\n", status);
		goto spi_add_err;
	}

#ifdef CONFIG_DEBUG
	pr_info(DRIVER_NAME " %s: spi device registered.\n", np->name);
#endif

	/* Point device node data to gpio_spi_device structure */
	if (np->data == NULL)
		np->data = gpio_spi_dev;

	if (gpio_spi_dev_id == 0) {
		/* Alloc Major & Minor number for char device */
		status = alloc_chrdev_region(&gpio_spi_dev_id, 0, MAX_DEV_NUM, DRIVER_NAME);
		if (status) {
			dev_err(&pdev->dev, "Character device region not allocated correctly: %d\n", status);
			goto err_alloc_chrdev_region;
		}
#ifdef CONFIG_DEBUG
		pr_info(DRIVER_NAME " : Char Device Region Registered, with Major: %d.\n",
			MAJOR(gpio_spi_dev_id));
#endif
	}

	if (gpio_spi_class == NULL) {
		/* Create emio-gpio-spi Device Class */
		gpio_spi_class = class_create(THIS_MODULE, DRIVER_NAME);
		if (IS_ERR(gpio_spi_class)) {
			status = PTR_ERR(gpio_spi_class);
			goto err_create_class;
		}
#ifdef CONFIG_DEBUG
		pr_info(DRIVER_NAME " : spi_gpio device class registered.\n");
#endif
	}

	if (spi_drv_registered == 0) {
		/* Register SPI Driver for gpio_spi Device */
		status = spi_register_driver(&gpio_spi_driver);
		if (status < 0) {
			dev_err(&pdev->dev, "gpio_spi_driver register failed: %d\n", status);
			goto err_spi_register;
		}
		spi_drv_registered = 1;
	}

	device_num++;

	return status;

err_spi_register:
	class_destroy(gpio_spi_class);
	gpio_spi_class = NULL;
err_create_class:
	unregister_chrdev_region(gpio_spi_dev_id, MAX_DEV_NUM);
	gpio_spi_dev_id = 0;
err_alloc_chrdev_region:
	spi_unregister_device(gpio_spi_dev->spi);
spi_add_err:
	platform_device_unregister(gpio_spi_dev->pdev);
pdev_reg_err:
	kfree(gpio_spi_pdev);
pdev_alloc_err:
	kfree(gpio_spi_pdata);
pdata_alloc_err:
	kfree(gpio_spi_dev->transfer_buf);
transfer_buf_alloc_err:
	kfree(gpio_spi_dev);
dev_alloc_err:
	return status;
}

/**
 * emio_spi_of_remove - Remove method for gpio_spi device.
 * @np: pointer to device tree node
 *
 * This function removes the gpio_spi device in the device tree. It frees the
 * gpio_spi driver data structure. It returns 0, if the driver is successfully
 * removed, or a negative value if there is an error.
 */
static int emio_spi_of_remove(struct platform_device *pdev)
{
	struct gpio_spi_device *gpio_spi_dev;
	struct device_node *np = pdev->dev.of_node;

	if (np->data == NULL) {
		dev_err(&pdev->dev, "spi %s: ERROR: No gpio_spi_device structure found!\n", np->name);
		return -ENOSYS;
	}
	gpio_spi_dev = (struct gpio_spi_device *)(np->data);

#ifdef CONFIG_DEBUG
	pr_info(DRIVER_NAME " %s : Free display buffer.\n", np->name);
#endif

	if (gpio_spi_dev->transfer_buf != NULL)
		kfree(gpio_spi_dev->transfer_buf);

#ifdef CONFIG_DEBUG
	pr_info(DRIVER_NAME " %s : Unregister gpio_spi Platform Devices.\n", np->name);
#endif

	if (gpio_spi_dev->pdev != NULL)
		platform_device_unregister(gpio_spi_dev->pdev);

	np->data = NULL;
	device_num--;

	/* Unregister SPI Driver, Destroy emio-spi class, Release device id Region after
	 * all emio-spi devices have been removed.
	 */
	if (device_num == 0) {
#ifdef CONFIG_DEBUG
		pr_info(DRIVER_NAME " : Unregister SPI Driver.\n");
#endif
		spi_unregister_driver(&gpio_spi_driver);
		spi_drv_registered = 0;

#ifdef CONFIG_DEBUG
		pr_info(DRIVER_NAME " : Destroy spi_gpio Class.\n");
#endif

		if (gpio_spi_class)
			class_destroy(gpio_spi_class);

		gpio_spi_class = NULL;

#ifdef CONFIG_DEBUG
		pr_info(DRIVER_NAME " : Release Char Device Region.\n");
#endif

		unregister_chrdev_region(gpio_spi_dev_id, MAX_DEV_NUM);
		gpio_spi_dev_id = 0;
	}

	return 0;
}

static struct platform_driver emio_spi_driver = {
	.driver			= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = emio_spi_of_match,
	},
	.probe			= emio_spi_of_probe,
	.remove			= emio_spi_of_remove,
};

module_platform_driver(emio_spi_driver);

MODULE_AUTHOR("Digilent, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME "emio_spi driver");
MODULE_ALIAS(DRIVER_NAME);
