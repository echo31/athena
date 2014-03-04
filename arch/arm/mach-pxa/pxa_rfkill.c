/*
 * pxa_rfkill.c
 *
 * from tosa
 * Copyright (c) 2009 Olivier Grisoni
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License.
 *
 */



#define DEBUG


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/rfkill.h>

#include <mach/pxa_rfkill.h>




static char *rfkill_string_type[] =
{ "All", "WLAN", "BLUETOOTH", "UWB", "WIMAX", "WWAN", "GPS", "Unknown"};

static void pxa_rfkill_enable(struct pxa_rfkill_data *data)
{
	if (data->gpio_reset) {
	    gpio_set_value(data->gpio_reset, data->gpio_reset_inverted);
	    mdelay(25);
	    gpio_set_value(data->gpio_reset, !data->gpio_reset_inverted);
	    }
	if (data->gpio_pwr1) gpio_set_value(data->gpio_pwr1, 1);
	if (data->gpio_pwr2) gpio_set_value(data->gpio_pwr2, 1);



}

static void pxa_rfkill_disable(struct pxa_rfkill_data *data)
{
	if (data->gpio_reset) {
	    gpio_set_value(data->gpio_reset, data->gpio_reset_inverted);
	    mdelay(25);
	    }
	if (data->gpio_pwr1) gpio_set_value(data->gpio_pwr1, 0);
	if (data->gpio_pwr2) gpio_set_value(data->gpio_pwr2, 0);
	if (data->gpio_reset) gpio_set_value(data->gpio_reset, !data->gpio_reset_inverted);
}

static int pxa_rfkill_set_block(void *data, bool blocked)
{
   pr_info("%s  going: %s\n",
   rfkill_string_type[((struct pxa_rfkill_data *)data)->pxa_rfkill_type]
	, blocked ? "off" : "on");

	if (!blocked)
		pxa_rfkill_enable(data);
	else
		pxa_rfkill_disable(data);

	return 0;
}

static const struct rfkill_ops pxa_rfkill_rfkill_ops = {
	.set_block = pxa_rfkill_set_block,
};

static int pxa_rfkill_probe(struct platform_device *dev)
{
	int rc;
	struct rfkill *rfk;

	struct pxa_rfkill_data *data = dev->dev.platform_data;

	if (data->gpio_reset)
	{
		rc = gpio_request(data->gpio_reset, "reset");
		if (rc)
			goto err_reset;
		rc = gpio_direction_output(data->gpio_reset,0);
		if (rc)
			goto err_reset_dir;
	}
	
	if (data->gpio_pwr1)
	{
		rc = gpio_request(data->gpio_pwr1, "power1");
		if (rc)
			goto err_pwr;
		rc = gpio_direction_output(data->gpio_pwr1, 0);
		if (rc)
			goto err_pwr_dir;
	}

	if (data->gpio_pwr2)
	{
		rc = gpio_request(data->gpio_pwr2, "power2");
		if (rc)
			goto err_pwr;
		rc = gpio_direction_output(data->gpio_pwr2, 0);
		if (rc)
			goto err_pwr_dir;
	}

	rfk = rfkill_alloc("rfkill", &dev->dev, data->pxa_rfkill_type,
			   &pxa_rfkill_rfkill_ops, data);
	if (!rfk) {
		rc = -ENOMEM;
		goto err_rfk_alloc;
	}
	rfkill_init_sw_state(rfk,!data->state);
	

	rc = rfkill_register(rfk);
	if (rc)
		goto err_rfkill;

	platform_set_drvdata(dev, rfk);

	return 0;

err_rfkill:
	rfkill_destroy(rfk);
err_rfk_alloc:
	pxa_rfkill_disable(data);
err_pwr_dir:
	if (data->gpio_pwr1) gpio_free(data->gpio_pwr1);
	if (data->gpio_pwr2) gpio_free(data->gpio_pwr2);
err_pwr:
err_reset_dir:
	if (data->gpio_reset) gpio_free(data->gpio_reset);
err_reset:
	return rc;
}

static int __devexit pxa_rfkill_remove(struct platform_device *dev)
{
	struct pxa_rfkill_data *data = dev->dev.platform_data;
	struct rfkill *rfk = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	if (rfk) {
		rfkill_unregister(rfk);
		rfkill_destroy(rfk);
	}
	rfk = NULL;

	pxa_rfkill_disable(data);

	if (data->gpio_pwr1) gpio_free(data->gpio_pwr1);
	if (data->gpio_pwr2) gpio_free(data->gpio_pwr2);
	if (data->gpio_reset) gpio_free(data->gpio_reset);

	return 0;
}

static struct platform_driver pxa_rfkill_driver = {
	.probe = pxa_rfkill_probe,
	.remove = __devexit_p(pxa_rfkill_remove),

	.driver = {
		.name = "pxa-rfkill",
		.owner = THIS_MODULE,
	},
};


static int __init pxa_rfkill_init(void)
{
	return platform_driver_register(&pxa_rfkill_driver);
}

static void __exit pxa_rfkill_exit(void)
{
	platform_driver_unregister(&pxa_rfkill_driver);
}

module_init(pxa_rfkill_init);
module_exit(pxa_rfkill_exit);



MODULE_AUTHOR("Oliver Grisoni");
MODULE_DESCRIPTION("PXA RFKILL Support Driver");
MODULE_LICENSE("GPL");












