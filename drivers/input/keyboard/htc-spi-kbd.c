/*
 * Support for an HTC customized keyboard micro-controller present on
 * several of their pdas and phones.
 *
 * (c) Copyright 2007 Kevin O'Connor <kevin@koconnor.net>
 *
 * This file may be distributed under the terms of the GNU GPL license.
 */

#include <linux/kernel.h>
#include <linux/device.h> // struct device
#include <linux/spinlock.h> // spinlock_t
#include <linux/spi/spi.h> // struct spi_message
#include <linux/input.h> // input_report_key
#include <linux/interrupt.h> // irqreturn_t
#include <linux/htc-spi-kbd.h> // struct htc_spi_kbd_platform_data*
#include <linux/module.h>


struct htckbd {
        struct htc_spi_kbd_platform_data *pdata;
        spinlock_t lock;
        struct input_dev *input;
        struct spi_device *spi;
	int keycount;

        struct spi_message read_msg;
        struct spi_transfer read_transfer;
	char databuf[2];
	int pending, queryagain;
};

// Report a key press to the input system.
static void reportkey(struct htckbd *kbd, int key)
{
        struct htc_spi_kbd_platform_data *pdata = kbd->pdata;
	int isdown = (key & 0x80) == 0;
	int i;

	key &= ~0x80;
	for (i=0; i<kbd->keycount; i++) {

		if (key != pdata->keys[i].id)
			continue;
		input_report_key(kbd->input, pdata->keys[i].keycode, isdown);
		input_sync(kbd->input);
	}
}

// SPI message read callback - find out what keys are present.
static void handledata(void *data)
{
        struct htckbd *kbd = data;
        unsigned long flags;
	int doquery;

#if 0
	printk("handledata %d %d pending=%d queryagain=%d\n"
	       , kbd->databuf[0], kbd->databuf[1]
	       , kbd->pending, kbd->queryagain);
#endif

	if (kbd->databuf[0])
		reportkey(kbd, kbd->databuf[0]);
	if (kbd->databuf[1])
		reportkey(kbd, kbd->databuf[1]);

        spin_lock_irqsave(&kbd->lock, flags);

	if (kbd->queryagain) {
		kbd->queryagain = 0;
		doquery = 1;
	} else {
		kbd->pending = 0;
		doquery = 0;
	}

	spin_unlock_irqrestore(&kbd->lock, flags);

	if (doquery)
		spi_async(kbd->spi, &kbd->read_msg);
}

// Interrupt handler - kick off the spi read (unless one is already
// pending).
static irqreturn_t htckbd_isr(int irq, void *handle)
{
        struct htckbd *kbd = handle;
	int doquery;

#if 0
	printk("htckbd_isr pending=%d queryagain=%d \n"
	       , kbd->pending, kbd->queryagain);
#endif

        spin_lock(&kbd->lock);
	if (!kbd->pending) {
		kbd->pending = 1;
		doquery = 1;
	} else {
		kbd->queryagain = 1;
		doquery = 0;
	}
	spin_unlock(&kbd->lock);

	if (doquery)
		spi_async(kbd->spi, &kbd->read_msg);

        return IRQ_HANDLED;
}

static int __devinit htckbd_probe(struct spi_device *spi)
{
	struct htckbd *kbd;
	struct input_dev *input = NULL;
	struct htc_spi_kbd_platform_data *pdata = spi->dev.platform_data;
	int ret, i;

	// Initialize kbd data structure.
	kbd = kzalloc(sizeof(*kbd), GFP_KERNEL);
	if (!kbd)
		return -ENOMEM;
	spi_set_drvdata(spi, kbd);
        spin_lock_init(&kbd->lock);
	kbd->pdata = pdata;
	kbd->spi = spi;

	// Initialize spi messages.
	kbd->read_transfer.rx_buf = kbd->databuf;
	kbd->read_transfer.len = sizeof(kbd->databuf);
	spi_message_init(&kbd->read_msg);
	kbd->read_msg.complete = handledata;
	kbd->read_msg.context = kbd;
	spi_message_add_tail(&kbd->read_transfer, &kbd->read_msg);

	// Initialize input device.
	input = input_allocate_device();
	if (! input)
		goto fail;
	input->name = spi->modalias;
	set_bit(EV_KEY, input->evbit);

	for (i = 0; i<ARRAY_SIZE(pdata->keys) && pdata->keys[i].id; i++)
		set_bit(pdata->keys[i].keycode, input->keybit);
	kbd->keycount = i;

	ret = input_register_device(input);
	if (ret)
		goto fail;
	kbd->input = input;

	// Get irq
	ret = request_irq(spi->irq, htckbd_isr, IRQF_TRIGGER_FALLING,
			  spi->modalias, kbd);
	if (ret)
		goto fail;

	// Drain any pending key events
	htckbd_isr(spi->irq, kbd);
	
	

	return 0;
fail:
	input_unregister_device(input);
	kfree(kbd);
	return -ENOSYS;
}

static int __devexit htckbd_remove(struct spi_device *spi)
{
        struct htckbd *kbd = dev_get_drvdata(&spi->dev);

        free_irq(kbd->spi->irq, kbd);
        input_unregister_device(kbd->input);
        kfree(kbd);

        return 0;
}

static struct spi_driver htckbd_driver = {
        .driver = {
                .name   = "htc-spi-kbd",
                .bus	= &spi_bus_type,
                .owner  = THIS_MODULE,
        },
        .probe          = htckbd_probe,
        .remove         = __devexit_p(htckbd_remove),
};

static int __init htckbd_init(void)
{
        return spi_register_driver(&htckbd_driver);
}

static void __exit htckbd_exit(void)
{
        spi_unregister_driver(&htckbd_driver);
}

module_init(htckbd_init);
module_exit(htckbd_exit);

MODULE_DESCRIPTION("HTC SPI keyboard driver");
MODULE_LICENSE("GPL v2");
