/*
 *	Support for the HTC Athena.
 *
 * Copyright (c) 2009 Olivier Grisoni
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/pwm_backlight.h>
#include <linux/wl12xx.h>
#include <linux/pda_power.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <linux/lis3lv02d.h>
#include <linux/htc-spi-kbd.h>
#include <linux/rfkill.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/mfd/htc-egpio.h>
#include <linux/mfd/tc6393xb.h>
#include <linux/mfd/w228x_base.h>

#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/ad7877.h>
#include <linux/udc_isp158x.h>
#include <linux/usb/gpio_vbus.h>
#include <mach/pxa_rfkill.h>
#include <mach/udc.h>

#include <mach/mmc.h>
#include <mach/ohci.h>
#include <mach/htcathena.h>

#include <mach/camera.h>
#include <media/soc_camera.h>
#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>


#include <mach/pata_athena.h>
#include <mach/pm.h>
#include <mach/hardware.h>
#include <mach/pxa27x.h>
#include <mach/audio.h>


#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/smemc.h>
#include "devices.h"
#include "generic.h"

#include <sound/ak4641.h>
#include <linux/memblock.h>

#include <video/w2284fb.h>
#include <video/platform_lcd.h>

#define PXA_NR_BUILTIN_GPIO	(128)

static __init post_init(void);
static int __init keyboard_init(void);

static unsigned long athena_pin_config[] __initdata = {

	/* SDRAM and Static Memory I/O Signals */
	GPIO15_nCS_1,
	GPIO78_nCS_2,
	GPIO79_nCS_3,
	GPIO80_nCS_4,
	GPIO33_nCS_5,

	/* Miscellaneous I/O and DMA Signals */
	GPIO18_RDY,
	GPIO115_DREQ_0,

	/* PC CARD for ATI */
	GPIO49_nPWE,

	/* I2C sound */
	GPIO117_I2C_SCL,
	GPIO118_I2C_SDA,

	/* FFUART   GPS */
	GPIO34_FFUART_RXD,
	GPIO39_FFUART_TXD,

	/* BTUART   */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* STUART  unknown */
	GPIO46_STUART_RXD,
	GPIO47_STUART_TXD,

	/* PWM 0 Backlight*/
	GPIO16_PWM0_OUT,

	/* SSP 1  battery_monitor+touchscreen */
	GPIO23_SSP1_SCLK,
	GPIO24_SSP1_SFRM,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,

	/* SSP 2 Keyboard */
	GPIO36_SSP2_SCLK,
	GPIO38_SSP2_TXD,
	GPIO40_SSP2_RXD,

	/* WIFI */
	GPIO32_MMC_CLK,
	GPIO92_MMC_DAT_0,
	GPIO109_MMC_DAT_1,
	GPIO110_MMC_DAT_2,
	GPIO111_MMC_DAT_3,
	GPIO112_MMC_CMD,


	/* USB OHCI */
	GPIO88_USBH1_PWR,
	GPIO89_USBH1_PEN,

};

/****************************************************************
 * GPIO Keys
 ****************************************************************/

#define INIT_KEY(_code, _gpio, _active_low, _desc)\
	{			\
	.code	= KEY_##_code,	\
	.gpio	= _gpio,	\
	.active_low = _active_low, \
	.desc   = _desc,	\
	.type   = EV_KEY,	\
	.wakeup = 1,		\
	}

static struct gpio_keys_button athena_button_table[] = {
 INIT_KEY(POWER, GPIO0_HTCATHENA_KEY_POWER,	1, "Power button"),
 INIT_KEY(VOLUMEUP, GPIO17_HTCATHENA_KEY_VOL_UP, 0, "Volume up"),
 INIT_KEY(CAMERA, GPIO93_HTCATHENA_KEY_CAMERA,	0, "Camera button"),
 INIT_KEY(RECORD, GPIO94_HTCATHENA_KEY_RECORD,	0, "Record button"),
 INIT_KEY(WWW, GPIO95_HTCATHENA_KEY_WWW,		0, "WWW button"),
 INIT_KEY(SEND, GPIO98_HTCATHENA_KEY_SEND,	0, "OK button"),
 INIT_KEY(END, GPIO99_HTCATHENA_KEY_END,	0, "Windows Button"),
 INIT_KEY(VOLUMEDOWN, GPIO102_HTCATHENA_KEY_VOL_DOWN, 0, "Volume down"),
 INIT_KEY(RIGHT, GPIO103_HTCATHENA_KEY_RIGHT,	1, "Right button"),
 INIT_KEY(UP, GPIO104_HTCATHENA_KEY_UP,		1, "Up button"),
 INIT_KEY(LEFT, GPIO105_HTCATHENA_KEY_LEFT,	1, "Left button"),
 INIT_KEY(DOWN, GPIO106_HTCATHENA_KEY_DOWN ,	1, "Down button"),
 INIT_KEY(KPENTER, GPIO107_HTCATHENA_KEY_ENTER,	1, "Action button"),

};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = athena_button_table,
	.nbuttons = ARRAY_SIZE(athena_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

/****************************************************************
 * Athena Keyboard
 ****************************************************************/

struct htc_spi_kbd_platform_data kb_info = {
	.keys = {
          /*row1*/
          
  /*  {0x11, KEY_PHONE}, {0x21, KEY_WWW}, {0x31, KEY_LEFTALT},
	{0x02, KEY_CYCLEWINDOWS}, {0x22, KEY_MAIL}, {0x32, KEY_PLAY},
	{0x03, KEY_BRIGHTNESSUP}, {0x33, KEY_RIGHTALT}, {0x04, KEY_EXIT},*/
	
	 {0x11, KEY_0}, {0x21, KEY_1}, {0x31, KEY_2},
	{0x02, KEY_APOSTROPHE}, {0x22, KEY_SEMICOLON}, {0x32, KEY_BACKSLASH},
	{0x03, KEY_SLASH}, {0x33, KEY_MINUS}, {0x04, KEY_EQUAL},
	
	 /*row2*/
	{0x41, KEY_Q}, {0x42, KEY_W}, {0x43, KEY_E}, {0x44, KEY_R},
	{0x35, KEY_T}, {0x36, KEY_Y}, {0x37, KEY_U}, {0x38, KEY_I},
	{0x06, KEY_O}, {0x07, KEY_P},
	 /*row3*/
	 {0x51, KEY_A}, {0x52, KEY_S },	{0x53, KEY_D}, {0x54, KEY_F},
	 {0x45, KEY_G}, {0x46, KEY_H}, 	{0x47, KEY_J}, {0x48, KEY_K},
	 {0x05, KEY_L}, {0x34, KEY_BACKSPACE},
	  /*row4*/	 
	{0x12, KEY_LEFTSHIFT}, {0x61, KEY_Z}, {0x62, KEY_X}, {0x63, KEY_C},	
	{0x55, KEY_V}, {0x56, KEY_B}, {0x57, KEY_N}, {0x58, KEY_M},
	{0x17, KEY_UP}, {0x08, KEY_ENTER},
	 /*row5*/
	 {0x14, KEY_LEFTALT }, {0x23, KEY_TAB},
	{0x01, KEY_COMMA}, {0x64, KEY_LEFTCTRL}, {0x65, KEY_SPACE},
	{0x67, KEY_DOT}, {0x26, KEY_LEFT}, {0x27, KEY_DOWN}, {0x28, KEY_RIGHT},
          
   
	}
};

/*
 * Keyboard spi slave -   connected to SSP2
 */

static void set_kb_cs(struct spi_board_info *spi, int cs, int pol)
{
	
}

static struct pxa2xx_spi_chip kb_chip = {
	.tx_threshold = 8,
	.rx_threshold = 8,
	.cs_control =  (void*)set_kb_cs,
	.gpio_cs  = GPIO10_HTCATHENA_KEYBOARD_IO ,
};

/****************************************************************
 * EGPIOs
 ****************************************************************/


static struct resource egpio_cpld2_resources[] = {

	[0] = {
		.start  = HTCATHENA_EGPIO_CPLD2_BASE_0,
		.end    = HTCATHENA_EGPIO_CPLD2_BASE_0 + 0x4 - 1,
		.flags  = IORESOURCE_MEM,
	},
};

// CPLD2 Settings from Wince values C902

static struct htc_egpio_chip egpio_cpld2_chips[] = {

	[0] = {
	.reg_start = 0,		 /* CPLD2  Output 16 bits */
	.gpio_base = CPLD2_EGPIO_BASE,  /* NR_BUILTIN_GPIO + 64  */
	.num_gpios = 16,
	.direction = 0x01FF,
	.initial_values =  0xc902 | 0x1f | 0x20  ,	/* LCD-noBL-i2c enabled- */
	},
};

static struct htc_egpio_platform_data egpio_cpld2_info = {
	.reg_width	= 16,
	.bus_width	= 16,
	.chip		= egpio_cpld2_chips,
	.num_chips	= ARRAY_SIZE(egpio_cpld2_chips),
};

static struct platform_device egpio_cpld2 = {
	.name		= "htc-egpio",
	.id		= 1,
	.resource	= egpio_cpld2_resources,
	.num_resources	= ARRAY_SIZE(egpio_cpld2_resources),
	.dev = {
	.platform_data = &egpio_cpld2_info,
	},
};

/*******************************************************************
 * HTC EGPIO on the Xilinx CPLD1 (8 16-bit aligned 8-bit registers)
 *******************************************************************/

static struct resource egpio_cpld1_resources[] = {

	[0] = {
	.start = HTCATHENA_CPLD1_BASE,
	.end   = HTCATHENA_CPLD1_BASE + 0x8 - 1 ,
	.flags = IORESOURCE_MEM,
	},
	[1] = {
	.start = PXA_GPIO_TO_IRQ(GPIO14_HTCATHENA_CPLD1_EXT_INT),
	.end   = PXA_GPIO_TO_IRQ(GPIO14_HTCATHENA_CPLD1_EXT_INT),
	.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};
// CPLD1 Settings from Wince values 0C 06 10 50

static struct htc_egpio_chip egpio_cpld1_chips[] = {

	[0] = {				/* Bank A output */
	.reg_start = 0,
	.gpio_base = CPLD1_EGPIO_BASE,	/* NR_BUILTIN_GPIO */
	.num_gpios = 8,
	.direction = HTC_EGPIO_OUTPUT,
	.initial_values =  0xc,		/* 4 for working Device */
	},
	[1] = {				/*  Bank B output */
	.reg_start = 1,
	.gpio_base = NR_BUILTIN_GPIO + 8 * 1,
	.num_gpios = 8,
	.direction = HTC_EGPIO_OUTPUT,
	.initial_values =  0x6,
	},
	[2] = {				/* Bank C output */
	.reg_start = 2,
	.gpio_base = NR_BUILTIN_GPIO + 8 * 2,
	.num_gpios = 8,
	.direction = HTC_EGPIO_OUTPUT,
	.initial_values =  0x10 ,  // wifi ,   	
					
	},
	[3] = {				/* Bank D output */
	.reg_start = 3,
	.gpio_base = NR_BUILTIN_GPIO + 8 * 3,
	.num_gpios = 8,
	.direction = HTC_EGPIO_OUTPUT,
	.initial_values = 0x50,  	
	},
	[4] = {				/* Bank F intput */
	.reg_start = 5,
	.gpio_base = NR_BUILTIN_GPIO + 8 * 5,
	.num_gpios = 8,
	.direction = HTC_EGPIO_INPUT,	
	},
	[5] = {				/* Bank G intput */
	.reg_start = 6,
	.gpio_base = NR_BUILTIN_GPIO + 8 * 6,
	.num_gpios = 8,
	.direction = HTC_EGPIO_INPUT,	
	},
	
	[6] = {
	.reg_start = 8,			/* Bank H  Output */
	.gpio_base = NR_BUILTIN_GPIO+8 * 8,
	.num_gpios = 8,
	.direction = HTC_EGPIO_OUTPUT,
	.initial_values = 0x0, 
	},

};

static struct htc_egpio_platform_data egpio_cpld1_info = {
	.reg_width	= 8,
	.bus_width	= 16,
	.irq_base	= IRQ_BOARD_START,
	.num_irqs	= 8,
	.ack_register	= 4,
	.chip	 = egpio_cpld1_chips,
	.num_chips	= ARRAY_SIZE(egpio_cpld1_chips),
	.invert_acks = 1 ,
};

static struct platform_device egpio_cpld1 = {
	.name		= "htc-egpio",
	.id		= 0,
	.resource	= egpio_cpld1_resources,
	.num_resources	= ARRAY_SIZE(egpio_cpld1_resources),
	.dev = {
	.platform_data	= &egpio_cpld1_info,
	},
};

/****************************************************************
 * MTD - Nand - DoC G4 - MD8331-d2G
 ****************************************************************/

static struct resource docg4_flash_resource[] = {
	{
	.start	= PXA_CS0_PHYS,
	.end	= PXA_CS0_PHYS +  0x00100001 , // SZ_8K-1 ,
	.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device docg4_flash = {
	.name		= "docg4",
	.id     = -1,
	.resource = docg4_flash_resource,
	.num_resources = ARRAY_SIZE(docg4_flash_resource),
};


/****************************************************************
 * Touchscreen AD7877 Chip
 ****************************************************************/

static const struct ad7877_platform_data ts_info = {
	.model			= 7877,
	.vref_delay_usecs	= 50,	/* internal, no capacitor */
	.x_plate_ohms	 	= 419,
	.y_plate_ohms		= 486,
	.x_min                  = 310,
	.y_min                  = 217,
	.x_max                  = 3716,
	.y_max                  = 3782,
	.pressure_max		= 1000,
	.pressure_min		= 0,
	.stopacq_polarity	= 1,
	.first_conversion_delay	= 3,
	.acquisition_time	= 1,
	.averaging		= 1,
	.pen_down_acc_interval  = 1,
};

static void set_ts_cs(struct spi_board_info *spi, int cs, int pol)
{
}

static struct pxa2xx_spi_chip ts_chip = {
	.tx_threshold = 1,
	.rx_threshold = 12,
	.timeout = 64,
	.cs_control = (void*)set_ts_cs,
};

/****************************************************************
 * LEDs
 ****************************************************************/

static struct gpio_led gpio_leds[] = {
	{
		.name = "athena::gsm",
		.default_trigger = "gsm",
		.gpio = EGPIO1_B3_HTCATHENA_GSM_LED,
	},
	{
		.name = "athena::gps",
		.default_trigger = "gps",
		.gpio = EGPIO1_B4_HTCATHENA_GPS_LED,
	},
	{
		.name = "athena::alert",
		.default_trigger = "alert",
		.gpio = EGPIO1_B5_HTCATHENA_ALERT_LED,
	},
	{
		.name = "athena::bt",
		.default_trigger = "bluetooth",
		.gpio = EGPIO1_B6_HTCATHENA_BT_LED,
	},
	
	{
		.name = "athena::wifi",
		.default_trigger = "wifi",
		.gpio = EGPIO1_B7_HTCATHENA_WIFI_LED,
	},
	{
		.name = "athena::backlight",
		.default_trigger = "backlight",
		.gpio = EGPIO1_C0_HTCATHENA_CHARGING_LED_RED,
		.default_state =0,
	},
	{
		.name = "athena::green-charged",
		.default_trigger = "green",
		.gpio = EGPIO1_C1_HTCATHENA_CHARGING_LED_GREEN,
		.default_state =0,
	},
	{
		.name = "athena::red-alert",
		.default_trigger = "red-blinking",
		.gpio = EGPIO1_C2_HTCATHENA_CHARGING_LED_REDBLK,
	},


};

static struct gpio_led_platform_data gpio_led_info = {
	.leds = gpio_leds,
	.num_leds = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name = "leds-gpio",
	.id   = -1,
	.dev  = {
		.platform_data = &gpio_led_info,
	},
};

/****************************************************************
* WLAN  WL1250and RFKILL
****************************************************************/

static void athena_wl1251_set_power(bool enable)  
{  
   /*  
    * Keep power always on until wl1251_sdio driver learns to re-init  
    * the chip after powering it down and back up.  
    */  
    printk(KERN_DEBUG "wifi power on %d \n", enable);
    
}  
 
static struct wl12xx_platform_data athena_wl1251_pdata = {  
   .set_power  = athena_wl1251_set_power,  
   .use_eeprom = true,  
   .irq = PXA_GPIO_TO_IRQ(GPIO35_HTCATHENA_WIFI_IRQ),
};  
 
static struct platform_device athena_wl1251_data = {  
   .name           = "wl1251_data",  
   .id             = -1,  
   .dev        = {  
       .platform_data  = &athena_wl1251_pdata,  
   },  
};  

static void athena_wl1251_init(void)
{
	int ret;
	printk(KERN_INFO "wl1251  init\n");
	ret = gpio_request(GPIO35_HTCATHENA_WIFI_IRQ, "wl1251 irq");
	if (ret < 0)
		goto fail;

	ret = gpio_direction_input(GPIO35_HTCATHENA_WIFI_IRQ);
	if (ret < 0)
		goto fail_irq;

	athena_wl1251_pdata.irq = PXA_GPIO_TO_IRQ(GPIO35_HTCATHENA_WIFI_IRQ);
	if (athena_wl1251_pdata.irq < 0)
		goto fail_irq;

	return;

fail_irq:
	gpio_free(GPIO35_HTCATHENA_WIFI_IRQ);
fail:
	printk(KERN_ERR "wl1251 board initialisation failed\n");
}

static int athena_mci_init(struct device *dev, irq_handler_t isr, void *data)
{
	
	
		printk(KERN_INFO "wl1251 mci init\n");
	    gpio_set_value(GPIO54_HTCATHENA_WIFI, 1);
	    gpio_set_value(GPIO72_HTCATHENA_WIFI_RESET, 1);
       
         
        gpio_set_value(EGPIO1_C6_HTCATHENA_WIFI_POWER4, 1);
        gpio_set_value(EGPIO2_HTCATHENA_WIFI_POWER1, 1);
        gpio_set_value(EGPIO2_HTCATHENA_WIFI_POWER2, 1);
        gpio_set_value(EGPIO1_C3_HTCATHENA_WIFI_POWER3, 1);
      
        
         return 0;
}

static void athena_mci_exit(struct device *dev, void *data)
{
	  gpio_set_value(GPIO72_HTCATHENA_WIFI_RESET, 0);
        gpio_set_value(GPIO54_HTCATHENA_WIFI,0);
         
        gpio_set_value(EGPIO1_C6_HTCATHENA_WIFI_POWER4, 0);
        gpio_set_value(EGPIO2_HTCATHENA_WIFI_POWER1, 0);
        gpio_set_value(EGPIO2_HTCATHENA_WIFI_POWER2, 0);
        gpio_set_value(EGPIO1_C3_HTCATHENA_WIFI_POWER3, 0);
		
}

static struct pxamci_platform_data athena_mci_info = {
	    .init                   = athena_mci_init,
        .exit                   = athena_mci_exit,
        .detect_delay_ms        = 100,
		.ocr_mask		= MMC_VDD_32_33|MMC_VDD_33_34,
        .gpio_card_detect       = -1,
        .gpio_card_ro           = -1,
        .gpio_power             = -1,
};

static struct pxa_rfkill_data athena_rfkill_wlan_data = {
	.name		= "wifi",
	.state		 = 0 ,
	.pxa_rfkill_type = RFKILL_TYPE_WLAN,

	
	
	

};

static struct platform_device athena_rfkill_wlan = {
	.name	= "pxa-rfkill",
	.id	= 0,
	.dev.platform_data = &athena_rfkill_wlan_data,
};

/****************************************************************
* BT via BTUART and RFKILL
****************************************************************/

static struct pxa_rfkill_data athena_rfkill_bt_data = {
	.name		= "bluetooth",
	.gpio_reset	= EGPIO1_A4_HTCATHENA_BT_PWR1,
	.gpio_pwr1	= EGPIO1_A5_HTCATHENA_BT_PWR2,
	.pxa_rfkill_type = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device athena_rfkill_bt = {
	.name	= "pxa-rfkill",
	.id	= 1,
	.dev.platform_data = &athena_rfkill_bt_data,
};

/****************************************************************
* GPS via FFUART and RFKILL
****************************************************************/

static struct pxa_rfkill_data athena_rfkill_gps_data = {
	.name		= "gps",
	.gpio_pwr1	= GPIO58_HTCATHENA_GPS,
	.gpio_pwr2	= GPIO61_HTCATHENA_GPS,
	.pxa_rfkill_type = RFKILL_TYPE_GPS,
};

static struct platform_device athena_rfkill_gps = {
	.name	= "pxa-rfkill",
	.id	= 2,
	.dev.platform_data = &athena_rfkill_gps_data,
};

/****************************************************************
* Backlight
****************************************************************/

static int athena_backlight_init(struct device *dev)
{
	int ret;

	ret = gpio_request(EGPIO2_HTCATHENA_BKL_POWER, "BL_POWER");
	if (ret)
		goto err;
	ret = gpio_request(EGPIO1_D7_HTCATHENA_LCD_BKL, "BL_LCD");
	if (ret)
		goto err2;
	return 0;


err2:
	gpio_free(EGPIO2_HTCATHENA_BKL_POWER);
	printk(KERN_ERR "bklight GPIO101_BACKLIGHT %x \n", ret);
err:
	printk(KERN_ERR "bklight EGPIO1_D7_LCD_BKL %x \n", ret);
	return ret;
}

static int athena_backlight_notify(struct device *dev, int brightness)
{
	gpio_set_value(EGPIO2_HTCATHENA_BKL_POWER, brightness);

if (brightness >= 200) {
	gpio_set_value(EGPIO1_D7_HTCATHENA_LCD_BKL, 1);
	printk(KERN_DEBUG "bk light brightness %d \n", brightness);
	return brightness - 72;
	} else {
	gpio_set_value(EGPIO1_D7_HTCATHENA_LCD_BKL, 0);
	return brightness;
	}
	return brightness;
}

static void athena_backlight_exit(struct device *dev)
{
	gpio_free(EGPIO2_HTCATHENA_BKL_POWER);
	gpio_free(EGPIO1_D7_HTCATHENA_LCD_BKL);
}

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id	 = 0,
	.max_brightness	= 272,
	.dft_brightness	= 100,
	.pwm_period_ns	= 30923,
	.init		= athena_backlight_init,
	.notify		 = athena_backlight_notify,
	.exit		= athena_backlight_exit,
};

static struct platform_device backlight = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.parent	= &pxa27x_device_pwm0.dev,
		.platform_data = &backlight_data,
	},
};



/******************************************************************************
 * s1r72v03 PATA IDE
 ******************************************************************************/
 


static int s1r72v03_init(void)
{
	int rc;


	rc = gpio_request(GPIO12_HTCATHENA_EPSON_INT, "EPSON_IRQ");
	if (rc) {
		
		printk(KERN_ERR "Epson Irq GPIO error %x \n", rc);
		goto err;
		}
	

	rc = gpio_request(GPIO68_HTCATHENA_EPSON_POWER, "EPSON_POWER");
	if (rc) {
		
		printk(KERN_ERR "Epson Power GPIO error %x \n", rc);
		goto err2;
		}
		
	gpio_direction_output(GPIO68_HTCATHENA_EPSON_POWER, 1);
	msleep(0x20);
	gpio_set_value(EGPIO1_C4_HTCATHENA_EPSON_POWER_SLEEP, 1);
	msleep(0x4b0);
		
	return 0;

err:
	gpio_free(GPIO12_HTCATHENA_EPSON_INT);
err2:	
	
	gpio_free(GPIO68_HTCATHENA_EPSON_POWER);
	
	return -ENODEV;
}





static struct pata_pxa_pdata s1r72v03_pata_pdata = {
	.reg_shift	= 2,
	.dma_dreq	= 0,
	.irq_flags	= IRQF_TRIGGER_RISING,
	.gpio_setup	=  s1r72v03_init,
};

static struct resource s1r72v03_ide_resources[] = {
	[0] = {	/* I/O Base address */
	       .start	= PXA_CS3_PHYS ,
	       .end	= PXA_CS3_PHYS + 4*0xFF-1,
	       .flags	= IORESOURCE_MEM
	},
	[1] = {	/* DMA DREQ<0> pin */
	       .start	=  0,
	       .end	=  0,
	       .flags	= IORESOURCE_IRQ
	},
	
	
};

static struct platform_device s1r72v03_ide_device = {
	.name		= "pata_athena",
	.num_resources	= ARRAY_SIZE(s1r72v03_ide_resources),
	.resource	= s1r72v03_ide_resources,
	.dev		= {
		.platform_data	= &s1r72v03_pata_pdata,
		.coherent_dma_mask	= 0xffffffff,
	}
};



#define w2284_lcd_resume	NULL
#define w2284_lcd_suspend	NULL

static struct w2284_tg_info w2284_tg_info = {
	.suspend	= w2284_lcd_suspend,
	.resume		= w2284_lcd_resume,
};

static struct w2284_gen_regs w2284_lcd_regs = {
	.lcd_format =        0x003,
	.lcdd_cntl1 =        0x00FC0000,
	.lcdd_cntl2 =        0x0003FFFF,
	.genlcd_cntl1 =      0x00A952DD,
        .genlcd_cntl2 =      0x003C00F0,	
	.genlcd_cntl3 =      0x000102AA,
};

static struct w2284_mode w2284_modes[] = {
{
        .xres 		= 480,
	.yres 		= 640,
	.left_margin 	= 15,
	.right_margin 	= 24,
	.upper_margin 	= 3,
	.lower_margin 	= 4,
	.crtc_ss	= 0x8010000F,
	.crtc_ls	= 0xE0090001,	
	.crtc_gs	= 0xC0000000,	
	.crtc_vpos_gs	= 0x00010287,
	.crtc_ps1_active =0xe0000100,	//not sure about this reg
	.crtc_rev	= 0x0,
	.crtc_dclk	= 0x80000000,
	.crtc_gclk	= 0x0,
	.crtc_goe	= 0x0,
	.pll_freq 	= 120,
    //    .fast_pll_freq   = 192,
        .sysclk_src     = 1,   //CLK_SRC_PLL,
        .sysclk_normal_divider = 6,
        .sysclk_fast_divider = 2,
        .sysclk_turbo_divider = 1,
	
	.pixclk_divider = 9,
	.pixclk_divider_rotated = 6,
	.pixclk_src     = 1,    //CLK_SRC_PLL,
    // .sysclk_divider = 8,
        
},

};

struct w2284_mem_info w2284_mem_info = {               //done
	.ext_cntl        = 0x00e00011,
	.sdram_mode_reg  = 0x017d0031,
	.ext_timing_cntl = 0x2c002a4a,	
	.io_cntl         = 0x733c7cc3,
	.size            = 0x400000,
};

struct w2284_bm_mem_info w2284_bm_mem_info = {
	.ext_mem_bw = 0,
	.offset = 0,
	.ext_timing_ctl = 0,
	.ext_cntl = 0,
	.mode_reg = 0,
	.io_cntl = 0,
	.config = 0,
};

static struct w2284_gpio_regs w2284_gpio_info = {
	.init_data1 = 0xffff0000,	/* GPIO_DATA */
    .init_data2 = 0xfff70000,	/* GPIO_DATA2 */
	.gpio_dir1  = 0x0000ffff,	/* GPIO_CNTL1 */
	.gpio_oe1   = 0x003c0000,	/* GPIO_CNTL2 */
	.gpio_dir2  = 0x0000ffff,	/* GPIO_CNTL3 */
	.gpio_oe2   = 0x003c0000,	/* GPIO_CNTL4 */
};

static struct w2284fb_mach_info w2284_info = {                //done
	.tg        = &w2284_tg_info,
	.mem       = &w2284_mem_info,
	.bm_mem    = &w2284_bm_mem_info,
	.gpio      = &w2284_gpio_info,
	.regs      = &w2284_lcd_regs,
	.modelist  = w2284_modes,
	.num_modes = 1,
	.xtal_freq = 16000000,
    //    .xtal_dbl   = 0,
}; 


static struct resource w2284_resources[] = {
	[0] = {
        .start	= HTCATHENA_W2284_BASE_PHYS,                       // 0x04000000
		.end	= HTCATHENA_W2284_BASE_PHYS + 0xffffff,	
		.flags	= IORESOURCE_MEM,
	      },
        [1] = {
		.start	= PXA_GPIO_TO_IRQ(GPIO21_HTCATHENA_ATI_SD_INT),
		.end	= PXA_GPIO_TO_IRQ(GPIO21_HTCATHENA_ATI_SD_INT),
		.flags	= IORESOURCE_IRQ,
               },
};

static struct platform_device w2284 = {
	.name	= "w2284fb",
	.id	= -1,
	.dev	= {
		.platform_data = &w2284_info,
	},
	.num_resources = ARRAY_SIZE(w2284_resources),
	.resource      = w2284_resources,
};




static void athena_atiw228x_sdio_setpower(struct device *dev, unsigned int vdd)
{
            gpio_set_value(EGPIO1_D6_HTCATHENA_SD_POWER, 1);
          printk(KERN_ERR "w2284 set sdio power");
        if ((1 << vdd) & MMC_VDD_32_33)
		{
		printk(KERN_INFO "MMC on %d \n",gpio_get_value(EGPIO1_D6_HTCATHENA_SD_POWER));
		gpio_set_value(GPIO66_HTCATHENA_SD_POWER, 1);
		
		}
	else
		{

		gpio_set_value(GPIO66_HTCATHENA_SD_POWER, 1);
		printk(KERN_INFO "MMC off %d\n",gpio_get_value(EGPIO1_D6_HTCATHENA_SD_POWER));
		}
}

static struct atiw_mmc_hwconfig athena_atiw_mmc_hwconfig = {
        .setpower = athena_atiw228x_sdio_setpower,
};

static struct w228x_platform_data athena_w2284_platform_data = {

	.bus_shift = 2,
	.atiw_mmc_hwconfig = &athena_atiw_mmc_hwconfig,
};

static struct resource athena_w2284_resources[] = {
	[0] = {
		.start	= HTCATHENA_W2284_BASE_PHYS,
		.end	= HTCATHENA_W2284_CTRL_PHYS + 0x10000,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= PXA_GPIO_TO_IRQ(GPIO21_HTCATHENA_ATI_SD_INT), // fake
		.end	= PXA_GPIO_TO_IRQ(GPIO21_HTCATHENA_ATI_SD_INT),
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start  = HTCATHENA_W2284_MMC_PHYS,
		.end    = HTCATHENA_W2284_MMC_PHYS + 0x4000,
		.flags  = IORESOURCE_MEM,
	},
	[3] = {
		.start  = PXA_GPIO_TO_IRQ(GPIO21_HTCATHENA_ATI_SD_INT),
		.end  	= PXA_GPIO_TO_IRQ(GPIO21_HTCATHENA_ATI_SD_INT),
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device athena_atiw_mmc = {
	.name           = "w228xasic",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(athena_w2284_resources),
	.resource       = athena_w2284_resources,
	.dev = { .platform_data = &athena_w2284_platform_data, },
};
EXPORT_SYMBOL_GPL(athena_atiw_mmc);

/* ----------------- e800 tc6393xb parameters ------------------ */

static struct tc6393xb_platform_data e800_tc6393xb_info = {
	.irq_base       = IRQ_BOARD_START,
	.scr_pll2cr     = 0x0cc1,
	.scr_gper       = 0,
	.gpio_base      = -1,
//	.suspend        = &eseries_tmio_suspend,
//	.resume         = &eseries_tmio_resume,
//	.enable         = &eseries_tmio_enable,
//	.disable        = &eseries_tmio_disable,
};

static struct platform_device e800_tc6393xb_device = {
	.name           = "tc6393xb",
	.id             = -1,
	.dev            = {
		.platform_data = &e800_tc6393xb_info,
	},
	.num_resources = 2,
	.resource      = athena_w2284_resources,
};




/****************************************************************
 * USB client controller
 ****************************************************************/
 

/* todo :memory range and irq to be checked */

static struct resource athena_isp1582_resources[] = {
         [0] = {
                .start      = 0x0b000000,
                .end        = 0x0b000000 + 0x304-1,
                .flags      = IORESOURCE_MEM,
         },
         [1] = {
                .start      = PXA_GPIO_TO_IRQ(GPIO1_HTCATHENA_ISP1582_INT),
                .end        = PXA_GPIO_TO_IRQ(GPIO1_HTCATHENA_ISP1582_INT),
                .flags      = IORESOURCE_IRQ,
         },
 };



/* ISP1582 UDC */
static void isp158x_udc_command(int cmd)
{
	switch (cmd) {
		case ISP158X_UDC_CMD_INIT:
		
		// EGPIO2_HTCATHENA_TV_OUT_INIT already enabled
		gpio_set_value(EGPIO1_H3_HTCATHENA_TV_OUT_RESET,0);
		msleep(10);
		gpio_set_value(EGPIO1_H3_HTCATHENA_TV_OUT_RESET,1);
		
		gpio_set_value(EGPIO1_H4_HTCATHENA_ISP1582_CS,1);
		mdelay(5);
	
		break;
		
		
	case ISP158X_UDC_CMD_DISCONNECT:
		gpio_set_value(EGPIO1_H5_HTCATHENA_ISP1582_CS,0); 
		
		break;
	case ISP158X_UDC_CMD_CONNECT:
		gpio_set_value(EGPIO1_H5_HTCATHENA_ISP1582_CS,1);	
		break;
	}
}



struct isp158x_udc_mach_info isp158x_platform_data = {
	.udc_command = isp158x_udc_command,
};

struct platform_device athena_isp1582_device = {
	.name       = "isp1582_udc",
	.id     = -1,
	.num_resources  = ARRAY_SIZE(athena_isp1582_resources),
	.resource   = athena_isp1582_resources,
	.dev		= {
		.platform_data	= &isp158x_platform_data,
	},
};
 
 

/* ISP1582 UDC */

static struct resource gpio_vbus_resource = {
	.flags = IORESOURCE_IRQ,
	.start = ATHENA_USB_CABLE_IRQ,
	.end   = ATHENA_USB_CABLE_IRQ,
};

static struct gpio_vbus_mach_info athena_gpio_vbus_info = {
	.gpio_vbus		= EGPIO1_F4_HTCATHENA_USB20_CABLE_PLUGGED,
	.gpio_pullup		= -1,
	.gpio_vbus_inverted = 0,
};

static struct platform_device athena_gpio_vbus = {
	.name	= "gpio-vbus",
	.id	= -1,
	.num_resources = 1,
	.resource      = &gpio_vbus_resource,
	.dev	= {
		.platform_data	= &athena_gpio_vbus_info,
	},
};

/*
 * External power
 */

static int power_supply_init(struct device *dev)
{
    printk(KERN_ERR "power_supply_init");
/*    
    printk(KERN_INFO "power_supply F3:%x F5:%x F6:%x F7:%x",gpio_get_value(EGPIO1_F3_HTCATHENA_TBD),gpio_get_value(EGPIO1_F5_HTCATHENA_TBD),gpio_get_value(EGPIO1_F6_HTCATHENA_ISP1582_RELATED) ,gpio_get_value(EGPIO1_F7_HTCATHENA_TV_OUT));
    printk(KERN_INFO "power_supply G0:%x G1:%x G2:%x G3:%x",gpio_get_value(EGPIO1_G0_HTCATHENA_BATTERY),gpio_get_value(EGPIO1_G1_HTCATHENA_TBD),gpio_get_value(EGPIO1_G2_HTCATHENA_TBD) ,gpio_get_value(EGPIO1_G3_HTCATHENA_TBD));
    printk(KERN_INFO "power_supply G4:%x G5:%x G6:%x G7:%x",gpio_get_value(EGPIO1_G4_HTCATHENA_TBD),gpio_get_value(EGPIO1_G5_HTCATHENA_ISP1582_CPLD2),gpio_get_value(EGPIO1_G6_HTCATHENA_TBD) ,gpio_get_value(EGPIO1_G7_HTCATHENA_TBD));
 */ 
             
    post_init();
         
    return gpio_get_value(EGPIO1_F4_HTCATHENA_USB20_CABLE_PLUGGED);
	
   
}

static int athena_is_usb_online(void)

{  
	int ret;
	ret = gpio_get_value(EGPIO1_F4_HTCATHENA_USB20_CABLE_PLUGGED);
	if (ret)
		{// athena_is_usb_online 		
		gpio_set_value(EGPIO1_C0_HTCATHENA_CHARGING_LED_RED, 1);
		gpio_set_value(EGPIO1_C1_HTCATHENA_CHARGING_LED_GREEN, 1);	
		gpio_set_value(EGPIO1_A3_HTCATHENA_BATTERY_RELATED, 0);	
		}
	else
		{ // athena_is_usb_offline 	
		gpio_set_value(EGPIO1_A3_HTCATHENA_BATTERY_RELATED, 1);	
		gpio_set_value(EGPIO1_C0_HTCATHENA_CHARGING_LED_RED, 0);
		gpio_set_value(EGPIO1_C1_HTCATHENA_CHARGING_LED_GREEN, 0);		
		}	
	return ret;
}

static void power_supply_exit(struct device *dev)
{
	return;
	
}

static char *athena_supplicants[] = {
	"main-battery",   
};

static struct pda_power_pdata power_supply_info = {
	.init            = power_supply_init,
	.is_usb_online    = athena_is_usb_online,
	.exit            = power_supply_exit,
	.supplied_to     = athena_supplicants,
	.num_supplicants = ARRAY_SIZE(athena_supplicants),
};

static struct resource power_supply_resources[] = {
	[0] = {
		.name  = "usb",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE |
		         IORESOURCE_IRQ_LOWEDGE,
		.start =  ATHENA_USB_CABLE_IRQ,
		.end   =  ATHENA_USB_CABLE_IRQ,
	},

};

static struct platform_device power_supply = {
	.name = "pda-power",
	.id   = -1,
	.dev  = {
		.platform_data = &power_supply_info,
	},
	.resource      = power_supply_resources,
	.num_resources = ARRAY_SIZE(power_supply_resources),
};

/*
****************************************************************
 * Battery monitoring
 ****************************************************************/





/*
 * Battery charger - tps65023 regulator is used  
 ****************************************************************/
	
static struct regulator_consumer_supply athena_tps65021_consumers[] = {

	REGULATOR_SUPPLY("vbus_draw", NULL),
	REGULATOR_SUPPLY("vcc_core", NULL),	
	REGULATOR_SUPPLY("Vdd", "0-001d"),
};

	
static struct regulator_consumer_supply athena_lis3lv02d_consumers[] = {
	
		REGULATOR_SUPPLY("Vdd_IO", "0-001d"), // from 1.7v to vdd
	
};



static struct regulator_init_data athena_tps65021_info[] = {
	{
		.constraints = {
			.name		= "vcc_core range", 
			.min_uV		= 1350000,
			.max_uV		= 1705000,
			.always_on	= 1,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies	= athena_tps65021_consumers,
		.num_consumer_supplies	= ARRAY_SIZE(athena_tps65021_consumers),
	}, {
		.constraints = {
			.name		= "DCDC2",
			.min_uV		= 3300000,
			.max_uV		= 3300000,
			.always_on	= 1,
		},

	}, {
		.constraints = {
			.name		= "DCDC3",
            .min_uV     = 800000,
            .max_uV     = 3300000,
			.always_on	= 1,
		},
		.consumer_supplies	= athena_lis3lv02d_consumers,
		.num_consumer_supplies	= ARRAY_SIZE(athena_lis3lv02d_consumers),
	}, {
		.constraints = {
			.name		= "LDO1",
			.min_uV		= 1000000,
			.max_uV		= 3150000,
			.always_on	= 1,
		},
	}, {
		.constraints = {
			.name		= "LDO2",
			.min_uV		= 1050000,
			.max_uV		= 3300000,
			.always_on	= 1,
		},
	}
};


/****************************************************************
 * PHONE
 ****************************************************************/

struct platform_device dpram = {
	.name	= "dpram-device",
	.id	= -1,
};


/****************************************************************
 * I2C CAMERA
 ****************************************************************/

/*
 * 
 * GPIO73_HTCATHENA_CAMERA   // main cam pwr
 * GPIO84_HTCATHENA_CAM_LED
 * GPIO86_HTCATHENA_CAMERA	// can be reset
 * GPIO87_HTCATHENA_CAMERA_POWER	// front camera 1 main 0
 * EGPIO1_A7_HTCATHENA_CAM_RELATED   //no reset should be sda
 * EGPIO1_B0_HTCATHENA_CAM_RELATED	//can be reset scl
 * 
 * 
 * 
 * */

static int front_cam_init1(void)
{
	int err;

	/*
	 * GPIO50_nCAM_EN is active low
	 * GPIO19_GEN1_CAM_RST is active on rising edge
	 */
/*	err = gpio_request(GPIO87_HTCATHENA_CAMERA_POWER, "nCAM_EN"); //86,73 works
	if (err) {
		pr_err("%s: Failed to request nCAM_EN\n", __func__);
		goto fail;
	}

	err = gpio_request(GPIO86_HTCATHENA_CAMERA, "CAM_RST");
	if (err) {
		pr_err("%s: Failed to request CAM_RST\n", __func__);
		goto fail_gpio_cam_rst;
	}*/
        gpio_set_value(GPIO87_HTCATHENA_CAMERA_POWER, 1);
        mdelay(50);
        gpio_set_value(GPIO73_HTCATHENA_CAMERA, 0);
//	gpio_direction_output(GPIO73_HTCATHENA_CAMERA, 1);
//	gpio_direction_output(EGPIO1_B0_HTCATHENA_CAM_RELATED, 0);

	return 0;

fail_gpio_cam_rst:
	gpio_free(GPIO87_HTCATHENA_CAMERA_POWER);
fail:
	return err;
}

static int front_camera_power(struct device *dev, int on)
{
	gpio_set_value(GPIO87_HTCATHENA_CAMERA_POWER, !on);
        mdelay(50);
	return 0;
}

static int front_camera_reset(struct device *dev)
{
	gpio_set_value(GPIO86_HTCATHENA_CAMERA, 0);
	msleep(10);
	gpio_set_value(GPIO86_HTCATHENA_CAMERA, 1);

	return 0;
}

/*
* Here we request the camera GPIOs and configure them. We power up the camera
 * module, deassert the reset pin, but put it into powerdown (low to no power
 * consumption) mode. This allows up later to bring the module up fast. */

static inline void __init front_cam_init(void)
{
	if (gpio_request(GPIO87_HTCATHENA_CAMERA_POWER, "Camera PWDN"))
		goto err1;
	if (gpio_request(GPIO86_HTCATHENA_CAMERA, "Camera RESET"))
		goto err2;
	if (gpio_request(GPIO73_HTCATHENA_CAMERA, "Camera DVDD"))
		goto err3;
	if (gpio_direction_output(GPIO87_HTCATHENA_CAMERA_POWER, 1))
		goto err4;
	if (gpio_direction_output(GPIO86_HTCATHENA_CAMERA, 0))
		goto err4;
	if (gpio_direction_output(GPIO73_HTCATHENA_CAMERA, 0))
		goto err4;
	return;

err4:
	gpio_free(GPIO73_HTCATHENA_CAMERA);
err3:
	gpio_free(GPIO86_HTCATHENA_CAMERA);
err2:
	gpio_free(GPIO87_HTCATHENA_CAMERA_POWER);
err1:
	
	return;
}



static void camera_power(int val)
{
	gpio_set_value(GPIO87_HTCATHENA_CAMERA_POWER, val); /* RST_CAM/RSTB */
	mdelay(10);
}
#ifdef CONFIG_I2C
/* support for the old ncm03j camera */
static unsigned char camera_ncm03j_magic[] =
{
	0x87, 0x00, 0x88, 0x08, 0x89, 0x01, 0x8A, 0xE8,
	0x1D, 0x00, 0x1E, 0x8A, 0x21, 0x00, 0x33, 0x36,
	0x36, 0x60, 0x37, 0x08, 0x3B, 0x31, 0x44, 0x0F,
	0x46, 0xF0, 0x4B, 0x28, 0x4C, 0x21, 0x4D, 0x55,
	0x4E, 0x1B, 0x4F, 0xC7, 0x50, 0xFC, 0x51, 0x12,
	0x58, 0x02, 0x66, 0xC0, 0x67, 0x46, 0x6B, 0xA0,
	0x6C, 0x34, 0x7E, 0x25, 0x7F, 0x25, 0x8D, 0x0F,
	0x92, 0x40, 0x93, 0x04, 0x94, 0x26, 0x95, 0x0A,
	0x99, 0x03, 0x9A, 0xF0, 0x9B, 0x14, 0x9D, 0x7A,
	0xC5, 0x02, 0xD6, 0x07, 0x59, 0x00, 0x5A, 0x1A,
	0x5B, 0x2A, 0x5C, 0x37, 0x5D, 0x42, 0x5E, 0x56,
	0xC8, 0x00, 0xC9, 0x1A, 0xCA, 0x2A, 0xCB, 0x37,
	0xCC, 0x42, 0xCD, 0x56, 0xCE, 0x00, 0xCF, 0x1A,
	0xD0, 0x2A, 0xD1, 0x37, 0xD2, 0x42, 0xD3, 0x56,
	0x5F, 0x68, 0x60, 0x87, 0x61, 0xA3, 0x62, 0xBC,
	0x63, 0xD4, 0x64, 0xEA, 0xD6, 0x0F,
};

static int camera_probe(void)
{
	struct i2c_adapter *a = i2c_get_adapter(0);
	struct i2c_msg msg;
	int ret;

	if (!a)
		return -ENODEV;

	camera_power(1);
	msg.addr = 0x6e;
	msg.buf = camera_ncm03j_magic;
	msg.len = 2;
	msg.flags = 0;
	ret = i2c_transfer(a, &msg, 1);
	camera_power(0);

	return ret;
}

static int camera_set_capture(struct soc_camera_platform_info *info,
			      int enable)
{
	struct i2c_adapter *a = i2c_get_adapter(0);
	struct i2c_msg msg;
	int ret = 0;
	int i;

	camera_power(0);
	if (!enable)
		return 0; /* no disable for now */

	camera_power(1);
	for (i = 0; i < ARRAY_SIZE(camera_ncm03j_magic); i += 2) {
		u_int8_t buf[8];

		msg.addr = 0x6e;
		msg.buf = buf;
		msg.len = 2;
		msg.flags = 0;

		buf[0] = camera_ncm03j_magic[i];
		buf[1] = camera_ncm03j_magic[i + 1];

		ret = (ret < 0) ? ret : i2c_transfer(a, &msg, 1);
	}

	return ret;
}

static struct soc_camera_platform_info camera_info = {
	//.iface = 0,
	.format_name = "UYVY",
	.format_depth = 16,
	.format = {
	//	.pixelformat = V4L2_PIX_FMT_UYVY,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.width = 640,
		.height = 480,
	},
	.set_capture = camera_set_capture,
	
	 .mbus_param = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
        V4L2_MBUS_VSYNC_ACTIVE_HIGH | V4L2_MBUS_HSYNC_ACTIVE_HIGH |
        SOCAM_DATAWIDTH_8,
	
	
};

static struct platform_device camera_device = {
	.name		= "soc_camera_platform",
	.dev		= {
		.platform_data	= &camera_info,
	},
};

static int __init camera_setup(void)
{
	if (camera_probe() > 0)
		platform_device_register(&camera_device);

	return 0;
}
late_initcall(camera_setup);

#endif /* CONFIG_I2C */

static int ov7725_power(struct device *dev, int mode)
{
	camera_power(0);
	if (mode)
		camera_power(1);

	return 0;
}
struct pxacamera_platform_data front_pxacamera_platform_data = {
	.flags  = PXA_CAMERA_MASTER | PXA_CAMERA_DATAWIDTH_8 |
		PXA_CAMERA_PCLK_EN | PXA_CAMERA_MCLK_EN,
	.mclk_10khz = 2600,
};


/*static struct ov772x_camera_info ov7725_info = {
	.buswidth  = SOCAM_DATAWIDTH_8,
	.flags = OV772X_FLAG_VFLIP | OV772X_FLAG_HFLIP,
	.link = {
		.power  = ov7725_power,
	},
};*/
static struct i2c_board_info frontcamera_i2c_board_info[] = {
	{
	I2C_BOARD_INFO("ov772x", 0x21),
	},
};

static struct soc_camera_link front_iclink = {
	.bus_id         = 0,/* Match id in pxa27x_device_camera in device.c */
	.flags          = SOCAM_DATAWIDTH_8,
	.i2c_adapter_id = 0,              //right
	.board_info     = &frontcamera_i2c_board_info,
	.power          = front_camera_power,
	.reset          = front_camera_reset,
        .module_name    = "ov772x",
};

static struct i2c_gpio_platform_data frontcamera_i2c_bus_data = {
     .sda_pin = EGPIO1_A7_HTCATHENA_CAM_RELATED,
     .scl_pin = EGPIO1_B0_HTCATHENA_CAM_RELATED,
     .udelay  = 40,
//     .timeout = 100,
};

static struct platform_device frontcamera_i2c_bus_device = {
    .name           = "i2c-gpio",
     .id             = 3, /* we use this as a replacement for i2c-pxa */
     .dev = {
             .platform_data = &front_iclink,
     }
};
static struct platform_device front_camera = {
	.name   = "soc-camera-pdrv",
	.id     = 2,
	.dev    = {
		.platform_data = &front_iclink,
	},
};

static struct i2c_gpio_platform_data tvout_i2c_bus_data = {
     .sda_pin = GPIO118_HTCATHENA_SDA,
     .scl_pin = GPIO117_HTCATHENA_SCL,
     .udelay  = 40,
//     .timeout = 100,
};

static struct platform_device tvout_i2c_bus_device = {   //drivers/gpu/drm/i2c/ch7006
    .name           = "i2c-gpio",
     .id             = 2, /* we use this as a replacement for i2c-pxa */
     .dev = {
             .platform_data = &tvout_i2c_bus_data,
     }
};

static struct i2c_gpio_platform_data sound_i2c_bus_data = {
 //    .sda_pin =  from ATI GPIO,
 //    .scl_pin =  from  ATI GPIO,
     .udelay  = 5,
//     .timeout = 100,
};

static struct platform_device sound_i2c_bus_device = {
    .name           = "i2c-gpio",
     .id             = 6, /* we use this as a replacement for i2c-pxa */
     .dev = {
             .platform_data = &sound_i2c_bus_data,
     }
};

static struct i2c_gpio_platform_data camera_i2c_bus_data = {
     .sda_pin = EGPIO1_A7_HTCATHENA_CAM_RELATED,
     .scl_pin = EGPIO1_B0_HTCATHENA_CAM_RELATED,
     .udelay  = 5,
//     .timeout = 100,
};

static struct platform_device camera_i2c_bus_device = {
    .name           = "i2c-gpio",
     .id             = 5, /* we use this as a replacement for i2c-pxa */
     .dev = {
             .platform_data = &camera_i2c_bus_data,
     }
};
/****************************************************************
 * devices settings on I2C
 ****************************************************************/

//run 'dump pxa27x' while playing sound
static inline void __init sound_init(void)
{        

	gpio_set_value(GPIO67_HTCATHENA_SOUND_RELATED, 1);
        gpio_set_value(GPIO57_HTCATHENA_SOUND, 1);
        mdelay(50);
     //   gpio_set_value(GPIO57_HTCATHENA_SOUND, 0);
        
	return;
};


static struct ak4641_platform_data ak4641_info = {
 .gpio_power = GPIO67_HTCATHENA_SOUND_RELATED, ///GPIO27_HX4700_AK4641_POWER,
 .gpio_npdn = GPIO57_HTCATHENA_SOUND, //GPIO109_HX4700_AK4641_nPDN,
};



static struct platform_device audio = {
 .name = "athena-audio",
 .id = -1,
};

static struct lis3lv02d_platform_data lis302dl_data = {
  .click_flags = 0,
};





/****************************************************************
 * I2C 
 ****************************************************************/




static struct i2c_board_info pxa_i2c_board_info0[] = {
	{
	I2C_BOARD_INFO("lis3lv02d", 0x1d),
	.platform_data = &lis302dl_data,
	},
	  
	
};

static struct i2c_board_info pwr_pxa_i2c_board_info1[] = {
      {
		I2C_BOARD_INFO("tps65021", 0x48),
		.platform_data	= &athena_tps65021_info,
	},
	
	        
};


static struct i2c_board_info gpio_i2c_board_info2[] = {   
    {
	I2C_BOARD_INFO("ch7013", 0x75),
	},

	
};
static struct i2c_board_info athena_i2c_board_info4[] = {   
        {
	I2C_BOARD_INFO("tlv320aic23", 0x1a),
        },
	
};

static struct i2c_board_info sound_i2c_board_info[] = {   
        {
	I2C_BOARD_INFO("ak4641", 0x12),
   //    .platform_data = &ak4641_info,
        },
	
};



static struct i2c_board_info maincamera_i2c_board_info[] = {   
        {
	I2C_BOARD_INFO("s5k3c1fx", 0x5a),
        },
	
};


/****************************************************************
 * USB host controller
 ****************************************************************/

static struct pxaohci_platform_data athena_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
	.flags		= ENABLE_PORT2 | POWER_SENSE_LOW ,
	.power_budget	= 500,  /* mA */
};

/****************************************************************
* SSP
****************************************************************/

static struct spi_board_info ssp_board_info[] __initdata = {
	{
		.modalias	= "ad7877",
		.bus_num	= 1,
		.max_speed_hz	= 12500000,
		.mode		= SPI_MODE_1,
		.irq		= PXA_GPIO_TO_IRQ(GPIO114_HTCATHENA_TS_DAV), 
		.chip_select	= 0,
		.platform_data	= &ts_info,
		.controller_data = &ts_chip,
	},
	{
		.modalias	= "htc-spi-kbd",
		.bus_num	 = 2,
		.max_speed_hz	= 100000,
		.mode		= SPI_MODE_3,
		.irq		= PXA_GPIO_TO_IRQ(GPIO9_HTCATHENA_KEYB_BUTTON_IRQ),
		.chip_select	= 0,
		.platform_data	= &kb_info,
		.controller_data = &kb_chip,
	},
};

static struct pxa2xx_spi_master pxa_ssp1_master_info = {
	.num_chipselect	= 1,
	.clock_enable	= CKEN_SSP1,
	.enable_dma     = 1,
};

static struct pxa2xx_spi_master pxa_ssp2_master_info = {
	.num_chipselect	= 1,
	.clock_enable	= CKEN_SSP2,
	.enable_dma     = 1,

};


static irqreturn_t jack_irq(int irq, void *p)
{
	printk(KERN_INFO  "athena: HeadPhone Jack %s  \n", !gpio_get_value(EGPIO1_F1_EXT_JACK)? "connected" : "disconnected");	
	return IRQ_HANDLED;
}

static int __init jack_init(void)
{
	int ret;

	ret = request_irq(ATHENA_HPEP_W_IRQ, 
	jack_irq, IRQF_TRIGGER_RISING ,
			 "Jack_plugged", NULL);
	if (ret)
		printk(KERN_ERR "Athena: jack error %d\n",-ret);
	return ret;
}

static irqreturn_t hpbutton_irq(int irq, void *p)
{
	printk(KERN_INFO  "athena: HP button %s  \n", gpio_get_value(EGPIO1_F2_HP_EXT)? "pressed" : "unpressed");
	return IRQ_HANDLED;
}

static int __init hpbutton_init(void)
{
	int ret;
	ret = request_irq(ATHENA_HPEXT_B_IRQ, 
	hpbutton_irq, IRQF_TRIGGER_RISING ,
			 "HP button", NULL);
	if (ret)
		printk(KERN_ERR "Athena: hp button error %d\n",-ret);
	return ret;
}

static irqreturn_t tvcable_irq(int irq, void *p)
{
	printk(KERN_INFO  "athena: tv 4-1 cable %s  \n", gpio_get_value(EGPIO1_F7_HTCATHENA_TV_OUT)? "connected" : "disconnected");	
	return IRQ_HANDLED;
}

static int __init tvcable_init(void)
{
	int ret;

	ret = request_irq(ATHENA_TV_OUT_IRQ, 
	tvcable_irq, IRQF_TRIGGER_RISING ,
			 "tvcable_plugged", NULL);
	if (ret)
		printk(KERN_ERR "Athena: tv cable error %d\n",-ret);
	return ret;
}

static int __init post_init(void)
{
	jack_init();
	hpbutton_init();
	tvcable_init();
}

/****************************************************************
 * Keyboard on/off
 ****************************************************************/


static irqreturn_t keyboard_on_irq(int irq, void *p)
{
	if (gpio_get_value(GPIO10_HTCATHENA_KEYBOARD_IO))
		gpio_set_value(GPIO69_HTCATHENA_KBD_POWER, 0);  /* Keyboard disabling */
	 else 
		gpio_set_value(GPIO69_HTCATHENA_KBD_POWER , 1);/* Keyboard enabling */		
	return IRQ_HANDLED;
}

static int __init keyboard_init(void)
{
	int ret;

	ret = gpio_request(GPIO69_HTCATHENA_KBD_POWER, "KBD_POWER");
	if (ret) {
		gpio_free(GPIO69_HTCATHENA_KBD_POWER);
		printk(KERN_ERR "Kbd Power GPIO error %x \n", ret);
		return ret;
		}

	ret = request_irq( PXA_GPIO_TO_IRQ(GPIO10_HTCATHENA_KEYBOARD_IO), 
	 keyboard_on_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			 "Athena keyboard connection", NULL);
	if (ret)
		printk(KERN_ERR "Athena: kb irq error %d\n",-ret);
	return ret;
}

static void keyboard_exit(void)
{
	free_irq(PXA_GPIO_TO_IRQ(GPIO10_HTCATHENA_KEYBOARD_IO), NULL);
}


/****************************************************************
 * Athena platform Initlialization
 ****************************************************************/
 
 
 static void __init athena_reserve(void)
{
memblock_reserve(0xa0000000, 0x1000);  // SPL
memblock_reserve(0xa2000000, 0x1000);
}
 
 

static struct platform_device *devices[] __initdata = {
	&egpio_cpld1,
	&egpio_cpld2,
	&w2284,
	&gpio_keys,
	&s1r72v03_ide_device,
	&backlight,
	&leds_gpio,
	
	//&audio,
    //&sound_i2c_bus_device,
    //&front_camera,
    //&frontcamera_i2c_bus_device,
    &tvout_i2c_bus_device,
    &dpram,
	
	
	&athena_gpio_vbus,
    &athena_isp1582_device,
    &athena_atiw_mmc,
    &power_supply,
    &docg4_flash,
	&athena_rfkill_bt,
	&athena_rfkill_gps,
	&athena_rfkill_wlan,
	&athena_wl1251_data,
};


static struct gpio athena_global_gpios[] = {
	
//	{ GPIO0_HTCATHENA_KEY_POWER,   GPIOF_INIT_HIGH, "GPIO0" },
//	{ GPIO3_HTCATHENA_TBD,   GPIOF_INIT_HIGH, "GPIO3" },
//	{ GPIO4_HTCATHENA_TBD,   GPIOF_INIT_HIGH, "GPIO4" },	
//	{ GPIO14_HTCATHENA_CPLD1_EXT_INT,   GPIOF_IN, "CPLD_IRQ" },
	{ GPIO56_HTCATHENA_GPS,   GPIOF_INIT_LOW, "GPIO56" },   // needed
//	{ GPIO70_HTCATHENA_FLASHLED,   GPIOF_INIT_LOW, "GPIO70" }, 
//	{ GPIO74_HTCATHENA_TBD,   GPIOF_INIT_LOW, "GPIO74" },
//	{ GPIO76_HTCATHENA_TBD,   GPIOF_INIT_HIGH, "GPIO76" },
//	{ GPIO77_HTCATHENA_TBD,   GPIOF_INIT_HIGH, "GPIO77" },
//	{ GPIO97_HTCATHENA_TBD,   GPIOF_INIT_HIGH, "GPIO97" },
//	{ GPIO108_HTCATHENA_TBD,   GPIOF_INIT_HIGH, "GPIO108" },
};

static void __init athena_init(void)
{
    int ret;	
	pxa2xx_mfp_config(ARRAY_AND_SIZE(athena_pin_config));	
	ret = gpio_request_array(ARRAY_AND_SIZE(athena_global_gpios));
	if (ret)
		printk(KERN_ERR "Athena: Failed to request GPIOs: %d\n",ret);

	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);
	
	keyboard_init();
	athena_wl1251_init();
	
	pxa_set_ohci_info(&athena_ohci_info);
	pxa27x_set_i2c_power_info(NULL);
	pxa_set_i2c_info(NULL);

	/* G-Sensor accelero and regulator */
	
	i2c_register_board_info(0 , ARRAY_AND_SIZE(pxa_i2c_board_info0));
    i2c_register_board_info(1 , ARRAY_AND_SIZE(pwr_pxa_i2c_board_info1));
    i2c_register_board_info(2 , ARRAY_AND_SIZE(gpio_i2c_board_info2));
 
 
    //sound_init();
    //front_cam_init1();
	//pxa_set_camera_info(&front_pxacamera_platform_data);
    //  i2c_register_board_info(2 , ARRAY_AND_SIZE(frontcamera_i2c_board_info));   
    //  i2c_register_board_info(5 , ARRAY_AND_SIZE(maincamera_i2c_board_info));
    //i2c_register_board_info(6 , ARRAY_AND_SIZE(sound_i2c_board_info));  
    

	
//	pxa_set_mci_info(&athena_mci_info);
	
	platform_add_devices(devices, ARRAY_SIZE(devices));

	pxa2xx_set_spi_info(1, &pxa_ssp1_master_info);  /* TS */
	pxa2xx_set_spi_info(2, &pxa_ssp2_master_info);  /* Kb */
	spi_register_board_info(ARRAY_AND_SIZE(ssp_board_info));
	
	
}

MACHINE_START(HTCATHENA, "HTC Athena")
	.atag_offset 	= 0x100,
	.map_io       	= pxa27x_map_io,
	.reserve		= athena_reserve,
	.nr_irqs      	= ATHENA_NR_IRQS,
	.init_irq     	= pxa27x_init_irq,
	.handle_irq     = pxa27x_handle_irq,
	.init_machine	= athena_init,
	.timer          = &pxa_timer,
	.restart	= pxa_restart,
MACHINE_END
