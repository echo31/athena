/** 
 * @file ch7013.c 
 * @brief Source file for CH7013B tvout driver. 
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 * 
 * Copyright (C) 2006 Sitek Hengke Electronics Wuxi. 
 *                              http://www.sitek.cn 
 * Modification history: 
 *  
 * Nov.  29, 2006.  Gary 
 *  Initial version of this file. 
 *  
 */  
 
#include <linux/kernel.h>  
#include <linux/module.h>  
#include <linux/mm.h>  
#include <linux/slab.h>  
#include <linux/init.h>  
#include <linux/i2c.h>  
//#include <linux/i2c-algo-bit.h>  
//#include <linux/i2c-id.h>  
//#include <linux/poll.h>  
//#include <linux/miscdevice.h>  
#include <linux/proc_fs.h>  
#include <linux/delay.h>  
#include <asm/types.h>  
#include <mach/hardware.h>  
#include <mach/pxa2xx-regs.h>  
#include <mach/regs-lcd.h> 

   
//#include "tvout.h"  
#include "ch7013.h"  
  
//extern unsigned int pxa27x_set_cpufreq(unsigned int, unsigned int, unsigned int, unsigned int, unsigned int);      
//extern unsigned int pxa27x_get_clk_frequency_khz(int info); 
//extern unsigned int pxa27x_get_lcdclk_frequency_10khz(void);
//extern unsigned  get_memclk_frequency_10khz(void); 
  
#define MODULE_NAME "tvout"  
static int g_tvout_major = 0;  
static struct lcdc_regs_t g_lcdc_regs;  
  
/************************************************************************************************* 
    i2c related 
*************************************************************************************************/  
  
#define I2C_DRIVERID_I2CCH7013 0xF000  
#define I2C_M_READ 0x01  
#define I2C_M_WT 0x02  
  
//CH7013 i2c addr  
//ADDR   Serial Address Selected  
//  1        111 0101 = 75h          //high: 0xEA(write), 0xEB(read)  
//  0        111 0110 = 76h      // low:  0xEC(write), 0xED(read)  
  
#define CH7013_I2C_ADDR  0x76   //0xEC  
  
  
#if 0 

void i2c_ch7013_init(void)  
{  
    /*  
     * set the address of the CH7013 to the client 
     */  
    i2c_ch7013_client.addr = CH7013_I2C_ADDR;  
  
    /*  
     * call the i2c_add_driver() to register the driver  
     * to the Linux I2C system 
     */  
    if (i2c_add_driver( &i2c_ch7013_driver ) != 0) {  
        printk("I2C add driver failed\n");  
        return;  
    }  
    /*  
     * attach the client to the adapter by calling the i2c_attach_client()  
     * function of the Linux I2C system 
     */  
    if (i2c_attach_client(&i2c_ch7013_client ) != 0) {    
        printk("I2C attach client failed\n");  
        i2c_del_driver(&i2c_ch7013_driver);  
        return;  
    }  
}  
  
  
void i2c_ch7013_cleanup(void)  
{  
    i2c_detach_client(&i2c_ch7013_client );  
    i2c_del_driver(&i2c_ch7013_driver);  
}  
#endif

#define TVOUT_FMT_OFF   0
#define TVOUT_FMT_NTSC  1
#define TVOUT_FMT_PAL   2

static int enabled;           /* enable power on or not */
static int pm_status;         /* status before suspend */

static struct i2c_client *ch7013_client;
static struct fb_info *ch7013_fbi;
static int ch7013_cur_mode;
static u32 detect_gpio;
static struct regulator *io_reg;
static struct regulator *core_reg;
static struct regulator *analog_reg;

//static void hp_detect_wq_handler(struct work_struct *);
//DECLARE_DELAYED_WORK(ch7013_wq, hp_detect_wq_handler);

  static inline int ch7013_read_reg(u8 reg)
{
      return i2c_smbus_read_byte_data(ch7013_client, reg);
}

static inline int ch7013_write_reg(u8 reg, u8 word)
{
      return i2c_smbus_write_byte_data(ch7013_client, reg, word);
}

#if 0
/** 
 * @brief Write one byte to CH7013 register 
 * 
 * @param reg   CH7013 register address 
 * @param data  Data to be written to the register 
 */  
static int i2c_ch7013_write(unsigned int reg, unsigned char data)  
{  
    struct i2c_msg msg;  
    char buf[2];  
    int ret;  
  
    /*  
     * store the register value to the first address of the buffer  
     * the adapter/algorithm driver will regard the first byte  
     * as the register value  
     */  
    buf[0] = (char)reg;  
    buf[1] = data;  
          
    /*  
     * initialize the message structure 
     */  
  //  msg.addr = i2c_ch7013_client.addr;  
    msg.flags = I2C_M_WT;  
    msg.len = 2;  
    msg.buf = buf;  
      
    ret = i2c_transfer( i2c_ch7013_client.adapter, &msg, 1 );  
  
    return ret;  
}  
  
static int i2c_ch7013_read(unsigned int reg_offset, unsigned char *buf)  
{  
    struct i2c_msg msg[2]  = {  
        { i2c_ch7013_client.addr, 0,        1, ®_offset },  
        { i2c_ch7013_client.addr, I2C_M_RD, 1, buf }  
    };  
  
    if (i2c_transfer( i2c_ch7013_client.adapter, msg, 2 ) == 2)  
        return 2;  
    else  
        return 0;  
}  

#endif
/************************************************************************************************* 
    End of i2c related 
*************************************************************************************************/  
  
/////////////////////////////////////////////////////////////////////  
//TVOUT CH7013 Part  
////////////////////////////////////////////////////////////////////  
//static int ch7013_program(void);  
//static void ch7013_hw_init(void);  
//static void ch7013_hw_exit(void);  
//static void ch7013_save_lcdc(void);  
  
static struct ch7013_display_mode ctable[] =  
{  
    {0x60, 0x2A, 0x50, 0x74, 0x31, 0x81, 0x18, 0x2D, 0x40,  0x41, 0x80, 0x300AE7C4, 0x10, 0x01},     // default
    {0x60, 0x2A, 0x50, 0x74, 0x31, 0x81, 0x18, 0x2D, 0x40,  0x0D, 0x14, 0x300AE7C4, 0x10, 0x00},    //display mode 13  
    {0x61, 0x2A, 0x50, 0x44, 0x31, 0x81, 0x36, 0x2C, 0x40,  0x04, 0x09, 0x266F1FD0, 0x10, 0x00},    //display mode 14  
    {0x69, 0x2A, 0x50, 0x6A, 0x30, 0x81, 0x1F, 0xF9, 0x40,  0x3F, 0x6E, 0x25249249, 0x00, 0x00},    //display mode 16  
    {0x6A, 0x2A, 0x50, 0x64, 0x30, 0x81, 0x2D, 0xFD, 0x40,  0x3F, 0x7E, 0x20800000, 0x00, 0x00},    //display mode 17  
};  
  
static int ch7013_program(void)  
{  
    int idx;  
 //   unsigned char chip_ver;   
      
    idx = 3;  
    ch7013_write_reg(REG_PMR, 0x00);           //soft reset  
    mdelay(1);  
    ch7013_write_reg(REG_PMR, 0x0B);           //all are active  
    mdelay(1);  
    printk("Chip version ID(0x22)= 0x%X \n", ch7013_read_reg(REG_VID));  
    
    return 0;
  
    ch7013_write_reg(REG_DMR, ctable[idx].DMR);    //display mode  
    ch7013_write_reg(REG_FFR, ctable[idx].FFR);    //flicker filter settings  
    ch7013_write_reg(REG_IDF, 0x00);                        //IDF Input data format         16bit 565  
    ch7013_write_reg(REG_CM, ctable[idx].CM); //clock mode  
    ch7013_write_reg(REG_SAV, ctable[idx].SAV);    //start of active video register  
    ch7013_write_reg(REG_PO, ctable[idx].PO); //position overflow  
    ch7013_write_reg(REG_BLR, ctable[idx].BLR);    //black level  
    ch7013_write_reg(REG_HPR, ctable[idx].HPR);    //horizontal position  
    ch7013_write_reg(REG_VPR, ctable[idx].VPR);    //vertical position  
    ch7013_write_reg(REG_MNE, ctable[idx].MNE);    //PLLM and PLLN  
    ch7013_write_reg(REG_PLLM, ctable[idx].PLLM);  
    ch7013_write_reg(REG_PLLN, ctable[idx].PLLN);  
    ch7013_write_reg(REG_PLLC, ctable[idx].PLLC);   //PLL control  
    ch7013_write_reg(REG_CIVC, ctable[idx].CIVC);   //turn off CIV  
    ch7013_write_reg(REG_FSI0, (ctable[idx].FSCI & 0xF0000000) >> 28);  
    ch7013_write_reg(REG_FSI1, (ctable[idx].FSCI & 0x0F000000) >> 24);  
    ch7013_write_reg(REG_FSI2, (ctable[idx].FSCI & 0x00F00000) >> 20);  
    ch7013_write_reg(REG_FSI3, (ctable[idx].FSCI & 0x000F0000) >> 16);  
    ch7013_write_reg(REG_FSI4, (ctable[idx].FSCI & 0x0000F000) >> 12);  
    ch7013_write_reg(REG_FSI5, (ctable[idx].FSCI & 0x00000F00) >> 8);  
    ch7013_write_reg(REG_FSI6, (ctable[idx].FSCI & 0x000000F0) >> 4);  
    ch7013_write_reg(REG_FSI7, ctable[idx].FSCI & 0x0000000F);  
      
    return 0;  
}  

#if 0
  @@ -925,6 +925,8 @@ static int __devinit pxafb_overlay_init(struct
> >> pxafb_info *fbi)
> >> 
> >>         /* place overlay(s) on top of base */
> >>         fbi->lccr0 |= LCCR0_OUC;
> >> +       lcd_writel(fbi, LCCR0, fbi->lccr0 & ~LCCR0_ENB);
> >> +
> >>         pr_info("PXA Overlay driver loaded successfully!\n");
> >>         return 0;
//This function config LCDC registers to fit TVOUT output.  
static void ch7013_set_lcdc(void)  
{  
    unsigned int reg_lccr0;  
    unsigned int reg_lccr1;  
    unsigned int reg_lccr2;  
    unsigned int reg_lccr3;  
    unsigned int PCD;  
      
    LCCR0 = 0;  
    LCCR1 = 0;  
    LCCR2 = 0;  
    LCCR3 = 0;  
    LCCR4 = 0;  
    LCCR5 = 0x3f3f3f3f;  
  
    g_lcdc_regs.fsadr0 = FSADR0;  
    FDADR0 = g_lcdc_regs.fdadr0;  
    FIDR0 = g_lcdc_regs.fidr0;  
    LDCMD0 = g_lcdc_regs.ldcmd0;  
      
    // Configure the LCD Controller Control Registers  
    PCD = 2;  
      
    reg_lccr0 = (LCCR0_BM | LCCR0_PAS | \  
                      LCCR0_EFM | LCCR0_IUM | LCCR0_SFM | LCCR0_LDM);  
    LCCR0 = reg_lccr0;    
  
     reg_lccr1 = (LCCR1_DisWdth(640) | LCCR1_HorSnchWdth(0x35) | LCCR1_EndLnDel(0x10)  | LCCR1_BegLnDel(0x4B));  
     LCCR1 = reg_lccr1;   
  
    reg_lccr2 = (LCCR2_DisHght(480) | LCCR2_EndFrmDel(0x30) | LCCR2_EndFrmDel(0x22)  | LCCR2_BegFrmDel(0x20));     
    LCCR2 = reg_lccr2;   
   
    reg_lccr3 = (LCCR3_PixClkDiv(PCD)  | LCCR3_Bpp(4) |LCCR3_OEP|LCCR3_PCP | LCD_PDFOR(PDFOR_00));   
    LCCR3 = reg_lccr3;     
    LCCR4 = 0x80000000;  
      
    // enable LCD Controller  
    LCCR0 |= LCCR0_ENB;  
      
    //printk("lccr0: 0x%08x \n", LCCR0);  
    //printk("lccr1: 0x%08x \n", LCCR1);  
    //printk("lccr2: 0x%08x \n", LCCR2);  
    //printk("lccr3: 0x%08x \n", LCCR3);  
    //printk("lccr4: 0x%08x \n", LCCR4);  
}  
  
  
static void ch7013_regs_dump(void)  
{  
    unsigned int k;  
    unsigned char v;  
      
    for (k = 0x00; k = 0x29; k++) {  
        i2c_ch7013_read(k, &v);  
        printk("REG:%x = 0x%x \n", k, v);  
    }  
}  
#endif  
  
//This function modify PXA27x CPU Clock and LCDC Clock  



static void ch7013_change_cpufreq(void)  
{  
    //unsigned int CCCR_L;  
    //int LCLK = 0;  
    unsigned int l = 26;  
    unsigned int n2 = 3;  
    unsigned int t = 2;   
    unsigned int b = 0;  
    unsigned int a = 1;  
    unsigned int memclk, lcdclk;  
  
      
    //printk("*******************First**************************\n");  
    //get_clk_frequency_khz(1);  
    //memclk = get_memclk_frequency_10khz();  
    //lcdclk = get_lcdclk_frequency_10khz();  
    //printk("memclk = %d,  lcdclk = %d\n", memclk, lcdclk);  
  
   // ch7013_save_lcdc();  
  
    // 1. turn off LCDC clock before change   
   // CKEN &= (~0x10000);  
      
    // 2. change CPU clock  
 //   pxa27x_set_cpufreq(l, n2, b, t, a);  
  
    // 3. turn on LCDC clock  
    // CKEN |= 0x110000;  
      
    //printk("********************Second*************************\n");  
  //  pxa27x_get_clk_frequency_khz(1);  
  //  memclk = get_memclk_frequency_10khz();  
  //  lcdclk = pxa27x_get_lcdclk_frequency_10khz();  
    printk("memclk = %d,  lcdclk = %d\n", memclk, lcdclk);  
      
    return;  
}  
  
  
/** 
 * @brief Save the register values/status that might be changed by tvout 
 */  
/*static void ch7013_save_lcdc(void)  
{  
    g_lcdc_regs.lccr0 = LCCR0;  
    g_lcdc_regs.lccr1 = LCCR1;  
    g_lcdc_regs.lccr2 = LCCR2;  
    g_lcdc_regs.lccr3 = LCCR3;  
    g_lcdc_regs.fsadr0 = FSADR0;  
    g_lcdc_regs.fdadr0 = FDADR0;  
    g_lcdc_regs.fidr0 = FIDR0;  
    g_lcdc_regs.ldcmd0 = LDCMD0;  
      
}  
  */


static int ch7013_open(struct inode * inode, struct file * filp)  
{  
    return 0;  
}  
  
static int ch7013_release(struct inode * inode, struct file * filp)  
{  
    return 0;  
}  
  
static int ch7013_ioctl(struct inode * inode, struct file *filp, u_int cmd, u_long arg)  
{  
    int ret = 0;  
      
    return ret;  
}  
  
struct file_operations g_tvout_fops = {  
    .owner=      THIS_MODULE,  
    .unlocked_ioctl=      ch7013_ioctl,  
    .open=       ch7013_open,  
    .release=        ch7013_release,  
};  
   

static int __devinit ch7013_probe(struct i2c_client *client,
					const struct i2c_device_id *id)  
{  
	

    printk(KERN_INFO "PXA27x CH7013 tvout driver.\n"); 
  
    g_tvout_major = register_chrdev( 0, MODULE_NAME, &g_tvout_fops);  
  if (g_tvout_major<0)
	{
        printk("Unable to register %s driver.\n",MODULE_NAME);  
        remove_proc_entry(MODULE_NAME, NULL);  
        return -ENODEV;  
    } else {  
        printk("/dev/tvout major device number is: %d\n", g_tvout_major);  
    }  
    ch7013_client = client;
  //  ch7013_change_cpufreq();        
  //  ch7013_set_lcdc(); 
  
	/*if (i2c_client->adapter == NULL) return -EINVAL;
		i2c_client->addr = 0x12;
		if (i2c_smbus_xfer(i2c_client->adapter, i2c_client->addr,
		0, 0, 0, I2C_SMBUS_QUICK, NULL) < 0)
		printk(KERN_INFO"error ch7013 I2C driver \n");*/
  
   
      
    mdelay(10);  
    ch7013_program();    
      
    printk("%s driver init successfully.\n",MODULE_NAME);  
    
  
    //ch7013_i2c_test();  
    //ch7013_regs_dump();  
      
    return 0;  
}  
  
static int __devexit ch7013_remove(struct i2c_client *client)  
{  
   ch7013_client=NULL;
  
    if (g_tvout_major > 0) {  
        unregister_chrdev(g_tvout_major, MODULE_NAME);  
        
    }  
} 



static const struct i2c_device_id ch7013_id[] = {
	{
	.name= "ch7013", 0 },
	{}
};

 
MODULE_DEVICE_TABLE(i2c, ch7013_id);

static struct i2c_driver i2c_ch7013_driver = {
	.probe = ch7013_probe,
	.remove = ch7013_remove,
	.driver = {
		.name = "ch7013",
	},
	.id_table = ch7013_id,
};
  

static int __init ch7013_init(void)
{
		
	return i2c_add_driver(&i2c_ch7013_driver);
}



static void __exit ch7013_exit(void)
{
	i2c_del_driver(&i2c_ch7013_driver);
}

module_init(ch7013_init);
module_exit(ch7013_exit);

  
MODULE_DESCRIPTION("PXA270 TVOUT CH7013 Driver ");  
MODULE_LICENSE("GPL");  
