/*
 * vsfb.h
 *
 * (c) 2006 Ian Molton <spyro@f2s.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _W2284FB_H
#define _W2284FB_H

#define W2284_GPIO_PORT_A	0
#define W2284_GPIO_PORT_B	1

#define CLK_SRC_XTAL  0
#define CLK_SRC_PLL   1

struct w2284fb_par;

unsigned long w2284fb_gpio_read(int port);
void w2284fb_gpio_write(int port, unsigned long value);
unsigned long w2284fb_get_hsynclen(struct device *dev);

/* LCD Specific Routines and Config */
struct w2284_tg_info {
	void (*change)(struct w2284fb_par*);
	void (*suspend)(struct w2284fb_par*);
	void (*resume)(struct w2284fb_par*);
};

/* General Platform Specific w2284 Register Values */
struct w2284_gen_regs {
	unsigned long lcd_format;
	unsigned long lcdd_cntl1;
	unsigned long lcdd_cntl2;
	unsigned long genlcd_cntl1;
	unsigned long genlcd_cntl2;
	unsigned long genlcd_cntl3;
};

struct w2284_gpio_regs {
	unsigned long init_data1;
	unsigned long init_data2;
	unsigned long gpio_dir1;
	unsigned long gpio_oe1;
	unsigned long gpio_dir2;
	unsigned long gpio_oe2;
};

/* Optional External Memory Configuration */
struct w2284_mem_info {
	unsigned long ext_cntl;
	unsigned long sdram_mode_reg;
	unsigned long ext_timing_cntl;
	unsigned long io_cntl;
	unsigned int size;
};

struct w2284_bm_mem_info {
	unsigned long ext_mem_bw;
	unsigned long offset;
	unsigned long ext_timing_ctl;
	unsigned long ext_cntl;
	unsigned long mode_reg;
	unsigned long io_cntl;
	unsigned long config;
};

/* LCD Mode definition */
struct w2284_mode {
	unsigned int xres;
	unsigned int yres;
	unsigned short left_margin;
	unsigned short right_margin;
	unsigned short upper_margin;
	unsigned short lower_margin;
	unsigned long crtc_ss;
	unsigned long crtc_ls;
	unsigned long crtc_gs;
	unsigned long crtc_vpos_gs;
	unsigned long crtc_rev;
	unsigned long crtc_dclk;
	unsigned long crtc_gclk;
	unsigned long crtc_goe;
	unsigned long crtc_ps1_active;
	char pll_freq;
	char fast_pll_freq;
	int sysclk_src;
	int sysclk_normal_divider;
        int sysclk_fast_divider;
        int sysclk_turbo_divider;
	int pixclk_src;
	int pixclk_divider;
	int pixclk_divider_rotated;
};

struct w2284_pll_info {
	uint16_t freq;  /* desired Fout for PLL (Mhz) */
	uint8_t M;      /* input divider */
	uint8_t N_int;  /* VCO multiplier */
	uint8_t N_fac;  /* VCO multiplier fractional part */
	uint8_t tfgoal;
	uint8_t lock_time;
};

/* Initial Video mode orientation flags */
#define INIT_MODE_ROTATED  0x1
#define INIT_MODE_FLIPPED  0x2

/*
 * This structure describes the machine which we are running on.
 * It is set by machine specific code and used in the probe routine
 * of drivers/video/w2284fb.c
 */
struct w2284fb_mach_info {
	/* General Platform Specific Registers */
	struct w2284_gen_regs *regs;
	/* Table of modes the LCD is capable of */
	struct w2284_mode *modelist;
	unsigned int num_modes;
	/* Hooks for any platform specific tg/lcd code (optional) */
	struct w2284_tg_info *tg;
	/* External memory definition (if present) */
	struct w2284_mem_info *mem;
	/* Additional External memory definition (if present) */
	struct w2284_bm_mem_info *bm_mem;
	/* GPIO definitions (optional) */
	struct w2284_gpio_regs *gpio;
	/* Initial Mode flags */
	unsigned int init_mode;
	/* Xtal Frequency */
	unsigned int xtal_freq;
	/* Enable Xtal input doubler (1 == enable) */
	unsigned int xtal_dbl;
};

/* General frame buffer data structure */
struct w2284fb_par {
	unsigned int chip_id;
	unsigned int xres;
	unsigned int yres;
	unsigned int extmem_active;
	unsigned int flip;
	unsigned int blanked;
	unsigned int fastpll_mode;
	unsigned long hsync_len;
	struct w2284_mode *mode;
	struct w2284_pll_info *pll_table;
	struct w2284fb_mach_info *mach;
	uint32_t *saved_intmem;
	uint32_t *saved_extmem;
};



/*  registry */



/* Block CIF Start: */
#define mmCHIP_ID           0x0000    //********************
#define mmREVISION_ID       0x0004
#define mmWRAP_BUF_A        0x0008
#define mmWRAP_BUF_B        0x000C
#define mmWRAP_TOP_DIR      0x0010
#define mmWRAP_START_DIR    0x0014
#define mmCIF_CNTL          0x0018
#define mmCFGREG_BASE       0x001C
#define mmCIF_IO            0x0020
#define mmCIF_READ_DBG      0x0024
#define mmCIF_WRITE_DBG     0x0028
#define cfgIND_ADDR_A_0     0x0000
#define cfgIND_ADDR_A_1     0x0001
#define cfgIND_ADDR_A_2     0x0002
#define cfgIND_DATA_A       0x0003
#define cfgREG_BASE         0x0004
#define cfgINTF_CNTL        0x0005
#define cfgSTATUS           0x0006
#define cfgCPU_DEFAULTS     0x0007
#define cfgIND_ADDR_B_0     0x0008
#define cfgIND_ADDR_B_1     0x0009
#define cfgIND_ADDR_B_2     0x000A
#define cfgIND_DATA_B       0x000B
#define cfgPM4_RPTR         0x000C
#define cfgSCRATCH          0x000D
#define cfgPM4_WRPTR_0      0x000E
#define cfgPM4_WRPTR_1      0x000F
/* Block CIF End: */

/* Block CG Start: */
#define mmCLK_PIN_CNTL      0x0080  
#define mmPLL_REF_FB_DIV    0x0084          //0x05101b00   M, N_int, N_frac, reset time, lock time 
#define mmPLL_CNTL          0x0088          // 0x2b009100 vdo voltage for core
#define mmSCLK_CNTL         0x008C          //divider for normal a fast mode
#define mmPCLK_CNTL         0x0090          //low 4-bits clkpixsrc, next 4-bits clkpixdiv
#define mmCLK_TEST_CNTL     0x0094         //Select the test clock source and reset
#define mmPWRMGT_CNTL       0x0098         //0x00ff40d1
#define mmPWRMGT_STATUS     0x009C         //
/* Block CG End: */

/* Block RBBM Start: */
#define mmRBBM_STATUS         0x0140
#define mmRBBM_CNTL           0x0144
#define mmRBBM_SOFT_RESET     0x0148      //newpll_re
#define mmNQWAIT_UNTIL        0x0150
#define mmRBBM_1              0x0158
#define mmRBBM_DEBUG          0x016c             //  0xfbadce46
#define mmRBBM_CMDFIFO_ADDR   0x0170       
#define mmRBBM_CMDFIFO_DATAL  0x0174       // 0x4622f75b  change value, but not in driver
#define mmRBBM_CMDFIFO_DATAH  0x0178        // 0x000037f9
#define mmRBBM_CMDFIFO_STAT   0x017c        // 0x00000202
/* Block RBBM End: */

/* Block MC Start: */
#define mmMEM_CNTL             0x0180    //0x0000004f
#define mmMEM_ARB              0x0184    //0x0000017f
#define mmMC_FB_LOCATION       0x0188    //adress range of intenal memory   /*********
#define mmMEM_EXT_CNTL         0x018C
#define mmMC_EXT_MEM_LOCATION  0x0190   //adress range of extenal memory   /**********
#define mmMEM_EXT_TIMING_CNTL  0x0194
#define mmMEM_SDRAM_MODE_REG   0x0198   //needs patch   /reset between modes?
#define mmMEM_IO_CNTL          0x019C
#define mmMC_DEBUG             0x01A0   //0x000c0100
#define mmMC_BIST_CTRL         0x01A4   //**********  till here same as w100  0x00007800
#define mmMC_BIST_COLLAR_READ  0x01A8
#define mmTC_MISMATCH          0x01AC
#define mmMC_PERF_MON_CNTL     0x01B0
#define mmMC_PERF_COUNTERS     0x01B4
/* Block MC End: */


/* Block CP Start:Sync */
#define mmGEN_INT_CNTL      0x0200
#define mmGEN_INT_STATUS    0x0204

//0x0210- 0x022c not importatnt data

#define mmUNKNOWN2          0x0240 //401f000b
#define mmDISP_INT_CNTL     0x0244 //like rotate
//#define mmGEN_INT_STATUS    0x0228   change value in wm   rotate change 250 (0x536dffaf>0x20007000) 244 (0>1) 24c (3e>0)
//4b0 (54a1>551f) 444 (29ca>ffff)
                        
#define mmROT2      0x024c  // value 0x00000008 in portrait mode
#define mmUNKNOWN3       0x0250  //values change from 0x20007000->0x00401084 in portrait


   
#define mmSCRATCH_UMSK      0x0280  //?
#define mmSCRATCH_ADDR      0x0284  //?
//#define mmUNKNOWN           0x02b4  // can be grctrl 5e3ac310 on rainy screen src0
//#define mmUNKNOWN1           0x02e8  // can be gr ctrl 0x6df6385b normal,0x6df62a5b in 16Mhz xtall and pll change, 0x6df6285b in vico vga
/* Block CP End: */

/* Block DISPLAY Start: */      /*  mmCRTC_PS1_ACTIVE mmCRTC_PS1_NACTIVE mmCRTC_GCLK_EXT mmCRTC_PSK
mmCRTC_ALW_VPOS mmCRTC_PSK_HPOS mmHW_INT  mmDISP_DB_BUF_CNTL mmDISP_CRC_SIG  mmDISP_DEBUG mmCRTC_ALW mmCRTC_CV4_START mmCRTC_CV4_END mmCRTC_CV4_HPOS mmCRTC_ECK  */
/* suspend
    0x017c  606-> 505
    0x021c  252-> 9c2
    0x0220  252-> 9c2
    0x0240 0x401f0015->0x401f0019
    0x0244  0->1
    0x0444 =0x0000ffff -> 0
    0x0494 =0x00000000 -> 0x003fffff
    0x04b0 =0x000054e7 -> 0x0000203b
    0x04b4 =0x000f0067 -> 0x00000060
    0x04d8 =0x01ef000f -> 0x000f000f
   0x04e4 =0x02830003 -> 0x00030003
   0x05c0 =0x003700d5 -> 0x001700c0
   0x05c4 =0x07ce2c01 -> 0x07ccb2cb
   0x1080-0x110bc  0-> 1
   0x1280-0x112bc  0->1
ati2284reg  ati2284regsetlcd4 ati2284regsetlcd4back */

#define mmLCD_FORMAT              0x0400       //**
#define mmLCD_BACKGROUND_COLOR    0x0404       //*********
#define mmLCDD_CNTL1              0x0408       //*****
#define mmGENLCD_CNTL1            0x0410   //*****************
#define mmGENLCD_CNTL2            0x0414  //************
#define mmGENLCD_CNTL3            0x0418    //***********buggy
#define mmGPIO_DATA               0x041C    //gpio pin upper direction xxxx0000( used bits in data, will be FFFF), lower gpio data 0000xxxx
#define mmGPIO_DATA2       0x0420                   //gpio pin upper direction xxxx0000, lower gpio data 0000xxxx
#define mmGPIO_CNTL1       0x0424         //control  lower gpio pulldown 0000xxxx
#define mmGPIO_CNTL2       0x0428         //
#define mmGPIO_CNTL3       0x042C         //control   lower gpio pulldown 0000xxxx
#define mmGPIO_CNTL4       0x0430         //         end gpio
#define mmVIDEO_CTRL        0x0434
//#define mmVIDEO_CTRL        0x0434
//#define mmGRAPHIC_KEY       0x0438
#define mmDISP_DB_BUF_CNTL   0x0444  // ?
#define mmCRTC_PS1_ACTIVE       0x0458  // mayby mmDISP_DB_BUF_CNTL?
#define mmLCDD_CNTL2       0x0488   //********
//#define            0x048C
//#define            0x0490
//#define           0x0494
//#define          0x049C
//#define            0x04A0

//#define       0x04A4
//#define          0x04A8
//#define           0x04AC
#define  mmGRAPHIC_CTRL1     0x04B0    //**** changes value in wince with changing screen
#define  mmGRAPHIC_CTRL        0x04B4   // ? change value in wince cause tv screen(look like sync), write 60 cause slow white screen, 67 do nothing and after suspend no vsync
     //0x000x006y    x seems to by some vsync, y seems be some power
//#define          0x04B8
#define mmDISP_DEBUG2      0x04BC   //or GRAPHIC_CTRL**** always 0x00800000 rgb 565
//#define  
//#define         0x04C0
//#define         0x04C4
//#define         0x04C8
//#define       0x04CC
#define mmGRAPHIC_OFFSET    0x04D0       // ? 0x000007f1
#define mmGRAPHIC_PITCH     0x04D4       //********** 0x000003c0
#define mmACTIVE_H_DISP     0x04D8       //***********
#define mmACTIVE_V_DISP     0x04DC      //************
#define mmGRAPHIC_H_DISP    0x04E0        //0x01ef000f******
#define mmGRAPHIC_V_DISP    0x04E4         //0x02830003********
#define mmCRTC_PS2          0x04E8      //?graphicctrl
#define mmCRTC_PS2_VPOS     0x04EC
#define mmCRTC_TOTAL        0x04F0          //0x02870207*************************
#define mmCRTC_SS           0x04F4              //*************************
#define mmCRTC_LS           0x04F8               //********************
#define mmCRTC_REV          0x04FC               //**
#define mmCRTC_DCLK         0x0500                    //******************
#define mmCRTC_GS           0x0504                //*******************
#define mmCRTC_VPOS_GS      0x0508               //******************
#define mmCRTC_GCLK         0x050C              //**
#define mmCRTC_GOE          0x0510              //**
#define mmCRTC_FRAME        0x0514             //*****
#define mmCRTC_FRAME_VPOS   0x051C             //*****
#define mmREFRESH_CNTL      0x0520
//#define       0x0524
//#define         0x0528
//#define         0x052C
//#define         0x0530
#define mmCHIP_STRAP        0x0534
//#define mmDISP_DEBUG2       0x0538
#define mmDEBUG_BUS_CNTL    0x053C
#define mmGAMMA_VALUE1      0x0540
#define mmGAMMA_VALUE2      0x0544
#define mmGAMMA_SLOPE       0x0548
#define mmGEN_STATUS        0x054C
#define mmCRTC_DEFAULT_COUNT            0x0550     //****
/* Block DISPLAY End: */

// 0x0598 -0x0604
// 0x0830 -0x0838
// 0x0908

/* Block BM Start: */                   //seems same as w100
#define mmBM_EXT_MEM_BANDWIDTH    0x0A00
#define mmBM_OFFSET               0x0A04
#define mmBM_MEM_EXT_TIMING_CNTL  0x0A08
#define mmBM_MEM_EXT_CNTL         0x0A0C
#define mmBM_MEM_MODE_REG         0x0A10
#define mmBM_MEM_IO_CNTL          0x0A18
#define mmBM_CONFIG               0x0A1C
#define mmBM_STATUS               0x0A20
#define mmBM_DEBUG                0x0A24
#define mmBM_PERF_MON_CNTL        0x0A28
#define mmBM_PERF_COUNTERS        0x0A2C
#define mmBM_PERF2_MON_CNTL       0x0A30
#define mmBM_PERF2_COUNTERS       0x0A34
/* Block BM End: */

/* Block IDCT Start: */              //seems same as w100
#define mmIDCT_RUNS         0x0C00
#define mmIDCT_LEVELS       0x0C04
#define mmIDCT_CONTROL      0x0C3C
#define mmIDCT_AUTH_CONTROL 0x0C08
#define mmIDCT_AUTH         0x0C0C
/* Block IDCT End: */
//0xfxx no important values
//new


#define mmSRC_OFFSET           0x1010
#define mmSRC_PITCH            0x1014
#define mmDP_GUI_MASTER_CNTL   0x100c
//#define mmDP_CNTL             0x0f10 //only last 2 value 0x000000xx
//#define mmDP_DATATYPE         0x0f0c //up to 30-bit
#define mmDP_MIX              0x12C8
#define mmDP_WRITE_MSK        0x12CC
//0x100c set from 2 to 0 draw only half of screen, 0x1010 write fffffff seem change dst offset,0x101c draw only half of screen write 0,

//old
/* Block GFX Start: */
#define mmDST_OFFSET          0x1004
#define mmDST_PITCH           0x1008
#define mmDST_Y_X             0x1038  //
#define mmDST_WIDTH_HEIGHT    0x1198  //
//#define mmDP_GUI_MASTER_CNTL  0x106C  //
#define mmBRUSH_OFFSET        0x108C
#define mmBRUSH_Y_X           0x1074
#define mmDP_BRUSH_FRGD_CLR   0x107C
//#define mmSRC_OFFSET          0x11AC
//#define mmSRC_PITCH           0x11B0
#define mmSRC_Y_X             0x1034
#define mmDEFAULT_PITCH_OFFSET      0x10A0
#define mmDEFAULT_SC_BOTTOM_RIGHT   0x10A8
#define mmDEFAULT2_SC_BOTTOM_RIGHT  0x10AC
#define mmSC_TOP_LEFT         0x11BC
#define mmSC_BOTTOM_RIGHT     0x11C0
#define mmSRC_SC_BOTTOM_RIGHT 0x11C4
#define mmGLOBAL_ALPHA        0x1210
#define mmFILTER_COEF         0x1214
#define mmMVC_CNTL_START      0x11E0
#define mmE2_ARITHMETIC_CNTL  0x1220
//#define mmDP_CNTL             0x11C8  //only last 2 value 0x000000xx
#define mmDP_CNTL_DST_DIR     0x11CC
//#define mmDP_DATATYPE         0x12C4  /// can be  0x10f18
//#define mmDP_MIX              0x12C8
//#define mmDP_WRITE_MSK        0x12CC
#define mmENG_CNTL            0x13E8
#define mmENG_PERF_CNT        0x13F0
/* Block GFX End: */
/*accel*/
#define mmHOST_DATA0          0x13C0
#define mmHOST_DATA1          0x13C4
#define mmHOST_DATA2          0x13C8
#define mmHOST_DATA3          0x13CC
#define mmHOST_DATA4          0x13D0
#define mmHOST_DATA5          0x13D4
#define mmHOST_DATA6          0x13D8
#define mmHOST_DATA7          0x13DC
#define mmHOST_DATA_LAST      0x13E0
#define mmDP_SRC_FRGD_CLR     0x1218  //0x1240
#define mmDP_SRC_BKGD_CLR     0x121c  // 0x1244



#define mmWAIT_UNTIL          0x1400
#define mmISYNC_CNTL          0x1404




/* default value definitions */
#define defWRAP_TOP_DIR        0x00000000
#define defWRAP_START_DIR      0x00000000
#define defCFGREG_BASE         0x00000000
#define defCIF_IO              0x80140930
#define defINTF_CNTL           0x00000011
#define defCPU_DEFAULTS        0x00000006
#define defHW_INT              0x00000000
#define defMC_EXT_MEM_LOCATION 0xbfff8000  
#define defTC_MISMATCH         0x00000000

#define W2284_CFG_BASE          0x0
#define W2284_CFG_LEN           0x10
#define W2284_REG_BASE          0x10000
#define W2284_REG_LEN           0x2000
#define MEM_INT_BASE_VALUE     0x100000     // used for MC_FB_LOCATION, internal mem(fb) at chipadress+MEM_INT_BASE_VALUE
#define MEM_EXT_BASE_VALUE     0x00800000   // used for MC_EXT_MEM_LOCATION, external mem at chipadress+MEM_EXT_BASE_VALUE
#define MEM_INT_SIZE           0x00030000
#define MEM_WINDOW_BASE        0xab600000
#define MEM_WINDOW_SIZE        0x00400000

#define WRAP_BUF_BASE_VALUE    0x80000
#define WRAP_BUF_TOP_VALUE     0xbffff

#define CHIP_ID_W2282          0x57411002
#define CHIP_ID_W2284          0x57491002

/* Register structure definitions */

struct wrap_top_dir_t {
	u32 top_addr  : 23;
	u32           : 9;
} __attribute__((packed));

union wrap_top_dir_u {
	u32 val : 32;
	struct wrap_top_dir_t f;
} __attribute__((packed));

struct wrap_start_dir_t {
	u32 start_addr : 23;
	u32            : 9;
} __attribute__((packed));

union wrap_start_dir_u {
	u32 val : 32;
	struct wrap_start_dir_t f;
} __attribute__((packed));

struct cif_cntl_t {
	u32 swap_reg                 : 2;
	u32 swap_fbuf_1              : 2;
	u32 swap_fbuf_2              : 2;
	u32 swap_fbuf_3              : 2;
	u32 pmi_int_disable          : 1;
	u32 pmi_schmen_disable       : 1;
	u32 intb_oe                  : 1;
	u32 en_wait_to_compensate_dq_prop_dly  : 1;
	u32 compensate_wait_rd_size  : 2;
	u32 wait_asserted_timeout_val  : 2;
	u32 wait_masked_val          : 2;
	u32 en_wait_timeout          : 1;
	u32 en_one_clk_setup_before_wait  : 1;
	u32 interrupt_active_high    : 1;
	u32 en_overwrite_straps      : 1;
	u32 strap_wait_active_hi     : 1;
	u32 lat_busy_count           : 2;
	u32 lat_rd_pm4_sclk_busy     : 1;
	u32 dis_system_bits          : 1;
	u32 dis_mr                   : 1;
	u32 cif_spare_1              : 4;
} __attribute__((packed));

union cif_cntl_u {
	u32 val : 32;
	struct cif_cntl_t f;
} __attribute__((packed));

struct cfgreg_base_t {
	u32 cfgreg_base  : 24;
	u32              : 8;
} __attribute__((packed));

union cfgreg_base_u {
	u32 val : 32;
	struct cfgreg_base_t f;
} __attribute__((packed));

struct cif_io_t {
	u32 dq_srp     : 1;
	u32 dq_srn     : 1;
	u32 dq_sp      : 4;
	u32 dq_sn      : 4;
	u32 waitb_srp  : 1;
	u32 waitb_srn  : 1;
	u32 waitb_sp   : 4;
	u32 waitb_sn   : 4;
	u32 intb_srp   : 1;
	u32 intb_srn   : 1;
	u32 intb_sp    : 4;
	u32 intb_sn    : 4;
	u32            : 2;
} __attribute__((packed));

union cif_io_u {
	u32 val : 32;
	struct cif_io_t f;
} __attribute__((packed));

struct cif_read_dbg_t {
	u32 unpacker_pre_fetch_trig_gen  : 2;
	u32 dly_second_rd_fetch_trig     : 1;
	u32 rst_rd_burst_id              : 1;
	u32 dis_rd_burst_id              : 1;
	u32 en_block_rd_when_packer_is_not_emp : 1;
	u32 dis_pre_fetch_cntl_sm        : 1;
	u32 rbbm_chrncy_dis              : 1;
	u32 rbbm_rd_after_wr_lat         : 2;
	u32 dis_be_during_rd             : 1;
	u32 one_clk_invalidate_pulse     : 1;
	u32 dis_chnl_priority            : 1;
	u32 rst_read_path_a_pls          : 1;
	u32 rst_read_path_b_pls          : 1;
	u32 dis_reg_rd_fetch_trig        : 1;
	u32 dis_rd_fetch_trig_from_ind_addr : 1;
	u32 dis_rd_same_byte_to_trig_fetch : 1;
	u32 dis_dir_wrap                 : 1;
	u32 dis_ring_buf_to_force_dec    : 1;
	u32 dis_addr_comp_in_16bit       : 1;
	u32 clr_w                        : 1;
	u32 err_rd_tag_is_3              : 1;
	u32 err_load_when_ful_a          : 1;
	u32 err_load_when_ful_b          : 1;
	u32                              : 7;
} __attribute__((packed));

union cif_read_dbg_u {
	u32 val : 32;
	struct cif_read_dbg_t f;
} __attribute__((packed));

struct cif_write_dbg_t {
	u32 packer_timeout_count          : 2;
	u32 en_upper_load_cond            : 1;
	u32 en_chnl_change_cond           : 1;
	u32 dis_addr_comp_cond            : 1;
	u32 dis_load_same_byte_addr_cond  : 1;
	u32 dis_timeout_cond              : 1;
	u32 dis_timeout_during_rbbm       : 1;
	u32 dis_packer_ful_during_rbbm_timeout : 1;
	u32 en_dword_split_to_rbbm        : 1;
	u32 en_dummy_val                  : 1;
	u32 dummy_val_sel                 : 1;
	u32 mask_pm4_wrptr_dec            : 1;
	u32 dis_mc_clean_cond             : 1;
	u32 err_two_reqi_during_ful       : 1;
	u32 err_reqi_during_idle_clk      : 1;
	u32 err_global                    : 1;
	u32 en_wr_buf_dbg_load            : 1;
	u32 en_wr_buf_dbg_path            : 1;
	u32 sel_wr_buf_byte               : 3;
	u32 dis_rd_flush_wr               : 1;
	u32 dis_packer_ful_cond           : 1;
	u32 dis_invalidate_by_ops_chnl    : 1;
	u32 en_halt_when_reqi_err         : 1;
	u32 cif_spare_2                   : 5;
	u32                               : 1;
} __attribute__((packed));

union cif_write_dbg_u {
	u32 val : 32;
	struct cif_write_dbg_t f;
} __attribute__((packed));


struct intf_cntl_t {
	unsigned char ad_inc_a            : 1;
	unsigned char ring_buf_a          : 1;
	unsigned char rd_fetch_trigger_a  : 1;
	unsigned char rd_data_rdy_a       : 1;
	unsigned char ad_inc_b            : 1;
	unsigned char ring_buf_b          : 1;
	unsigned char rd_fetch_trigger_b  : 1;
	unsigned char rd_data_rdy_b       : 1;
} __attribute__((packed));

union intf_cntl_u {
	unsigned char val : 8;
	struct intf_cntl_t f;
} __attribute__((packed));

struct cpu_defaults_t {
	unsigned char unpack_rd_data     : 1;
	unsigned char access_ind_addr_a  : 1;
	unsigned char access_ind_addr_b  : 1;
	unsigned char access_scratch_reg : 1;
	unsigned char pack_wr_data       : 1;
	unsigned char transition_size    : 1;
	unsigned char en_read_buf_mode   : 1;
	unsigned char rd_fetch_scratch   : 1;
} __attribute__((packed));

union cpu_defaults_u {
	unsigned char val : 8;
	struct cpu_defaults_t f;
} __attribute__((packed));

struct crtc_total_t {
	u32 crtc_h_total : 10;
	u32              : 6;
	u32 crtc_v_total : 10;
	u32              : 6;
} __attribute__((packed));

union crtc_total_u {
	u32 val : 32;
	struct crtc_total_t f;
} __attribute__((packed));

struct crtc_ss_t {
	u32 ss_start    : 10;
	u32             : 6;
	u32 ss_end      : 10;
	u32             : 2;
	u32 ss_align    : 1;
	u32 ss_pol      : 1;
	u32 ss_run_mode : 1;
	u32 ss_en       : 1;
} __attribute__((packed));

union crtc_ss_u {
	u32 val : 32;
	struct crtc_ss_t f;
} __attribute__((packed));

struct active_h_disp_t {
	u32 active_h_start  : 10;
	u32                 : 6;
	u32 active_h_end    : 10;
	u32                 : 6;
} __attribute__((packed));

union active_h_disp_u {
	u32 val : 32;
	struct active_h_disp_t f;
} __attribute__((packed));

struct active_v_disp_t {
	u32 active_v_start  : 10;
	u32                 : 6;
	u32 active_v_end    : 10;
	u32                 : 6;
} __attribute__((packed));

union active_v_disp_u {
	u32 val : 32;
	struct active_v_disp_t f;
} __attribute__((packed));

struct graphic_h_disp_t {
	u32 graphic_h_start : 10;
	u32                 : 6;
	u32 graphic_h_end   : 10;
	u32                 : 6;
} __attribute__((packed));

union graphic_h_disp_u {
	u32 val : 32;
	struct graphic_h_disp_t f;
} __attribute__((packed));

struct graphic_v_disp_t {
	u32 graphic_v_start : 10;
	u32                 : 6;
	u32 graphic_v_end   : 10;
	u32                 : 6;
} __attribute__((packed));

union graphic_v_disp_u{
	u32 val : 32;
	struct graphic_v_disp_t f;
} __attribute__((packed));

struct graphic_ctrl_t_w2284 {
	u32 color_depth       : 3;
	u32 portrait_mode     : 2;
	u32 low_power_on      : 1;
	u32 req_freq          : 4;
	u32 en_crtc           : 1;
	u32 en_graphic_req    : 1;
	u32 en_graphic_crtc   : 1;
	u32 total_req_graphic : 9;
	u32 lcd_pclk_on       : 1;
	u32 lcd_sclk_on       : 1;
	u32 pclk_running      : 1;
	u32 sclk_running      : 1;
	u32                   : 6;
} __attribute__((packed));


union graphic_ctrl_u {
	u32 val : 32;
	struct graphic_ctrl_t_w2284 f_w2284;
} __attribute__((packed));

struct video_ctrl_t {
	u32 video_mode       : 1;
	u32 keyer_en         : 1;
	u32 en_video_req     : 1;
	u32 en_graphic_req_video  : 1;
	u32 en_video_crtc    : 1;
	u32 video_hor_exp    : 2;
	u32 video_ver_exp    : 2;
	u32 uv_combine       : 1;
	u32 total_req_video  : 9;
	u32 video_ch_sel     : 1;
	u32 video_portrait   : 2;
	u32 yuv2rgb_en       : 1;
	u32 yuv2rgb_option   : 1;
	u32 video_inv_hor    : 1;
	u32 video_inv_ver    : 1;
	u32 gamma_sel        : 2;
	u32 dis_limit        : 1;
	u32 en_uv_hblend     : 1;
	u32 rgb_gamma_sel    : 2;
} __attribute__((packed));

union video_ctrl_u {
	u32 val : 32;
	struct video_ctrl_t f;
} __attribute__((packed));

struct disp_db_buf_cntl_rd_t {
	u32 en_db_buf           : 1;
	u32 update_db_buf_done  : 1;
	u32 db_buf_cntl         : 6;
	u32                     : 24;
} __attribute__((packed));

union disp_db_buf_cntl_rd_u {
	u32 val : 32;
	struct disp_db_buf_cntl_rd_t f;
} __attribute__((packed));

struct disp_db_buf_cntl_wr_t {
	u32 en_db_buf      : 1;
	u32 update_db_buf  : 1;
	u32 db_buf_cntl    : 6;
	u32                : 24;
} __attribute__((packed));

union disp_db_buf_cntl_wr_u {
	u32 val : 32;
	struct disp_db_buf_cntl_wr_t f;
} __attribute__((packed));

struct gamma_value1_t {
	u32 gamma1   : 8;
	u32 gamma2   : 8;
	u32 gamma3   : 8;
	u32 gamma4   : 8;
} __attribute__((packed));

union gamma_value1_u {
	u32 val : 32;
	struct gamma_value1_t f;
} __attribute__((packed));

struct gamma_value2_t {
	u32 gamma5   : 8;
	u32 gamma6   : 8;
	u32 gamma7   : 8;
	u32 gamma8   : 8;
} __attribute__((packed));

union gamma_value2_u {
	u32 val : 32;
	struct gamma_value2_t f;
} __attribute__((packed));

struct gamma_slope_t {
	u32 slope1   : 3;
	u32 slope2   : 3;
	u32 slope3   : 3;
	u32 slope4   : 3;
	u32 slope5   : 3;
	u32 slope6   : 3;
	u32 slope7   : 3;
	u32 slope8   : 3;
	u32          : 8;
} __attribute__((packed));

union gamma_slope_u {
	u32 val : 32;
	struct gamma_slope_t f;
} __attribute__((packed));

struct mc_ext_mem_location_t {
	u32 mc_ext_mem_start : 16;
	u32 mc_ext_mem_top   : 16;
} __attribute__((packed));

union mc_ext_mem_location_u {
	u32 val : 32;
	struct mc_ext_mem_location_t f;
} __attribute__((packed));

struct mc_fb_location_t {
	u32 mc_fb_start      : 16;
	u32 mc_fb_top        : 16;
} __attribute__((packed));

union mc_fb_location_u {
	u32 val : 32;
	struct mc_fb_location_t f;
} __attribute__((packed));

struct clk_pin_cntl_t {
	u32 osc_en           : 1;
	u32 osc_gain         : 5;
	u32 dont_use_xtalin  : 1;
	u32 xtalin_pm_en     : 1;
	u32 xtalin_dbl_en    : 1;
	u32                  : 7;
	u32 cg_debug         : 16;
} __attribute__((packed));

union clk_pin_cntl_u {
	u32 val : 32;
	struct clk_pin_cntl_t f;
} __attribute__((packed));

struct pll_ref_fb_div_t {
	u32 pll_ref_div      : 8;
	u32 pll_fb_div_int   : 8;
	u32 pll_fb_div_frac  : 4;
	u32 pll_reset_time   : 4;
	u32 pll_lock_time    : 8;
} __attribute__((packed));

union pll_ref_fb_div_u {
	u32 val : 32;
	struct pll_ref_fb_div_t f;
} __attribute__((packed));

struct pll_cntl_t {
	u32 pll_pwdn        : 1;
	u32 pll_reset       : 1;
	u32 pll_pm_en       : 1;
	u32 pll_mode        : 1;
	u32 pll_refclk_sel  : 1;
	u32 pll_fbclk_sel   : 1;
	u32 pll_tcpoff      : 1;
	u32 pll_pcp         : 3;
	u32 pll_pvg         : 3;
	u32 pll_vcofr       : 1;
	u32 pll_ioffset     : 2; //this value is changing
	u32 pll_pecc_mode   : 2;
	u32 pll_pecc_scon   : 2;
	u32 pll_dactal      : 4;
	u32 pll_cp_clip     : 2;
	u32 pll_conf        : 3;
	u32 pll_mbctrl      : 2;
	u32 pll_ring_off    : 1;
} __attribute__((packed));

union pll_cntl_u {
	u32 val : 32;
	struct pll_cntl_t f;
} __attribute__((packed));

struct sclk_cntl_t {                 //modified (value of divider decrease by 1)(if ClkSysNormSrc:0 , sclk_normal_ppl_src_divider is replaced by clksysnormxtalsrcdiv)
	u32 sclk_src_sel                 : 2;
	u32 sclk_turbo_ppl_src_divider   : 4;
	u32 sclk_fast_ppl_src_divider    : 5;
	u32 sclk_normal_ppl_src_divider  : 5;
        u32 sclk_unknown                 : 1;
	u32                              : 15;
} __attribute__((packed));

union sclk_cntl_u {
	u32 val : 32;
	struct sclk_cntl_t f;
} __attribute__((packed));

struct pclk_cntl_t {
	u32 pclk_pix_src     : 2;
	u32                  : 2;
	u32 pclk_pix_div     : 4;
	u32                  : 8;
	u32 pclk_force_disp  : 1;
	u32                  : 15;
} __attribute__((packed));

union pclk_cntl_u {
	u32 val : 32;
	struct pclk_cntl_t f;
} __attribute__((packed));


#define TESTCLK_SRC_PLL   0x01
#define TESTCLK_SRC_SCLK  0x02
#define TESTCLK_SRC_PCLK  0x03
/* 4 and 5 seem to by XTAL/M */
#define TESTCLK_SRC_XTAL  0x06

struct clk_test_cntl_t {
	u32 testclk_sel      : 4;
	u32                  : 3;
	u32 start_check_freq : 1;
	u32 tstcount_rst     : 1;
	u32                  : 15;
	u32 test_count       : 8;
} __attribute__((packed));

union clk_test_cntl_u {
	u32 val : 32;
	struct clk_test_cntl_t f;
} __attribute__((packed));

struct pwrmgt_cntl_t {            //in progress  0x00ff40d1->0x00ff4091 after reset bit HwAutoModeSwitch
 //ExtMemPowerManagement:0 2b4->5e3ac710 ,2e8->6df6385b
 //0x02e8 6df6385b->6df6285b
 //  0x02b4
// no dynvolten:1 it goes to dark
	u32 pwm_enable           : 2;
	u32 pwm_mode_req         : 2;
	u32 pwm_wakeup_cond      : 2;
	u32 Hw_Auto_Mode_Switch  : 1;
	u32 pwm_noml_fast_hw_en  : 1;
	u32 pwm_fast_noml_cond   : 4;
	u32 pwm_noml_fast_cond   : 4; 
	u32 pwm_idle_timer       : 8;
	u32 pwm_busy_timer       : 8;
} __attribute__((packed));

union pwrmgt_cntl_u {
	u32 val : 32;
	struct pwrmgt_cntl_t f;
} __attribute__((packed));

struct pwrmgt_status_t {          //in progress
	u32     status1             : 4;
        u32     Clk_Sys_Norm_Src    : 4; //ClkSysNormSrc=1 (pll) value 0x7 ,ClkSysNormSrc=0 (xtal)reset 4, 6 bit or set 0x2, replace divider
        u32     status2             : 4;       
        u32     Hw_Auto_Fast_Turbo_Switch    : 1;
        u32     status3                 : 3;     
	u32                      : 16;
} __attribute__((packed));

union pwrmgt_status_u {
	u32 val : 32;
	struct pwrmgt_status_t f;
} __attribute__((packed));

#define DP_SRC_1BPP_OPA          0
#define DP_SRC_COLOR_SAME_AS_DST 3
#define DP_DST_16BPP_1555        3

#define ROP3_SRCCOPY	0xcc
#define ROP3_PATCOPY	0xf0

#define GMC_BRUSH_SOLID_COLOR	13
#define GMC_BRUSH_NONE		15

#define DP_SRC_MEM_RECTANGULAR	2

#define DP_SRC_HOSTDATA_BIT     3
#define DP_SRC_HOSTDATA_BYTE    4

#define DP_PIX_ORDER_MSB2LSB    0
#define DP_PIX_ORDER_LSB2MSB    1
#define DP_OP_ROP	0

struct dp_gui_master_cntl_t {
	u32 gmc_src_pitch_offset_cntl : 1;
	u32 gmc_dst_pitch_offset_cntl : 1;
	u32 gmc_src_clipping          : 1;
	u32 gmc_dst_clipping          : 1;
	u32 gmc_brush_datatype        : 4;
	u32 gmc_dst_datatype          : 4;
	u32 gmc_src_datatype          : 3;
	u32 gmc_byte_pix_order        : 1;
	u32 gmc_default_sel           : 1;
	u32 gmc_rop3                  : 8;
	u32 gmc_dp_src_source         : 3;
	u32 gmc_clr_cmp_fcn_dis       : 1;
	u32                           : 1;
	u32 gmc_wr_msk_dis            : 1;
	u32 gmc_dp_op                 : 1;
} __attribute__((packed));

union dp_gui_master_cntl_u {
	u32 val : 32;
	struct dp_gui_master_cntl_t f;
} __attribute__((packed));

struct rbbm_status_t {
	u32 cmdfifo_avail   : 7;
	u32                 : 1;
	u32 hirq_on_rbb     : 1;
	u32 cprq_on_rbb     : 1;
	u32 cfrq_on_rbb     : 1;
	u32 hirq_in_rtbuf   : 1;
	u32 cprq_in_rtbuf   : 1;
	u32 cfrq_in_rtbuf   : 1;
	u32 cf_pipe_busy    : 1;
	u32 eng_ev_busy     : 1;
	u32 cp_cmdstrm_busy : 1;
	u32 e2_busy         : 1;
	u32 rb2d_busy       : 1;
	u32 rb3d_busy       : 1;
	u32 se_busy         : 1;
	u32 re_busy         : 1;
	u32 tam_busy        : 1;
	u32 tdm_busy        : 1;
	u32 pb_busy         : 1;
	u32                 : 6;
	u32 gui_active      : 1;
} __attribute__((packed));

union rbbm_status_u {
	u32 val : 32;
	struct rbbm_status_t f;
} __attribute__((packed));

struct dp_datatype_t {
	u32 dp_dst_datatype   : 4;
	u32                   : 4;
	u32 dp_brush_datatype : 4;
	u32 dp_src2_type      : 1;
	u32 dp_src2_datatype  : 3;
	u32 dp_src_datatype   : 3;
	u32                   : 11;
	u32 dp_byte_pix_order : 1;
	u32                   : 1;
} __attribute__((packed));

union dp_datatype_u {
	u32 val : 32;
	struct dp_datatype_t f;
} __attribute__((packed));

struct dp_mix_t {
	u32                : 8;
	u32 dp_src_source  : 3;
	u32 dp_src2_source : 3;
	u32                : 2;
	u32 dp_rop3        : 8;
	u32 dp_op          : 1;
	u32                : 7;
} __attribute__((packed));

union dp_mix_u {
	u32 val : 32;
	struct dp_mix_t f;
} __attribute__((packed));

struct eng_cntl_t {
	u32 erc_reg_rd_ws            : 1;
	u32 erc_reg_wr_ws            : 1;
	u32 erc_idle_reg_wr          : 1;
	u32 dis_engine_triggers      : 1;
	u32 dis_rop_src_uses_dst_w_h : 1;
	u32 dis_src_uses_dst_dirmaj  : 1;
	u32                          : 6;
	u32 force_3dclk_when_2dclk   : 1;
	u32                          : 19;
} __attribute__((packed));

union eng_cntl_u {
	u32 val : 32;
	struct eng_cntl_t f;
} __attribute__((packed));

struct dp_cntl_t {
	u32 dst_x_dir   : 1;
	u32 dst_y_dir   : 1;
	u32 src_x_dir   : 1;
	u32 src_y_dir   : 1;
	u32 dst_major_x : 1;
	u32 src_major_x : 1;
	u32             : 26;
} __attribute__((packed));

union dp_cntl_u {
	u32 val : 32;
	struct dp_cntl_t f;
} __attribute__((packed));

struct dp_cntl_dst_dir_t {
	u32           : 15;
	u32 dst_y_dir : 1;
	u32           : 15;
	u32 dst_x_dir : 1;
} __attribute__((packed));

union dp_cntl_dst_dir_u {
	u32 val : 32;
	struct dp_cntl_dst_dir_t f;
} __attribute__((packed));




#endif
