/*
 * linux/drivers/usb/gadget/isp1582_udc.c
 *
 * Samsung S3C24xx series on-chip full speed USB device controllers
 *
 * Copyright (C) 2004-2007 guiming zhuo <gmzhuo@gmail.com>
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
 */

#ifndef _ISP1582_UDC_H
#define _ISP1582_UDC_H
#include <linux/udc_isp158x.h>
#include <linux/usb/otg.h>

/* dma burst register access */
#define ISP_DMA_BURST_LEN      32

/* the following definitions are used for unlock access */
#define ISP_UNLOCK_REGS        0xAA37

#define DIR_RX                      0
#define DIR_TX                      1

#define ISP1582_UNLOCK_CODE         (unsigned short)0xAA37

/* Initialization OTG register bits */
#define INIT_OTG_BSESS_VALID        (1 << 4)

/* Initialization Mode register bits */
#define INIT_MODE_TEST2             (1 << 15)
#define INIT_MODE_TEST1             (1 << 14)
#define INIT_MODE_TEST0             (1 << 13)
#define INIT_MODE_DMA_CLKON         (1 << 9)
#define INIT_MODE_VBUSSTAT          (1 << 8)
#define INIT_MODE_CLKAON            (1 << 7)
#define INIT_MODE_SNDRSU            (1 << 6)
#define INIT_MODE_GOSUSP            (1 << 5)
#define INIT_MODE_SFRESET           (1 << 4)
#define INIT_MODE_GLINTENA          (1 << 3)
#define INIT_MODE_WKUPCS            (1 << 2)
#define INIT_MODE_PWRON             (1 << 1)
#define INIT_MODE_SOFTCT            (1 << 0)

/* Initialization Interrupt Enable register bits */
#define INIT_INTEN_IEP7TX           (1 << 25)
#define INIT_INTEN_IEP7RX           (1 << 24)
#define INIT_INTEN_IEP6TX           (1 << 23)
#define INIT_INTEN_IEP6RX           (1 << 22)
#define INIT_INTEN_IEP5TX           (1 << 21)
#define INIT_INTEN_IEP5RX           (1 << 20)
#define INIT_INTEN_IEP4TX           (1 << 19)
#define INIT_INTEN_IEP4RX           (1 << 18)
#define INIT_INTEN_IEP3TX           (1 << 17)
#define INIT_INTEN_IEP3RX           (1 << 16)
#define INIT_INTEN_IEP2TX           (1 << 15)
#define INIT_INTEN_IEP2RX           (1 << 14)
#define INIT_INTEN_IEP1TX           (1 << 13)
#define INIT_INTEN_IEP1RX           (1 << 12)
#define INIT_INTEN_IEP0TX           (1 << 11)
#define INIT_INTEN_IEP0RX           (1 << 10)
#define INIT_INTEN_IEP0SETUP        (1 << 8)
#define INIT_INTEN_IEVBUS           (1 << 7)
#define INIT_INTEN_IEDMA            (1 << 6)
#define INIT_INTEN_IEHS_STA         (1 << 5)
#define INIT_INTEN_IERESM           (1 << 4)
#define INIT_INTEN_IESUSP           (1 << 3)
#define INIT_INTEN_IEPSOF           (1 << 2)
#define INIT_INTEN_IESOF            (1 << 1)
#define INIT_INTEN_IEBRST           (1 << 0)

/* Initialization Interrupt Configuration register bits */
#define INIT_INTCONF_INTLVL         (1 << 1)
#define INIT_INTCONF_INTPOL         (1 << 0)

/* Initialization Address register bits */
#define INIT_ADDRESS_DEVEN          (1 << 7)

/* Data Flow registers' bits */

/* Data Flow Endpoint Index register bits */
#define DFLOW_EPINDEX_EP0SETUP      (1 << 5)

/* Data Flow Control Function register bits */
#define DFLOW_CTRLFUN_CLBUF         (1 << 4)
#define DFLOW_CTRLFUN_VENDP         (1 << 3)
#define DFLOW_CTRLFUN_DSEN          (1 << 2)
#define DFLOW_CTRLFUN_STATUS        (1 << 1)
#define DFLOW_CTRLFUN_STALL         (1 << 0)

/* Data Flow Endpoint Type register bits */
#define DFLOW_EPTYPE_NOEMPKT        (1 << 4)
#define DFLOW_EPTYPE_ENABLE         (1 << 3)
#define DFLOW_EPTYPE_DBLBUF         (1 << 2)

/* General registers' bits */

/* General Test Mode register bits */
#define GEN_TSTMOD_FORCEHS          (1 << 7)
#define GEN_TSTMOD_FORCEFS          (1 << 4)
#define GEN_TSTMOD_PRBS             (1 << 3)
#define GEN_TSTMOD_KSTATE           (1 << 2)
#define GEN_TSTMOD_JSTATE           (1 << 1)
#define GEN_TSTMOD_SE0_NAK          (1 << 0)

/* Interrupts */
#define INT_IEP7TX                  (1 << 25)
#define INT_IEP7RX                  (1 << 24)
#define INT_IEP6TX                  (1 << 23)
#define INT_IEP6RX                  (1 << 22)
#define INT_IEP5TX                  (1 << 21)
#define INT_IEP5RX                  (1 << 20)
#define INT_IEP4TX                  (1 << 19)
#define INT_IEP4RX                  (1 << 18)
#define INT_IEP3TX                  (1 << 17)
#define INT_IEP3RX                  (1 << 16)
#define INT_IEP2TX                  (1 << 15)
#define INT_IEP2RX                  (1 << 14)
#define INT_IEP1TX                  (1 << 13)
#define INT_IEP1RX                  (1 << 12)
#define INT_IEP0TX                  (1 << 11)
#define INT_IEP0RX                  (1 << 10)
#define INT_IEP0SETUP               (1 << 8)
#define INT_IEVBUS                  (1 << 7)
#define INT_IEDMA                   (1 << 6)
#define INT_IEHS_STA                (1 << 5)
#define INT_IERESM                  (1 << 4)
#define INT_IESUSP                  (1 << 3)
#define INT_IEPSOF                  (1 << 2)
#define INT_IESOF                   (1 << 1)
#define INT_IEBRST                  (1 << 0)

#define INT_EP_MASK                 (INT_IEP0RX | INT_IEP0TX | INT_IEP1RX |\
		    INT_IEP1TX | INT_IEP2RX | INT_IEP2TX | INT_IEP3RX |\
		    INT_IEP3TX | INT_IEP4RX | INT_IEP4TX | INT_IEP5RX |\
		    INT_IEP5TX | INT_IEP6RX | INT_IEP6TX | INT_IEP7RX |\
		    INT_IEP7TX)

#define STANDARD_INTEN              (INIT_INTEN_IEBRST | INIT_INTEN_IEHS_STA |\
		    INT_IESUSP | INT_IERESM | INIT_INTEN_IEVBUS |\
		    INIT_INTEN_IEP0SETUP | INIT_INTEN_IEP0RX |\
		    INIT_INTEN_IEP0TX)

#define STANDARD_INIT_MODE          (INIT_MODE_CLKAON | INIT_MODE_GLINTENA)

/*
  * Definitions of the bit fields in the EndpointControl register.
  */
#define USB_EPCONTROL_CLEAR                     0x00000010
#define USB_EPCONTROL_VALIDATE                  0x00000008
#define USB_EPCONTROL_DSEN		        0x00000004
#define USB_EPCONTROL_STATUS_ACK                0x00000002
#define USB_EPCONTROL_STALL                     0x00000001

/*
  * Definitions of the bit fields in the EndpointIndex register.
  */
#define USB_ENDPOINT_CONTROL_OUT                0x00000000
#define USB_ENDPOINT_CONTROL_IN                 0x00000001
#define USB_ENDPOINT_ONE_OUT                    0x00000002
#define USB_ENDPOINT_ONE_IN                     0x00000003
#define USB_ENDPOINT_TWO_OUT                    0x00000004
#define USB_ENDPOINT_TWO_IN                     0x00000005
#define USB_ENDPOINT_THREE_OUT                  0x00000006
#define USB_ENDPOINT_THREE_IN                   0x00000007
#define USB_ENDPOINT_FOUR_OUT                   0x00000008
#define USB_ENDPOINT_FOUR_IN                    0x00000009
#define USB_ENDPOINT_FIVE_OUT                   0x0000000a
#define USB_ENDPOINT_FIVE_IN                    0x0000000b
#define USB_ENDPOINT_SIX_OUT                    0x0000000c
#define USB_ENDPOINT_SIX_IN                     0x0000000d
#define USB_ENDPOINT_SEVEN_OUT                  0x0000000e
#define USB_ENDPOINT_SEVEN_IN                   0x0000000f
#define USB_ENDPOINT_SETUP                      0x00000020

#define USB_ENDPOINT_STATUS_STALLED             0x01

/*
  * Definitions of the bit fields in the EndpointType register.
  */
#define USB_EPTYPE_NO_EMPTY                     0x00000010
#define USB_EPTYPE_ENABLE                       0x00000008
#define USB_EPTYPE_DOUBLE_BUFFER                0x00000004
#define USB_EPTYPE_TYPE_MASK                    0x00000003
#define USB_EPTYPE_TYPE_CONTROL                 0x00000000
#define USB_EPTYPE_TYPE_ISOCHRONOUS             0x00000001
#define USB_EPTYPE_TYPE_BULK                    0x00000002
#define USB_EPTYPE_TYPE_INTERRUPT               0x00000003

#define ISP_DMA_DATA_PORT_OFFSET   0x440

#define MAX_DMA_SIZE  4096

#define ISP1582_ADDR_REG	0X00
#define ISP1582_MODE_REG	0X0C
#define ISP1582_INTCONF_REG	0X10
#define ISP1582_OTG_REG		0X12
#define ISP1582_INTEN_L_REG	0X14
#define ISP1582_INTEN_M_REG	0X16

#define ISP1582_EPINDEX_REG	0X2C
#define ISP1582_EPCFG_REG	0X28
#define ISP1582_EPDATA_REG	0X20
#define ISP1582_BUFFLEN_REG	0X1C
#define ISP1582_BUFFST_REG	0X1E
#define ISP1582_MAXPKT_REG	0X04
#define ISP1582_EPTYPE_REG	0X08

#define ISP1582_DMACMD_REG	0X30
#define ISP1582_DMACNT_L_REG	0X34
#define ISP1582_DMACNT_H_REG	0X36
#define ISP1582_DMACFG_REG	0X38
#define ISP1582_DMAHW_REG	0X3C
#define ISP1582_DMATASK_REG	0X40
#define ISP1582_DMAINTS_REG	0X50
#define ISP1582_DMAINTEN_REG	0X54
#define ISP1582_DMAEP_REG	0X58
#define ISP1582_DMASTROBETM_REG	0X60
#define ISP1582_DMABSTCNT_REG	0X64

#define ISP1582_INTS_L_REG	0X18
#define ISP1582_INTS_M_REG	0X1A
#define ISP1582_CHIPID_L_REG	0X70
#define ISP1582_CHIPID_M_REG	0X72
#define ISP1582_FRAMENUM_REG	0X74
#define ISP1582_SCRATCH_REG	0X78
#define ISP1582_UNLOCK_REG	0X7C
#define ISP1582_TESTMODE_REG	0X84

/*****************************************************************************/

/*
  *The USB Control process
  */

/******************************************************************************/

/*
  *Setup control process status
  */
#define SETUPCOMMAND_DataPhase_OUTPID      1
#define SETUPCOMMAND_DataPhase_INPID         2
#define SETUPCOMMAND_StatusPhase_INPID      3
#define SETUPCOMMAND_StatusPhase_OUTPID   4
#define SETUPCOMMAND_STATUS_DEFAULT        0

/*
  *USB Speed
  */
#define HighSpeed  2
#define FullSpeed   1
#define LowSpeed   0

/*
  *default languale is 0409--- english
  */
#define DEF_LANG 0x0409

struct isp1582_ep {
	struct list_head queue;
	unsigned long last_io;	/* jiffies timestamp */
	struct usb_gadget *gadget;
	struct isp1582_udc *dev;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep ep;
	u8 num;

	unsigned short fifo_size;
	u8 bEndpointAddress;
	u8 bmAttributes;

	unsigned halted:1;
	unsigned already_seen:1;
	unsigned setup_stage:1;
};

#define EP_FIFO_SIZE		64
#define DEFAULT_POWER_STATE	0x00

#define EP0_FIFO_SIZE	        ((unsigned)64)
#define BULK_FIFO_SIZE          ((unsigned)512)

#define ISO_FIFO_SIZE	        ((unsigned)256)
#define INT_FIFO_SIZE	        ((unsigned)8)

static const char ep0name[] = "ep0";

static const char *const ep_name[] = {
	ep0name,		/* everyone has ep0 */
	/* isp1582 four bidirectional bulk endpoints */
	"ep1in-", "ep1out-", "ep2in-", "ep2out-",
	"ep3in-", "ep3out-", "ep4in-", "ep4out-",
	"ep5in-", "ep5out-", "ep6in-", "ep6out-",
	"ep7in-", "ep7out-"
};

#define ISP1582_ENDPOINTS       ARRAY_SIZE(ep_name)

struct isp1582_request {
	struct list_head queue;	/* ep's requests */
	struct usb_request req;
	int isIn;
	int mapped;
};

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_IN_STATUS_PHASE,
	EP0_OUT_STATUS_PHASE,
	EP0_END_XFER,
	EP0_STALL,
};

#pragma pack(1)

/* ISP1582 register definition */
union ADDRESS_REG {
	struct ADDRESS_BITS {
		u8 DEVADDR:7;
		u8 DEVEN:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union USB_MODE {
	struct USB_MODE_BITS {
		u8 SOFTCT:1;
		u8 PWRON:1;
		u8 WKUPCS:1;
		u8 GLINTENA:1;
		u8 SFRESET:1;
		u8 GOSUSP:1;
		u8 SNDRSU:1;
		u8 CLKAON:1;
		u8 VBUS_STATUS:1;
		u8 DMACLKON:1;
		u8 RESERVED:3;
		u8 BUS_CONFIG:1;
		u8 MODE0:1;
		u8 MODE1:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

#define ISP_MODE_GLINTENA           0x0008
#define ISP_MODE_MASK_GLINTENA      0xFFF7

union INT_CONFIG {
	struct INT_CONFIG_BITS {
		u8 INTPOL:1;
		u8 INTLVL:1;
		u8 DDBGMODOUT:2;
		u8 DDBGMODIN:2;
		u8 CDBGMOD:2;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union OTG_MODE {
	struct OTG_MODE_BITS {
		u8 OTG:1;
		u8 VP:1;
		u8 DISCV:1;
		u8 INIT:1;
		u8 BSESSVALID:1;
		u8 DP:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union INT_ENABLE_LSB {
	struct INT_ENABLE_BITS_LSB {
		u8 IERST:1;
		u8 IESOF:1;
		u8 IEPSOF:1;
		u8 IESUSP:1;
		u8 IERESM:1;
		u8 IEHS_STA:1;
		u8 IEDMA:1;
		u8 IEVBUS:1;
		u8 IEP0SETUP:1;
		u8 RESERVED2:1;
		u8 IEP0RX:1;
		u8 IEP0TX:1;
		u8 IEP1RX:1;
		u8 IEP1TX:1;
		u8 IEP2RX:1;
		u8 IEP2TX:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union INT_ENABLE_MSB {
	struct INT_ENABLE_BITS_MSB {
		u8 IEP3RX:1;
		u8 IEP3TX:1;
		u8 IEP4RX:1;
		u8 IEP4TX:1;
		u8 IEP5RX:1;
		u8 IEP5TX:1;
		u8 IEP6RX:1;
		u8 IEP6TX:1;
		u8 IEP7RX:1;
		u8 IEP7TX:1;
		u8 RESERVED1:6;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union CONTROL_REG {
	struct CONTROL_BITS {
		u8 STALL:1;
		u8 STATUS:1;
		u8 DSEN:1;
		u8 VENDP:1;
		u8 CLBUF:1;
		u8 RESERVED1:3;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union BUFFER_STATUS {
	struct BUFFER_STATUS_BIT {
		u8 BUF0:1;
		u8 BUF1:1;
		u8 RES:6;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union ENDPT_MAXSIZE {
	struct ENDPT_MAXSIZE_BITS {
		u8 FFOSZ7_0:8;
		u8 FFOSZ10_8:3;
		u8 NTRANS:2;
		u8 RESERVED2:3;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union ENDPT_TYPE {
	struct ENDPT_TYPE_BITS {
		u8 ENDPTYP:2;
		u8 DBLBUF:1;
		u8 ENABLE:1;
		u8 NOEMPKT:1;
		u8 RESERVED1:3;
		u8 RESRVED2:8;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

#define ISP_EPT_NOEMPKT    0x10
#define ISP_EPT_ENABLE     0x08
#define ISP_EPT_DBLBUF     0x04

union DMA_CONFIG {
	struct DMA_CONFIG_BITS {
		u8 WIDTH:1;
		u8 DREQ_MODE:1;
		u8 MODE:2;
		u8 RES1:3;
		u8 DIS_XFER_CNT:1;
		u8 PIO_MODE:3;
		u8 DMA_MODE:2;
		u8 ATA_MODE:1;
		u8 IGNORE_IORDY:1;
		u8 STREAMING:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union DMA_HARDWARE {
	struct DMA_HARDWARE_BITS {
		u8 READ_POL:1;
		u8 WRITE_POL:1;
		u8 DREQ_POL:1;
		u8 ACK_POL:1;
		u8 MASTER:1;
		u8 EOT_POL:1;
		u8 ENDIAN:2;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union DMA_STROBE {
	struct DMA_STROBE_BITS {
		u8 DMA_STROBE:5;
		u8 RES:3;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union DMA_INT {
	struct DMA_INT_BITS {
		u8 RES:1;
		u8 CMD_INTRQ_OK:1;
		u8 TASKFILE_READ_COMPLETE:1;
		u8 BSY_DRQ_POLL_DONE:1;
		u8 START_READ_1F0_RD_FIFO:1;
		u8 RD_1F0_FIFO_EMPTY:1;
		u8 WR_1F0_FIFO_FULL:1;
		u8 WR_1F0_FIFO_EMPTY:1;
		u8 DMA_XFER_OK:1;
		u8 INTRQ_SEEN:1;
		u8 INT_EOT:1;
		u8 EXT_EOT:1;
		u8 RES1:3;
		u8 TEST3:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union DMA_INT_ENABLE {
	struct DMA_INT_ENABLE_BITS {
		u8 RES:1;
		u8 IE_CMD_INTRQ_OK:1;
		u8 IE_TASKFILE_READ_COMPLETE:1;
		u8 IE_BSY_DRQ_POLL_DONE:1;
		u8 IE_START_READ_1F0_RD_FIFO:1;
		u8 IE_RD_1F0_FIFO_EMPTY:1;
		u8 IE_WR_1F0_FIFO_FULL:1;
		u8 IE_WR_1F0_FIFO_EMPTY:1;
		u8 IE_DMA_XFER_OK:1;
		u8 IE_INTRQ_SEEN:1;
		u8 IE_INT_EOT:1;
		u8 IE_EXT_EOT:1;
		u8 RES1:3;
		u8 TEST4:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union INTERRUPT_STATUS_LSB {
	struct INTERRUPT_STATUS_BITS_LSB {
		u8 RESET:1;
		u8 SOF:1;
		u8 PSOF:1;
		u8 SUSP:1;
		u8 RESUME:1;
		u8 HS_STAT:1;
		u8 DMA:1;
		u8 VBUS:1;
		u8 EP0SETUP:1;
		u8 RESERVED2:1;
		u8 EP0RX:1;
		u8 EP0TX:1;
		u8 EP1RX:1;
		u8 EP1TX:1;
		u8 EP2RX:1;
		u8 EP2TX:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union INTERRUPT_STATUS_MSB {
	struct INTERRUPT_STATUS_BITS_MSB {
		u8 EP3RX:1;
		u8 EP3TX:1;
		u8 EP4RX:1;
		u8 EP4TX:1;
		u8 EP5RX:1;
		u8 EP5TX:1;
		u8 EP6RX:1;
		u8 EP6TX:1;
		u8 EP7RX:1;
		u8 EP7TX:1;
		u8 RESERVED1:6;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

union FRAME_NO {
	struct FRAME_NO_BITS {
		u8 SOFL:8;
		u8 SOFH:3;
		u8 USOFP:3;
		u8 RESERVED:2;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

#define ISP_FNO_MASK		0x07FF
#define ISP_FNO_MICRMASK	0x3800
#define ISP_FNO_MICROFF		11

union TESTMODE {
	struct TESTMODE_BITS {
		u8 SE0_NAK:1;
		u8 JSTATE:1;
		u8 KSTATE:1;
		u8 PRBS:1;
		u8 FORCEFS:1;
		u8 LPBK:1;
		u8 PHYTEST:1;
		u8 FORCEHS:1;
	} __attribute__ ((packed)) BITS;
	u16 VALUE;
} __attribute__ ((packed));

#define ISP_DMA_RESET		0x11
#define ISP_DMA_CLEAR_BUFFER    0x0F
#define ISP_GDMA_Write_Command	0x01
#define ISP_GDMA_Read_Command	0x00

struct isp1582_udc {
	spinlock_t lock;

	struct isp1582_ep ep[ISP1582_ENDPOINTS];
	int address;
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct isp1582_request fifo_req;
	u8 fifo_buf[EP_FIFO_SIZE];
	u16 devstatus;

	u32 port_status;
	int ep0state;

	unsigned got_irq:1;

	unsigned req_std:1;
	unsigned req_config:1;
	unsigned req_pending:1;
	u8 vbus;
	struct dentry *regs_info;
	unsigned short *regs;
	int irq;
	int disabled;
	int hiSpeed;
	struct otg_transceiver *transceiver;

	int dma_channel;
	int isp_dma_flag;
	int isp_dma_index;
	int isp_dma_len;
	int isp_dma_index_len;
	struct isp1582_ep *dma_ep;
	unsigned long phy_base;
	struct delayed_work vbus_check;
	struct isp158x_udc_mach_info *mach;
};

void isp_init(void);
int udc_init(void);
void check_vbus(struct work_struct *work);

static inline int ep_index(int n, bool dir)
{
	return (n << 1) | dir;
}

static inline bool epidx_dir(int idx)
{
	return idx & 1;
}

static inline int epidx_n(int idx)
{
	return idx >> 1;
}

#endif
