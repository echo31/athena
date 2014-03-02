/*
 * Specific PXA PATA driver of s1r72v03 device for athena
 *
 * Copyright (C) 2011 Olivier Grisoni
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/ata.h>
#include <linux/libata.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/completion.h>

#include <scsi/scsi_host.h>

#include <mach/pxa2xx-regs.h>
#include <mach/pata_athena.h>
#include <mach/dma.h>

#define DRV_NAME	"pata_athena"
#define DRV_VERSION	"0.1"

#define S1R72V03_HW_REVISION 0x30

/* parameters for changing power management states */
#define S1R72V03_PM_WAIT_USEC          50
#define S1R72V03_MAX_PM_WAIT_USEC      5000

/* chip config:
   bit 7: XINT active high
   bit 6: XINT 1/0 mode
   bit 5: XDREQ active low
   bit 4: XDACK active low
   bit 3: DMA0/1 is enabled by assertion of chip select and XDACK
   bit 2: endianness (even address is lower byte)
   bit 1-0: 16-bit BE mode
*/
#define S1R72V03_CHIPCONFIG_VALUE       0x7e 

/* S1R72V03 IDE taskfile register identifiers */
#define S1R72V03_DATA_TF_REG           0x0
#define S1R72V03_ERROR_TF_REG          0x1
#define S1R72V03_FEATURES_TF_REG       0x1
#define S1R72V03_SECTOR_CNT_TF_REG     0x2
#define S1R72V03_LBA_LOW_TF_REG        0x3
#define S1R72V03_LBA_MID_TF_REG        0x4
#define S1R72V03_LBA_HIGH_TF_REG       0x5
#define S1R72V03_DEVICE_TF_REG         0x6
#define S1R72V03_STATUS_TF_REG         0x7
#define S1R72V03_COMMAND_TF_REG        0x7
#define S1R72V03_DEVICE_CONTROL_TF_REG 0xE
#define S1R72V03_ALT_STATUS_TF_REG     0xE
#define S1R72V03_UNSUPPORTED           -1

#define S1R72V03_TF_WRITE              0x80
#define S1R72V03_TF_READ               0x40

/* S1R72V03 register map (byte offsets from start of chip select) */
#define S1R72V03_MAININTSTAT_OFFSET             	0x00
#define   S1R72V03_MAININTSTAT_SIE_INTSTAT      0x80
#define   S1R72V03_MAININTSTAT_IDE_INTSTAT      0x04
#define   S1R72V03_MAININTSTAT_FIFO_INTSTAT     0x10
#define   S1R72V03_MAININTSTAT_ALLINTS          0xFF
#define S1R72V03_EPRINTSTAT_OFFSET           		0x01
#define S1R72V03_SIE_INTSTAT_OFFSET             	0x02
#define   S1R72V03_SIE_INTSTAT_FINISHEDPM      0x20
#define   S1R72V03_SIE_INTSTAT_INIT		0x00
#define S1R72V03_CPUINTSTAT_OFFSET              	0x03
#define S1R72V03_FIFO_INTSTAT_OFFSET             	0x04
#define   S1R72V03_FIFO_INTSTAT_CMP		0x01
#define   S1R72V03_FIFO_INTSTAT_EMPTY		0x04
#define   S1R72V03_FIFO_INTSTAT_FULL		0x08
#define   S1R72V03_FIFO_INTSTAT_ALLINTS 	0x0D
#define S1R72V03_BULKINTSTAT_OFFSET              	0x05
#define S1R72V03_IDE_INTSTAT_OFFSET             	0x06
#define   S1R72V03_IDE_INTSTAT_DETECTINTRQ      0x02
#define   S1R72V03_IDE_INTSTAT_IDE_CMP		0x04
#define   S1R72V03_IDE_INTSTAT_ALLINTS          0xF7
#define S1R72V03_DBG_INTSTAT_OFFSET			0x07
#define   S1R72V04_DBG_INTSTAT_EMPTY		0x01
#define   S1R72V04_DBG_INTSTAT_FULL		0x02
#define   S1R72V04_DBG_INTSTAT_NOTEMPTY		0x04
#define   S1R72V04_DBG_INTSTAT_CMP		0x40
#define   S1R72V04_DBG_INTSTAT_ALLINTS		0x47
#define S1R72V03_MAININTENB_OFFSET              	0x10
#define   S1R72V03_MAININTENB_ENSIE_INTSTAT	0x80
#define   S1R72V03_MAININTENB_ENFIFO_INTSTAT	0x10
#define   S1R72V03_MAININTENB_ENIDE_INTSTAT     0x04
#define S1R72V03_SIE_INTENB_OFFSET              	0x12
#define S1R72V03_ENDETECTRESET 			0x10
#define S1R72V03_ENFINISHEDPM			0x20
#define S1R72V03_FIFOINTENB_OFFSET			0x14
#define   S1R72V03_FIFOINTENB_CMP		0x01
#define   S1R72V03_FIFOINTENB_EMPTY		0x04
#define   S1R72V03_FIFOINTENB_FULL		0x08
#define   S1R72V03_FIFOINTENB_NOTEMPTY		0x00     //ol


#define S1R72V03_IDE_INTENB_OFFSET              	0x16
#define   S1R72V03_IDE_INTENB_ENDETECTINTRQ     0x02
#define   S1R72V03_IDE_INTENB_ENIDE_CMP		0x04
#define   S1R72V03_IDE_INTENB_ALL_INTRQ 	0x17

#define S1R72V03_REVISIONNUM_OFFSET             	0x20
#define S1R72V03_CHIPRESET_SWAPPED_OFFSET       0x20
#define S1R72V03_CHIPRESET_OFFSET               	0x21
#define S1R72V03_REVISIONNUM_SWAPPED_OFFSET     0x21
#define S1R72V03_PM_CONTROL_OFFSET            		0x22
#define   S1R72V03_PM_CONTROL_GOACTIVE60      	0x02
#define   S1R72V03_PM_CONTROL_GOSLEEP		0x08
#define S1R72V03_USB_CONTROL_OFFSET            		0x23
#define S1R72V03_USB_STATUS_OFFSET            		0x24
#define S1R72V03_XCVRCONTROL_OFFSET            		0x25

#define S1R72V03_EPNCONTROL_OFFSET            		0x28
#define S1R72V03_EPRFIFOCLR_OFFSET            		0x29
#define S1R72V03_EPAFIFOCLR            		0x1
#define S1R72V03_EPBFIFOCLR            		0x2
#define S1R72V03_EPCFIFOCLR            		0x4

#define S1R72V03_WAKEUPTIM_H_OFFSET            		0x2E
#define S1R72V03_WAKEUPTIM_L_OFFSET            		0x2F
#define S1R72V03_CLRALLJOIN_OFFSET            		0x2A

#define S1R72V03_EP0SETUP_0 _OFFSET			0x30
#define S1R72V03_EP0SETUP_1 _OFFSET			0x31
#define S1R72V03_EP0SETUP_2 _OFFSET			0x32
#define S1R72V03_EP0SETUP_3 _OFFSET			0x33
#define S1R72V03_EP0SETUP_4 _OFFSET			0x34
#define S1R72V03_EP0SETUP_5 _OFFSET			0x35
#define S1R72V03_EP0SETUP_6 _OFFSET			0x36
#define S1R72V03_EP0SETUP_7 _OFFSET			0x37
#define S1R72V03_USB_ADDRESS_OFFSET			0x38
#define S1R72V03_SETUP_CONTROL_OFFSET			0x3A


#define S1R72V03_EP0MAXSIZE_OFFSET			0x40
#define S1R72V03_EP0CONTROL_OFFSET			0x41
#define S1R72V03_EP0CONTROLIN_OFFSET			0x42
#define S1R72V03_EP0CONTROLOUT_OFFSET			0x43
#define S1R72V03_EP0JOIN_OFFSET				0x45

#define S1R72V03_EPAMAXSIZE_OFFSET			0x51
#define S1R72V03_EPACONFIG_0_OFFSET			0x52
#define S1R72V03_EPACONTROL_OFFSET			0x54
#define S1R72V03_EPAJOIN_OFFSET				0x55

#define S1R72V03_EPBMAXSIZE_H_OFFSET			0x58
#define S1R72V03_EPBMAXSIZE_L_OFFSET			0x59
#define S1R72V03_EPBCONFIG_0_OFFSET			0x5A
#define S1R72V03_EPBCONTROL_OFFSET			0x5C
#define S1R72V03_EPBJOIN_OFFSET				0x5D

#define S1R72V03_EPCMAXSIZE_H_OFFSET			0x60
#define S1R72V03_EPCMAXSIZE_L_OFFSET			0x61
#define S1R72V03_EPCCONFIG_0_OFFSET			0x62
#define S1R72V03_EPCCONTROL_OFFSET			0x64
#define S1R72V03_EPCJOIN_OFFSET				0x65


#define S1R72V03_FIFO_RD_H_OFFSET			0x70
#define S1R72V03_FIFO_RD_L_OFFSET			0x71
#define S1R72V03_FIFO_WR_H_OFFSET			0x72
#define S1R72V03_FIFO_WR_L_OFFSET			0x73
#define S1R72V03_FIFO_RDREMAIN_H_OFFSET		0x74
#define   S1R72V03_FIFO_RDREMAIN_VALID                  0x8000
#define S1R72V03_FIFO_RDREMAIN_L_OFFSET 		0x75
#define S1R72V03_FIFO_WRREMAIN_H_OFFSET		0x76
#define S1R72V03_FIFO_WRREMAIN_L_OFFSET 		0x77

#define S1R72V03_DMA0_FIFO_CONTROL_OFFSET		0x80
#define S1R72V03_DMA0_CONFIG_OFFSET			0x81
		#define S1R72V03_DMA0_FREERUN	0x80
		#define S1R72V03_DMA0_MODE	0x40
		#define S1R72V03_ACTIVEDMA0	0x8
		#define S1R72V03_REQASSERTCOUNT1  0x2
		#define S1R72V03_REQASSERTCOUNT0  0x1

#define S1R72V03_DMA0_CONTROL_OFFSET			0x82
		#define S1R72V03_DMA0_GO	0x1
		#define S1R72V03_DMA0_STOP 	0x2
		#define S1R72V03_DMA0_CLR 	0x10
		#define S1R72V03_DMA0_RUNNING 	0x80
		
#define S1R72V03_DMA0_REMAIN_H_OFFSET   		0x84
#define S1R72V03_DMA0_REMAIN_L_OFFSET   		0x85

#define S1R72V03_DMA0_COUNT_HH_OFFSET			0x88
#define S1R72V03_DMA0_COUNT_HL_OFFSET			0x89
#define S1R72V03_DMA0_COUNT_LH_OFFSET			0x8A
#define S1R72V03_DMA0_COUNT_LL_OFFSET			0x8B


#define S1R72V03_DMA0_RDDATA_H_OFFSET			0x8C
#define S1R72V03_DMA0_RDDATA_L_OFFSET			0x8D
#define S1R72V03_DMA0_WRDATA_H_OFFSET			0x8E
#define S1R72V03_DMA0_WRDATA_L_OFFSET			0x8F

#define S1R72V03_IDE_STATUS_OFFSET              	0xA0
#define 	S1R72V03_IDE_STATUS_INTRQ	              	0x20
#define S1R72V03_IDE_CONTROL_OFFSET             	0xA1
#define   S1R72V03_IDE_CONTROL_IDE_GO           0x01
#define   S1R72V03_IDE_CONTROL_IDE_CLR          0x40
#define S1R72V03_IDE_CONFIG_0_OFFSET            	0xA2
#define   S1R72V03_IDE_CONFIG_0_DMA		0x01
#define   S1R72V03_IDE_CONFIG_0_ULTRA		0x02
#define   S1R72V03_IDE_CONFIG_0_IDE_BUSRESET    0x80
#define S1R72V03_IDE_CONFIG_1_OFFSET            	0xA3
#define   S1R72V03_IDE_CONFIG_1_SWAP            0x04
#define   S1R72V03_IDE_CONFIG_1_ACTIVEIDE       0x80
#define S1R72V03_IDE_RMOD_OFFSET                	0xA4
#define   S1R72V03_IDE_RMOD_MAX_TIMING          0x10
#define S1R72V03_IDE_TMOD_OFFSET                	0xA5
#define   S1R72V03_IDE_TMOD_MAX_TIMING          0x0
#define S1R72V03_IDE_UMOD_OFFSET                	0xA6
#define   S1R72V03_IDE_UMOD_MAX_TIMING          0x03


#define S1R72V03_IDE_COUNT_H_OFFSET             	0xAD
#define S1R72V03_IDE_COUNT_M_OFFSET             	0xAE
#define S1R72V03_IDE_COUNT_L_OFFSET             	0xAF
#define S1R72V03_IDE_REGADRS_OFFSET             	0xB0
#define S1R72V03_IDE_RDREGVALUE_H_OFFSET        	0xB2
#define S1R72V03_IDE_RDREGVALUE_L_OFFSET        	0xB3
#define S1R72V03_IDE_WRREGVALUE_H_OFFSET        	0xB4
#define S1R72V03_IDE_WRREGVALUE_L_OFFSET        	0xB5
#define S1R72V03_IDE_REGCONFIG_OFFSET        		0xBC
#define   S1R72V03_IDE_AUTOSTSRDS	       	0x80

#define S1R72V03_MODEPROTECT_OFFSET 			0xEB

#define S1R72V03_CLKSELECT_OFFSET 			0xED
#define    S1R72V03_CLKSELECT 			0x01
#define    S1R72V03_XACTIDE_TERM 		0x80


#define S1R72V03_CHIPCONFIG_SWAPPED_OFFSET      	0xEE
#define S1R72V03_CHIPCONFIG_OFFSET              	0xEF
#define S1R72V03_CPU_CHGENDIAN_SWAPPED_OFFSET   	0xE8
#define S1R72V03_CPU_CHGENDIAN_OFFSET           	0xE9
#define S1R72V03_F0_OFFSET           	0xF0
#define S1R72V03_F1_OFFSET           	0xF1
#define S1R72V03_F2_OFFSET           	0xF2
#define S1R72V03_F3_OFFSET           	0xF3
#define S1R72V03_F6_OFFSET           	0xF6
#define S1R72V03_F7_OFFSET           	0xF7

#define S1R72V03_REG_ADDR(reg) \
  (s1r72v03_data->virt_io_addr + 4* (S1R72V03_##reg##_OFFSET) - 3*((S1R72V03_##reg##_OFFSET) & 0x01))

/* Uncomment to turn on output of taskfile register accesses. */


#ifdef S1R72V03_DEBUG
#define S1R72V03__DBG dev_dbg
#else
#define S1R72V03_DBG(...)
#endif

#define DPCSR(n)		DMAC_REG(0x00A4 + ((n) << 4))


struct pata_pxa_data {
	uint32_t		dma_channel;
	struct pxa_dma_desc	*dma_desc;
	dma_addr_t		dma_desc_addr;
	uint32_t		dma_desc_id;
	/* S1R75V03 IO physical address */
	void 		__iomem *virt_io_addr;
	uint32_t		irq;
	/* DMA IO physical address */
	uint32_t		dma_read_addr;
	uint32_t		dma_write_addr;
	/* PXA DREQ<0:2> pin selector */
	uint32_t		dma_dreq;
	/* DMA DCSR register value */
	uint32_t		dma_dcsr;

	struct completion	dma_done;
};

static struct pata_pxa_data *s1r72v03_data;

static inline void s1r72v03_read_tfreg(void* port)
{
	unsigned long timeout;
	iowrite8(S1R72V03_TF_READ | (int)port,
		 S1R72V03_REG_ADDR(IDE_REGADRS));
	timeout = jiffies + 2;
	while ((ioread8(S1R72V03_REG_ADDR(IDE_REGADRS)) &
		S1R72V03_TF_READ) && time_before(jiffies, timeout))
		cpu_relax();
}

static inline void s1r72v03_write_tfreg(void* port)
{
	unsigned long timeout;
	iowrite8(S1R72V03_TF_WRITE | (int)port, S1R72V03_REG_ADDR(IDE_REGADRS));
	timeout = jiffies + 2;
	while ((ioread8(S1R72V03_REG_ADDR(IDE_REGADRS)) &
		S1R72V03_TF_WRITE) && time_before(jiffies, timeout))
		cpu_relax();
}

static u8 s1r72v03_inb(void* port)
{
	u8 retval;
				
	s1r72v03_read_tfreg(port);
	retval = ioread8(S1R72V03_REG_ADDR(IDE_RDREGVALUE_H));

	
	return retval;
}

static u16 s1r72v03_inw(void* port)
{
	
	s1r72v03_read_tfreg(port);

	return ioread16(S1R72V03_REG_ADDR(IDE_RDREGVALUE_H));
}

static void s1r72v03_insw(void* port, void *addr, u32 count)
{
	u16 *to = addr;
	for (; count; count--) {
		s1r72v03_read_tfreg(port);
		*(to++) = ioread16(S1R72V03_REG_ADDR(IDE_RDREGVALUE_H));
	}
}

static void s1r72v03_outb(void* port,u8 val)
{

	iowrite8(val, S1R72V03_REG_ADDR(IDE_WRREGVALUE_H));
	s1r72v03_write_tfreg(port);
}

static void s1r72v03_outw(void* port,u16 val)
{
	
	iowrite16(val, S1R72V03_REG_ADDR(IDE_WRREGVALUE_H));
	s1r72v03_write_tfreg(port);
}

static void s1r72v03_outsw(void* port, void *addr, u32 count)
{
	u16 *from = addr;

	
	for (; count; count--) {
		iowrite16(*from++, S1R72V03_REG_ADDR(IDE_WRREGVALUE_H));
		s1r72v03_write_tfreg(port);
	}
}

static u8 s1r72v03_read_status(struct ata_port *ap)
{

u8 status =s1r72v03_inb(ap->ioaddr.status_addr);
	return status;
}

static void s1r72v03_exec_command (struct ata_port *ap,
			      const struct ata_taskfile *tf)
{
	s1r72v03_outb(ap->ioaddr.command_addr, tf->command);
	ata_sff_pause(ap);
}

static u8 s1r72v03_read_altstatus (struct ata_port *ap)
{

u8 status =  s1r72v03_inb(ap->ioaddr.altstatus_addr);

	return status;
}

static void s1r72v03_dev_select (struct ata_port *ap, unsigned int device)
{
	u8 tmp;

	if (device == 0)
		tmp = ATA_DEVICE_OBS;
	else
		tmp = ATA_DEVICE_OBS | ATA_DEV1;

	s1r72v03_outb(ap->ioaddr.device_addr, tmp);
	ata_sff_pause(ap);
}

static void s1r72v03_write_devctl(struct ata_port *ap, u8 ctl)
{
	s1r72v03_outb(ap->ioaddr.ctl_addr, ctl);
}



static void s1r72v03_tf_load (struct ata_port *ap, const struct ata_taskfile *tf)
{
	struct ata_ioports *ioaddr = &ap->ioaddr;
	unsigned int is_addr = tf->flags & ATA_TFLAG_ISADDR;
	

	if (tf->ctl != ap->last_ctl) {
		s1r72v03_outb(ioaddr->ctl_addr, tf->ctl);
		ap->last_ctl = tf->ctl;
		ata_wait_idle(ap);
	}

	if (is_addr && (tf->flags & ATA_TFLAG_LBA48)) {
		s1r72v03_outb(ioaddr->feature_addr, tf->hob_feature);
		s1r72v03_outb(ioaddr->nsect_addr, tf->hob_nsect);
		s1r72v03_outb(ioaddr->lbal_addr, tf->hob_lbal);
		s1r72v03_outb(ioaddr->lbam_addr, tf->hob_lbam);
		s1r72v03_outb(ioaddr->lbah_addr, tf->hob_lbah);
		VPRINTK("hob: feat 0x%X nsect 0x%X, lba 0x%X 0x%X 0x%X\n",
			tf->hob_feature,
			tf->hob_nsect,
			tf->hob_lbal,
			tf->hob_lbam,
			tf->hob_lbah);
	}

	if (is_addr) {
		s1r72v03_outb(ioaddr->feature_addr, tf->feature);
		s1r72v03_outb(ioaddr->nsect_addr, tf->nsect);
		s1r72v03_outb(ioaddr->lbal_addr, tf->lbal);
		s1r72v03_outb(ioaddr->lbam_addr, tf->lbam);
		s1r72v03_outb(ioaddr->lbah_addr, tf->lbah);
		VPRINTK("feat 0x%X nsect 0x%X lba 0x%X 0x%X 0x%X\n",
			tf->feature,
			tf->nsect,
			tf->lbal,
			tf->lbam,
			tf->lbah);
	}

	if (tf->flags & ATA_TFLAG_DEVICE) {
		s1r72v03_outb(ioaddr->device_addr, tf->device);
		VPRINTK("device 0x%X\n", tf->device);
	}

	ata_wait_idle(ap);
}



static void s1r72v03_tf_read (struct ata_port *ap, struct ata_taskfile *tf)
{
	struct ata_ioports *ioaddr = &ap->ioaddr;
	
	
	tf->command = s1r72v03_read_status(ap);
	tf->feature = s1r72v03_inb(ioaddr->error_addr);
	tf->nsect = s1r72v03_inb(ioaddr->nsect_addr);
	tf->lbal = s1r72v03_inb(ioaddr->lbal_addr);
	tf->lbam = s1r72v03_inb(ioaddr->lbam_addr);
	tf->lbah = s1r72v03_inb(ioaddr->lbah_addr);
	tf->device = s1r72v03_inb(ioaddr->device_addr);

	if (tf->flags & ATA_TFLAG_LBA48) {
		s1r72v03_outb(ioaddr->ctl_addr, tf->ctl | ATA_HOB);
		tf->hob_feature = s1r72v03_inb(ioaddr->error_addr);
		tf->hob_nsect = s1r72v03_inb(ioaddr->nsect_addr);
		tf->hob_lbal = s1r72v03_inb(ioaddr->lbal_addr);
		tf->hob_lbam = s1r72v03_inb(ioaddr->lbam_addr);
		tf->hob_lbah = s1r72v03_inb(ioaddr->lbah_addr);
		s1r72v03_outb(ioaddr->ctl_addr, tf->ctl);
		ap->last_ctl = tf->ctl;
	}
}



int s1r72v03_softreset(struct ata_link *link, unsigned int *classes,
		      unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	unsigned int slave_possible = ap->flags & ATA_FLAG_SLAVE_POSS;
	unsigned int devmask = 0;
	int rc;
	u8 err;

	DPRINTK("ENTER\n");
	devmask |= (1 << 0);
	classes[0]=ata_sff_dev_classify(&ap->link.device[0],
					devmask & (1 << 0), &err);
		
	return  0;
}


unsigned int s1r72v03_data_xfer(struct ata_device *dev, unsigned char *buffer,
			       unsigned int buflen, int rw)
{
	struct ata_port *ap		= dev->link->ap;
	void __iomem *data_addr		= ap->ioaddr.data_addr;
	unsigned long words;
	int count;

	words = buflen / 2;
	
	
	if (rw) {
		count = 16;
		while (words--) {
			s1r72v03_outw(data_addr,*(uint16_t *)buffer);
			buffer += sizeof(uint16_t);
			/*
			 * Every 16 writes do a read so the bootbus
			 * FIFO doesn't fill up.
			 */
			if (--count == 0) {
				s1r72v03_read_altstatus(ap);
				count = 16;
			}
		}
	} else {
		while (words--) {
			*(uint16_t *)buffer = s1r72v03_inw(data_addr);
			buffer += sizeof(uint16_t);
		}
	}
	/* Transfer trailing 1 byte, if any. */
	if (unlikely(buflen & 0x01)) {
		__le16 align_buf[1] = { 0 };

		if (rw == READ) {
			align_buf[0] = cpu_to_le16(s1r72v03_inw(data_addr));
			memcpy(buffer, align_buf, 1);
		} else {
			memcpy(align_buf, buffer, 1);
			s1r72v03_outw(data_addr,le16_to_cpu(align_buf[0]));
			}
		words++;
	}

	
	return buflen;	
}

void s1r72v03_drain_fifo(struct ata_queued_cmd *qc)
{
	int count;
	struct ata_port *ap;

	/* We only need to flush incoming data when a command was running */
	if (qc == NULL || qc->dma_dir == DMA_TO_DEVICE)
		return;

	ap = qc->ap;
	/* Drain up to 64K of data before we give up this recovery method */
	for (count = 0; (ap->ops->sff_check_status(ap) & ATA_DRQ)
						&& count < 65536; count += 2)
				s1r72v03_inw(ap->ioaddr.data_addr);
		//ioread16(ap->ioaddr.data_addr);

	/* Can become DEBUG later */
	if (count)
		ata_port_printk(ap, KERN_DEBUG,
			"drained %d bytes to clear DRQ.\n", count);

}


static void s1r72v03_set_piomode(struct ata_port *ap, struct ata_device *adev)
{
	u8 reg;
	/* Register values taken from s1r72v03 technical manual */
	switch (adev->pio_mode) {
	case XFER_PIO_0:
		reg = 0xFF;
		break;
	case XFER_PIO_1:
		reg = 0x88;
		break;
	case XFER_PIO_2:
		reg = 0x44;
		break;
	case XFER_PIO_3:
		reg = 0x22;
		break;
	case XFER_PIO_4:
		reg = 0x10;
		break;
	default:
		printk(KERN_WARNING "%s: Unexpected PIO mode: %d\n",
		       DRV_NAME,adev->pio_mode);
		return;
	}
	iowrite8(reg, S1R72V03_REG_ADDR(IDE_TMOD));


}
static void s1r72v03_set_dmamode(struct ata_port *ap, struct ata_device *adev)
{
	u8 reg;
	/* Register values taken from s1r72v03 technical manual */
	switch (adev->dma_mode) {
	case XFER_UDMA_0:  
		reg = 0x06;
		break;
	case XFER_UDMA_1:
		reg = 0x04;
		break;
	case XFER_UDMA_2:
		reg = 0x03;
		break;
	case XFER_UDMA_3:
		reg = 0x02;
		break;
	case XFER_UDMA_4:
		reg = 0x01;
		break;
	case XFER_UDMA_5:
		reg = 0x0;
		break;
	default:
		printk(KERN_WARNING "%s: Unexpected PIO mode: %d\n",
		       DRV_NAME, adev->dma_mode);
		return;
	}
	iowrite8(reg, S1R72V03_REG_ADDR(IDE_UMOD));
}


/*
 * Setup the DMA descriptors. The size is transfer capped at 4k per descriptor,
 * if the transfer is longer, it is split into multiple chained descriptors.
 */
static void pxa_load_dmac(struct scatterlist *sg, struct ata_queued_cmd *qc)
{
	struct pata_pxa_data *pd = qc->ap->private_data;

	uint32_t cpu_len, seg_len;
	dma_addr_t cpu_addr;

	cpu_addr = sg_dma_address(sg);
	cpu_len = sg_dma_len(sg) ; 
	  

	do {
		seg_len = (cpu_len > 0x1000) ? 0x1000 : cpu_len;

		pd->dma_desc[pd->dma_desc_id].ddadr = pd->dma_desc_addr +
			((pd->dma_desc_id + 1) * sizeof(struct pxa_dma_desc));

		if (qc->tf.flags & ATA_TFLAG_WRITE) {
			pd->dma_desc[pd->dma_desc_id].dcmd =  DCMD_BURST32 |
					DCMD_WIDTH2 | (DCMD_LENGTH & seg_len);
			pd->dma_desc[pd->dma_desc_id].dsadr = cpu_addr;
			pd->dma_desc[pd->dma_desc_id].dtadr = pd->dma_write_addr;
			pd->dma_desc[pd->dma_desc_id].dcmd |= DCMD_INCSRCADDR |
						 DCMD_FLOWSRC;
		} else {
			pd->dma_desc[pd->dma_desc_id].dcmd =  DCMD_BURST16 |
					DCMD_WIDTH2 | (DCMD_LENGTH & seg_len);
			pd->dma_desc[pd->dma_desc_id].dsadr = pd->dma_read_addr;
			pd->dma_desc[pd->dma_desc_id].dtadr = cpu_addr;
			pd->dma_desc[pd->dma_desc_id].dcmd |= DCMD_INCTRGADDR  |    
						DCMD_FLOWTRG;
		}

		cpu_len -= seg_len;
		cpu_addr += seg_len;
		pd->dma_desc_id++;

	} while (cpu_len);

	/* Should not happen */
	if (seg_len & 0x1f)
		DALGN |= (1 << pd->dma_channel);
		
		
		
}

/*
 * Prepare taskfile for submission.
 */
static void pxa_qc_prep(struct ata_queued_cmd *qc)
{
	struct pata_pxa_data *pd = qc->ap->private_data;
	int si = 0;
	struct scatterlist *sg;
	
	if (!(qc->flags & ATA_QCFLAG_DMAMAP))
		return;
	

	pd->dma_desc_id = 0;

	DCSR(pd->dma_channel) = 0;
	DALGN &= ~(1 << pd->dma_channel);
	
	for_each_sg(qc->sg, sg, qc->n_elem, si)
		pxa_load_dmac(sg, qc);
	

	pd->dma_desc[pd->dma_desc_id - 1].ddadr = DDADR_STOP;

	/* Fire IRQ only at the end of last block */
	
	pd->dma_desc[pd->dma_desc_id - 1].dcmd |= DCMD_ENDIRQEN;

	DDADR(pd->dma_channel) = pd->dma_desc_addr;
	DRCMR(pd->dma_dreq) = DRCMR_MAPVLD | pd->dma_channel;
	DPCSR(0x80000000);
	

}




/*
 * Configure the DMA controller, load the DMA descriptors, but don't start the
 * DMA controller yet. Only issue the ATA command.
 */
static void pxa_bmdma_setup(struct ata_queued_cmd *qc)
{
       // struct pata_pxa_data *pd = qc->ap->private_data;
            
   unsigned int transferSize= qc->nbytes;  
	
	
	iowrite8( 0, S1R72V03_REG_ADDR(IDE_CONTROL));
	iowrite8(S1R72V03_DMA0_STOP  , S1R72V03_REG_ADDR(DMA0_CONTROL)); 

	
	if (qc->tf.flags & ATA_TFLAG_WRITE) {
	
	iowrite8(S1R72V03_ACTIVEDMA0|S1R72V03_REQASSERTCOUNT1 |S1R72V03_DMA0_MODE,S1R72V03_REG_ADDR(DMA0_CONFIG));
	iowrite8(0x0, S1R72V03_REG_ADDR(EPBMAXSIZE_H));   //60
	iowrite8(0x00, S1R72V03_REG_ADDR(EPBMAXSIZE_L));   //61
	iowrite8( 0x20 | 0x3, S1R72V03_REG_ADDR(EPBCONFIG_0));     //62 OUT
	iowrite8(0x00, S1R72V03_REG_ADDR(EPBCONTROL));     //64
	iowrite8(0x80 |0x10 , S1R72V03_REG_ADDR(EPBJOIN));     //65  dma0 wr*/  
	
	} else {
	
	iowrite8(   S1R72V03_ACTIVEDMA0  |S1R72V03_REQASSERTCOUNT0  |  S1R72V03_DMA0_MODE  , S1R72V03_REG_ADDR(DMA0_CONFIG));
	iowrite8(0x00, S1R72V03_REG_ADDR(EPBMAXSIZE_H)); //58
	iowrite8(0x10, S1R72V03_REG_ADDR(EPBMAXSIZE_L));   //59
	iowrite8(0x80 | 0x20 |  0x2 , S1R72V03_REG_ADDR(EPBCONFIG_0));     //5A  IN
	iowrite8(0x00, S1R72V03_REG_ADDR(EPBCONTROL));     //5c
	iowrite8(0x80 | 0x20 , S1R72V03_REG_ADDR(EPBJOIN));     //5d  dma0 rd  
	
	}	
		
	iowrite8((u8)( transferSize / 0x10000 ), S1R72V03_REG_ADDR(IDE_COUNT_H));
	iowrite8((u8)( transferSize / 0x100 ) , S1R72V03_REG_ADDR(IDE_COUNT_M));
	iowrite8((u8)( transferSize ) , S1R72V03_REG_ADDR(IDE_COUNT_L));
	
	iowrite8((u8)(transferSize >> 24), S1R72V03_REG_ADDR(DMA0_COUNT_HH));
	iowrite8((u8)(transferSize >> 16), S1R72V03_REG_ADDR(DMA0_COUNT_HL));
	iowrite8((u8)(transferSize >> 8), S1R72V03_REG_ADDR(DMA0_COUNT_LH));
	iowrite8((u8)(transferSize ), S1R72V03_REG_ADDR(DMA0_COUNT_LL));
	
	       
    qc->ap->ops->sff_exec_command(qc->ap, &qc->tf);   
      

}

/*
 * Execute the DMA transfer.
 */
static void pxa_bmdma_start(struct ata_queued_cmd *qc)
{
	struct pata_pxa_data *pd = qc->ap->private_data;
	init_completion(&pd->dma_done);		

	iowrite8( S1R72V03_IDE_CONTROL_IDE_GO, S1R72V03_REG_ADDR(IDE_CONTROL));
	iowrite8(S1R72V03_DMA0_GO , S1R72V03_REG_ADDR(DMA0_CONTROL));
	DCSR(pd->dma_channel) |= DCSR_RUN;
}

/*
 * Wait until the DMA transfer completes, then stop the DMA controller.
 */
static void pxa_bmdma_stop(struct ata_queued_cmd *qc)
{
	
	struct pata_pxa_data *pd = qc->ap->private_data;
	
	
	if ((DCSR(pd->dma_channel) & DCSR_RUN) &&
		wait_for_completion_timeout(&pd->dma_done, HZ))
		dev_err(qc->ap->dev, "Timeout waiting for DMA completion!");
	else
	{
	iowrite8( 0, S1R72V03_REG_ADDR(IDE_CONTROL));
	iowrite8(S1R72V03_DMA0_STOP | S1R72V03_DMA0_CLR , S1R72V03_REG_ADDR(DMA0_CONTROL));	
	iowrite8(0x20, S1R72V03_REG_ADDR(EPNCONTROL));	
	DCSR(pd->dma_channel) &= ~ DCSR_RUN;
	}
	
}

/*
 * Read DMA status. The bmdma_stop() will take care of properly finishing the
 * DMA transfer so we always have DMA-complete interrupt here.
 */
static unsigned char pxa_bmdma_status(struct ata_port *ap)
{
	struct pata_pxa_data *pd = ap->private_data;
	unsigned char ret = ATA_DMA_INTR;
	
	printk(KERN_INFO " pxa_bmdma status error %x :\n",pd->dma_dcsr);
	
	if (pd->dma_dcsr & DCSR_BUSERR)
		{
		ret |= ATA_DMA_ERR;
		
		}
		
	
	return ret;
}

/*
 * No IRQ register present so we do nothing.
 */
static void pxa_irq_clear(struct ata_port *ap)
{
}

/*
 * Check for ATAPI DMA. ATAPI DMA is unsupported by this driver. It's still
 * unclear why ATAPI has DMA issues.
 */
static int pxa_check_atapi_dma(struct ata_queued_cmd *qc)
{
	return -EOPNOTSUPP;
}

static struct scsi_host_template pxa_ata_sht = {
	ATA_BMDMA_SHT(DRV_NAME),
};

static struct ata_port_operations pxa_ata_port_ops = {
	.inherits		= &ata_bmdma_port_ops,
	.cable_detect		= ata_cable_40wire,
	.set_piomode		= s1r72v03_set_piomode,
	.set_dmamode		= s1r72v03_set_dmamode,
	
	.sff_tf_load		= s1r72v03_tf_load,
	.sff_tf_read		= s1r72v03_tf_read,
	.sff_exec_command	= s1r72v03_exec_command,
	.sff_check_status	= s1r72v03_read_status,
	.sff_check_altstatus	= s1r72v03_read_altstatus,
	.sff_dev_select		= s1r72v03_dev_select,
	.sff_set_devctl		= s1r72v03_write_devctl,
	.softreset			= s1r72v03_softreset,
	.sff_data_xfer		= s1r72v03_data_xfer,
	.sff_drain_fifo		= s1r72v03_drain_fifo,
	
	.bmdma_setup		= pxa_bmdma_setup,
	.bmdma_start		= pxa_bmdma_start,
	.bmdma_stop			= pxa_bmdma_stop,
	.bmdma_status		= pxa_bmdma_status,

	.check_atapi_dma	= pxa_check_atapi_dma,
	.port_start			= ATA_OP_NULL,	/* don't need PRD table */
	.sff_irq_clear		= pxa_irq_clear,
	.qc_prep			= pxa_qc_prep,
	
};

/*
 * DMA interrupt handler.
 */
static void pxa_ata_dma_irq(int dma, void *port)
{
	struct ata_port *ap = port;
	struct pata_pxa_data *pd = ap->private_data;
		
		pd->dma_dcsr = DCSR(dma);
	DCSR(dma) = pd->dma_dcsr;
	
	if (pd->dma_dcsr & DCSR_STOPSTATE)
		complete(&pd->dma_done);
	
	  unsigned int transferSize= 0x10000* ioread8( S1R72V03_REG_ADDR(IDE_COUNT_H))+
	  0x100*ioread8( S1R72V03_REG_ADDR(IDE_COUNT_M))+
	  ioread8(S1R72V03_REG_ADDR(IDE_COUNT_L));
	  
	  unsigned int remainbyte=0x10000* ioread8( S1R72V03_REG_ADDR(DMA0_COUNT_HL))+
	  0x100*ioread8( S1R72V03_REG_ADDR(DMA0_COUNT_LH))+
	  ioread8(S1R72V03_REG_ADDR(DMA0_COUNT_LL));
		
	
	 if (!remainbyte)
	 {
		iowrite8( 0, S1R72V03_REG_ADDR(IDE_CONTROL));
		iowrite8(S1R72V03_DMA0_STOP , S1R72V03_REG_ADDR(DMA0_CONTROL));	
		if (ioread16( S1R72V03_REG_ADDR(DMA0_REMAIN_L))) 
			while (!(S1R72V03_IDE_STATUS_INTRQ&ioread8(S1R72V03_REG_ADDR(IDE_STATUS)))) mdelay(2);
		else
		     while (!(S1R72V03_IDE_STATUS_INTRQ&ioread8(S1R72V03_REG_ADDR(IDE_STATUS)))) ndelay(100);
		
		ata_sff_interrupt(s1r72v03_data->irq, ap->host) ;
	}
	 else 
		printk(KERN_INFO " Error pxa_bmdma irq with %x : fifo %x remain %x  len %x   \n",transferSize,ioread16( S1R72V03_REG_ADDR(DMA0_REMAIN_L)),remainbyte, DCMD(pd->dma_channel) );
  
 
	
}



static int __devinit pxa_ata_probe(struct platform_device *pdev)
{
	struct ata_host *host;
	struct ata_port *ap;
	struct resource *io_res;
	struct resource *ctl_res;
	struct resource *dma_res;
	struct resource *drq_res;
	struct pata_pxa_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;

	
	if (pdev->num_resources != 2) {
		dev_err(&pdev->dev, "invalid number of resources\n");
		return -EINVAL;
	}

	if (pdata != NULL) {
		if (pdata->gpio_setup != NULL) {
			ret = pdata->gpio_setup();
			if (ret)
				return ret;
		}
	}


/*
	 * Allocate and load driver's internal data structure
	 */
	s1r72v03_data = devm_kzalloc(&pdev->dev, sizeof(struct pata_pxa_data),
								GFP_KERNEL);
	if (!s1r72v03_data)
		return -ENOMEM;


	/*
	 * IO base address
	 */
	io_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(io_res == NULL))
		return -EINVAL;

	/*
	 * DRQ pin
	 */
	drq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(drq_res == NULL))
		return -EINVAL;
		
	// DREQ<0>	
		
	s1r72v03_data->dma_dreq = 0;
		

	/*
	 * Allocate the host
	 */
	host = ata_host_alloc(&pdev->dev, 1);
	if (!host)
		return -ENOMEM;

	ap		= host->ports[0];
	ap->ops		= &pxa_ata_port_ops;
	ap->pio_mask	= ATA_PIO4_ONLY;
	ap->mwdma_mask	= ATA_MWDMA2;   
	ap->udma_mask	= ATA_UDMA2;		
	
	ap->flags |=  ATA_FLAG_PIO_POLLING |  ATA_FLAG_NO_ATAPI  ; 
	ap->ioaddr.cmd_addr	= NULL;	/* Don't have a classic reg block */
	ap->ioaddr.bmdma_addr	= NULL;

	/*
	 * Adjust register offsets
	 */
	ap->ioaddr.altstatus_addr = S1R72V03_ALT_STATUS_TF_REG;
	ap->ioaddr.ctl_addr 	= S1R72V03_DEVICE_CONTROL_TF_REG;
	ap->ioaddr.data_addr	= ATA_REG_DATA ;
	ap->ioaddr.error_addr	= ATA_REG_ERR ;
	ap->ioaddr.feature_addr	= ATA_REG_FEATURE;
	ap->ioaddr.nsect_addr	= ATA_REG_NSECT ;
	ap->ioaddr.lbal_addr	= ATA_REG_LBAL ;
	ap->ioaddr.lbam_addr	= ATA_REG_LBAM ;
	ap->ioaddr.lbah_addr	= ATA_REG_LBAH ;
	ap->ioaddr.device_addr	= ATA_REG_DEVICE ;
	ap->ioaddr.status_addr	= ATA_REG_STATUS ;
	ap->ioaddr.command_addr	= ATA_REG_CMD ;

	

	if (!request_mem_region(io_res->start,
			resource_size(io_res), DRV_NAME)) {
		printk(KERN_ALERT
		       "%s: could not allocate io region\n", DRV_NAME);
		return -EBUSY;
	}

	s1r72v03_data->virt_io_addr = ioremap(io_res->start,
					resource_size(io_res));
	if (!s1r72v03_data->virt_io_addr) {
		printk(KERN_ALERT
		       "%s: could not ioremap io region\n", DRV_NAME);
		ret = -ENOMEM;
		goto release_mem_region;
	}

	printk(KERN_INFO   "ata region=%x size=%x\n", s1r72v03_data->virt_io_addr,resource_size(io_res));
	
	ap->private_data = s1r72v03_data;
    s1r72v03_data->dma_read_addr = io_res->start + 4*S1R72V03_DMA0_RDDATA_H_OFFSET;
	s1r72v03_data->dma_write_addr = io_res->start + 4*S1R72V03_DMA0_WRDATA_H_OFFSET;
	
		
	/* Reset chip. We don't know if the endianness is wrong, so
	   write to both bytes. The other one is the revision register
	   which is read-only (so this won't break anything).
	 */
	iowrite8(0x1, S1R72V03_REG_ADDR(CHIPRESET));
	iowrite8(0x1, S1R72V03_REG_ADDR(CHIPRESET_SWAPPED));
	mdelay(1);
	iowrite8(0x0, S1R72V03_REG_ADDR(CHIPRESET));
	iowrite8(0x0, S1R72V03_REG_ADDR(CHIPRESET_SWAPPED));
	
	
	
	/* Before we fix the endianness, there's a strange issue where reading
	   the revision register can return 0 unless you read the reset
	   register first.
	 */
		
	if (ioread8(S1R72V03_REG_ADDR(REVISIONNUM)) != S1R72V03_HW_REVISION) {
		printk(KERN_ALERT
			"revision register not detected, assuming "
			"endianness is reversed\n %x",ioread8(S1R72V03_REG_ADDR(REVISIONNUM)));
		/* Make sure revision is showing up in its byte-swapped
		   position.
		 */
		if (ioread8(S1R72V03_REG_ADDR(REVISIONNUM_SWAPPED)) !=
		    S1R72V03_HW_REVISION) {
			printk(KERN_ALERT
			       "%s: could not read revision register before "
			       "setting endianness\n", DRV_NAME);
			goto release_iomap;
		}

		/* Initialize chip config register and fix endianness. */
		iowrite8(S1R72V03_CHIPCONFIG_VALUE,
			 S1R72V03_REG_ADDR(CHIPCONFIG_SWAPPED));
		ioread8(S1R72V03_REG_ADDR(CPU_CHGENDIAN_SWAPPED));
		printk(KERN_ALERT
			"revision register not detected, assuming "
			"endianness 111 %x",ioread16(S1R72V03_REG_ADDR(REVISIONNUM)));
	} else {
		printk(KERN_ALERT
			"revision register detected, assuming "
			"endianness is correct\n");
		/* Initialize chip config register. */
		iowrite8(S1R72V03_CHIPCONFIG_VALUE,
			 S1R72V03_REG_ADDR(CHIPCONFIG));
	}
	ioread8(S1R72V03_REG_ADDR(CPU_CHGENDIAN));

	/* Check revision, it should show up correctly now. */
	if (ioread8(S1R72V03_REG_ADDR(REVISIONNUM)) != S1R72V03_HW_REVISION) {
		printk(KERN_ALERT "%s: could not read revision register "
		       "after setting endianness\n", DRV_NAME);
		goto release_iomap;
	}

	/* The chip turns on in sleep mode. Go into Active60. */
	
	iowrite8(S1R72V03_CLKSELECT | S1R72V03_XACTIDE_TERM,S1R72V03_REG_ADDR(CLKSELECT));
		
	
	iowrite8(0xFF,S1R72V03_REG_ADDR(WAKEUPTIM_H ));
	iowrite8(0xFF, S1R72V03_REG_ADDR(WAKEUPTIM_L));
	
	
	iowrite8(S1R72V03_PM_CONTROL_GOACTIVE60,
		 S1R72V03_REG_ADDR(PM_CONTROL));

	/* Wait for transition to Active60 state. */
	int num_sleeps = 0;
	do {
		udelay(S1R72V03_PM_WAIT_USEC);
		num_sleeps += S1R72V03_PM_WAIT_USEC;
	} while (!(ioread8(S1R72V03_REG_ADDR(SIE_INTSTAT)) &
		   S1R72V03_SIE_INTSTAT_FINISHEDPM) &&
		 num_sleeps < S1R72V03_MAX_PM_WAIT_USEC);
	if (!(ioread8(S1R72V03_REG_ADDR(SIE_INTSTAT)) &
	      S1R72V03_SIE_INTSTAT_FINISHEDPM)) {
		printk(KERN_ALERT "%s: could not switch to Active60 state\n",
		       DRV_NAME);
		goto release_iomap;
	}
	


	/* Turn on IDE interface in the chip. */
	iowrite8(S1R72V03_IDE_CONFIG_1_SWAP | S1R72V03_IDE_CONFIG_1_ACTIVEIDE,
		 S1R72V03_REG_ADDR(IDE_CONFIG_1));

	/* "Initialize the IDE circuit." */
	iowrite8(S1R72V03_CLKSELECT,  S1R72V03_REG_ADDR(CLKSELECT));
	

	/* Set the assert pulse and negate pulse width to the max for now. */
	iowrite8(S1R72V03_IDE_RMOD_MAX_TIMING, S1R72V03_REG_ADDR(IDE_RMOD));
	iowrite8(S1R72V03_IDE_TMOD_MAX_TIMING, S1R72V03_REG_ADDR(IDE_TMOD));
	iowrite8(S1R72V03_IDE_UMOD_MAX_TIMING, S1R72V03_REG_ADDR(IDE_UMOD));
	
		
	/*
	 * Allocate space for the DMA descriptors
	 */
	s1r72v03_data->dma_desc = dmam_alloc_coherent(&pdev->dev, PAGE_SIZE,
					&s1r72v03_data->dma_desc_addr, GFP_KERNEL);
	if (!s1r72v03_data->dma_desc)
		return -EINVAL;

	/*
	 * Request the DMA channel
	 */
	s1r72v03_data->dma_channel = pxa_request_dma(DRV_NAME, DMA_PRIO_LOW,
						pxa_ata_dma_irq, ap);
	if (s1r72v03_data->dma_channel < 0)
		return -EBUSY;

	/*
	 * Stop and clear the DMA channel
	 */
	DCSR(s1r72v03_data->dma_channel) = 0;
	
	/* Reset the IDE bus and set to ultra DMA mode. */
	
	iowrite8( S1R72V03_IDE_CONFIG_0_ULTRA | S1R72V03_IDE_CONFIG_0_DMA ,
		 S1R72V03_REG_ADDR(IDE_CONFIG_0));
		

	/*
	 * Activate the ATA host
	 */
	 
	ret = ata_host_activate(host, 0,  NULL,0, &pxa_ata_sht);

	if (ret)
		pxa_free_dma(s1r72v03_data->dma_channel);
     	return ret;
     	
	
release_iomap:
	iounmap(s1r72v03_data->virt_io_addr);

release_mem_region:
	release_mem_region(io_res->start,
			resource_size(io_res));
	return ret;
	
	
	
}

static int __devexit pxa_ata_remove(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	struct pata_pxa_data *pxadata = host->ports[0]->private_data;

	pxa_free_dma(pxadata->dma_channel);

	ata_host_detach(host);

	return 0;
}

static struct platform_driver pxa_ata_driver = {
	.probe		= pxa_ata_probe,
	.remove		= __devexit_p(pxa_ata_remove),
	.driver		= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init pxa_ata_init(void)
{
	return platform_driver_register(&pxa_ata_driver);
}

static void __exit pxa_ata_exit(void)
{
	platform_driver_unregister(&pxa_ata_driver);
}

module_init(pxa_ata_init);
module_exit(pxa_ata_exit);

MODULE_AUTHOR("Grisoni Olivier");
MODULE_DESCRIPTION("DMA-capable driver for PATA S1rv7203 on PXA CPU");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:" DRV_NAME);
