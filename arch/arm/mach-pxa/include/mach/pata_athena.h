/*
 * Generic PXA ATHENA driver for HTC Athena
 *
 * Copyright (C) 2011 Grisoni Olivier
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

#ifndef	__MACH_PATA_ATHENA_H__
#define	__MACH_PATA_ATHENA_H__

struct pata_pxa_pdata {
	/* PXA DMA DREQ<0:2> pin */
	uint32_t	dma_dreq;
	/* Register shift */
	uint32_t	reg_shift;
	/* IRQ flags */
	uint32_t	irq_flags;
	int (*gpio_setup)(void);
};

#endif	/* __MACH_PATA_PXA_H__ */
