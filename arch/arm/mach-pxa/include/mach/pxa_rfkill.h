/*
 * pxa rfkill from Tosa bluetooth built-in chip control.
 *
 * Later it may be shared with some other platforms.
 * from 2008 Dmitry Baryshkov
 *
 * Copyright (c) 2009 Olivier Grisoni
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef PXA_RFKILL_H
#define PXA_RFKILL_H

struct pxa_rfkill_data {
	const char * name;
	int gpio_pwr1;
	int gpio_pwr2;
	int gpio_reset;
	int gpio_reset_inverted;
	int pxa_rfkill_type;
	bool state;

	};



#endif

