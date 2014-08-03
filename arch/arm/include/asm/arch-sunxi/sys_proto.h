/*
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SYS_PROTO_H_
#define _SYS_PROTO_H_

#include <linux/types.h>

void sdelay(unsigned long);

int  sunxi_cons_index(void);
int  soc_is_sun4i(void);
int  soc_is_sun5i(void);
int  soc_is_sun7i(void);

#endif
