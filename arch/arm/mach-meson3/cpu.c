/*
 * arch/arm/mach-meson6/cpu.c
 *
 * Copyright (C) 2011-2012 Amlogic, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <plat/regops.h>
#include <linux/string.h>
#include <asm/hardware/cache-l2x0.h>

static int __init meson_cache_init(void)
{

    /*
     * Early BRESP, I/D prefetch enabled
     * Non-secure enabled
     * 128kb (16KB/way),
     * 8-way associativity,
     * evmon/parity/share disabled
     * Full Line of Zero enabled
         * Bits:  .111 .... .100 0010 0000 .... .... ...1
     */
    l2x0_init((void __iomem *)IO_PL310_BASE, 0x7c420001, 0xff800fff);
    return 0;

}

early_initcall(meson_cache_init);

unsigned (*get_cpu_temperature_celius)(void)=NULL;
EXPORT_SYMBOL_GPL(get_cpu_temperature_celius);
int get_cpu_temperature(void)
{
    return get_cpu_temperature_celius?get_cpu_temperature_celius():-1;
}
int mali_revb_flag = -1;
//int mali_version(void)

EXPORT_SYMBOL_GPL(mali_revb_flag);
static int __init maliversion(char *str)
{
    mali_revb_flag=-1;
    if(strncasecmp(str,"a",1)==0)
        mali_revb_flag = 0;
    else if(strncasecmp(str,"b",1)==0)
       mali_revb_flag = 1;


    return 1;
}
__setup("mali_version=",maliversion);
