/*
 * customer/boards/board-m6g02.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>

#include <linux/io.h>
#include <plat/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <linux/io.h>
#include <plat/io.h>

#include <mach/map.h>
#include <mach/i2c_aml.h>
#include <mach/nand.h>
#include <mach/usbclock.h>
#include <mach/usbsetting.h>
#include <mach/pm.h>
#include <mach/gpio_data.h>
#include <mach/pinmux.h>

#include <linux/uart-aml.h>
#include <linux/i2c-aml.h>

#include "board-m6refg02.h"

#ifdef CONFIG_MMC_AML
#include <mach/mmc.h>
#endif

#ifdef CONFIG_CARDREADER
#include <mach/card_io.h>
#endif // CONFIG_CARDREADER

#include <mach/gpio.h>
#ifdef CONFIG_EFUSE
#include <linux/efuse.h>
#endif

#define CONFIG_SND_SOC_DUMMY_CODEC
#ifdef CONFIG_SND_SOC_DUMMY_CODEC
#include <sound/dummy_codec.h>
#endif
#ifdef CONFIG_AM_WIFI
#include <plat/wifi_power.h>
#endif
#ifdef CONFIG_AML_HDMI_TX
#include <plat/hdmi_config.h>
#endif

#ifdef CONFIG_AM_ETHERNET
#include <mach/am_regs.h>
#include <mach/am_eth_reg.h>
#endif

#define RESERVED_MEM_END    (STREAMBUF_ADDR_END)
static struct resource meson_fb_resource[] = {
    [0] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = OSD2_ADDR_START,
        .end   = OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },

};

static struct resource meson_codec_resource[] = {
    [0] = {
        .start = CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = STREAMBUF_ADDR_START,
        .end   = STREAMBUF_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

#ifdef CONFIG_POST_PROCESS_MANAGER
static struct resource ppmgr_resources[] = {
    [0] = {
        .start = PPMGR_ADDR_START,
        .end   = PPMGR_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device ppmgr_device = {
    .name       = "ppmgr",
    .id         = 0,
    .num_resources = ARRAY_SIZE(ppmgr_resources),
    .resource      = ppmgr_resources,
};
#endif

#ifdef CONFIG_FREE_SCALE
static struct resource freescale_resources[] = {
    [0] = {
        .start = FREESCALE_ADDR_START,
        .end   = FREESCALE_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device freescale_device =
{
    .name           = "freescale",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(freescale_resources),
    .resource       = freescale_resources,
};
#endif

#if defined(CONFIG_AM_DEINTERLACE) || defined (CONFIG_DEINTERLACE)
static struct resource deinterlace_resources[] = {
    [0] = {
        .start =  DI_ADDR_START,
        .end   = DI_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device deinterlace_device = {
    .name       = "deinterlace",
    .id         = 0,
    .num_resources = ARRAY_SIZE(deinterlace_resources),
    .resource      = deinterlace_resources,
};
#endif

static  int __init setup_devices_resource(void)
{
    setup_fb_resource(meson_fb_resource, ARRAY_SIZE(meson_fb_resource));
#ifdef CONFIG_AM_STREAMING
    setup_codec_resource(meson_codec_resource, ARRAY_SIZE(meson_codec_resource));
#endif
    return 0;
}

/***********************************************************
*Remote Section
************************************************************/
#ifdef CONFIG_AM_REMOTE
#include <plat/remote.h>

static pinmux_item_t aml_remote_pins[] = {
    {
	.reg = PINMUX_REG(AO),
	.clrmask = 0,
	.setmask = 1 << 0,
    },
    PINMUX_END_ITEM
};

static struct aml_remote_platdata aml_remote_pdata __initdata = {
    .pinmux_items  = aml_remote_pins,
    .ao_baseaddr = P_AO_IR_DEC_LDR_ACTIVE, 
};

static void __init setup_remote_device(void)
{
    meson_remote_set_platdata(&aml_remote_pdata);
}
#endif
/***********************************************************************
 * I2C Section
 **********************************************************************/
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
static bool pinmux_dummy_share(bool select)
{
    return select;
}

static pinmux_item_t aml_i2c_a_pinmux_item[] = {
    {
        .reg = 5,
        //.clrmask = (3<<24)|(3<<30),
        .setmask = 3<<26
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_a = {
    .wait_count             = 50000,
    .wait_ack_interval   = 5,
    .wait_read_interval  = 5,
    .wait_xfer_interval   = 5,
    .master_no          = AML_I2C_MASTER_A,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_a_pinmux_item[0]
    }
};

static pinmux_item_t aml_i2c_b_pinmux_item[]={
    {
        .reg = 5,
        //.clrmask = (3<<28)|(3<<26),
        .setmask = 3<<30
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_b = {
    .wait_count         = 50000,
    .wait_ack_interval = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no          = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_300K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_b_pinmux_item[0]
    }
};

static pinmux_item_t aml_i2c_ao_pinmux_item[] = {
    {
        .reg = AO,
        .clrmask  = 3<<1,
        .setmask = 3<<5
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_ao = {
    .wait_count         = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no          = AML_I2C_MASTER_AO,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux      = {
        .pinmux         = &aml_i2c_ao_pinmux_item[0]
    }
};

static struct resource aml_i2c_resource_a[] = {
    [0] = {
        .start = MESON_I2C_MASTER_A_START,
        .end   = MESON_I2C_MASTER_A_END,
        .flags = IORESOURCE_MEM,
    }
};

static struct resource aml_i2c_resource_b[] = {
    [0] = {
        .start = MESON_I2C_MASTER_B_START,
        .end   = MESON_I2C_MASTER_B_END,
        .flags = IORESOURCE_MEM,
    }
};

static struct resource aml_i2c_resource_ao[] = {
    [0]= {
        .start =    MESON_I2C_MASTER_AO_START,
        .end   =    MESON_I2C_MASTER_AO_END,
        .flags =    IORESOURCE_MEM,
    }
};

static struct platform_device aml_i2c_device_a = {
    .name         = "aml-i2c",
    .id       = 0,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource_a),
    .resource     = aml_i2c_resource_a,
    .dev = {
        .platform_data = &aml_i2c_plat_a,
    },
};

static struct platform_device aml_i2c_device_b = {
    .name         = "aml-i2c",
    .id       = 1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource_b),
    .resource     = aml_i2c_resource_b,
    .dev = {
        .platform_data = &aml_i2c_plat_b,
    },
};

static struct platform_device aml_i2c_device_ao = {
    .name         = "aml-i2c",
    .id       = 2,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource_ao),
    .resource     = aml_i2c_resource_ao,
    .dev = {
        .platform_data = &aml_i2c_plat_ao,
    },
};

static struct i2c_board_info __initdata aml_i2c_bus_info_b[] = {
#if (defined CONFIG_TVIN_IT660X)
    {
        I2C_BOARD_INFO("it660x_i2c", 0x90>>1),
        //.irq = ITK_INT,
        //.platform_data = (void *)&itk_pdata,
    },
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_a[] = {
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_HI2056
    {
        /*hi2056 i2c address is 0x48/0x49*/
        I2C_BOARD_INFO("hi2056_i2c",  0x48 >> 1),
        .platform_data = (void *)&mipi_hi2056_data,
    },
#endif
};

static struct i2c_board_info __initdata aml_i2c_bus_info_ao[] = {

};
static int __init aml_i2c_init(void)
{
    i2c_register_board_info(0, aml_i2c_bus_info_a,
        ARRAY_SIZE(aml_i2c_bus_info_a));
    i2c_register_board_info(1, aml_i2c_bus_info_b,
        ARRAY_SIZE(aml_i2c_bus_info_b));
    i2c_register_board_info(2, aml_i2c_bus_info_ao,
        ARRAY_SIZE(aml_i2c_bus_info_ao));
  printk("%s\n", __func__);
    return 0;
}

#endif

#if defined(CONFIG_I2C_SW_AML)
//#include "gpio_data.c"
static struct aml_sw_i2c_platform aml_sw_i2c_plat_1 = {
    .sw_pins = {
        .scl_reg_out        = P_PREG_PAD_GPIO4_O,
        .scl_reg_in     = P_PREG_PAD_GPIO4_I,
        .scl_bit            = 26,
        .scl_oe         = P_PREG_PAD_GPIO4_EN_N,
        .sda_reg_out        = P_PREG_PAD_GPIO4_O,
        .sda_reg_in     = P_PREG_PAD_GPIO4_I,
        .sda_bit            = 25,
        .sda_oe         = P_PREG_PAD_GPIO4_EN_N,
    },  
    .udelay         = 2,
    .timeout            = 100,
};

static struct platform_device aml_sw_i2c_device_1 = {
    .name         = "aml-sw-i2c",
    .id       = -1,
    .dev = {
        .platform_data = &aml_sw_i2c_plat_1,
    },
};
#endif

/***********************************************************************
 * UART Section
 **********************************************************************/
static pinmux_item_t uart_pins[] = {
    {
        .reg = PINMUX_REG(AO),
        .setmask = 3 << 11
    },
    {
        .reg = PINMUX_REG(4),
        .setmask = 3 << 12
    },
    PINMUX_END_ITEM
};

static pinmux_set_t aml_uart_ao = {
    .chip_select = NULL,
    .pinmux = &uart_pins[0]
};
static pinmux_set_t aml_uart_a = {
    .chip_select = NULL,
    .pinmux = &uart_pins[1]
};
static struct aml_uart_platform aml_uart_plat = {
    .uart_line[0]   = UART_AO,
    .uart_line[1]   = UART_A,
    .uart_line[2]   = UART_B,
    .uart_line[3]   = UART_C,
    .uart_line[4]   = UART_D,

    .pinmux_uart[0] = (void*)&aml_uart_ao,
    .pinmux_uart[1] = (void*)&aml_uart_a,
    .pinmux_uart[2] = NULL,
    .pinmux_uart[3] = NULL,
    .pinmux_uart[4] = NULL
};

static struct platform_device aml_uart_device = {
    .name       = "mesonuart",
    .id     = -1,
    .num_resources  = 0,
    .resource   = NULL,
    .dev = {
        .platform_data = &aml_uart_plat,
    },
};

/***********************************************************************
 * Nand Section
 **********************************************************************/

#ifdef CONFIG_AM_NAND
static struct mtd_partition normal_partition_info[] = {
    {
        .name 	= "logo",
        .offset = 8*1024*1024,
        .size	= 4*1024*1024
    },
    {
        .name 	= "boot",
        .offset = 12*1024*1024,
        .size 	= 8*1024*1024,
    },
    {
        .name 	= "system",
        .offset = 20*1024*1024,
        .size 	= 512*1024*1024,
    },
    {
        .name	= "cache",
        .offset = 532*1024*1024,
        .size 	= 500*1024*1024,
    },
    {
        .name 	= "backup",
        .offset = 1032*1024*1024,
        .size 	= 300*1024*1024,
    },
    {
        .name 	= "userdata",
        .offset = MTDPART_OFS_APPEND,
        .size 	= MTDPART_SIZ_FULL,
    },
};

static struct aml_nand_platform aml_nand_mid_platform[] = {
#ifndef CONFIG_AMLOGIC_SPI_NOR
    {
        .name = NAND_BOOT_NAME,
        .chip_enable_pad = AML_NAND_CE0,
        .ready_busy_pad = AML_NAND_CE0,
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE),
            },
        },
        .T_REA = 20,
        .T_RHOH = 15,
    },
#endif
    {
        .name = NAND_NORMAL_NAME,
        .chip_enable_pad = (AML_NAND_CE0/* | (AML_NAND_CE1 << 4) | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)*/),
        .ready_busy_pad = (AML_NAND_CE0 /*| (AML_NAND_CE0 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)*/),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .nr_partitions = ARRAY_SIZE(normal_partition_info),
                .partitions = normal_partition_info,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_1K_MODE | NAND_TWO_PLANE_MODE),
            },
        },
        .T_REA = 20,
        .T_RHOH = 15,
    }
};

static struct aml_nand_device aml_nand_mid_device = {
    .aml_nand_platform = aml_nand_mid_platform,
    .dev_num = ARRAY_SIZE(aml_nand_mid_platform),
};

static struct resource aml_nand_resources[] = {
    {
        .start = 0xc1108600,
        .end = 0xc1108624,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device aml_nand_device = {
    .name = "aml_nand",
    .id = 0,
    .num_resources = ARRAY_SIZE(aml_nand_resources),
    .resource = aml_nand_resources,
    .dev = {
        .platform_data = &aml_nand_mid_device,
    },
};
#endif

#if defined(CONFIG_AMLOGIC_SPI_NOR)
static struct mtd_partition spi_partition_info[] = {
    {
        .name = "bootloader",
        .offset = 0,
        .size = 0x200000,
    },

    {
        .name = "ubootenv",
        .offset = 0x3e000,
        .size = 0x2000,
    },

};

static struct flash_platform_data amlogic_spi_platform = {
    .parts = spi_partition_info,
    .nr_parts = ARRAY_SIZE(spi_partition_info),
};

static struct resource amlogic_spi_nor_resources[] = {
    {
        .start = 0xcc000000,
        .end = 0xcfffffff,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device amlogic_spi_nor_device = {
    .name = "AMLOGIC_SPI_NOR",
    .id = -1,
    .num_resources = ARRAY_SIZE(amlogic_spi_nor_resources),
    .resource = amlogic_spi_nor_resources,
    .dev = {
        .platform_data = &amlogic_spi_platform,
    },
};
#endif

/***********************************************************************
* Audio section
**********************************************************************/
static struct resource aml_m6_audio_resource[] = {
    [0] = {
        .start = 0,
        .end = 0,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device aml_audio = {
    .name = "aml-audio",
    .id = 0,
};

static struct platform_device aml_audio_dai = {
    .name = "aml-dai",
    .id = 0,
};

#if defined(CONFIG_SND_SOC_DUMMY_CODEC)
static pinmux_item_t dummy_codec_pinmux[] = {
    /* I2S_MCLK I2S_BCLK I2S_LRCLK I2S_DOUT */
    {
        .reg = PINMUX_REG(9),
        .setmask = (1 << 7) | (1 << 5) | (1 << 9) | (1 << 4),
        .clrmask = (7 << 19) | (7 << 1) | (3 << 10) | (1 << 6),
    },
    {
        .reg = PINMUX_REG(8),
        .clrmask = (0x7f << 24),
    },
    /* spdif out from GPIOC_9 */
    {
        .reg = PINMUX_REG(3),
        .setmask = (1<<24),
    },
    /* mask spdif out from GPIOE_8 */
    {
        .reg = PINMUX_REG(9),
        .clrmask = (1<<0),
    },
    PINMUX_END_ITEM
};

static pinmux_set_t dummy_codec_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &dummy_codec_pinmux[0],
};

static void dummy_codec_device_init(void)
{
    /* audio pinmux */
    pinmux_set(&dummy_codec_pinmux_set);
}

static void dummy_codec_device_deinit(void)
{
    pinmux_clr(&dummy_codec_pinmux_set);
}


static struct dummy_codec_platform_data dummy_codec_pdata = {
    .device_init = dummy_codec_device_init,
    .device_uninit = dummy_codec_device_deinit,
};

static struct platform_device aml_dummy_codec_audio = {
    .name = "aml_dummy_codec_audio",
    .id = 0,
    .resource = aml_m6_audio_resource,
    .num_resources = ARRAY_SIZE(aml_m6_audio_resource),
    .dev = {
        .platform_data = &dummy_codec_pdata,
    },
};

static struct platform_device aml_dummy_codec = {
    .name = "dummy_codec",
    .id = 0,
};
#endif

/***********************************************************************
 * Card Reader Section
 **********************************************************************/
#ifdef CONFIG_CARDREADER
static struct resource meson_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};

static struct aml_card_info meson_card_info[] = {
    [0] = {
        .name           = "sd_card",
        .work_mode      = CARD_HW_MODE,
        .io_pad_type        = SDHC_CARD_0_5,
        .card_ins_en_reg    = CARD_GPIO_ENABLE,
        .card_ins_en_mask   = PREG_IO_29_MASK,
        .card_ins_input_reg = CARD_GPIO_INPUT,
        .card_ins_input_mask    = PREG_IO_29_MASK,
        .card_power_en_reg  = CARD_GPIO_ENABLE,
        .card_power_en_mask = PREG_IO_31_MASK,
        .card_power_output_reg  = CARD_GPIO_OUTPUT,
        .card_power_output_mask = PREG_IO_31_MASK,
        .card_power_en_lev  = 0,
        .card_wp_en_reg     = 0,
        .card_wp_en_mask    = 0,
        .card_wp_input_reg  = 0,
        .card_wp_input_mask = 0,
        .card_extern_init   = 0,
    },
};

static struct aml_card_platform meson_card_platform = {
    .card_num   = ARRAY_SIZE(meson_card_info),
    .card_info  = meson_card_info,
};

static struct platform_device meson_card_device = {
    .name   = "AMLOGIC_CARD",
    .id     = -1,
    .num_resources  = ARRAY_SIZE(meson_card_resource),
    .resource   = meson_card_resource,
    .dev = {
        .platform_data = &meson_card_platform,
    },
};

#endif // CONFIG_CARDREADER

/***********************************************************************
 * IO Mapping
 **********************************************************************/
/*
#define IO_CBUS_BASE        0xf1100000  ///2M
#define IO_AXI_BUS_BASE     0xf1300000  ///1M
#define IO_PL310_BASE       0xf2200000  ///4k
#define IO_PERIPH_BASE      0xf2300000  ///4k
#define IO_APB_BUS_BASE     0xf3000000  ///8k
#define IO_DOS_BUS_BASE     0xf3010000  ///64k
#define IO_AOBUS_BASE       0xf3100000  ///1M
#define IO_USB_A_BASE       0xf3240000  ///256k
#define IO_USB_B_BASE       0xf32C0000  ///256k
#define IO_WIFI_BASE        0xf3300000  ///1M
#define IO_SATA_BASE        0xf3400000  ///64k
#define IO_ETH_BASE         0xf3410000  ///64k

#define IO_SPIMEM_BASE      0xf4000000  ///64M
#define IO_A9_APB_BASE      0xf8000000  ///256k
#define IO_DEMOD_APB_BASE   0xf8044000  ///112k
#define IO_MALI_APB_BASE    0xf8060000  ///128k
#define IO_APB2_BUS_BASE    0xf8000000
#define IO_AHB_BASE         0xf9000000  ///128k
#define IO_BOOTROM_BASE     0xf9040000  ///64k
#define IO_SECBUS_BASE      0xfa000000
#define IO_EFUSE_BASE       0xfa000000  ///4k
*/
static __initdata struct map_desc meson_io_desc[] = {
    {
        .virtual    = IO_CBUS_BASE,
        .pfn        = __phys_to_pfn(IO_CBUS_PHY_BASE),
        .length     = SZ_2M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AXI_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AXI_BUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_PL310_BASE,
        .pfn        = __phys_to_pfn(IO_PL310_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_PERIPH_BASE,
        .pfn        = __phys_to_pfn(IO_PERIPH_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
           .virtual    = IO_APB_BUS_BASE,
           .pfn        = __phys_to_pfn(IO_APB_BUS_PHY_BASE),
           .length     = SZ_1M,
           .type       = MT_DEVICE,
       } , /*{

           .virtual    = IO_DOS_BUS_BASE,
           .pfn        = __phys_to_pfn(IO_DOS_BUS_PHY_BASE),
           .length     = SZ_64K,
           .type       = MT_DEVICE,
       } , */{
           .virtual    = IO_AOBUS_BASE,
        .pfn        = __phys_to_pfn(IO_AOBUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_BUS_PHY_BASE),
        .length     = SZ_8M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_SPIMEM_BASE,
        .pfn        = __phys_to_pfn(IO_SPIMEM_PHY_BASE),
        .length     = SZ_64M,
        .type       = MT_ROM,
    } , {
        .virtual    = IO_APB2_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB2_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_PHY_BASE),
        .length     = SZ_128K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_BOOTROM_BASE,
        .pfn        = __phys_to_pfn(IO_BOOTROM_PHY_BASE),
        .length     = SZ_64K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_SECBUS_BASE,
        .pfn        = __phys_to_pfn(IO_SECBUS_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_SECURE_BASE,
        .pfn        = __phys_to_pfn(IO_SECURE_PHY_BASE),
        .length     = SZ_16K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END - RESERVED_MEM_START + 1,
        .type       = MT_MEMORY_NONCACHED,
    },
#ifdef CONFIG_MESON_SUSPEND
        {
        .virtual    = PAGE_ALIGN(__phys_to_virt(0x9ff00000)),
        .pfn        = __phys_to_pfn(0x9ff00000),
        .length     = SZ_1M,
        .type       = MT_MEMORY_NONCACHED,
        },
#endif

};

static void __init meson_map_io(void)
{
    iotable_init(meson_io_desc, ARRAY_SIZE(meson_io_desc));
}

static void __init meson_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    mach->video_start    = RESERVED_MEM_START;
    mach->video_end      = RESERVED_MEM_END;

    m->nr_banks = 0;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
    m->nr_banks++;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END + 1);
#ifdef CONFIG_MESON_SUSPEND
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END-SZ_1M) & PAGE_MASK;
#else
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
#endif
    m->nr_banks++;
}

/***********************************************************************
 *USB Setting section
 **********************************************************************/
static  int __init setup_usb_devices(void)
{
    struct lm_device * usb_ld_a, *usb_ld_b;
    usb_ld_a = alloc_usb_lm_device(USB_PORT_IDX_A);
    usb_ld_b = alloc_usb_lm_device(USB_PORT_IDX_B);

    lm_device_register(usb_ld_a); 
    lm_device_register(usb_ld_b);
    return 0;
}

/***********************************************************************
 *WiFi power section
 **********************************************************************/
/* built-in usb wifi power ctrl, usb dongle must register NULL to power_ctrl! 1:power on  0:power off */
#ifdef CONFIG_AM_WIFI_USB
static int usb_wifi_power(int is_power)
{
    printk(KERN_INFO "usb_wifi_power %s\n", is_power ? "On" : "Off");
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1,(1<<11));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0,(1<<18));
    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<5));
    if (is_power)//is_power
        SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<5));
    else
        CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<5));
    return 0;
}

static struct wifi_power_platform_data wifi_plat_data = {
    .usb_set_power = usb_wifi_power,
};

static struct platform_device wifi_power_device = {
    .name   = "wifi_power",
    .id     = -1,
    .dev = {
        .platform_data = &wifi_plat_data,
    },
};
#endif

/***********************************************************************
* ETHERNET Section
**********************************************************************/
 
#ifdef CONFIG_AM_ETHERNET
#include <plat/eth.h>
//#define ETH_MODE_RGMII
#define ETH_MODE_RMII_INTERNAL
//#define ETH_MODE_RMII_EXTERNAL
static void aml_eth_reset(void)
{
    unsigned int val = 0;

    printk(KERN_INFO "****** aml_eth_reset() ******\n");
#ifdef ETH_MODE_RGMII
    val = 0x211;
#else
    val = 0x241;
#endif
    /* setup ethernet mode */
    aml_set_reg32_mask(P_PREG_ETHERNET_ADDR0, val);

    /* setup ethernet interrupt */
    aml_set_reg32_mask(P_SYS_CPU_0_IRQ_IN0_INTR_MASK, 1 << 8);
    aml_set_reg32_mask(P_SYS_CPU_0_IRQ_IN1_INTR_STAT, 1 << 8);

    /* hardware reset ethernet phy */
    gpio_out(PAD_GPIOY_15, 0);
    msleep(20);
    gpio_out(PAD_GPIOY_15, 1);
}

static void aml_eth_clock_enable(void)
{
    unsigned int val = 0;

    printk(KERN_INFO "****** aml_eth_clock_enable() ******\n");
#ifdef ETH_MODE_RGMII
    val = 0x309;
#elif defined(ETH_MODE_RMII_EXTERNAL)
    val = 0x130;
#else
    val = 0x702;
#endif
    /* setup ethernet clk */
    aml_set_reg32_mask(P_HHI_ETH_CLK_CNTL, val);
}

static void aml_eth_clock_disable(void)
{
    printk(KERN_INFO "****** aml_eth_clock_disable() ******\n");
    /* disable ethernet clk */
    aml_clr_reg32_mask(P_HHI_ETH_CLK_CNTL, 1 << 8);
}

static pinmux_item_t aml_eth_pins[] = {
    /* RMII pin-mux */
    {
        .reg = PINMUX_REG(6),
        .clrmask = 0,
#ifdef ETH_MODE_RMII_EXTERNAL
        .setmask = 0x8007ffe0,
#else
        .setmask = 0x4007ffe0,
#endif
    },
    PINMUX_END_ITEM
};

static pinmux_set_t aml_eth_pinmux = {
    .chip_select = NULL,
    .pinmux = aml_eth_pins,
};

static void aml_eth_pinmux_setup(void)
{
    printk(KERN_INFO "****** aml_eth_pinmux_setup() ******\n");

    pinmux_set(&aml_eth_pinmux);
}

static void aml_eth_pinmux_cleanup(void)
{
    printk(KERN_INFO "****** aml_eth_pinmux_cleanup() ******\n");

    pinmux_clr(&aml_eth_pinmux);
}

static void aml_eth_init(void)
{
    aml_eth_pinmux_setup();
    aml_eth_clock_enable();
    aml_eth_reset();
}

static struct aml_eth_platdata aml_eth_pdata __initdata = {
    .pinmux_items = aml_eth_pins,
    .pinmux_setup = aml_eth_pinmux_setup,
    .pinmux_cleanup = aml_eth_pinmux_cleanup,
    .clock_enable = aml_eth_clock_enable,
    .clock_disable = aml_eth_clock_disable,
    .reset = aml_eth_reset,
};

static void __init setup_eth_device(void)
{
    meson_eth_set_platdata(&aml_eth_pdata);
    aml_eth_init();
}
#endif


/***********************************************************************
 * Efuse section
 **********************************************************************/
#ifdef CONFIG_EFUSE
static bool efuse_data_verify(unsigned char *usid)
{
    int len;

    len = strlen(usid);
    if((len > 0)&&(len<58) )
        return true;
    else
        return false;
}

static struct efuse_platform_data aml_efuse_plat = {
    .pos = 454,
    .count = 58,
    .data_verify = efuse_data_verify,
};

static struct platform_device aml_efuse_device = {
    .name   = "efuse",
    .id = -1,
    .dev = {
        .platform_data = &aml_efuse_plat,
    },
};

// BSP EFUSE layout setting
static efuseinfo_item_t aml_efuse_setting[] = {
    // usid layout can be defined by customer
    {
	.title = "usid",   
	.id = EFUSE_USID_ID,
	.offset = 454,      // customer can modify the offset which must >= 454
	.enc_len = 58,     // customer can modify the encode length by self must <=58
	.data_len = 58,	// customer can modify the data length by self must <=58
	.bch_en = 0,		// customer can modify do bch or not
	.bch_reverse = 0,
    },	
    // customer also can add new EFUSE item to expand, but must be correct and bo conflict
};

static int aml_efuse_getinfoex_byID(unsigned param, efuseinfo_item_t *info)
{
    unsigned num = sizeof(aml_efuse_setting)/sizeof(efuseinfo_item_t);
    int i=0;
    int ret = -1;
    for(i=0; i<num; i++) {
        if(aml_efuse_setting[i].id == param) {
            strcpy(info->title, aml_efuse_setting[i].title);
            info->offset = aml_efuse_setting[i].offset;
            info->id = aml_efuse_setting[i].id;
            info->data_len = aml_efuse_setting[i].data_len;
            info->enc_len = aml_efuse_setting[i].enc_len;
            info->bch_en = aml_efuse_setting[i].bch_en;
            info->bch_reverse = aml_efuse_setting[i].bch_reverse;
            ret = 0;
            break;
        }
    }
    return ret;
}

static int aml_efuse_getinfoex_byPos(unsigned param, efuseinfo_item_t *info)
{
    unsigned num = sizeof(aml_efuse_setting)/sizeof(efuseinfo_item_t);
    int i=0;
    int ret = -1;
    for(i=0; i<num; i++) {
        if(aml_efuse_setting[i].offset == param) {
            strcpy(info->title, aml_efuse_setting[i].title);
            info->offset = aml_efuse_setting[i].offset;
            info->id = aml_efuse_setting[i].id;
            info->data_len = aml_efuse_setting[i].data_len;
            info->enc_len = aml_efuse_setting[i].enc_len;
            info->bch_en = aml_efuse_setting[i].bch_en;
            info->bch_reverse = aml_efuse_setting[i].bch_reverse;
            ret = 0;
            break;
        }
    }
    return ret;
}

extern pfn efuse_getinfoex;
extern pfn efuse_getinfoex_byPos;

static __init void setup_aml_efuse(void)
{
    efuse_getinfoex = aml_efuse_getinfoex_byID;
    efuse_getinfoex_byPos = aml_efuse_getinfoex_byPos;
}
#endif

/***********************************************************************
 * RTC section
 **********************************************************************/
#if defined(CONFIG_AML_RTC)
static  struct platform_device aml_rtc_device = {
            .name            = "aml_rtc",
            .id               = -1,
    };
#endif

#if defined(CONFIG_TVIN_VDIN)
static struct resource vdin_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,  //pbufAddr
        .end   = VDIN_ADDR_END,     //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = VDIN_ADDR_START,
        .end   = VDIN_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [2] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
    [3] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device vdin_device = {
    .name       = "vdin",
    .id         = -1,
    .num_resources = ARRAY_SIZE(vdin_resources),
    .resource      = vdin_resources,
};
#endif

#ifdef CONFIG_TVIN_BT656IN
//add pin mux info for bt656 input
#if 0
static struct resource bt656in_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,      //pbufAddr
        .end   = VDIN_ADDR_END,             //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {     //bt656/camera/bt601 input resource pin mux setting
        .start =  0x3000,       //mask--mux gpioD 15 to bt656 clk;  mux gpioD 16:23 to be bt656 dt_in
        .end   = PERIPHS_PIN_MUX_5 + 0x3000,
        .flags = IORESOURCE_MEM,
    },

    [2] = {         //camera/bt601 input resource pin mux setting
        .start =  0x1c000,      //mask--mux gpioD 12 to bt601 FIQ; mux gpioD 13 to bt601HS; mux gpioD 14 to bt601 VS;
        .end   = PERIPHS_PIN_MUX_5 + 0x1c000,
        .flags = IORESOURCE_MEM,
    },

    [3] = {         //bt601 input resource pin mux setting
        .start =  0x800,        //mask--mux gpioD 24 to bt601 IDQ;;
        .end   = PERIPHS_PIN_MUX_5 + 0x800,
        .flags = IORESOURCE_MEM,
    },

};
#endif

static struct platform_device bt656in_device = {
    .name       = "amvdec_656in",
    .id         = -1,
//    .num_resources = ARRAY_SIZE(bt656in_resources),
//    .resource      = bt656in_resources,
};
#endif

#if defined(CONFIG_SUSPEND)
static void m6ref_set_vccx2(int power_on)
{
    if (power_on) {
        printk(KERN_INFO "%s() Power ON\n", __FUNCTION__);
        aml_clr_reg32_mask(P_PREG_PAD_GPIO0_EN_N,(1<<26));
        aml_clr_reg32_mask(P_PREG_PAD_GPIO0_O,(1<<26));
    }
    else {
        printk(KERN_INFO "%s() Power OFF\n", __FUNCTION__);
        aml_clr_reg32_mask(P_PREG_PAD_GPIO0_EN_N,(1<<26));
        aml_set_reg32_mask(P_PREG_PAD_GPIO0_O,(1<<26));
        //save_pinmux();
    }
}

static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = (void *)IO_APB_BUS_BASE,
    .mmc_reg_base = (void *)APB_REG_ADDR(0x1000),
    .hiu_reg_base = (void *)CBUS_REG_ADDR(0x1000),
    .power_key = (1<<8),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = m6ref_set_vccx2,
    .core_voltage_adjust = 7,  //5,8
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif

#if defined (CONFIG_AML_HDMI_TX)
static void m6ref_hdmi_5v_ctrl(unsigned int pwr)
{
    if(pwr){
        printk("HDMI 5V Power On\n");
        aml_clr_reg32_mask(P_PREG_PAD_GPIO2_O,(1<<21));
        aml_clr_reg32_mask(P_PREG_PAD_GPIO2_EN_N,(1<<21));
    }
    else{
        printk("HDMI 5V Power Off\n");
        aml_clr_reg32_mask(P_PREG_PAD_GPIO2_O,(0<<21));
        aml_set_reg32_mask(P_PREG_PAD_GPIO2_EN_N,(1<<21));
    }
}

static struct hdmi_phy_set_data brd_phy_data[] = {
//    {27, 0x16, 0x30},   // 480i/p 576i/p
//    {74, 0x16, 0x40},   // 720p 1080i
//    {148, 0x16, 0x40},  // 1080p
    {-1,   -1},         //end of phy setting
};

static struct hdmi_config_platform_data aml_hdmi_pdata = {
    .hdmi_5v_ctrl = m6ref_hdmi_5v_ctrl,
    .hdmi_3v3_ctrl = NULL,
    .hdmi_pll_vdd_ctrl = NULL,
    .phy_data = brd_phy_data,
};
#endif
/***********************************************************************
 * Meson CS DCDC section
 **********************************************************************/
#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
#include <mach/voltage.h>
#include <linux/regulator/meson_cs_dcdc_regulator.h>
#include <linux/regulator/machine.h>
static struct regulator_consumer_supply vcck_data[] = {
    {
        .supply = "vcck-armcore",
    },
};

static struct regulator_init_data vcck_init_data = {
    .constraints = { /* VCCK default 1.2V */
        .name = "vcck",
        .min_uV =  972000,
        .max_uV =  1253000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
    },
    .num_consumer_supplies = ARRAY_SIZE(vcck_data),
    .consumer_supplies = vcck_data,
};

// pwm duty for vcck voltage
static unsigned int vcck_pwm_table[MESON_CS_MAX_STEPS] = {
    0x0c001c, 0x0c001c, 0x0c001c, 0x0c001c, 
    0x100018, 0x100018, 0x100018, 0x100018, 
    0x180010, 0x180010, 0x180010, 0x180010, 
    0x230005, 0x230005, 0x230005, 0x230005, 
};
static int get_voltage() {
//    printk("***vcck: get_voltage");
    int i;
    unsigned int reg = aml_read_reg32(P_PWM_PWM_C);
    for(i=0; i<MESON_CS_MAX_STEPS; i++) {
        if(reg == vcck_pwm_table[i])
         break;
    }
    if(i >= MESON_CS_MAX_STEPS)
        return -1;
    else 
        return i;
}

static int set_voltage(unsigned int level) {
//    printk("***vcck: set_voltage");
    aml_write_reg32(P_PWM_PWM_C, vcck_pwm_table[level]);

}

static void vcck_pwm_init() {
    printk("***vcck: vcck_pwm_init");
    //enable pwm clk & pwm output
    aml_write_reg32(P_PWM_MISC_REG_CD, (aml_read_reg32(P_PWM_MISC_REG_CD) & ~(0x7f << 8)) | ((1 << 15) | (0 << 8) | (1 << 0)));
    aml_write_reg32(P_PWM_PWM_C, vcck_pwm_table[0]);
    //enable pwm_C pinmux    1<<3 pwm_D
    aml_write_reg32(P_PERIPHS_PIN_MUX_2, aml_read_reg32(P_PERIPHS_PIN_MUX_2) | (1 << 2));
}

static struct meson_cs_pdata_t vcck_pdata = {
    .meson_cs_init_data = &vcck_init_data,
    .voltage_step_table = {
/*        1209000, 1192000, 1176000, 1159000,
        1143000, 1127000, 1110000, 1094000,
        1077000, 1061000, 1044000, 1028000,
        1012000, 996000,  979000,  962000,
*/
    1253000, 1253000, 1253000,1253000,
    1203000, 1203000, 1203000, 1203000, 
    1107000, 1107000, 1107000, 1107000, 
    972000,   972000,   972000,   972000, 	
    },
    .default_uV = 1451000,
    .get_voltage = get_voltage,
    .set_voltage = set_voltage,
};

static struct meson_opp vcck_opp_table[] = {
    /* freq must be in descending order */
    {
        .freq   = 1500000,
        .min_uV = 1253000,
        .max_uV = 1253000,
    },
    {
        .freq   = 1320000,
        .min_uV = 1253000,
        .max_uV = 1253000,
    },
    {
        .freq   = 1200000,
        .min_uV = 1253000,
        .max_uV = 1253000,
    },
    {
        .freq   = 1080000,
        .min_uV = 1203000,
        .max_uV = 1203000,
    },
    {
        .freq   = 840000,
        .min_uV = 1203000,
        .max_uV = 1203000,
    },
    {
        .freq   = 600000,
        .min_uV = 1107000,
        .max_uV = 1107000,
    },
    {
        .freq   = 200000,
        .min_uV = 972000,
        .max_uV = 972000,
    }
};

static struct platform_device meson_cs_dcdc_regulator_device = {
    .name = "meson-cs-regulator",
    .dev = {
        .platform_data = &vcck_pdata,
    }
};
#endif
/***********************************************************************
 * Meson CPUFREQ section
 **********************************************************************/
#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#include <plat/cpufreq.h>

#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
#include <mach/voltage.h>
static struct regulator *vcck;
static struct meson_cpufreq_config cpufreq_info;

static unsigned int vcck_cur_max_freq(void)
{
    return meson_vcck_cur_max_freq(vcck, vcck_opp_table, ARRAY_SIZE(vcck_opp_table));
}

static int vcck_scale(unsigned int frequency)
{
    return meson_vcck_scale(vcck, vcck_opp_table, ARRAY_SIZE(vcck_opp_table), frequency);
}

static int vcck_regulator_init(void)
{
    vcck = regulator_get(NULL, vcck_data[0].supply);
    if (WARN(IS_ERR(vcck), "Unable to obtain voltage regulator for vcck;"
                    " voltage scaling unsupported\n")) {
        return PTR_ERR(vcck);
    }

    return 0;
}

static struct meson_cpufreq_config cpufreq_info = {
    .freq_table = NULL,
    .init = vcck_regulator_init,
    .cur_volt_max_freq = vcck_cur_max_freq,
    .voltage_scale = vcck_scale,
};
#endif //CONFIG_MESON_CS_DCDC_REGULATOR


static struct platform_device meson_cpufreq_device = {
    .name   = "cpufreq-meson",
    .dev = {
#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
        .platform_data = &cpufreq_info,
#else
        .platform_data = NULL,
#endif
    },
    .id = -1,
};
#endif //CONFIG_CPU_FREQ

#ifdef CONFIG_SARADC_AM
#include <linux/saradc.h>
static struct platform_device saradc_device = {
    .name = "saradc",
    .id = 0,
    .dev = {
        .platform_data = NULL,
    },
};
#endif

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static struct adc_key adc_kp_key[] = {
    {KEY_POWER,    "power", CHAN_4, 0, 40},
//    {KEY_VOLUMEDOWN,    "vol-", CHAN_4, 150, 40},
//    {KEY_VOLUMEUP,      "vol+", CHAN_4, 275, 40},
};

static struct adc_kp_platform_data adc_kp_pdata = {
    .key = &adc_kp_key[0],
    .key_num = ARRAY_SIZE(adc_kp_key),
};

static struct platform_device adc_kp_device = {
    .name = "m1-adckp",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &adc_kp_pdata,
    }
};
#endif

/***********************************************************************
 * Power Key Section
 **********************************************************************/


#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
#include <linux/input.h>
#include <linux/input/key_input.h>

static int _key_code_list[] = {KEY_POWER};

static inline int key_input_init_func(void)
{
    WRITE_AOBUS_REG(AO_RTC_ADDR0, (READ_AOBUS_REG(AO_RTC_ADDR0) &~(1<<11)));
    WRITE_AOBUS_REG(AO_RTC_ADDR1, (READ_AOBUS_REG(AO_RTC_ADDR1) &~(1<<3)));
    return 0;
}
static inline int key_scan(void* data)
{
    int *key_state_list = (int*)data;
    int ret = 0;
    key_state_list[0] = ((READ_AOBUS_REG(AO_RTC_ADDR1) >> 2) & 1) ? 0 : 1;
    return ret;
}

static struct key_input_platform_data key_input_pdata = {
    .scan_period = 20,
    .fuzz_time = 60,
    .key_code_list = &_key_code_list[0],
    .key_num = ARRAY_SIZE(_key_code_list),
    .scan_func = key_scan,
    .init_func = key_input_init_func,
    .config = 0,
};

static struct platform_device input_device_key = {
    .name = "meson-keyinput",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &key_input_pdata,
    }
};
#endif

/***********************************************************************
 * Device Register Section
 **********************************************************************/
static struct platform_device  *platform_devs[] = {
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    &aml_i2c_device_a,
    &aml_i2c_device_b,
    &aml_i2c_device_ao,
#endif
#if defined(CONFIG_I2C_SW_AML)
    &aml_sw_i2c_device_1,
#endif
    &aml_uart_device,
    &meson_device_fb,
    &meson_device_vout,
#ifdef CONFIG_AM_STREAMING
    &meson_device_codec,
#endif
#if defined(CONFIG_AM_NAND)
    &aml_nand_device,
#endif
#ifdef CONFIG_SARADC_AM
    &saradc_device,
#endif
#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
    &adc_kp_device,
#endif
#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
    &input_device_key,
#endif
#if defined(CONFIG_CARDREADER)
    &meson_card_device,
#endif // CONFIG_CARDREADER
#if defined(CONFIG_SUSPEND)
    &aml_pm_device,
#endif
#ifdef CONFIG_EFUSE
    &aml_efuse_device,
#endif
#if defined(CONFIG_AML_RTC)
    &aml_rtc_device,
#endif
    &aml_audio,
    &aml_audio_dai,
#if defined(CONFIG_SND_SOC_DUMMY_CODEC)
    &aml_dummy_codec_audio,
    &aml_dummy_codec,
#endif
#ifdef CONFIG_AM_WIFI
    &wifi_power_device,
#endif
#if defined(CONFIG_TVIN_VDIN)
    &vdin_device,
#endif
#if 0 //defined(CONFIG_TVIN_BT656IN)
    &bt656in_device,
#endif
#ifdef CONFIG_AM_REMOTE
    &meson_device_remote,
#endif
#if defined(CONFIG_AMLOGIC_SPI_NOR)
    &amlogic_spi_nor_device,
#endif
#ifdef CONFIG_POST_PROCESS_MANAGER
    &ppmgr_device,
#endif
#if defined(CONFIG_AM_DEINTERLACE) || defined (CONFIG_DEINTERLACE)
    &deinterlace_device,
#endif
#ifdef CONFIG_FREE_SCALE
        &freescale_device,
#endif 
#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
    &meson_cs_dcdc_regulator_device,
#endif
#ifdef CONFIG_CPU_FREQ
    &meson_cpufreq_device,
#endif
#ifdef CONFIG_BT_DEVICE  
    &bt_device,
#endif
};

#ifdef CONFIG_AML_HDMI_TX
    extern int setup_hdmi_dev_platdata(void* platform_data);
#endif
#if defined(CONFIG_SUSPEND)
    extern  int console_suspend_enabled;
#endif

static __init void meson_init_machine(void)
{
//    meson_cache_init();

    /**
     *  Meson6 socket board ONLY
     *  Do *NOT* merge for other BSP
     */
    aml_set_reg32_bits(AOBUS_REG_ADDR(0x24), 0,  3, 1);
    aml_set_reg32_bits(AOBUS_REG_ADDR(0x24), 1, 19, 1);
    aml_set_reg32_bits(AOBUS_REG_ADDR(0x24), 0,  2, 1);
    aml_set_reg32_bits(AOBUS_REG_ADDR(0x24), 1, 18, 1);

    // Enable +5V VCCx2/VCCk
    gpio_out(PAD_GPIOAO_3, 0);

#ifdef CONFIG_MESON_CS_DCDC_REGULATOR
    vcck_pwm_init();
#endif

#ifdef CONFIG_AM_ETHERNET
    setup_eth_device();
#endif

#ifdef CONFIG_AML_HDMI_TX
    setup_hdmi_dev_platdata(&aml_hdmi_pdata);
#endif
#ifdef CONFIG_AM_REMOTE
    setup_remote_device();
#endif
#ifdef CONFIG_EFUSE
    setup_aml_efuse();
#endif
    setup_usb_devices();
    setup_devices_resource();
    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));
#ifdef CONFIG_AM_WIFI_USB
    if(wifi_plat_data.usb_set_power)
        wifi_plat_data.usb_set_power(1);//power off built-in usb wifi
#endif
#if defined(CONFIG_SUSPEND)
    {//todo: remove it after verified. need set it in uboot environment variable.
    console_suspend_enabled = 0;
    }
#endif

    CLEAR_CBUS_REG_MASK(PREG_PAD_GPIO2_EN_N, (1<<21));
    SET_CBUS_REG_MASK(PREG_PAD_GPIO2_O, (1<<21));

#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    aml_i2c_init();
#endif
    
}
static __init void meson_init_early(void)
{
    ///boot seq 1

}

MACHINE_START(MESON6_REFG02, "Amlogic Meson6 G02 reference board")
	.boot_params    = BOOT_PARAMS_OFFSET,
	.map_io     = meson_map_io,///2
	.init_early = meson_init_early,///3
	.init_irq   = meson_init_irq,///0
	.timer      = &meson_sys_timer,
	.init_machine   = meson_init_machine,
	.fixup      = meson_fixup,///1
MACHINE_END
