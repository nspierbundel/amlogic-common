/*
 * arch/arm/mach-meson6tv/board-m6-ref.c
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
#include <linux/reboot.h>
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

#ifdef CONFIG_CARDREADER
#include <mach/card_io.h>
#endif // CONFIG_CARDREADER

#include <mach/gpio.h>
#include <mach/gpio_data.h>


#ifdef CONFIG_EFUSE
#include <linux/efuse.h>
#endif
#ifdef CONFIG_SND_AML_M6_AUDIO_CODEC
#include <sound/aml_m6_audio.h>
#endif
#ifdef CONFIG_SND_SOC_DUMMY_CODEC
#include <sound/dummy_codec.h>
#endif

#ifdef CONFIG_AM_WIFI
#include <plat/wifi_power.h>
#endif

#ifdef CONFIG_AM_ETHERNET
#include <mach/am_regs.h>
#include <mach/am_eth_reg.h>
#endif

#ifdef CONFIG_AML_HDMI_TX
#include <plat/hdmi_config.h>
extern int setup_hdmi_dev_platdata(void* platform_data);
#endif

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
    .name   = "ppmgr",
    .id     = 0,
    .num_resources = ARRAY_SIZE(ppmgr_resources),
    .resource      = ppmgr_resources,
};
#endif

#ifdef CONFIG_V4L_AMLOGIC_VIDEO2
static struct resource amlvideo2_resources[] = {
    [0] = {
        .start = AMLVIDEO2_ADDR_START,
        .end   = AMLVIDEO2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device amlvideo2_device = {
    .name   = "amlvideo2",
    .id     = 0,
    .num_resources = ARRAY_SIZE(amlvideo2_resources),
    .resource      = amlvideo2_resources,
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

#ifdef CONFIG_FREE_SCALE
static struct resource freescale_resources[] = {
    [0] = {
        .start = FREESCALE_ADDR_START,
        .end   = FREESCALE_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device freescale_device = {
    .name       = "freescale",
    .id         = 0,
    .num_resources  = ARRAY_SIZE(freescale_resources),
    .resource       = freescale_resources,
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


/***********************************************************************
 *  HDMI  Section
 **********************************************************************/

#if defined (CONFIG_AML_HDMI_TX)
static void m6ref_hdmi_5v_ctrl(unsigned int pwr)
{
    if(pwr){
        printk("HDMI 5V Power On\n");
    }
    else{
        printk("HDMI 5V Power Off\n");
    }
}

static struct hdmi_phy_set_data brd_phy_data[] = {
//    {27, 0x16, 0x30},   // 480i/p 576i/p
//	  {74, 0x16, 0x40},   // 720p 1080i
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

static pinmux_item_t aml_i2c_b_pinmux_item[]= {
    {
        .reg = 5,
        //.clrmask = (3<<28)|(3<<26),
        .setmask = 3<<30
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat_b = {
    .wait_count     	= 50000,
    .wait_ack_interval 	= 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no      	= AML_I2C_MASTER_B,
    .use_pio        	= 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux  = {
        .chip_select    = pinmux_dummy_share,
        .pinmux     	= &aml_i2c_b_pinmux_item[0]
    }
};

static struct resource aml_i2c_resource_b[] = {
    [0] = {
        .start = MESON_I2C_MASTER_B_START,
        .end   = MESON_I2C_MASTER_B_END,
        .flags = IORESOURCE_MEM,
    }
};

static struct platform_device aml_i2c_device_b = {
    .name     		= "aml-i2c",
    .id      		= 0,
    .num_resources  = ARRAY_SIZE(aml_i2c_resource_b),
    .resource     	= aml_i2c_resource_b,
    .dev = {
        .platform_data = &aml_i2c_plat_b,
    },
};

static struct i2c_board_info __initdata aml_i2c_bus_info_b[] = {
};

static int __init aml_i2c_init(void)
{
    i2c_register_board_info(0, aml_i2c_bus_info_b, ARRAY_SIZE(aml_i2c_bus_info_b));
    return 0;
}
#endif

/***********************************************************************
 * UART Section
 **********************************************************************/
static pinmux_item_t uart_pins[] = {
    {
        .reg = PINMUX_REG(AO),
        .setmask = 3 << 11
    },
    PINMUX_END_ITEM
};

static pinmux_set_t aml_uart_ao = {
    .chip_select = NULL,
    .pinmux = &uart_pins[0]
};

static struct aml_uart_platform  aml_uart_plat = {
    .uart_line[0]   = UART_AO,
    .uart_line[1]   = UART_A,
    .uart_line[2]   = UART_B,
    .uart_line[3]   = UART_C,
    .uart_line[4]   = UART_D,

    .pinmux_uart[0] = (void*)&aml_uart_ao,
    .pinmux_uart[1] = NULL,
    .pinmux_uart[2] = NULL,
    .pinmux_uart[3] = NULL,
    .pinmux_uart[4] = NULL
};

static struct platform_device aml_uart_device = {
    .name   = "mesonuart",
    .id     = -1,
    .num_resources  = 0,
    .resource   = NULL,
    .dev = {
        .platform_data = &aml_uart_plat,
    },
};

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
    .pinmux_items   = aml_eth_pins,
    .pinmux_setup   = aml_eth_pinmux_setup,
    .pinmux_cleanup = aml_eth_pinmux_cleanup,
    .clock_enable   = aml_eth_clock_enable,
    .clock_disable  = aml_eth_clock_disable,
    .reset      = aml_eth_reset,
};

static void __init setup_eth_device(void)
{
    meson_eth_set_platdata(&aml_eth_pdata);
    aml_eth_init();
}
#endif

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
    {
        .name = NAND_NORMAL_NAME,
        .chip_enable_pad = (AML_NAND_CE0/* | (AML_NAND_CE1 << 4) | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)*/),
        .ready_busy_pad = (AML_NAND_CE0 /*| (AML_NAND_CE0 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)*/),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .nr_partitions = ARRAY_SIZE(normal_partition_info),
                .partitions = normal_partition_info,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH30_1K_MODE | NAND_TWO_PLANE_MODE),
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
        .name       = "sd_card",
        .work_mode  = CARD_HW_MODE,
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
#define IO_ETH_BASE     0xf3410000  ///64k

#define IO_SPIMEM_BASE      0xf4000000  ///64M
#define IO_A9_APB_BASE      0xf8000000  ///256k
#define IO_DEMOD_APB_BASE   0xf8044000  ///112k
#define IO_MALI_APB_BASE    0xf8060000  ///128k
#define IO_APB2_BUS_BASE    0xf8000000
#define IO_AHB_BASE     0xf9000000  ///128k
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
    }, {
        .virtual    = IO_AXI_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AXI_BUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_PL310_BASE,
        .pfn        = __phys_to_pfn(IO_PL310_PHY_BASE),
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_PERIPH_BASE,
        .pfn        = __phys_to_pfn(IO_PERIPH_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_APB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB_BUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    }, /*{
    .virtual    = IO_DOS_BUS_BASE,
    .pfn        = __phys_to_pfn(IO_DOS_BUS_PHY_BASE),
    .length     = SZ_64K,
    .type       = MT_DEVICE,
    }, */{
        .virtual    = IO_AOBUS_BASE,
        .pfn        = __phys_to_pfn(IO_AOBUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_AHB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_BUS_PHY_BASE),
        .length     = SZ_8M,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_SPIMEM_BASE,
        .pfn        = __phys_to_pfn(IO_SPIMEM_PHY_BASE),
        .length     = SZ_64M,
        .type       = MT_ROM,
    }, {
        .virtual    = IO_APB2_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB2_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_AHB_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_PHY_BASE),
        .length     = SZ_128K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = IO_BOOTROM_BASE,
        .pfn        = __phys_to_pfn(IO_BOOTROM_PHY_BASE),
        .length     = SZ_64K,
        .type       = MT_DEVICE,
    }, {
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
    mach->video_end  = RESERVED_MEM_END;

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
        .data_len = 58, // customer can modify the data length by self must <=58
        .bch_en = 0,        // customer can modify do bch or not
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
static struct platform_device aml_rtc_device = {
    .name        = "aml_rtc",
    .id           = -1,
};
#endif

#if defined(CONFIG_SUSPEND)
typedef struct {
    char name[32];
    unsigned bank;
    unsigned bit;
    gpio_mode_t mode;
    unsigned value;
    unsigned enable;
    unsigned keep_last;
} gpio_data_t;

#define MAX_GPIO 1

static gpio_data_t gpio_data[MAX_GPIO] = {
    // ----------------------------------- bl ----------------------------------
    {"GPIOX2 -- BL_EN",         GPIOX_bank_bit0_31(2),     GPIOX_bit_bit0_31(2),  GPIO_OUTPUT_MODE, 1, 1, 1},
    // ----------------------------------- panel ----------------------------------
    // {"GPIOZ5 -- PANEL_PWR",     GPIOZ_bank_bit0_19(5),     GPIOZ_bit_bit0_19(5),  GPIO_OUTPUT_MODE, 1, 1, 1},
    // ----------------------------------- i2c ----------------------------------
    //{"GPIOZ6 -- iic",           GPIOZ_bank_bit0_19(6),     GPIOZ_bit_bit0_19(6),  GPIO_OUTPUT_MODE, 1, 1, 1},
    //{"GPIOZ7 -- iic",           GPIOZ_bank_bit0_19(7),     GPIOZ_bit_bit0_19(7),  GPIO_OUTPUT_MODE, 1, 1, 1},
};

static void save_gpio(int port)
{
    gpio_data[port].mode = get_gpio_mode(gpio_data[port].bank, gpio_data[port].bit);
    if (gpio_data[port].mode==GPIO_OUTPUT_MODE)
    {
        if (gpio_data[port].enable){
            printk("%d---change %s output %d to input\n", port, gpio_data[port].name, gpio_data[port].value);
            gpio_data[port].value = get_gpio_val(gpio_data[port].bank, gpio_data[port].bit);
            set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_INPUT_MODE);
        } else{
            printk("%d---no change %s output %d\n", port, gpio_data[port].name, gpio_data[port].value);
        }
    } else {
        printk("%d---%s input %d\n", port, gpio_data[port].name, gpio_data[port].mode);
    }
}

static void restore_gpio(int port)
{
    if ((gpio_data[port].mode==GPIO_OUTPUT_MODE)&&(gpio_data[port].enable))
    {
        set_gpio_val(gpio_data[port].bank, gpio_data[port].bit, gpio_data[port].value);
        set_gpio_mode(gpio_data[port].bank, gpio_data[port].bit, GPIO_OUTPUT_MODE);
        printk("%d---%s output %d\n", port, gpio_data[port].name, gpio_data[port].value);
    } else {
        printk("%d---%s output/input:%d, enable:%d\n", port, gpio_data[port].name, gpio_data[port].mode, gpio_data[port].value);
    }
}

typedef struct {
    char name[32];
    unsigned reg;
    unsigned bits;
    unsigned enable;
} pinmux_data_t;


#define MAX_PINMUX 10

pinmux_data_t pinmux_data[MAX_PINMUX] = {
    {"PERIPHS_PIN_MUX_0",         0, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_1",         1, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_2",         2, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_3",         3, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_4",         4, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_5",         5, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_6",         6, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_7",         7, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_8",         8, 0xffffffff,               1},
    {"PERIPHS_PIN_MUX_9",         9, 0xffffffff,               1},
};
#define MAX_INPUT_MODE 9
pinmux_data_t gpio_inputmode_data[MAX_INPUT_MODE] = {
    {"LCDGPIOA",         P_PREG_PAD_GPIO0_EN_N, 0x3fffffff,                 1},
    {"LCDGPIOB",         P_PREG_PAD_GPIO1_EN_N, 0x00ffffff,                 1},
    {"GPIOX0_12",        P_PREG_PAD_GPIO4_EN_N, 0x00001fff,                 1},
    {"BOOT0_17",         P_PREG_PAD_GPIO3_EN_N, 0x0003ffff,                 1},
    {"GPIOZ0_19",        P_PREG_PAD_GPIO6_EN_N, 0x000fffff,                 1},
    {"GPIOY0_27",        P_PREG_PAD_GPIO2_EN_N, 0x0fffffff,                 1},
    {"GPIOW0_19",        P_PREG_PAD_GPIO5_EN_N, 0x000fffff,                 1},
    {"CRAD0_8",          P_PREG_PAD_GPIO5_EN_N, 0xff000000,                 1},
    {"GPIOP6",           P_PREG_PAD_GPIO1_EN_N, 0x40000000,                 1},
};

#define MAX_RESUME_OUTPUT_MODE 1
pinmux_data_t gpio_outputmode_data[MAX_RESUME_OUTPUT_MODE] = {
    {"GPIOZ6_7",        P_PREG_PAD_GPIO6_EN_N, 0x000fff3f,                 1},
};
static unsigned pinmux_backup[10];

#define MAX_PADPULL 7

pinmux_data_t pad_pull[MAX_PADPULL] = {
    {"PAD_PULL_UP_REG0",         P_PAD_PULL_UP_REG0, 0xffffffff,               1},
    {"PAD_PULL_UP_REG1",         P_PAD_PULL_UP_REG1, 0x00000000,               1},
    {"PAD_PULL_UP_REG2",         P_PAD_PULL_UP_REG2, 0xffffffff,               1},
    {"PAD_PULL_UP_REG3",         P_PAD_PULL_UP_REG3, 0xffffffff,               1},
    {"PAD_PULL_UP_REG4",         P_PAD_PULL_UP_REG4, 0xffffffff,               1},
    {"PAD_PULL_UP_REG5",         P_PAD_PULL_UP_REG5, 0xffffffff,               1},
    {"PAD_PULL_UP_REG6",         P_PAD_PULL_UP_REG6, 0xffffffff,               1},
};

static unsigned pad_pull_backup[MAX_PADPULL];

int  clear_mio_mux(unsigned mux_index, unsigned mux_mask)
{
    unsigned mux_reg[] = {PERIPHS_PIN_MUX_0, PERIPHS_PIN_MUX_1, PERIPHS_PIN_MUX_2,PERIPHS_PIN_MUX_3,
        PERIPHS_PIN_MUX_4,PERIPHS_PIN_MUX_5,PERIPHS_PIN_MUX_6,PERIPHS_PIN_MUX_7,PERIPHS_PIN_MUX_8,
        PERIPHS_PIN_MUX_9,PERIPHS_PIN_MUX_10,PERIPHS_PIN_MUX_11,PERIPHS_PIN_MUX_12};
    if (mux_index < 13) {
        CLEAR_CBUS_REG_MASK(mux_reg[mux_index], mux_mask);
        return 0;
    }
    return -1;
}

static void save_pinmux(void)
{
    int i;
    for (i=0;i<10;i++){
        pinmux_backup[i] = READ_CBUS_REG(PERIPHS_PIN_MUX_0+i);
        printk("--PERIPHS_PIN_MUX_%d = %x\n", i,pinmux_backup[i]);
    }
    for (i=0;i<MAX_PADPULL;i++){
        pad_pull_backup[i] = aml_read_reg32(pad_pull[i].reg);
        printk("--PAD_PULL_UP_REG%d = %x\n", i,pad_pull_backup[i]);
    }
    for (i=0;i<MAX_PINMUX;i++){
        if (pinmux_data[i].enable){
            printk("%s %x\n", pinmux_data[i].name, pinmux_data[i].bits);
            clear_mio_mux(pinmux_data[i].reg, pinmux_data[i].bits);
        }
    }
    for (i=0;i<MAX_PADPULL;i++){
        if (pad_pull[i].enable){
            printk("%s %x\n", pad_pull[i].name, pad_pull[i].bits);
            aml_write_reg32(pad_pull[i].reg, aml_read_reg32(pad_pull[i].reg) | pad_pull[i].bits);
        }
    }
    for (i=0;i<MAX_INPUT_MODE;i++){
        if (gpio_inputmode_data[i].enable){
            printk("%s %x\n", gpio_inputmode_data[i].name, gpio_inputmode_data[i].bits);
            aml_write_reg32(gpio_inputmode_data[i].reg, aml_read_reg32(gpio_inputmode_data[i].reg) | gpio_inputmode_data[i].bits);
        }
    }
}

static void restore_pinmux(void)
{
    int i;
    /*for (i=0;i<MAX_RESUME_OUTPUT_MODE;i++){
        if (gpio_outputmode_data[i].enable){
            printk("%s %x\n", gpio_outputmode_data[i].name, gpio_outputmode_data[i].bits);
            aml_write_reg32(gpio_outputmode_data[i].reg, aml_read_reg32(gpio_outputmode_data[i].reg) & gpio_outputmode_data[i].bits);
        }
    }
    set_gpio_val(GPIOZ_bank_bit0_19(6), GPIOZ_bit_bit0_19(6), 1);
    set_gpio_mode(GPIOZ_bank_bit0_19(6), GPIOZ_bit_bit0_19(6), GPIO_OUTPUT_MODE);
    set_gpio_val(GPIOZ_bank_bit0_19(7), GPIOZ_bit_bit0_19(7), 1);
    set_gpio_mode(GPIOZ_bank_bit0_19(7), GPIOZ_bit_bit0_19(7), GPIO_OUTPUT_MODE);*/
    aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,(1<<19));
    aml_set_reg32_mask(P_PREG_PAD_GPIO5_O,(1<<19));
    for (i=0;i<10;i++){
    	 printk("++PERIPHS_PIN_MUX_%d = %x\n", i,pinmux_backup[i]);
         WRITE_CBUS_REG(PERIPHS_PIN_MUX_0+i, pinmux_backup[i]);
     }
     for (i=0;i<MAX_PADPULL;i++){
    	 printk("++PAD_PULL_UP_REG%d = %x\n", i,pad_pull_backup[i]);
         aml_write_reg32(pad_pull[i].reg, pad_pull_backup[i]);
     }
}

static void m6ref_set_vccx2(int power_on)
{
    if (power_on) {
        printk(KERN_INFO "%s() Power ON\n", __FUNCTION__);
    } else {
        printk(KERN_INFO "%s() Power OFF\n", __FUNCTION__);
    }
}

static void m6ref_set_pinmux(int power_on)
{
    int i = 0;
    if (power_on) {
        restore_pinmux();
        for (i=0;i<MAX_GPIO;i++)
            restore_gpio(i);
        printk(KERN_INFO "%s() Power ON\n", __FUNCTION__);
    } else {
        save_pinmux();
        for (i=0;i<MAX_GPIO;i++)
            save_gpio(i);
        printk(KERN_INFO "%s() Power OFF\n", __FUNCTION__);
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
    // .set_pinmux = m6ref_set_pinmux,
};

static struct platform_device aml_pm_device = {
    .name       = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id     = -1,
 };
#endif /* CONFIG_SUSPEND */

/***********************************************************************
 * Meson CPUFREQ section
 **********************************************************************/
#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#include <plat/cpufreq.h>

static struct platform_device meson_cpufreq_device = {
    .name   = "cpufreq-meson",
    .dev = {
        .platform_data = NULL,
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
    {KEY_FIND,      "vol+",     CHAN_0, 0,  60},
    {KEY_MENU,      "menu",     CHAN_0, 253,    60},
    {KEY_ENTER,     "enter",    CHAN_0, 506,    60},
    {KEY_BACK,      "back",     CHAN_0, 763,    60},
    {KEY_CUT,       "ch-",      CHAN_1, 0,  60},
    {KEY_MENU,      "vol-",     CHAN_1, 763,    60},

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

#define GPIO_PENIRQ ((GPIOAO_bank_bit0_11(11)<<16)|GPIOAO_bank_bit0_11(11))

static int _key_code_list[] = {
    KEY_POWER,
    KEY_UP,
    KEY_UP,
};

static inline int key_input_init_func(void)
{
    WRITE_AOBUS_REG(AO_RTC_ADDR0, (READ_AOBUS_REG(AO_RTC_ADDR0) &~(1<<11)));
    WRITE_AOBUS_REG(AO_RTC_ADDR1, (READ_AOBUS_REG(AO_RTC_ADDR1) &~(1<<3)));
    return 0;
}

static inline int key_scan(void *data)
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
 * Audio section
 **********************************************************************/
static struct resource aml_m6_audio_resource[] = {
    [0] =   {
        .start      = 0,
        .end        = 0,
        .flags      = IORESOURCE_MEM,
    },
};

static struct platform_device aml_audio = {
    .name       = "aml-audio",
    .id         = 0,
};

static struct platform_device aml_audio_dai = {
    .name       = "aml-dai",
    .id         = 0,
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
    .device_init    = dummy_codec_device_init,
    .device_uninit  = dummy_codec_device_deinit,
};

static struct platform_device aml_dummy_codec_audio = {
    .name       = "aml_dummy_codec_audio",
    .id         = 0,
    .resource   = aml_m6_audio_resource,
    .num_resources  = ARRAY_SIZE(aml_m6_audio_resource),
    .dev = {
        .platform_data = &dummy_codec_pdata,
    },
};

static struct platform_device aml_dummy_codec = {
    .name       = "dummy_codec",
    .id     = 0,
};
#endif

#if defined(CONFIG_SND_AML_M6_AUDIO_CODEC)
static pinmux_item_t m6_audio_codec_pinmux[] = {
    /* I2S_MCLK I2S_BCLK I2S_LRCLK I2S_DOUT */
    {
        .reg = PINMUX_REG(8),
        .setmask = (1 << 26) | (1 << 25) | (1 << 23) | (1 << 21),
        .clrmask = (1 << 12) | (1 << 18) | (1 << 17) | (1 << 22)|(1 << 24),
    },
    /* mute gpio pinmux clear */
    {
        .reg = PINMUX_REG(1),
        .clrmask = (1 << 0),
    },
    {
        .reg = PINMUX_REG(3),
        .clrmask = (1 << 22),
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
    /* amplifier reset pinmux clear */
    {
    	.reg = PINMUX_REG(3),
		.clrmask = (24<<0),	
    },
    PINMUX_END_ITEM
};

static pinmux_set_t m6_audio_codec_pinmux_set = {
    .chip_select = NULL,
    .pinmux = &m6_audio_codec_pinmux[0],
};

static void m6_audio_codec_device_init(void)
{
    /*  audio MUTE pin goio setting,by default  unmute< pull high >  */
    WRITE_CBUS_REG_BITS(0x201b,0,19,1);
    WRITE_CBUS_REG_BITS(0x201c,1,19,1);
    pinmux_set(&m6_audio_codec_pinmux_set);
}

static void m6_audio_codec_device_deinit(void)
{
    pinmux_clr(&m6_audio_codec_pinmux_set);
}
static struct m6_audio_codec_platform_data m6_audio_codec_pdata = {
    .device_init    = m6_audio_codec_device_init,
    .device_uninit  = m6_audio_codec_device_deinit,
};

static struct platform_device aml_m6_audio = {
    .name       = "aml_m6_audio",
    .id         = 0,
    .resource   = aml_m6_audio_resource,
    .num_resources  = ARRAY_SIZE(aml_m6_audio_resource),
    .dev = {
        .platform_data = &m6_audio_codec_pdata,
    },
};
#endif

#ifdef CONFIG_D2D3_PROCESS
static struct resource d2d3_resource[] = {
    [0] = {
        .start = D2D3_ADDR_START,
        .end = D2D3_ADDR_END,
        .flags = IORESOURCE_MEM,
        .name = "d2d3_mem"
    },
    [1] = {
        .start = INT_D2D3,
        .end  = INT_D2D3,
        .flags = IORESOURCE_IRQ,
        .name = "d2d3_irq"
    },
};

static struct platform_device d2d3_device = {
    .name = "d2d3",
    .id = -1,
    .num_resources = ARRAY_SIZE(d2d3_resource),
    .resource = d2d3_resource,
};
#endif//de CONFIG_D2D3_PROCESS

/***********************************************************************
 * Device Register Section
 **********************************************************************/
static struct platform_device  *platform_devs[] = {
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    &aml_i2c_device_b,
#endif
    &aml_uart_device,
#ifdef CONFIG_AM_ETHERNET
    &meson_device_eth,
#endif
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
#if defined(CONFIG_MMC_AML)
    &aml_mmc_device,
#endif
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
#ifdef CONFIG_SND_AML_M6_AUDIO_CODEC
    &aml_m6_audio,
#endif    
#ifdef CONFIG_AM_WIFI
    &wifi_power_device,
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
#ifdef CONFIG_CPU_FREQ
    &meson_cpufreq_device,
#endif
#ifdef CONFIG_D2D3_PROCESS
    &d2d3_device,
#endif
};

static __init void meson_init_machine(void)
{
//  meson_cache_init();

#ifdef CONFIG_AM_ETHERNET
    setup_eth_device();
#endif

#ifdef CONFIG_AM_REMOTE
    setup_remote_device();
#endif

#ifdef CONFIG_EFUSE
    setup_aml_efuse();
#endif

#ifdef CONFIG_AML_HDMI_TX
    setup_hdmi_dev_platdata(&aml_hdmi_pdata);
#endif

    setup_usb_devices();
    setup_devices_resource();
    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

#ifdef CONFIG_AM_WIFI_USB
    if(wifi_plat_data.usb_set_power)
        wifi_plat_data.usb_set_power(0);//power off built-in usb wifi
#endif

#if defined(CONFIG_SUSPEND)
    {
        //todo: remove it after verified. need set it in uboot environment variable.
       //  extern  int console_suspend_enabled;
        // console_suspend_enabled = 0;
    }
#endif

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
