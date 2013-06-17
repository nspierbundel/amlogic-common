/*
 * arch/arm/mach-meson3/board-m3ref.c
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
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <plat/platform.h>
#include <plat/plat_dev.h>
#include <plat/platform_data.h>
#include <plat/lm.h>
#include <plat/regops.h>
#include <mach/am_regs.h>
#include <mach/clock.h>
#include <mach/map.h>
#include <mach/i2c_aml.h>
#include <mach/nand.h>
#include <mach/usbclock.h>
#include <mach/usbsetting.h>
#include <mach/gpio.h>

#ifdef CONFIG_AM_ETHERNET
#include <mach/am_regs.h>
#include <mach/am_eth_reg.h>
#endif

#include "board-a11.h"

#ifdef CONFIG_MMC_AML
#include <mach/mmc.h>
#endif
#include <linux/i2c-aml.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
//#ifdef CONFIG_AM_UART
#include <linux/uart-aml.h>
//#endif
#ifdef CONFIG_SUSPEND
#include <mach/pm.h>
#endif

#ifdef CONFIG_CARDREADER
#include <mach/card_io.h>
#include <mach/gpio.h>
#endif

#include "board-m3ref-pinmux.h"

#if defined(CONFIG_AML_HDMI_TX)
#include <plat/hdmi_config.h>
#endif

#define DEBUG_GPIO_INTERFACE
#ifdef DEBUG_GPIO_INTERFACE
#include <mach/gpio.h>
#include <mach/gpio_data.h>
#endif

#ifdef CONFIG_SND_AML_M3
#include <sound/soc.h>
#include <sound/aml_platform.h>
#endif

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


#if defined(CONFIG_LEDS_GPIO_PLATFORM)
#include <linux/leds.h>
#endif

/* GPIO Defines */
// LEDS
#define GPIO_LED_STATUS	GPIO_AO(10)
#define GPIO_LED_POWER	GPIO_AO(11)
// ETHERNET
#define GPIO_ETH_RESET	GPIO_D(7)
// BUTTONS
#define GPIO_KEY_POWER	GPIO_AO(3)
// POWERSUPPLIES
#define GPIO_PWR_WIFI	GPIO_C(5)
#define GPIO_PWR_VCCIO	GPIO_AO(2)
#define GPIO_PWR_VCCx2	GPIO_AO(6)
#define GPIO_PWR_HDMI	GPIO_D(6)
#define GPIO_PWR_SD	GPIO_CARD(8)
// SD CARD
#define GPIO_SD_WP	GPIO_CARD(6)
#define GPIO_SD_DET	GPIO_CARD(7)

#if defined(CONFIG_LEDS_GPIO_PLATFORM)
/* LED Class Support for the leds */
static struct gpio_led aml_led_pins[] = {
	{	
		.name	= "Powerled",
		.default_trigger = "default-on",
		.gpio	= GPIO_LED_POWER,
		.active_low	= 0,
	},
	{
		.name	= "Statusled",
#if defined(CONFIG_LEDS_TRIGGER_REMOTE_CONTROL)
		.default_trigger = "rc",
#else
		.default_trigger = "none",
#endif
		.gpio	= GPIO_LED_STATUS,
		.active_low	= 1,
	},
};

static struct gpio_led_platform_data aml_led_data = {
	.leds	= aml_led_pins,
	.num_leds = ARRAY_SIZE(aml_led_pins),
};

static struct platform_device aml_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &aml_led_data,
	}
};
#endif

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static struct adc_key adc_kp_key[] = {
    {KEY_MENU,          "menu", CHAN_4, 0, 60},
    {KEY_VOLUMEDOWN,    "vol-", CHAN_4, 140, 60},
    {KEY_VOLUMEUP,      "vol+", CHAN_4, 266, 60},
    {KEY_BACK,          "exit", CHAN_4, 386, 60},
    {KEY_HOME,          "home", CHAN_4, 508, 60},
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

#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
#include <linux/input.h>
#include <linux/input/key_input.h>

static int _key_code_list[] = {KEY_POWER};

static inline int key_input_init_func(void)
{
	// Power Button, GPIO AO3, ACTIVE LOW
	gpio_direction_input(GPIO_KEY_POWER);
	return 0;
}

static inline int key_scan(void* data)
{
        int *key_state_list = (int*)data;
        key_state_list[0] = gpio_get_value(GPIO_KEY_POWER) ? 0 : 1;
        return 0;
}

static  struct key_input_platform_data  key_input_pdata = {
    .scan_period	= 20,
    .fuzz_time		= 60,
    .key_code_list	= &_key_code_list[0],
    .key_num		= ARRAY_SIZE(_key_code_list),
    .scan_func		= key_scan,
    .init_func		= key_input_init_func,
    .config		= 0,
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

static void set_usb_a_vbus_power(char is_power_on)
{
}

static  int __init setup_usb_devices(void)
{
	struct lm_device * usb_ld_a;
	usb_ld_a = alloc_usb_lm_device(USB_PORT_IDX_A);
	usb_ld_a->param.usb.set_vbus_power = set_usb_a_vbus_power;
	lm_device_register(usb_ld_a);
	return 0;
}

#if defined (CONFIG_AMLOGIC_VIDEOIN_MANAGER)
static struct resource vm_resources[] = {
    [0] = {
        .start = VM_ADDR_START,
        .end   = VM_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vm_device =
{
    .name = "vm",
    .id = 0,
    .num_resources = ARRAY_SIZE(vm_resources),
    .resource      = vm_resources,
};
#endif /* AMLOGIC_VIDEOIN_MANAGER */

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
#endif /* CONFIG_POST_PROCESS_MANAGER */

#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
static bool pinmux_dummy_share(bool select)
{
    return select;
}

static pinmux_item_t aml_i2c_0_pinmux_item[] = {
    {
        .reg = 5,
        //.clrmask = (3<<24)|(3<<30),
        .setmask = 3<<26
    },
    PINMUX_END_ITEM
};

static struct aml_i2c_platform aml_i2c_plat = {
    .wait_count         = 50000,
    .wait_ack_interval  = 5,
    .wait_read_interval	= 5,
    .wait_xfer_interval	= 5,
    .master_no          = AML_I2C_MASTER_A,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux      = {
        .chip_select    = pinmux_dummy_share,
        .pinmux         = &aml_i2c_0_pinmux_item[0]
    }
};

static struct resource aml_i2c_resource[] = {
    [0] = {
        .start = MESON_I2C_MASTER_A_START,
        .end   = MESON_I2C_MASTER_A_END,
        .flags = IORESOURCE_MEM,
	}
};

static struct platform_device aml_i2c_device = {
    .name         = "aml-i2c",
    .id       = 0,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};

static struct i2c_board_info __initdata aml_i2c_bus_info[] = {
	{
		I2C_BOARD_INFO("at88scxx", 0xB6),
	},
};

static int __init aml_i2c_init(void)
{
	return i2c_register_board_info(0, aml_i2c_bus_info, ARRAY_SIZE(aml_i2c_bus_info));
}
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
	 .end = STREAMBUF_ADDR_END,
	 .flags = IORESOURCE_MEM,
    },
};

static  int __init setup_devices_resource(void)
{
     setup_fb_resource(meson_fb_resource, ARRAY_SIZE(meson_fb_resource));
     setup_codec_resource(meson_codec_resource,ARRAY_SIZE(meson_codec_resource));
     return 0;
}

#if 0
static  int __init setup_i2c_devices(void)
{
    //just a sample 
    meson_i2c1_set_platdata(&aml_i2c_data_a,sizeof(struct aml_i2c_platform_a ));
    meson_i2c2_set_platdata(&aml_i2c_data_b,sizeof(struct aml_i2c_platform_b ));    
    return 0;
}

static  int __init setup_uart_devices(void)
{
#if 0    
    static pinmux_set_t temp;
//#if defined(CONFIG_AM_UART)
    aml_uart_plat.public.pinmux_cfg.setup=NULL; //NULL repsent use defaut pinmux_set.
    aml_uart_plat.public.pinmux_cfg.clear=NULL;
    aml_uart_plat.public.clk_src=clk_get_sys("clk81", NULL);
    temp.pinmux=cur_board_pinmux[DEVICE_PIN_ITEM_UART];
    aml_uart_plat.public.pinmux_set.pinmux=cur_board_pinmux[DEVICE_PIN_ITEM_UART];
    aml_uart_plat.pinmux_uart[0]=&temp;
    meson_uart_set_platdata(&aml_uart_plat,sizeof(struct aml_uart_platform));
#endif   
    return 0;
//#endif    
}
#endif

static void __init device_pinmux_init(void)
{
	
    // clearall_pinmux();
    /*other deivce power on*/
    // uart_set_pinmux(UART_PORT_AO, UART_AO_GPIO_AO0_AO1_STD);
    /*pinmux of eth*/
    // eth_pinmux_init();
//    set_audio_pinmux(AUDIO_IN_JTAG); // for MIC input
//    set_audio_pinmux(AUDIO_OUT_TEST_N); //External AUDIO DAC
//    set_audio_pinmux(SPDIF_OUT_GPIOA); //SPDIF GPIOA_6
}

static void __init  device_clk_setting(void)
{
#if 0 ///@todo Jerry Yu, Compile break , enable it later	
    /*Configurate the ethernet clock*/
    eth_clk_set(ETH_CLKSRC_MISC_CLK, get_misc_pll_clk(), (50 * CLK_1M), 0);
#endif    
}

static void disable_unused_model(void)
{
#if 0 ///@todo Jerry Yu, Compile break , enable it later	
    CLK_GATE_OFF(VIDEO_IN);
    CLK_GATE_OFF(BT656_IN);
#endif    
}
static void __init meson_cache_init(void)
{
#ifdef CONFIG_CACHE_L2X0
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
#endif
}

static void  __init backup_board_pinmux(void)
{//devices_pins in __initdata section ,it will be released .
#if 0
        cur_board_pinmux=kmemdup(devices_pins, sizeof(devices_pins), GFP_KERNEL);
        printk(KERN_INFO " cur_board_pinmux=%p",cur_board_pinmux[0]);
        printk(KERN_INFO " cur_board_pinmux=%p",&(cur_board_pinmux[0]));
	printk(KERN_INFO " cur_board_pinmux=%x",cur_board_pinmux[0][0]);
	printk(KERN_INFO " cur_board_pinmux=%x",cur_board_pinmux[0][0]->reg);
#endif    
}

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

static struct __init aml_uart_platform aml_uart_plat = {
    .uart_line[0]   = UART_AO,
    .uart_line[1]   = UART_A,
    .uart_line[2]   = UART_B,
    .uart_line[3]   = UART_C,

    .pinmux_uart[0] = (void*)&aml_uart_ao,
    .pinmux_uart[1] = NULL,
    .pinmux_uart[2] = NULL,
    .pinmux_uart[3] = NULL,
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

#ifdef CONFIG_AM_ETHERNET
//#define ETH_MODE_RGMII
//#define ETH_MODE_RMII_INTERNAL
#define ETH_MODE_RMII_EXTERNAL
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
    aml_set_reg32_mask(P_A9_0_IRQ_IN0_INTR_MASK, 1 << 8);
    aml_set_reg32_mask(P_A9_0_IRQ_IN1_INTR_STAT, 1 << 8);

    /* hardware reset ethernet phy */
    gpio_direction_output(GPIO_ETH_RESET, 0);
    msleep(20);
    gpio_set_value(GPIO_ETH_RESET, 1);

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




#if defined(CONFIG_AML_RTC)
static struct platform_device meson_rtc_device = {
	.name	= "aml_rtc",
	.id	= -1,
};
#endif /* CONFIG_AML_RTC */

#ifdef CONFIG_AM_NAND

/*** Default NAND layout for A11 ***
Creating 7 MTD partitions on "nandmulti":
4M    0x000000800000-0x000000c00000 : "logo"      : 8M
12M   0x000000c00000-0x000001400000 : "boot"      : 8M
20M   0x000001400000-0x000021400000 : "system"    : 512M
532M  0x000021400000-0x000034000000 : "cache"     : 300M
832M  0x000034000000-0x000074000000 : "NFTL_Part" : 1024M
1856M 0x000074000000-0x000080800000 : "backup"    : 200M
2056M 0x000080800000-0x000100000000 : "userdata"  : 2040M
*/ 

static struct mtd_partition normal_partition_info[] = { // 4G
#ifndef CONFIG_AMLOGIC_SPI_NOR
/* Hide uboot partition
	{
		.name = "uboot",
		.offset = 0,
		.size = 4*1024*1024,
	},
//*/
    {
        .name = "ubootenv",
        .offset = 4*1024*1024,
        .size = 0x2000,
    },
/* Hide recovery partition
    {
        .name = "recovery",
        .offset = 6*1024*1024,
        .size = 2*1024*1024,
    },
//*/
#endif
	{//4M for logo
		.name   = "logo",
		.offset = 8*1024*1024,
		.size   = 4*1024*1024,
	},
	{//8M for kernel
		.name   = "boot",
		.offset = (8+4)*1024*1024,
		.size   = 8*1024*1024,
	},
	{//512M for android system.
		.name   = "system",
		.offset = (8+4+8)*1024*1024,
		.size   = 512*1024*1024,
	},
	{//300M for cache
		.name   = "cache",
		.offset = (8+4+8+512)*1024*1024,
		.size   = 300*1024*1024,
	},
#ifdef CONFIG_AML_NFTL
	{//1G for NFTL_part
		.name   = "NFTL_Part",
		.offset = (8+4+8+512+300)*1024*1024,
		.size   = 1024*1024*1024,
	},
	{//other 2040M for user data
		.name = "userdata",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL,
	},
#else
	{
		.name = "userdata",
		.offset=MTDPART_OFS_APPEND,
		.size=MTDPART_SIZ_FULL,
	},
#endif
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
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH30_1K_MODE),
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

#ifdef CONFIG_CARDREADER
static struct resource amlogic_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};

void extern_wifi_power(int is_power)
{
    // TODO: Add Right Registers
//    reg_on_control(is_power);
}

EXPORT_SYMBOL(extern_wifi_power);

static struct aml_card_info  amlogic_card_info[] = {
    [0] = {
        .name = "sd_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_B_CARD_0_5,
        .card_ins_en_reg = CARD_GPIO_ENABLE,
        .card_ins_en_mask = PREG_IO_29_MASK,
        .card_ins_input_reg = CARD_GPIO_INPUT,
        .card_ins_input_mask = PREG_IO_29_MASK,
        .card_power_en_reg = CARD_GPIO_ENABLE,
        .card_power_en_mask = PREG_IO_31_MASK,
        .card_power_output_reg = CARD_GPIO_OUTPUT,
        .card_power_output_mask = PREG_IO_31_MASK,
        .card_power_en_lev = 0,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = 0,
    },
};

static struct aml_card_platform amlogic_card_platform = {
    .card_num = ARRAY_SIZE(amlogic_card_info),
    .card_info = amlogic_card_info,
};

static struct platform_device amlogic_card_device = { 
    .name = "AMLOGIC_CARD", 
    .id    = -1,
    .num_resources = ARRAY_SIZE(amlogic_card_resource),
    .resource = amlogic_card_resource,
    .dev = {
        .platform_data = &amlogic_card_platform,
    },
};	
#endif //END CONFIG_CARDREADER	

#ifdef CONFIG_MMC_AML
struct platform_device;
struct mmc_host;
struct mmc_card;
struct mmc_ios;

//return 1: no inserted  0: inserted
static int aml_sdio_detect(struct aml_sd_host * host)
{
	SET_CBUS_REG_MASK(PREG_PAD_GPIO5_EN_N,1<<29);//CARD_6
	return READ_CBUS_REG(PREG_PAD_GPIO5_I)&(1<<29)?1:0;
}

static void  cpu_sdio_pwr_prepare(unsigned port)
{
    switch(port)
    {
        case M3_SDIO_PORT_A:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO4_EN_N,0x30f);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO4_O   ,0x30f);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8,0x3f);
            break;
        case M3_SDIO_PORT_B:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,0x3f<<23);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO5_O   ,0x3f<<23);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,0x3f<<10);
            break;
        case M3_SDIO_PORT_C:
            aml_clr_reg32_mask(P_PREG_PAD_GPIO3_EN_N,0xc0f);
            aml_clr_reg32_mask(P_PREG_PAD_GPIO3_O   ,0xc0f);
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            break;
        case M3_SDIO_PORT_XC_A:
            break;
        case M3_SDIO_PORT_XC_B:
            break;
        case M3_SDIO_PORT_XC_C:
            break;
    }
}

static int cpu_sdio_init(unsigned port)
{
    switch(port)
    {
        case M3_SDIO_PORT_A:
        		aml_set_reg32_mask(P_PERIPHS_PIN_MUX_8,0x3f);\
        		break;
        case M3_SDIO_PORT_B:
        		aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,0x3f<<10);
        		break;
        case M3_SDIO_PORT_C://SDIOC GPIOB_2~GPIOB_7
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,(0x1f<<22));
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            break;
        case M3_SDIO_PORT_XC_A:
            #if 0
            //sdxc controller can't work
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_8,(0x3f<<0));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_3,(0x0f<<27));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_7,((0x3f<<18)|(0x7<<25)));
            //aml_set_reg32_mask(P_PERIPHS_PIN_MUX_5,(0x1f<<10));//data 8 bit
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_5,(0x1b<<10));//data 4 bit
            #endif
            break;
        case M3_SDIO_PORT_XC_B:
            //sdxc controller can't work
            //aml_set_reg32_mask(P_PERIPHS_PIN_MUX_2,(0xf<<4));
            break;
        case M3_SDIO_PORT_XC_C:
            #if 0
            //sdxc controller can't work
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_6,(0x3f<<24));
            aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_2,((0x13<<22)|(0x3<<16)));
            aml_set_reg32_mask(P_PERIPHS_PIN_MUX_4,(0x1f<<26));
            printk(KERN_INFO "inand sdio xc-c init\n");
            #endif
            break;
        default:
            return -1;
    }
    return 0;
}

static void aml_sdio_pwr_prepare(unsigned port)
{
    /// @todo NOT FINISH
	///do nothing here
	cpu_sdio_pwr_prepare(port);
}

static void aml_sdio_pwr_on(unsigned port)
{
	
	if((aml_read_reg32(P_PREG_PAD_GPIO5_O) & (1<<31)) != 0){
	aml_clr_reg32_mask(P_PREG_PAD_GPIO5_O,(1<<31));
	aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,(1<<31));
	udelay(1000);
	}
    /// @todo NOT FINISH
}
static void aml_sdio_pwr_off(unsigned port)
{

	if((aml_read_reg32(P_PREG_PAD_GPIO5_O) & (1<<31)) == 0){
		aml_set_reg32_mask(P_PREG_PAD_GPIO5_O,(1<<31));
		aml_clr_reg32_mask(P_PREG_PAD_GPIO5_EN_N,(1<<31));//GPIOD13
		udelay(1000);
	}
	/// @todo NOT FINISH
}

static int aml_sdio_init(struct aml_sd_host * host)
{ //set pinumx ..
	aml_set_reg32_mask(P_PREG_PAD_GPIO5_EN_N,1<<29);//CARD_6
	cpu_sdio_init(host->sdio_port);	
	host->clk = clk_get_sys("clk81",NULL);
	if(!IS_ERR(host->clk))
		host->clk_rate = clk_get_rate(host->clk);
	else
		host->clk_rate = 0;
	return 0;
}

static struct resource aml_mmc_resource[] = {
   [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x1200248,
        .flags = IORESOURCE_MEM, //0x200
    },
};

static u64 aml_mmc_device_dmamask = 0xffffffffUL;
static struct aml_mmc_platform_data aml_mmc_def_platdata = {
	.no_wprotect = 1,
	.no_detect = 0,
	.wprotect_invert = 0,
	.detect_invert = 0,
	.use_dma = 0,
	.gpio_detect=1,
	.gpio_wprotect=0,
	.ocr_avail = MMC_VDD_33_34,
	
	.sdio_port = M3_SDIO_PORT_B,
	.max_width	= 4,
	.host_caps	= (MMC_CAP_4_BIT_DATA |	MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_NEEDS_POLL),

	.f_min = 200000,
	.f_max = 50000000,
	.clock = 300000,

	.sdio_init = aml_sdio_init,
	.sdio_detect = aml_sdio_detect,
	.sdio_pwr_prepare = aml_sdio_pwr_prepare,
	.sdio_pwr_on = aml_sdio_pwr_on,
	.sdio_pwr_off = aml_sdio_pwr_off,
};

static struct platform_device aml_mmc_device = {
	.name		= "aml_sd_mmc",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(aml_mmc_resource),
	.resource	= aml_mmc_resource,
	.dev		= {
		.dma_mask		=       &aml_mmc_device_dmamask,
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data		= &aml_mmc_def_platdata,
	},
};
#endif //CONFIG_MMC_AML

#if defined(CONFIG_AML_AUDIO_DSP)
static struct resource audiodsp_resources[] = {
    [0] = {
        .start = AUDIODSP_ADDR_START,
        .end   = AUDIODSP_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device audiodsp_device = {
    .name       = "audiodsp",
    .id         = 0,
    .num_resources = ARRAY_SIZE(audiodsp_resources),
    .resource      = audiodsp_resources,
};
#endif //CONFIG_AML_AUDIO_DSP

#if defined(CONFIG_SND_AML_M3)
static struct resource aml_m3_audio_resource[] = {
    [0] =   {
        .start  =   0,
        .end    =   0,
        .flags  =   IORESOURCE_MEM,
    },
};

extern char * get_vout_mode_internal(void);

/* Check current mode, 0: panel; 1: !panel*/
static int get_display_mode(void) {
	return 1;
}

static int aml_m3_is_hp_plugged(void)
{
	return 0;
}

static void mute_spk(int flag)
{
}

static struct aml_audio_platform aml_audio_pdata = {
    .is_hp_pluged	= &aml_m3_is_hp_plugged,
    .mute_spk		= &mute_spk,
};

/* machine platform device */
static struct platform_device aml_m3_audio = {
    .name           = "aml_m3_audio",
    .id             = -1,
    .resource       = aml_m3_audio_resource,
    .num_resources  = ARRAY_SIZE(aml_m3_audio_resource),
    .dev = {
	.platform_data = &aml_audio_pdata,
    },
};

/* dai platform device */
static struct platform_device aml_dai = {
    .name           = "aml-dai",
    .id             = 0,
    .num_resources  = 0,
};

/* pcm audio platform device */
static struct platform_device aml_audio = {
    .name           = "aml-audio",
    .id             = -1,
    .num_resources  = 0,
};
#endif //CONFIG_SND_AML_M3

#if defined(CONFIG_SUSPEND)
typedef struct {
	char name[32];
	unsigned reg;
	unsigned bits;
	unsigned enable;
} pinmux_data_t;

#define MAX_PINMUX	1
static pinmux_data_t pinmux_data[MAX_PINMUX] = {
	{"HDMI", 0, (1<<2)|(1<<1)|(1<<0), 1},
};

static unsigned pinmux_backup[6];
static void save_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		pinmux_backup[i] = aml_read_reg32(P_PERIPHS_PIN_MUX_0+i);
	for (i=0;i<MAX_PINMUX;i++){
		if (pinmux_data[i].enable){
			printk("%s %x\n", pinmux_data[i].name, pinmux_data[i].bits);	
			aml_clr_reg32_mask(P_PERIPHS_PIN_MUX_0+pinmux_data[i].reg, pinmux_data[i].bits);
		}
	}
}

static void restore_pinmux(void)
{
	int i;
	for (i=0;i<6;i++)
		aml_write_reg32(P_PERIPHS_PIN_MUX_0+i,pinmux_backup[i]);
}

void m3ref_set_vccx2(int power_on)
{
    if (power_on) {
        //restore_pinmux();
        printk(KERN_INFO "%s() Power ON\n", __FUNCTION__);
	// TODO: Add vccx2 enable

	// VCCIO +3V3 -- GPIO AO2, ACTIVE HIGH
	gpio_direction_output( GPIO_PWR_VCCIO, 1);

	// VCCx2 +5V -- GPIO AO6, ACTIVE HIGH.
	gpio_direction_output( GPIO_PWR_VCCx2, 1);

	// HDMI Power +5V -- GPIO D6, ACTIVE HIGH
	gpio_direction_output( GPIO_PWR_HDMI, 1);

    } else {
        printk(KERN_INFO "%s() Power OFF\n", __FUNCTION__);
        //save_pinmux();
	// TODO: Add vccx2 enable

	// HDMI Power +5V -- GPIO D6, ACTIVE HIGH
	gpio_direction_output( GPIO_PWR_HDMI, 0);

	// VCCx2 +5V -- GPIO AO6, ACTIVE HIGH.
	gpio_direction_output( GPIO_PWR_VCCx2, 0);

	// VCCIO +3V3 -- GPIO AO2, ACTIVE HIGH
	gpio_direction_output( GPIO_PWR_VCCIO, 0);
    }
}

static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = (void *)IO_APB_BUS_BASE,
    .mmc_reg_base = (void *)APB_REG_ADDR(0x1000),
    .hiu_reg_base = (void *)CBUS_REG_ADDR(0x1000),
    .power_key = (1<<8),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = m3ref_set_vccx2,
    .core_voltage_adjust = 7,  //5,8
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif //CONFIG_SUSPEND

#if defined(CONFIG_AML_HDMI_TX)
static struct hdmi_phy_set_data brd_phy_data[] = {
	{-1,   -1},         //end of phy setting
};

static struct hdmi_config_platform_data aml_hdmi_pdata ={
	.hdmi_5v_ctrl = NULL,
	.hdmi_3v3_ctrl = NULL,
	.hdmi_pll_vdd_ctrl = NULL,
	.hdmi_sspll_ctrl = NULL,
	.phy_data = brd_phy_data,
};

static struct platform_device aml_hdmi_device = {
	.name = "amhdmitx",
	.id   = -1,
	.dev  = {
		.platform_data = &aml_hdmi_pdata,
	}
};
#endif

static struct platform_device  *platform_devs[] = {
    &aml_uart_device,
#if defined(CONFIG_AML_HDMI_TX)
    &aml_hdmi_device,
#endif
    &meson_device_fb,
    //&meson_device_codec,
#if defined(CONFIG_SND_AML_M3)
    //&aml_audio,
    //&aml_dai,
    //&aml_m3_audio,
#endif
#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_VIRTUAL_REMOTE)||defined(CONFIG_KEYPADS_AM_MODULE)
    &input_device,
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
#if defined(CONFIG_MMC_AML)
    &aml_mmc_device,
#endif
#if defined(CONFIG_CARDREADER)
    &amlogic_card_device,
#endif
#if defined(CONFIG_SUSPEND)
    &aml_pm_device,
#endif
#if defined(CONFIG_AM_NAND)
    &aml_nand_device,
#endif
#if defined(CONFIG_TVIN_VDIN)
    //&vdin_device,
#endif
#if defined(CONFIG_AML_AUDIO_DSP)
    &audiodsp_device,
#endif //CONFIG_AML_AUDIO_DSP
#ifdef CONFIG_POST_PROCESS_MANAGER
    &ppmgr_device,
#endif
#if defined(CONFIG_AML_RTC)
    &meson_rtc_device,
#endif
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
    &aml_i2c_device,
#endif
#if defined(CONFIG_LEDS_GPIO_PLATFORM)
    &aml_leds,
#endif
};

static void __init power_hold(void)
{
	printk(KERN_INFO "power hold set high!\n");

	printk(KERN_INFO "set_vccio and set_vccx2 power up\n");
	// VCCIO +3V3 -- GPIO AO2, ACTIVE HIGH
	gpio_direction_output( GPIO_PWR_VCCIO, 1);

	// VCCx2 +5V -- GPIO AO6, ACTIVE HIGH.
	gpio_direction_output( GPIO_PWR_VCCx2, 1);

	printk(KERN_INFO "set_hdmi power up\n");
	// HDMI Power +5V -- GPIO D6, ACTIVE HIGH
	gpio_direction_output( GPIO_PWR_HDMI, 1);

	// Turn On Wifi Power. So the wifi-module can be detected.
	// extern_usb_wifi_power(1);
}

static __init void meson_m3ref_init(void)
{
	// backup_board_pinmux();
	meson_cache_init();
	setup_devices_resource();
	power_hold();
	platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));
	
#if defined(CONFIG_I2C_AML) || defined(CONFIG_I2C_HW_AML)
	aml_i2c_init();
#endif
#ifdef CONFIG_AM_ETHERNET
	setup_eth_device();
#endif
    disable_unused_model();
}


/***********************************************************************
 * IO Mapping
 **********************************************************************/
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
        .length     = SZ_4K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_APB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AOBUS_BASE,
        .pfn        = __phys_to_pfn(IO_AOBUS_PHY_BASE),
        .length     = SZ_1M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_AHB_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_AHB_BUS_PHY_BASE),
        .length     = SZ_16M,
        .type       = MT_DEVICE,
    } , {
        .virtual    = IO_APB2_BUS_BASE,
        .pfn        = __phys_to_pfn(IO_APB2_BUS_PHY_BASE),
        .length     = SZ_512K,
        .type       = MT_DEVICE,
    }, {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END - RESERVED_MEM_START + 1,
        .type       = MT_DEVICE,
    },
#ifdef CONFIG_MESON_SUSPEND
	{
        .virtual    = PAGE_ALIGN(0xdff00000),
        .pfn        = __phys_to_pfn(0x1ff00000),
        .length     = SZ_1M,
        .type       = MT_MEMORY,
	},
#endif
};

static  void __init meson_map_io(void)
{
    iotable_init(meson_io_desc, ARRAY_SIZE(meson_io_desc));
}

static __init void m3_irq_init(void)
{
    meson_init_irq();
}

static __init void m3_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    m->nr_banks = 0;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
///    pbank->node  = PHYS_TO_NID(PHYS_MEM_START);
    m->nr_banks++;
    pbank = &m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END + 1);
#ifdef CONFIG_MESON_SUSPEND
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END-SZ_1M) & PAGE_MASK;
#else
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
#endif
   /// pbank->node  = PHYS_TO_NID(RESERVED_MEM_END + 1);
    m->nr_banks++;

}

MACHINE_START(M3_REF, "Amlogic Meson3 reference development platform")
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = meson_map_io,
    .init_irq       = m3_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = meson_m3ref_init,
    .fixup          = m3_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END

MACHINE_START(VMX25, "Amlogic Meson3 reference development platform (legacy)")
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = meson_map_io,
    .init_irq       = m3_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = meson_m3ref_init,
    .fixup          = m3_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END