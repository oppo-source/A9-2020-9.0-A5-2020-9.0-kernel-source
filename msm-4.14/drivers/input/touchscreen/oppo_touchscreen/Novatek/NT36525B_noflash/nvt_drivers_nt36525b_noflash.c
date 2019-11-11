/*****************************************************************************************
 * Copyright (c)  2008- 2019  Oppo Mobile communication Corp.ltd.
 * VENDOR_EDIT
 * File       : novatek_drivers_nt36525b.c
 * Description: Source file for novatek nt36525b driver
 * Version   : 1.0
 * Date        : 2019/04/22
 * Author    : Ping.Zhang@PSW.BSP.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/input/mt.h>
//#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "nvt_drivers_nt36525b_noflash.h"

/*******Part0:LOG TAG Declear********************/

//static uint8_t ilm_dlm_num = 2;
static struct timeval start, end;

/****************** Start of Log Tag Declear and level define*******************************/
#define TPD_DEVICE "novatek,nf_nt36525b"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (LEVEL_DEBUG == tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DEBUG_NTAG(a, arg...)\
    do{\
        if (tp_debug)\
            printk(a, ##arg);\
    }while(0)
/******************** End of Log Tag Declear and level define*********************************/

static int8_t nvt_cmd_store(struct chip_data_nt36525b *chip_info, uint8_t u8Cmd);

static fw_update_state nvt_fw_update_sub(void *chip_data, const struct firmware *fw, bool force);
static fw_update_state nvt_fw_update(void *chip_data, const struct firmware *fw, bool force);
static int nvt_reset(void *chip_data);
static int nvt_get_chip_info(void *chip_data);

static struct chip_data_nt36525b *g_chip_info = NULL;

/***************************** start of id map table******************************************/
static const struct nvt_ts_mem_map NT36672A_memory_map = {
    .EVENT_BUF_ADDR           = 0x21C00,
    .RAW_PIPE0_ADDR           = 0x20000,
    .RAW_PIPE0_Q_ADDR         = 0,
    .RAW_PIPE1_ADDR           = 0x23000,
    .RAW_PIPE1_Q_ADDR         = 0,
    .BASELINE_ADDR            = 0x20BFC,
    .BASELINE_Q_ADDR          = 0,
    .BASELINE_BTN_ADDR        = 0x23BFC,
    .BASELINE_BTN_Q_ADDR      = 0,
    .DIFF_PIPE0_ADDR          = 0x206DC,
    .DIFF_PIPE0_Q_ADDR        = 0,
    .DIFF_PIPE1_ADDR          = 0x236DC,
    .DIFF_PIPE1_Q_ADDR        = 0,
    .RAW_BTN_PIPE0_ADDR       = 0x20510,
    .RAW_BTN_PIPE0_Q_ADDR     = 0,
    .RAW_BTN_PIPE1_ADDR       = 0x23510,
    .RAW_BTN_PIPE1_Q_ADDR     = 0,
    .DIFF_BTN_PIPE0_ADDR      = 0x20BF0,
    .DIFF_BTN_PIPE0_Q_ADDR    = 0,
    .DIFF_BTN_PIPE1_ADDR      = 0x23BF0,
    .DIFF_BTN_PIPE1_Q_ADDR    = 0,
    .READ_FLASH_CHECKSUM_ADDR = 0x24000,
    .RW_FLASH_DATA_ADDR       = 0x24002,
    .DOZE_GM_S1D_SCAN_RAW_ADDR = 0x23C1C,
    .DOZE_GM_BTN_SCAN_RAW_ADDR = 0x23CAC,
    /* Phase 2 Host Download */
    .BOOT_RDY_ADDR            = 0x3F10D,
    /* BLD CRC */
    .BLD_LENGTH_ADDR          = 0x3F10E,    //0x3F10E ~ 0x3F10F (2 bytes)
    .ILM_LENGTH_ADDR          = 0x3F118,    //0x3F118 ~ 0x3F119 (2 bytes)
    .DLM_LENGTH_ADDR          = 0x3F130,    //0x3F130 ~ 0x3F131 (2 bytes)
    .BLD_DES_ADDR             = 0x3F114,    //0x3F114 ~ 0x3F116 (3 bytes)
    .ILM_DES_ADDR             = 0x3F128,    //0x3F128 ~ 0x3F12A (3 bytes)
    .DLM_DES_ADDR             = 0x3F12C,    //0x3F12C ~ 0x3F12E (3 bytes)
    .G_ILM_CHECKSUM_ADDR      = 0x3F100,    //0x3F100 ~ 0x3F103 (4 bytes)
    .G_DLM_CHECKSUM_ADDR      = 0x3F104,    //0x3F104 ~ 0x3F107 (4 bytes)
    .R_ILM_CHECKSUM_ADDR      = 0x3F120,    //0x3F120 ~ 0x3F123 (4 bytes)
    .R_DLM_CHECKSUM_ADDR      = 0x3F124,    //0x3F124 ~ 0x3F127 (4 bytes)
    .BLD_CRC_EN_ADDR          = 0x3F30E,
    .DMA_CRC_EN_ADDR          = 0x3F132,
    .BLD_ILM_DLM_CRC_ADDR     = 0x3F133,
    .DMA_CRC_FLAG_ADDR        = 0x3F134,
};

static const struct nvt_ts_trim_id_table trim_id_table[] = {
    {
        .id = {0x0B, 0xFF, 0xFF, 0x25, 0x65, 0x03},
        .mask = {1, 0, 0, 1, 1, 1},
        .mmap = &NT36672A_memory_map,
        .carrier_system = 0,
        .support_hw_crc = 1
    },
};

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifdef CONFIG_SPI_MT65XX
static const struct mtk_chip_config spi_ctrdata = {
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .cs_pol = 0,
};
#else
static const struct mt_chip_conf spi_ctrdata = {
    .setuptime = 25,
    .holdtime = 25,
    .high_time = 3, /* 16.6MHz */
    .low_time = 3,
    .cs_idletime = 2,
    .ulthgh_thrsh = 0,

    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,

    .com_mod = DMA_TRANSFER,

    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#endif //CONFIG_SPI_MT65XX
#endif // end of  CONFIG_TOUCHPANEL_MTK_PLATFORM

/*******************************************************
Description:
    Novatek touchscreen write data to specify address.

return:
    Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
static int32_t nvt_write_addr(struct spi_device *client, uint32_t addr, uint8_t data)
{
    int32_t ret = 0;
    uint8_t buf[4] = {0};

    //---set xdata index---
    buf[0] = 0xFF;  //set index/page/addr command
    buf[1] = (addr >> 15) & 0xFF;
    buf[2] = (addr >> 7) & 0xFF;
    ret = CTP_SPI_WRITE(client, buf, 3);
    if (ret) {
        TPD_INFO("set page 0x%06X failed, ret = %d\n", addr, ret);
        return ret;
    }

    //---write data to index---
    buf[0] = addr & (0x7F);
    buf[1] = data;
    ret = CTP_SPI_WRITE(client, buf, 2);
    if (ret) {
        TPD_INFO("write data to 0x%06X failed, ret = %d\n", addr, ret);
        return ret;
    }

    return ret;
}

/*******************************************************
Description:
    Novatek touchscreen reset MCU (boot) function.

return:
    n.a.
*******************************************************/
void nvt_bootloader_reset_noflash(struct chip_data_nt36525b *chip_info)
{
    //---reset cmds to SWRST_N8_ADDR---
    TPD_INFO("%s is called!\n", __func__);
    nvt_write_addr(chip_info->s_client, SWRST_N8_ADDR, 0x69);

    mdelay(5);  //wait tBRST2FR after Bootload RST
}

/*******************************************************
Description:
    Novatek touchscreen set index/page/addr address.

return:
    Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
static int32_t nvt_set_page(struct chip_data_nt36525b *chip_info, uint32_t addr)
{
    uint8_t buf[4] = {0};

    buf[0] = 0xFF;      //set index/page/addr command
    buf[1] = (addr >> 15) & 0xFF;
    buf[2] = (addr >> 7) & 0xFF;

    return CTP_SPI_WRITE(chip_info->s_client, buf, 3);
}

static uint8_t nvt_wdt_fw_recovery(struct chip_data_nt36525b *chip_info, uint8_t *point_data)
{
    uint32_t recovery_cnt_max = 10;
    uint8_t recovery_enable = false;
    uint8_t i = 0;

    chip_info->recovery_cnt++;

    /* Pattern Check */
    for (i = 1 ; i < 7 ; i++) {
        if ((point_data[i] != 0xFD) && (point_data[i] != 0xFE)) {
            chip_info->recovery_cnt = 0;
            break;
        }
    }

    if (chip_info->recovery_cnt > recovery_cnt_max) {
        recovery_enable = true;
        chip_info->recovery_cnt = 0;
    }

    if (chip_info->recovery_cnt) {
        TPD_INFO("recovery_cnt is %d (0x%02X)\n", chip_info->recovery_cnt, point_data[1]);
    }

    return recovery_enable;
}

/*********************************************************
Description:
        Novatek touchscreen host esd recovery function.

return:
        Executive outcomes. false-detect 0x77. true-not detect 0x77
**********************************************************/
static bool nvt_fw_recovery(uint8_t *point_data)
{
    uint8_t i = 0;
    bool detected = true;

    /* check pattern */
    for (i = 1 ; i < 7 ; i++) {
        if (point_data[i] != 0x77) {
            detected = false;
            break;
        }
    }

    return detected;
}

static void nvt_esd_check_update_timer(struct chip_data_nt36525b *chip_info)
{
    TPD_DEBUG("%s\n", __func__);

    /* update interrupt timer */
    chip_info->irq_timer = jiffies;
}

static void nvt_esd_check_enable(struct chip_data_nt36525b *chip_info, bool enable)
{
    TPD_DEBUG("%s enable=%d\n", __func__, enable);

    /* update interrupt timer */
    chip_info->irq_timer = jiffies;
    /* enable/disable esd check flag */
    chip_info->esd_check_enabled = enable;
    /* clear esd_retry counter, if protect function is enabled */
    chip_info->esd_retry = enable ? 0 : chip_info->esd_retry;
}

static int nvt_esd_handle(void *chip_data)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    unsigned int timer = jiffies_to_msecs(jiffies - chip_info->irq_timer);

    if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && chip_info->esd_check_enabled) {
        TPD_INFO("do ESD recovery, timer = %d, retry = %d\n", timer, chip_info->esd_retry);
        /* do esd recovery, bootloader reset */
        nvt_reset(chip_info);
        tp_touch_btnkey_release();
        /* update interrupt timer */
        chip_info->irq_timer = jiffies;
        /* update esd_retry counter */
        chip_info->esd_retry++;
        return -1;
    }

    return 0;
}

/*******************************************************
Description:
        Novatek touchscreen check chip version trim function.

return:
        Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(struct chip_data_nt36525b *chip_info)
{
    uint8_t buf[8] = {0};
    int32_t retry = 0;
    int32_t list = 0;
    int32_t i = 0;
    int32_t found_nvt_chip = 0;
    int32_t ret = -1;

    //---Check for 5 times---
    for (retry = 5; retry > 0; retry--) {

        nvt_bootloader_reset_noflash(chip_info);

        //---set xdata index to 0x1F600---
        nvt_set_page(chip_info, 0x1F600);       //read chip id

        buf[0] = 0x4E;  //offset
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = 0x00;
        buf[4] = 0x00;
        buf[5] = 0x00;
        buf[6] = 0x00;
        CTP_SPI_READ(chip_info->s_client, buf, 7);
        TPD_INFO("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
                 buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

        // compare read chip id on supported list
        for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
            found_nvt_chip = 0;

            // compare each byte
            for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
                if (trim_id_table[list].mask[i]) {
                    if (buf[i + 1] != trim_id_table[list].id[i])    //set parameter from chip id
                        break;
                }
            }

            if (i == NVT_ID_BYTE_MAX) {
                found_nvt_chip = 1;
            }

            if (found_nvt_chip) {
                TPD_INFO("This is NVT touch IC\n");
                chip_info->trim_id_table.mmap = trim_id_table[list].mmap;
                chip_info->trim_id_table.carrier_system = trim_id_table[list].carrier_system;
                chip_info->trim_id_table.support_hw_crc = trim_id_table[list].support_hw_crc;
                ret = 0;
                goto out;
            } else {
                chip_info->trim_id_table.mmap = NULL;
                ret = -1;
            }
        }

        msleep(10);
    }

    if (chip_info->trim_id_table.mmap == NULL) {  //set default value
        chip_info->trim_id_table.mmap = &NT36672A_memory_map;
        chip_info->trim_id_table.carrier_system = 0;
        ret = 0;
    }

out:
    TPD_INFO("list = %d, support_hw_crc is %d\n", list, chip_info->trim_id_table.support_hw_crc);
    return ret;
}

/*******************************************************
Description:
        Novatek touchscreen check FW reset state function.

return:
        Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
static int32_t nvt_check_fw_reset_state_noflash(struct chip_data_nt36525b *chip_info, RST_COMPLETE_STATE check_reset_state)
{
    uint8_t buf[8] = {0};
    int32_t ret = 0;
    int32_t retry = 0;
    int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 100;

    while (1) {
        msleep(10);

        //---read reset state---
        buf[0] = EVENT_MAP_RESET_COMPLETE;
        buf[1] = 0x00;
        CTP_SPI_READ(chip_info->s_client, buf, 6);

        if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
            ret = 0;
            break;
        }

        retry++;
        if(unlikely(retry > retry_max)) {
            TPD_INFO("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
            ret = -1;
            break;
        }
    }

    return ret;
}

static int nvt_enter_sleep(struct chip_data_nt36525b *chip_info, bool config)
{
    int ret = -1;

    if (config) {
        ret = nvt_cmd_store(chip_info, CMD_ENTER_SLEEP);
        if (ret < 0) {
            TPD_INFO("%s: enter sleep mode failed!\n", __func__);
            return -1;
        } else {
            chip_info->is_sleep_writed = true;
            TPD_INFO("%s: enter sleep mode sucess!\n", __func__);
        }
    }

    return ret;
}

/*******************************************************
Description:
        Novatek touchscreen get novatek project id information
        function.

return:
        Executive outcomes. 0---success. -1---fail.
*******************************************************/
static void nvt_read_pid_noflash(struct chip_data_nt36525b *chip_info)
{
    uint8_t buf[4] = {0};

    //---set xdata index to EVENT BUF ADDR---
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

    //---read project id---
    buf[0] = EVENT_MAP_PROJECTID;
    buf[1] = 0x00;
    buf[2] = 0x00;
    CTP_SPI_READ(chip_info->s_client, buf, 3);

    chip_info->nvt_pid = (buf[2] << 8) + buf[1];

    //---set xdata index to EVENT BUF ADDR---
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR);

    TPD_DETAIL("PID=%04X\n", chip_info->nvt_pid);

}

static int32_t nvt_get_fw_info_noflash(struct chip_data_nt36525b *chip_info)
{
    uint8_t buf[64] = {0};
    uint32_t retry_count = 0;
    int32_t ret = 0;

info_retry:
    //---set xdata index to EVENT BUF ADDR---
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

    //---read fw info---
    buf[0] = EVENT_MAP_FWINFO;
    CTP_SPI_READ(chip_info->s_client, buf, 17);
    chip_info->fw_ver = buf[1];
    chip_info->fw_sub_ver = buf[14];
    TPD_INFO("fw_ver = 0x%x, fw_sub_ver = 0x%x\n", chip_info->fw_ver, chip_info->fw_sub_ver);

    //---clear x_num, y_num if fw info is broken---
    if ((buf[1] + buf[2]) != 0xFF) {
        TPD_INFO("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
        chip_info->fw_ver = 0;

        if(retry_count < 3) {
            retry_count++;
            TPD_INFO("retry_count=%d\n", retry_count);
            goto info_retry;
        } else {
            TPD_INFO("Set default fw_ver=0, x_num=18, y_num=32, abs_x_max=1080, abs_y_max=1920, max_button_num=0!\n");
            ret = -1;
        }
    } else {
        ret = 0;
    }

    //---Get Novatek PID---
    nvt_read_pid_noflash(chip_info);

    return ret;
}

static uint32_t byte_to_word(const uint8_t *data)
{
    return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

static uint32_t CheckSum(const u8 *data, size_t len)
{
    uint32_t i = 0;
    uint32_t checksum = 0;

    for (i = 0 ; i < len + 1 ; i++)
        checksum += data[i];

    checksum += len;
    checksum = ~checksum + 1;

    return checksum;
}

static int32_t nvt_bin_header_parser(struct chip_data_nt36525b *chip_info, const u8 *fwdata, size_t fwsize)
{
    uint32_t list = 0;
    uint32_t pos = 0x00;
    uint32_t tmp_end = 0x00;
    uint8_t info_sec_num = 0;
    uint8_t ovly_sec_num = 0;
    uint8_t ovly_info = 0;

    /* Find the header size */
    tmp_end = fwdata[0] + (fwdata[1] << 8) + (fwdata[2] << 16) + (fwdata[3] << 24);
    pos = 0x30;     // info section start at 0x30 offset
    while (pos < tmp_end) {
        info_sec_num ++;
        pos += 0x10;    /* each header info is 16 bytes */
    }

    /*
     * Find the DLM OVLY section
     * [0:3] Overlay Section Number
     * [4]   Overlay Info
     */
    ovly_info = (fwdata[0x28] & 0x10) >> 4;
    ovly_sec_num = (ovly_info) ? (fwdata[0x28] & 0x0F) : 0;

    /*
     * calculate all partition number
     * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
     */
    chip_info->partition = chip_info->ilm_dlm_num + ovly_sec_num + info_sec_num;
    TPD_INFO("ovly_info = %d, ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
             ovly_info, chip_info->ilm_dlm_num, ovly_sec_num, info_sec_num, chip_info->partition);

    /* allocated memory for header info */
    chip_info->bin_map = (struct nvt_ts_bin_map *)kzalloc((chip_info->partition + 1) * sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
    if(chip_info->bin_map == NULL) {
        TPD_INFO("kzalloc for bin_map failed!\n");
        return -ENOMEM;
    }

    for (list = 0; list < chip_info->partition; list++) {
        /*
         * [1] parsing ILM & DLM header info
         * BIN_addr : SRAM_addr : size (12-bytes)
         * crc located at 0x18 & 0x1C
         */
        if (list < chip_info->ilm_dlm_num) {
            chip_info->bin_map[list].BIN_addr = byte_to_word(&fwdata[0 + list * 12]);
            chip_info->bin_map[list].SRAM_addr = byte_to_word(&fwdata[4 + list * 12]);
            chip_info->bin_map[list].size = byte_to_word(&fwdata[8 + list * 12]);
            if (chip_info->trim_id_table.support_hw_crc) {
                chip_info->bin_map[list].crc = byte_to_word(&fwdata[0x18 + list * 4]);
            }
            if (list == 0)
                snprintf(chip_info->bin_map[list].name, 12, "ILM");
            else if (list == 1)
                snprintf(chip_info->bin_map[list].name, 12, "DLM");
        }

        /*
         * [2] parsing others header info
         * SRAM_addr : size : BIN_addr : crc (16-bytes)
         */
        if ((list >= chip_info->ilm_dlm_num) && (list < (chip_info->ilm_dlm_num + info_sec_num))) {
            /* others partition located at 0x30 offset */
            pos = 0x30 + (0x10 * (list - chip_info->ilm_dlm_num));

            chip_info->bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
            chip_info->bin_map[list].size = byte_to_word(&fwdata[pos + 4]);
            chip_info->bin_map[list].BIN_addr = byte_to_word(&fwdata[pos + 8]);
            if (chip_info->trim_id_table.support_hw_crc) {
                chip_info->bin_map[list].crc = byte_to_word(&fwdata[pos + 12]);
            }
            /* detect header end to protect parser function */
            if ((chip_info->bin_map[list].BIN_addr == 0) && (chip_info->bin_map[list].size != 0)) {
                snprintf(chip_info->bin_map[list].name, 12, "Header");
            } else {
                snprintf(chip_info->bin_map[list].name, 12, "Info-%d", (list - chip_info->ilm_dlm_num));
            }
        }

        /*
         * [3] parsing overlay section header info
         * SRAM_addr : size : BIN_addr : crc (16-bytes)
         */
        if (list >= (chip_info->ilm_dlm_num + info_sec_num)) {
            /* overlay info located at DLM (list = 1) start addr */
            pos = chip_info->bin_map[1].BIN_addr + (0x10 * (list - chip_info->ilm_dlm_num - info_sec_num));

            chip_info->bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
            chip_info->bin_map[list].size = byte_to_word(&fwdata[pos + 4]);
            chip_info->bin_map[list].BIN_addr = byte_to_word(&fwdata[pos + 8]);
            if (chip_info->trim_id_table.support_hw_crc) {
                chip_info->bin_map[list].crc = byte_to_word(&fwdata[pos + 12]);
            }
            snprintf(chip_info->bin_map[list].name, 12, "Overlay-%d", (list - chip_info->ilm_dlm_num - info_sec_num));
        }

        /* BIN size error detect */
        if ((chip_info->bin_map[list].BIN_addr + chip_info->bin_map[list].size) > fwsize) {
            TPD_INFO("access range (0x%08X to 0x%08X) is larger than bin size!\n",
                     chip_info->bin_map[list].BIN_addr, chip_info->bin_map[list].BIN_addr + chip_info->bin_map[list].size);
            return -EINVAL;
        }

    }

    return 0;
}

/*******************************************************
Description:
        Novatek touchscreen init variable and allocate buffer
for download firmware function.

return:
        n.a.
*******************************************************/
static int32_t Download_Init(struct chip_data_nt36525b *chip_info)
{
    /* allocate buffer for transfer firmware */
    //NVT_LOG("SPI_TANSFER_LEN = %ld\n", SPI_TANSFER_LEN);

    if (chip_info->fwbuf == NULL) {
        chip_info->fwbuf = (uint8_t *)kzalloc((SPI_TANSFER_LEN + 1), GFP_KERNEL);
        if(chip_info->fwbuf == NULL) {
            TPD_INFO("kzalloc for fwbuf failed!\n");
            return -ENOMEM;
        }
    }

    return 0;
}

#if NVT_DUMP_SRAM
/*******************************************************
Description:
        Novatek touchscreen dump flash partition function.

return:
        n.a.
*******************************************************/
static void nvt_read_ram_test(struct chip_data_nt36525b *chip_info, uint32_t addr, uint16_t len, char *name)
{
    char file[256] = "";
    uint8_t *fbufp = NULL;
    int32_t ret = 0;
    struct file *fp = NULL;
    mm_segment_t org_fs;

    snprintf(file, 256, "/sdcard/dump_%s.bin", name);
    TPD_INFO("Dump [%s] from 0x%08X to 0x%08X\n", file, addr, addr + len);

    fbufp = (uint8_t *)kzalloc(len + 1, GFP_KERNEL);
    if(fbufp == NULL) {
        TPD_INFO("kzalloc for fbufp failed!\n");
        return;
    }

    org_fs = get_fs();
    set_fs(KERNEL_DS);
    fp = filp_open(file, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fp == NULL || IS_ERR(fp)) {
        TPD_INFO("open file failed\n");
        goto open_file_fail;
    }

    /* SPI read */
    //---set xdata index to addr---
    nvt_set_page(chip_info, addr);

    fbufp[0] = addr & 0x7F; //offset
    CTP_SPI_READ(chip_info->s_client, fbufp, len + 1);

    /* Write to file */
    ret = vfs_write(fp, (char __user *)fbufp + 1, len, &offset);
    if (ret <= 0) {
        TPD_INFO("write file failed\n");
        goto open_file_fail;
    }

open_file_fail:
    if (!IS_ERR(fp)) {
        filp_close(fp, NULL);
        set_fs(org_fs);
        fp = NULL;
    }

    if (fbufp) {
        kfree(fbufp);
        fbufp = NULL;
    }

    return;
}
#endif

/*******************************************************
Description:
        Novatek touchscreen Write_Partition function to write
firmware into each partition.

return:
        n.a.
*******************************************************/
static int32_t Write_Partition(struct chip_data_nt36525b *chip_info, const u8 *fwdata, size_t fwsize)
{
    uint32_t list = 0;
    uint32_t BIN_addr, SRAM_addr, size;
    int32_t ret = 0;
    u32 *len_array = NULL;
    u8 array_len;
    u8 *buf;
    u8 count = 0;

    array_len = chip_info->partition * 2;
    len_array = kzalloc(sizeof(u32) * array_len, GFP_KERNEL);

    if (len_array == NULL) {
        return -1;
    }
    buf = chip_info->fw_buf_dma;

    for (list = 0; list < chip_info->partition; list++) {
        SRAM_addr = chip_info->bin_map[list].SRAM_addr;
        size = chip_info->bin_map[list].size;
        BIN_addr = chip_info->bin_map[list].BIN_addr;
        //TPD_INFO("SRAM_addr %x, size %d, BIN_addr %x\n", SRAM_addr, size, BIN_addr);
        if (size == 0) {
            array_len = array_len - 2;
            continue;
        }
        // Check data size
        if ((BIN_addr + size) > fwsize) {
            TPD_INFO("access range (0x%08X to 0x%08X) is larger than bin size!\n",
                     BIN_addr, BIN_addr + size);
            ret = -1;
            goto out;
        }
        buf[0] = 0xFF;      //set index/page/addr command
        buf[1] = (SRAM_addr >> 15) & 0xFF;
        buf[2] = (SRAM_addr >> 7) & 0xFF;
        len_array[count * 2] = 3;

        buf = buf + 3;
        buf[0] = (SRAM_addr & 0x7F) | 0x80; //offset
        size = size + 1;
        memcpy(buf + 1, &fwdata[BIN_addr], size);
        len_array[count * 2 + 1] = size + 1;

        buf = buf + size + 1;
        count++;
    }

    spi_write_firmware(chip_info->s_client,
                       chip_info->fw_buf_dma,
                       len_array,
                       array_len);

out:
    kfree(len_array);
    return ret;
}

/*
static int32_t Write_Partition(struct chip_data_nt36525b *chip_info, const u8 *fwdata, size_t fwsize)
{
    uint32_t list = 0;
    char *name;
    uint32_t BIN_addr, SRAM_addr, size;
    uint32_t i = 0;
    uint16_t len = 0;
    int32_t count = 0;
    int32_t ret = 0;

    memset(chip_info->fwbuf, 0, (SPI_TANSFER_LEN + 1));

    for (list = 0; list < chip_info->partition; list++) {
        // initialize variable
        SRAM_addr = chip_info->bin_map[list].SRAM_addr;
        size = chip_info->bin_map[list].size;
        BIN_addr = chip_info->bin_map[list].BIN_addr;
        name = chip_info->bin_map[list].name;

        //              TPD_INFO("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X)\n",
        //                              list, name, SRAM_addr, size, BIN_addr);

        // Check data size
        if ((BIN_addr + size) > fwsize) {
            TPD_INFO("access range (0x%08X to 0x%08X) is larger than bin size!\n",
                     BIN_addr, BIN_addr + size);
            ret = -1;
            goto out;
        }

        // ignore reserved partition (Reserved Partition size is zero)
        if (!size)
            continue;
        else
            size = size + 1;

        // write data to SRAM
        if (size % SPI_TANSFER_LEN)
            count = (size / SPI_TANSFER_LEN) + 1;
        else
            count = (size / SPI_TANSFER_LEN);

        for (i = 0 ; i < count ; i++) {
            len = (size < SPI_TANSFER_LEN) ? size : SPI_TANSFER_LEN;

            //---set xdata index to start address of SRAM---
            nvt_set_page(chip_info, SRAM_addr);

            //---write data into SRAM---
            chip_info->fwbuf[0] = SRAM_addr & 0x7F; //offset
            memcpy(chip_info->fwbuf + 1, &fwdata[BIN_addr], len);   //payload
            CTP_SPI_WRITE(chip_info->s_client, chip_info->fwbuf, len + 1);

#if NVT_DUMP_SRAM
            // dump for debug download firmware
            nvt_read_ram_test(chip_info, SRAM_addr, len, name);
#endif
            SRAM_addr += SPI_TANSFER_LEN;
            BIN_addr += SPI_TANSFER_LEN;
            size -= SPI_TANSFER_LEN;
        }

#if NVT_DUMP_SRAM
        offset = 0;
#endif
    }

out:
    return ret;
}
*/

static void nvt_bld_crc_enable(struct chip_data_nt36525b *chip_info)
{
    uint8_t buf[4] = {0};

    //---set xdata index to BLD_CRC_EN_ADDR---
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->BLD_CRC_EN_ADDR);

    //---read data from index---
    buf[0] = chip_info->trim_id_table.mmap->BLD_CRC_EN_ADDR & (0x7F);
    buf[1] = 0xFF;
    CTP_SPI_READ(chip_info->s_client, buf, 2);

    //---write data to index---
    buf[0] = chip_info->trim_id_table.mmap->BLD_CRC_EN_ADDR & (0x7F);
    buf[1] = buf[1] | (0x01 << 7);
    CTP_SPI_WRITE(chip_info->s_client, buf, 2);
}

/*******************************************************
Description:
    Novatek touchscreen clear status & enable fw crc function.

return:
    N/A.
*******************************************************/
static void nvt_fw_crc_enable(struct chip_data_nt36525b *chip_info)
{
    uint8_t buf[4] = {0};

    //---set xdata index to EVENT BUF ADDR---
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR);

    //---clear fw reset status---
    buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
    buf[1] = 0x00;
    CTP_SPI_WRITE(chip_info->s_client, buf, 2);

    //---enable fw crc---
    buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
    buf[1] = 0xAE;  //enable fw crc command
    CTP_SPI_WRITE(chip_info->s_client, buf, 2);
}

/*******************************************************
Description:
        Novatek touchscreen set boot ready function.

return:
        Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
static void nvt_boot_ready(struct chip_data_nt36525b *chip_info, uint8_t ready)
{
    //---write BOOT_RDY status cmds---
    nvt_write_addr(chip_info->s_client, chip_info->trim_id_table.mmap->BOOT_RDY_ADDR, 1);

    mdelay(5);

    //---set xdata index to EVENT BUF ADDR---
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
        Novatek touchscreen eng reset cmd
    function.

return:
        n.a.
*******************************************************/
static void nvt_eng_reset(struct chip_data_nt36525b *chip_info)
{
    //---eng reset cmds to ENG_RST_ADDR---
    TPD_INFO("%s is called!\n", __func__);
    nvt_write_addr(chip_info->s_client, chip_info->ENG_RST_ADDR, 0x5A);

    mdelay(1);      //wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen set bootload crc reg bank function.
This function will set hw crc reg before enable crc function.

return:
	n.a.
*******************************************************/
static void nvt_set_bld_crc_bank(struct chip_data_nt36525b *chip_info,
                                 uint32_t DES_ADDR, uint32_t SRAM_ADDR,
                                 uint32_t LENGTH_ADDR, uint32_t size,
                                 uint32_t G_CHECKSUM_ADDR, uint32_t crc)
{
    /* write destination address */
    nvt_set_page(chip_info, DES_ADDR);
    chip_info->fwbuf[0] = DES_ADDR & 0x7F;
    chip_info->fwbuf[1] = (SRAM_ADDR) & 0xFF;
    chip_info->fwbuf[2] = (SRAM_ADDR >> 8) & 0xFF;
    chip_info->fwbuf[3] = (SRAM_ADDR >> 16) & 0xFF;
    CTP_SPI_WRITE(chip_info->s_client, chip_info->fwbuf, 4);

    /* write length */
    chip_info->fwbuf[0] = LENGTH_ADDR & 0x7F;
    chip_info->fwbuf[1] = (size) & 0xFF;
    chip_info->fwbuf[2] = (size >> 8) & 0xFF;
    CTP_SPI_WRITE(chip_info->s_client, chip_info->fwbuf, 3);

    /* write golden dlm checksum */
    chip_info->fwbuf[0] = G_CHECKSUM_ADDR & 0x7F;
    chip_info->fwbuf[1] = (crc) & 0xFF;
    chip_info->fwbuf[2] = (crc >> 8) & 0xFF;
    chip_info->fwbuf[3] = (crc >> 16) & 0xFF;
    chip_info->fwbuf[4] = (crc >> 24) & 0xFF;
    CTP_SPI_WRITE(chip_info->s_client, chip_info->fwbuf, 5);

    return;
}

/*******************************************************
Description:
	Novatek touchscreen check DMA hw crc function.
This function will check hw crc result is pass or not.

return:
	n.a.
*******************************************************/
static void nvt_set_bld_hw_crc(struct chip_data_nt36525b *chip_info)
{
    /* [0] ILM */
    /* write register bank */
    nvt_set_bld_crc_bank(chip_info,
                         chip_info->trim_id_table.mmap->ILM_DES_ADDR, chip_info->bin_map[0].SRAM_addr,
                         chip_info->trim_id_table.mmap->ILM_LENGTH_ADDR, chip_info->bin_map[0].size,
                         chip_info->trim_id_table.mmap->G_ILM_CHECKSUM_ADDR, chip_info->bin_map[0].crc);

    /* [1] DLM */
    /* write register bank */
    nvt_set_bld_crc_bank(chip_info,
                         chip_info->trim_id_table.mmap->DLM_DES_ADDR, chip_info->bin_map[1].SRAM_addr,
                         chip_info->trim_id_table.mmap->DLM_LENGTH_ADDR, chip_info->bin_map[1].size,
                         chip_info->trim_id_table.mmap->G_DLM_CHECKSUM_ADDR, chip_info->bin_map[1].crc);
}

/*******************************************************
Description:
        Novatek touchscreen Download_Firmware with HW CRC
function. It's complete download firmware flow.

return:
        n.a.
*******************************************************/
static int32_t Download_Firmware_HW_CRC(struct chip_data_nt36525b *chip_info, const struct firmware *fw)
{
    uint8_t retry = 0;
    int32_t ret = 0;

    TPD_DETAIL("Enter Download_Firmware_HW_CRC\n");
    do_gettimeofday(&start);

    while (1) {
        nvt_esd_check_update_timer(chip_info);

        /* bootloader reset to reset MCU */
        nvt_bootloader_reset_noflash(chip_info);

        /* Start Write Firmware Process */
        ret = Write_Partition(chip_info, fw->data, fw->size);
        if (ret) {
            TPD_INFO("Write_Firmware failed. (%d)\n", ret);
            goto fail;
        }

        /* set ilm & dlm reg bank */
        nvt_set_bld_hw_crc(chip_info);

        /* enable hw bld crc function */
        nvt_bld_crc_enable(chip_info);

        /* clear fw reset status & enable fw crc check */
        nvt_fw_crc_enable(chip_info);

        /* Set Boot Ready Bit */
        nvt_boot_ready(chip_info, true);

        ret = nvt_check_fw_reset_state_noflash(chip_info, RESET_STATE_INIT);
        if (ret) {
            TPD_INFO("nvt_check_fw_reset_state_noflash failed. (%d)\n", ret);
            goto fail;
        } else {
            break;
        }

fail:
        retry++;
        if(unlikely(retry > 2) || chip_info->using_headfile) {
            TPD_INFO("error, retry=%d\n", retry);
            break;
        }
    }

    do_gettimeofday(&end);

    return ret;
}

int32_t nvt_nf_detect_chip(struct chip_data_nt36525b *chip_info)
{
    int32_t ret = 0;
    int i;
    uint8_t buf[8] = {0};

    ret = nvt_set_page(chip_info, 0x1F600);

    buf[0] = 0x4E;  //offset
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x00;
    ret = CTP_SPI_READ(chip_info->s_client, buf, 2);
    for(i = 1; i < 7; i++) {
        TPD_INFO("buf[%d] is 0x%02X\n", i, buf[i]);
        if(buf[i] != 0) {
            return 0;
        }
    }
    return -ENODEV;
}


/********* Start of implementation of oppo_touchpanel_operations callbacks********************/
//extern int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data);

static int nvt_ftm_process(void *chip_data)
{
    int ret = -1;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    const struct firmware *fw = NULL;

    TPD_INFO("%s is called!\n", __func__);

    ret = nvt_get_chip_info(chip_info);
    if (!ret) {
        ret = nvt_fw_update_sub(chip_info, fw, 0);
        if(ret > 0) {
            TPD_INFO("%s fw update failed!\n", __func__);
        } else {
            ret = nvt_enter_sleep(chip_info, true);
        }
    }

    return ret;
}

static int nvt_power_control(void *chip_data, bool enable)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    nvt_eng_reset(chip_info);       //make sure before nvt_bootloader_reset_noflash
    gpio_direction_output(chip_info->hw_res->reset_gpio, 1);

    return 0;
}

static int nvt_get_chip_info(void *chip_data)
{
    int ret = -1;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    //---check chip version trim---
    ret = nvt_ts_check_chip_ver_trim(chip_info);
    if (ret) {
        TPD_INFO("chip is not identified\n");
        ret = -EINVAL;
    }

    return ret;
}

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
extern unsigned int upmu_get_rgs_chrdet(void);
static int nvt_get_usb_state(void)
{
    return upmu_get_rgs_chrdet();
}
#else
static int nvt_get_usb_state(void)
{
    return 0;
}
#endif

static int nvt_get_vendor(void *chip_data, struct panel_info *panel_data)
{
    int len = 0;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    len = strlen(panel_data->fw_name);
    if ((len > 3) && (panel_data->fw_name[len - 3] == 'i') && \
        (panel_data->fw_name[len - 2] == 'm') && (panel_data->fw_name[len - 1] == 'g')) {
        panel_data->fw_name[len - 3] = 'b';
        panel_data->fw_name[len - 2] = 'i';
        panel_data->fw_name[len - 1] = 'n';
    }
    chip_info->tp_type = panel_data->tp_type;
    TPD_INFO("chip_info->tp_type = %d, panel_data->fw_name = %s\n", chip_info->tp_type, panel_data->fw_name);

    return 0;
}

static int nvt_reset_gpio_control(void *chip_data, bool enable)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    if (gpio_is_valid(chip_info->hw_res->reset_gpio)) {
        TPD_INFO("%s: set reset state %d\n", __func__, enable);
        gpio_set_value(chip_info->hw_res->reset_gpio, enable);
    }

    return 0;
}

int nvt_lcd_reset_gpio_init(void *chip_data)
{
    int rc;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    struct touchpanel_data *ts = spi_get_drvdata(chip_info->s_client);
    struct device_node *np;
    np = ts->dev->of_node;
    chip_info->lcd_reset_gpio = of_get_named_gpio(np, "lcd-reset-gpio", 0);
    if (gpio_is_valid(chip_info->lcd_reset_gpio)) {
        rc = gpio_request(chip_info->lcd_reset_gpio, "lcd-reset-gpio");
        if (rc)
            TPD_INFO("unable to request gpio [%d]\n", chip_info->lcd_reset_gpio);
    } else {
        TPD_INFO("chip_info->lcd-reset-gpio not specified\n");
    }
    return 0;
}

int nvt_lcd_reset_gpio_control(void *chip_data, bool enable)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    if (gpio_is_valid(chip_info->lcd_reset_gpio)) {
        TPD_INFO("%s: set reset state %d\n", __func__, enable);
        gpio_set_value(chip_info->lcd_reset_gpio, enable);
    }

    return 0;
}

static u8 nvt_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
    if ((gesture_enable == 1) && is_suspended) {
        return IRQ_GESTURE;
    } else if (is_suspended) {
        return IRQ_IGNORE;
    }

    return IRQ_TOUCH;
}


static int32_t nvt_ts_point_data_checksum(uint8_t *buf, uint8_t length)
{
    uint8_t checksum = 0;
    int32_t i = 0;

    // Generate checksum
    for (i = 0; i < length - 1; i++) {
        checksum += buf[i + 1];
    }
    checksum = (~checksum + 1);

    // Compare ckecksum and dump fail data
    if (checksum != buf[length]) {
        TPD_INFO("i2c/spi packet checksum not match. (point_data[%d]=0x%02X, checksum=0x%02X)\n",
                 length, buf[length], checksum);

        for (i = 0; i < 10; i++) {
            TPD_INFO("%02X %02X %02X %02X %02X %02X\n",
                     buf[1 + i * 6], buf[2 + i * 6], buf[3 + i * 6], buf[4 + i * 6], buf[5 + i * 6], buf[6 + i * 6]);
        }

        for (i = 0; i < (length - 60); i++) {
            TPD_INFO("%02X ", buf[1 + 60 + i]);
        }

        return -1;
    }

    return 0;
}


static int nvt_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    int obj_attention = 0;
    int i = 0;
    int32_t ret = -1;
    uint32_t position = 0;
    uint32_t input_x = 0;
    uint32_t input_y = 0;
    uint32_t input_w = 0;
    uint32_t input_p = 0;
    uint8_t pointid = 0;
    uint8_t point_data[POINT_DATA_LEN + 2] = {0};
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    ret = CTP_SPI_READ(chip_info->s_client, point_data, POINT_DATA_LEN + 1);
    if (ret < 0) {
        TPD_INFO("CTP_SPI_READ failed.(%d)\n", ret);
        return -1;
    }

    //some kind of protect mechanism, after WDT firware redownload and try to save tp
    ret = nvt_wdt_fw_recovery(chip_info, point_data);
    if (ret) {
        TPD_INFO("Recover for fw reset %02X\n", point_data[1]);
        nvt_reset(chip_info);
        return -1;
    }

    if (nvt_fw_recovery(point_data)) {  //receive 0x77
        nvt_esd_check_enable(chip_info, true);
        return -1;
    }

    ret = nvt_ts_point_data_checksum(point_data, POINT_DATA_CHECKSUM_LEN);
    if (ret) {
        return -1;
    }


    for(i = 0; i < max_num; i++) {
        position = 1 + 6 * i;
        pointid = (uint8_t)(point_data[position + 0] >> 3) - 1;
        if (pointid >= max_num) {
            continue;
        }

        if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {
            chip_info->irq_timer = jiffies;    //reset esd check trigger base time

            input_x = (uint32_t)(point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
            input_y = (uint32_t)(point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);

            input_w = (uint32_t)(point_data[position + 4]);
            if (input_w == 0) {
                input_w = 1;
            }
            if (i < 2) {
                input_p = (uint32_t)(point_data[position + 5]) + (uint32_t)(point_data[i + 63] << 8);
                if (input_p > 1000) {
                    input_p = 1000;
                }
            } else {
                input_p = (uint32_t)(point_data[position + 5]);
            }
            if (input_p == 0) {
                input_p = 1;
            }

            obj_attention = obj_attention | (1 << pointid);
            points[pointid].x = input_x;
            points[pointid].y = input_y;
            points[pointid].z = input_p;
            points[pointid].width_major = input_w;
            points[pointid].touch_major = input_w;
            points[pointid].status = 1;
        }
    }

    return obj_attention;
}

static int8_t nvt_extend_cmd_store(struct chip_data_nt36525b *chip_info, uint8_t u8Cmd, uint8_t u8SubCmd)
{
    int i, retry = 5;
    uint8_t buf[4] = {0};

    //---set xdata index to EVENT BUF ADDR---(set page)
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR);

    for (i = 0; i < retry; i++) {
        //---set cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = u8Cmd;
        buf[2] = u8SubCmd;
        CTP_SPI_WRITE(chip_info->s_client, buf, 3);

        msleep(20);

        //---read cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        CTP_SPI_READ(chip_info->s_client, buf, 3);
        if (buf[1] == 0x00)
            break;
    }

    if (unlikely(i == retry)) {
        TPD_INFO("send Cmd 0x%02X 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, u8SubCmd, buf[1]);
        return -1;
    } else {
        TPD_INFO("send Cmd 0x%02X 0x%02X success, tried %d times\n", u8Cmd, u8SubCmd, i);
    }

    return 0;
}

static int8_t nvt_cmd_store(struct chip_data_nt36525b *chip_info, uint8_t u8Cmd)
{
    int i, retry = 5;
    uint8_t buf[3] = {0};

    //---set xdata index to EVENT BUF ADDR---(set page)
    nvt_set_page(chip_info, chip_info->trim_id_table.mmap->EVENT_BUF_ADDR);

    for (i = 0; i < retry; i++) {
        //---set cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = u8Cmd;
        CTP_SPI_WRITE(chip_info->s_client, buf, 2);

        msleep(20);

        //---read cmd status---
        buf[0] = EVENT_MAP_HOST_CMD;
        buf[1] = 0xFF;
        CTP_SPI_READ(chip_info->s_client, buf, 2);
        if (buf[1] == 0x00)
            break;
    }

    if (unlikely(i == retry)) {
        TPD_INFO("send Cmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
        return -1;
    } else {
        TPD_INFO("send Cmd 0x%02X success, tried %d times\n", u8Cmd, i);
    }

    return 0;
}



static void nvt_ts_wakeup_gesture_coordinate(uint8_t *data, uint8_t max_num)
{
    uint32_t position = 0;
    uint32_t input_x = 0;
    uint32_t input_y = 0;
    int32_t i = 0;
    uint8_t input_id = 0;

    for (i = 0; i < max_num; i++) {
        position = 1 + 6 * i;
        input_id = (uint8_t)(data[position + 0] >> 3);
        if ((input_id == 0) || (input_id > max_num))
            continue;

        if (((data[position] & 0x07) == 0x01) || ((data[position] & 0x07) == 0x02)) {
            input_x = (uint32_t)(data[position + 1] << 4) + (uint32_t) (data[position + 3] >> 4);
            input_y = (uint32_t)(data[position + 2] << 4) + (uint32_t) (data[position + 3] & 0x0F);
        }
        TPD_INFO("(%d: %d, %d)\n", i, input_x, input_y);
    }
}


static int nvt_get_gesture_info(void *chip_data, struct gesture_info *gesture)
{
    uint8_t gesture_id = 0;
    uint8_t func_type = 0;
    int ret = -1;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    uint8_t point_data[POINT_DATA_LEN + 2] = {0};
    uint8_t max_num = 10;	//TODO: define max_num by oppo common driver

    memset(point_data, 0, sizeof(point_data));
    ret = CTP_SPI_READ(chip_info->s_client, point_data, POINT_DATA_LEN + 1);
    if (ret < 0) {
        TPD_INFO("%s: read gesture data failed\n", __func__);
        return -1;
    }

    //some kind of protect mechanism, after WDT firware redownload and try to save tp
    ret = nvt_wdt_fw_recovery(chip_info, point_data);
    if (ret) {
        TPD_INFO("receive all %02X, no gesture interrupts. recover for fw reset\n",
                 point_data[1]);
        nvt_reset(chip_info);
        /* auto go back to wakeup gesture mode */
        ret = nvt_cmd_store(chip_info, 0x13);
        return 0;
    }

    gesture_id = (uint8_t)(point_data[1] >> 3);
    func_type = (uint8_t)point_data[2];
    if ((gesture_id == 30) && (func_type == 1)) {
        gesture_id = (uint8_t)point_data[3];
    } else if (gesture_id > 30) {
        TPD_INFO("invalid gesture id= %d, no gesture event\n", gesture_id);
        return 0;
    }


    if ((gesture_id > 0) && (gesture_id <= max_num)) {
        nvt_ts_wakeup_gesture_coordinate(point_data, max_num);
        return 0;
    }

    switch (gesture_id) {   //judge gesture type
    case RIGHT_SLIDE_DETECT :
        gesture->gesture_type  = Left2RightSwip;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case LEFT_SLIDE_DETECT :
        gesture->gesture_type  = Right2LeftSwip;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case DOWN_SLIDE_DETECT  :
        gesture->gesture_type  = Up2DownSwip;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case UP_SLIDE_DETECT :
        gesture->gesture_type  = Down2UpSwip;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case DTAP_DETECT:
        gesture->gesture_type  = DouTap;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end     = gesture->Point_start;
        break;

    case UP_VEE_DETECT :
        gesture->gesture_type  = UpVee;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case DOWN_VEE_DETECT :
        gesture->gesture_type  = DownVee;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case LEFT_VEE_DETECT:
        gesture->gesture_type = LeftVee;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case RIGHT_VEE_DETECT :
        gesture->gesture_type  = RightVee;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        break;

    case CIRCLE_DETECT  :
        gesture->gesture_type = Circle;
        gesture->clockwise = (point_data[43] == 0x20) ? 1 : 0;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;    //ymin
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;  //xmin
        gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_3rd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;  //ymax
        gesture->Point_3rd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
        gesture->Point_4th.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;  //xmax
        gesture->Point_4th.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[24] & 0xFF) | (point_data[25] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[26] & 0xFF) | (point_data[27] & 0x0F) << 8;
        break;

    case DOUSWIP_DETECT  :
        gesture->gesture_type  = DouSwip;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        gesture->Point_2nd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
        gesture->Point_2nd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
        break;

    case M_DETECT  :
        gesture->gesture_type  = Mgestrue;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_3rd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
        gesture->Point_3rd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
        break;

    case W_DETECT :
        gesture->gesture_type  = Wgestrue;
        gesture->Point_start.x = (point_data[4] & 0xFF) | (point_data[5] & 0x0F) << 8;
        gesture->Point_start.y = (point_data[6] & 0xFF) | (point_data[7] & 0x0F) << 8;
        gesture->Point_1st.x   = (point_data[8] & 0xFF) | (point_data[9] & 0x0F) << 8;
        gesture->Point_1st.y   = (point_data[10] & 0xFF) | (point_data[11] & 0x0F) << 8;
        gesture->Point_2nd.x   = (point_data[12] & 0xFF) | (point_data[13] & 0x0F) << 8;
        gesture->Point_2nd.y   = (point_data[14] & 0xFF) | (point_data[15] & 0x0F) << 8;
        gesture->Point_3rd.x   = (point_data[16] & 0xFF) | (point_data[17] & 0x0F) << 8;
        gesture->Point_3rd.y   = (point_data[18] & 0xFF) | (point_data[19] & 0x0F) << 8;
        gesture->Point_end.x   = (point_data[20] & 0xFF) | (point_data[21] & 0x0F) << 8;
        gesture->Point_end.y   = (point_data[22] & 0xFF) | (point_data[23] & 0x0F) << 8;
        break;

    default:
        gesture->gesture_type = UnkownGesture;
        break;
    }

    TPD_INFO("%s, gesture_id: 0x%x, func_type: 0x%x, gesture_type: %d, clockwise: %d, points: (%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)(%d, %d)\n", \
             __func__, gesture_id, func_type, gesture->gesture_type, gesture->clockwise, \
             gesture->Point_start.x, gesture->Point_start.y, \
             gesture->Point_end.x, gesture->Point_end.y, \
             gesture->Point_1st.x, gesture->Point_1st.y, \
             gesture->Point_2nd.x, gesture->Point_2nd.y, \
             gesture->Point_3rd.x, gesture->Point_3rd.y, \
             gesture->Point_4th.x, gesture->Point_4th.y);

    return 0;
}

static int nvt_reset(void *chip_data)
{
    int ret = -1;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    struct touchpanel_data *ts = spi_get_drvdata(chip_info->s_client);
    //const struct firmware *fw = NULL;

    TPD_INFO("%s.\n", __func__);
    mutex_lock(&chip_info->mutex_testing);

    if(!ts->fw_update_app_support || chip_info->probe_done) {

        ret = nvt_fw_update(chip_info, NULL, 0);
        if(ret > 0) {
            TPD_INFO("g_fw_buf update failed!\n");
        }
    }
    /*
    if(chip_info->g_fw != NULL) {
        release_firmware(chip_info->g_fw);
    }
    */

    chip_info->is_sleep_writed = false;
    mutex_unlock(&chip_info->mutex_testing);
    return 0;
}


static int nvt_enable_black_gesture(struct chip_data_nt36525b *chip_info, bool enable)
{
    int ret = -1;

    TPD_INFO("%s, enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (chip_info->is_sleep_writed) {
        //disable_irq_nosync(chip_info->irq_num);
        chip_info->need_judge_irq_throw = true;
        nvt_lcd_reset_gpio_control(chip_info, true);
        mdelay(5);
        if (get_lcd_status() != 1) {
            TPD_INFO("LCD not init, control the reset low!");
            nvt_lcd_reset_gpio_control(chip_info, false);
            mdelay(5);
            nvt_lcd_reset_gpio_control(chip_info, true);
            mdelay(10);
            nvt_reset(chip_info);
        }
        chip_info->need_judge_irq_throw = false;
    }


    if (enable) {
        if (get_lcd_status() > 0) {
            TPD_INFO("Will power on soon!");
            return ret;
        }

        ret = nvt_cmd_store(chip_info, CMD_OPEN_BLACK_GESTURE);
        TPD_INFO("%s: enable gesture %s !\n", __func__, (ret < 0) ? "failed" : "success");
    } else {
        ret = 0;
    }

    return ret;
}

static uint8_t edge_limit_level = 10; /* 0 ~ 255 */
static int nvt_enable_edge_limit(struct chip_data_nt36525b *chip_info, int state)
{
    int8_t ret = -1;
    struct touchpanel_data *ts = spi_get_drvdata(chip_info->s_client);
    TPD_INFO("%s:state = %d, limit_corner = %d, level = %d, chip_info->is_sleep_writed = %d\n", __func__, state, ts->limit_corner, edge_limit_level, chip_info->is_sleep_writed);

    if (state == 1 || VERTICAL_SCREEN == chip_info->touch_direction) {
        ret = nvt_extend_cmd_store(chip_info,
                                   EVENTBUFFER_EDGE_LIMIT_VERTICAL, edge_limit_level);
    } else {
        if (LANDSCAPE_SCREEN_90 == chip_info->touch_direction) {
            ret = nvt_extend_cmd_store(chip_info,
                                       EVENTBUFFER_EDGE_LIMIT_RIGHT_UP, edge_limit_level);
        } else if (LANDSCAPE_SCREEN_270 == chip_info->touch_direction) {
            ret = nvt_extend_cmd_store(chip_info,
                                       EVENTBUFFER_EDGE_LIMIT_LEFT_UP, edge_limit_level);
        }
    }

    return ret;
}

static int nvt_enable_charge_mode(struct chip_data_nt36525b *chip_info, bool enable)
{
    int8_t ret = -1;

    TPD_INFO("%s:enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (enable) {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_PWR_PLUG_IN);
    } else {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_PWR_PLUG_OUT);
    }

    return ret;
}

static int nvt_enable_game_mode(struct chip_data_nt36525b *chip_info, bool enable)
{
    int8_t ret = -1;

    TPD_INFO("%s:enable = %d, chip_info->is_sleep_writed = %d\n", __func__, enable, chip_info->is_sleep_writed);

    if (enable) {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_GAME_ON);
    } else {
        ret = nvt_cmd_store(chip_info, EVENTBUFFER_GAME_OFF);
    }

    return ret;
}


static int nvt_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = -1;
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    nvt_esd_check_update_timer(chip_info);

    switch(mode) {
    case MODE_NORMAL:
        ret = 0;
        break;

    case MODE_SLEEP:
        ret = nvt_enter_sleep(chip_info, true);
        if (ret < 0) {
            TPD_INFO("%s: nvt enter sleep failed\n", __func__);
        }
        nvt_esd_check_enable(chip_info, false);
        break;

    case MODE_GESTURE:
        ret = nvt_enable_black_gesture(chip_info, flag);
        if (ret < 0) {
            TPD_INFO("%s: nvt enable gesture failed.\n", __func__);
            return ret;
        }

        if (flag) {
            nvt_esd_check_enable(chip_info, false);
        }
        break;

    case MODE_EDGE:
        ret = nvt_enable_edge_limit(chip_info, flag);
        if (ret < 0) {
            TPD_INFO("%s: nvt enable edg limit failed.\n", __func__);
            return ret;
        }
        break;

    case MODE_CHARGE:
        ret = nvt_enable_charge_mode(chip_info, flag);
        if (ret < 0) {
            TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
        }
        break;

    case MODE_GAME:
        ret = nvt_enable_game_mode(chip_info, flag);
        if (ret < 0) {
            TPD_INFO("%s: enable charge mode : %d failed\n", __func__, flag);
        }
        break;


    default:
        TPD_INFO("%s: Wrong mode.\n", __func__);
    }

    return ret;
}

static fw_check_state nvt_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    uint8_t ver_len = 0;
    int ret = 0;
    char dev_version[MAX_DEVICE_VERSION_LENGTH] = {0};
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    nvt_esd_check_update_timer(chip_info);

    ret |= nvt_get_fw_info_noflash(chip_info);
    if (ret < 0) {
        TPD_INFO("%s: get fw info failed\n", __func__);
        return FW_ABNORMAL;
    } else {
        panel_data->TP_FW = chip_info->fw_ver;
        snprintf(dev_version, MAX_DEVICE_VERSION_LENGTH,
                 "%02X", panel_data->TP_FW);
        if (panel_data->manufacture_info.version) {

            if (panel_data->vid_len == 0) {
                ver_len = strlen(panel_data->manufacture_info.version);
                if (ver_len <= 8) {
                    strlcat(panel_data->manufacture_info.version, dev_version, MAX_DEVICE_VERSION_LENGTH);
                } else {
                    strlcpy(&panel_data->manufacture_info.version[8], dev_version, MAX_DEVICE_VERSION_LENGTH - 8);
                }

            } else {
                ver_len = panel_data->vid_len;
                if (ver_len > MAX_DEVICE_VERSION_LENGTH - 4) {
                    ver_len = MAX_DEVICE_VERSION_LENGTH - 4;
                }

                strlcpy(&panel_data->manufacture_info.version[ver_len],
                        dev_version, MAX_DEVICE_VERSION_LENGTH - ver_len);

            }

        }
    }

    return FW_NORMAL;
}

/*******************************************************
Description:
    Novatek touchscreen nvt_check_bin_checksum function.
Compare checksum from bin and calculated results to check
bin file is correct or not.
return:
    n.a.
*******************************************************/
static int32_t nvt_check_bin_checksum(const u8 *fwdata, size_t fwsize)
{
    uint32_t checksum_calculated = 0;
    uint32_t checksum_bin = 0;
    int32_t ret = 0;
    /* calculate the checksum reslut */
    checksum_calculated = CheckSum(fwdata, fwsize - FW_BIN_CHECKSUM_LEN - 1);
    /* get the checksum from file directly */
    checksum_bin = byte_to_word(fwdata + (fwsize - FW_BIN_CHECKSUM_LEN));
    if (checksum_calculated != checksum_bin) {
        TPD_INFO("%s checksum_calculated = 0x%08X\n", __func__, checksum_calculated);
        TPD_INFO("%s checksum_bin = 0x%08X\n", __func__, checksum_bin);
        ret = -EINVAL;
    }
    return ret;
}

static fw_update_state nvt_fw_update_sub(void *chip_data, const struct firmware *fw, bool force)
{
    int ret = 0;
    uint8_t point_data[POINT_DATA_LEN + 2] = {0};
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;
    struct touchpanel_data *ts = spi_get_drvdata(chip_info->s_client);
    struct firmware *request_fw_headfile = NULL;

    //check bin file size(116kb)
    if(fw != NULL && fw->size != FW_BIN_SIZE) {
        TPD_INFO("bin file size not match (%zu), change to use headfile fw.\n", fw->size);
        goto out_fail;
        //release_firmware(fw);
        //fw = NULL;
    }

    //request firmware failed, get from headfile
    if(fw == NULL) {
        request_fw_headfile = kzalloc(sizeof(struct firmware), GFP_KERNEL);
        if(request_fw_headfile == NULL) {
            TPD_INFO("%s kzalloc failed!\n", __func__);
            goto out_fail;
        }

        if (chip_info->g_fw_sta) {
            TPD_INFO("request firmware failed, get from g_fw_buf\n");
            request_fw_headfile->size = chip_info->g_fw_len;
            request_fw_headfile->data = chip_info->g_fw_buf;
            fw = request_fw_headfile;

        } else {
            TPD_INFO("request firmware failed, get from headfile\n");

            if(chip_info->p_firmware_headfile->firmware_data) {
                request_fw_headfile->size = chip_info->p_firmware_headfile->firmware_size;
                request_fw_headfile->data = chip_info->p_firmware_headfile->firmware_data;
                fw = request_fw_headfile;

            } else {
                TPD_INFO("firmware_data is NULL! exit firmware update!\n");
                goto out_fail;
            }
        }
    }

    //check bin file size(116kb)
    if(fw->size != FW_BIN_SIZE) {
        TPD_INFO("fw file size not match. (%zu)\n", fw->size);
        goto out_fail;
    }

    // check if FW version add FW version bar equals 0xFF
    if (*(fw->data + FW_BIN_VER_OFFSET) + * (fw->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
        TPD_INFO("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
        TPD_INFO("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(fw->data + FW_BIN_VER_OFFSET), *(fw->data + FW_BIN_VER_BAR_OFFSET));
        goto out_fail;
    }

    //fw checksum compare
    ret = nvt_check_bin_checksum(fw->data, fw->size);
    if (ret) {
        if (fw != request_fw_headfile) {
            TPD_INFO("Image fw check checksum failed, reload fw from array\n");

            goto out_fail;

        } else {
            TPD_INFO("array fw check checksum failed, but use still\n");
        }
    } else {
        TPD_INFO("fw check checksum ok\n");
    }
    /* show fw type info (0:MP, 1:Normal)*/
    TPD_INFO("%s FW type is 0x%02X\n", __func__, *(fw->data + FW_BIN_TYPE_OFFSET));

    /* BIN Header Parser */
    ret = nvt_bin_header_parser(chip_info, fw->data, fw->size);
    if (ret) {
        TPD_INFO("bin header parser failed\n");
        goto out_fail;
    }

    /* initial buffer and variable */
    ret = Download_Init(chip_info);
    if (ret) {
        TPD_INFO("Download Init failed. (%d)\n", ret);
        goto out_fail;
    }

    /* download firmware process */
    if (chip_info->trim_id_table.support_hw_crc) {
        ret = Download_Firmware_HW_CRC(chip_info, fw);
    }
    if (ret) {
        TPD_INFO("Download Firmware failed. (%d)\n", ret);
        goto out_fail;
    }

    TPD_INFO("Update firmware success! <%ld us>\n",
             (end.tv_sec - start.tv_sec) * 1000000L + (end.tv_usec - start.tv_usec));

    /* Get FW Info */
    ret = CTP_SPI_READ(chip_info->s_client, point_data, POINT_DATA_LEN + 1);
    if (ret < 0 || nvt_fw_recovery(point_data)) {
        nvt_esd_check_enable(chip_info, true);
    }
    ret = nvt_get_fw_info_noflash(chip_info);
    if (ret) {
        TPD_INFO("nvt_get_fw_info_noflash failed. (%d)\n", ret);
        goto out_fail;
    }
    ret = CTP_SPI_READ(chip_info->s_client, point_data, POINT_DATA_LEN + 1);
    if (ret < 0 || nvt_fw_recovery(point_data)) {
        nvt_esd_check_enable(chip_info, true);
    }
    nvt_fw_check(ts->chip_data, &ts->resolution_info, &ts->panel_data);

    if(chip_info->bin_map != NULL) {
        kfree(chip_info->bin_map);
        chip_info->bin_map = NULL;
    }

    if(request_fw_headfile != NULL) {
        kfree(request_fw_headfile);
        request_fw_headfile = NULL;
        //fw = NULL;
    }
    return FW_UPDATE_SUCCESS;

out_fail:
    if(chip_info->bin_map != NULL) {
        kfree(chip_info->bin_map);
        chip_info->bin_map = NULL;
    }
    if(request_fw_headfile != NULL) {
        kfree(request_fw_headfile);
        request_fw_headfile = NULL;
        //fw = NULL;
    }
    return FW_NO_NEED_UPDATE;
}



static fw_update_state nvt_fw_update(void *chip_data, const struct firmware *fw, bool force)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    if (fw) {
        if (chip_info->g_fw_buf) {
            chip_info->g_fw_len = fw->size;
            memcpy(chip_info->g_fw_buf, fw->data, fw->size);
            chip_info->g_fw_sta = true;
        }
    }
    return nvt_fw_update_sub(chip_data, fw, force);

}

static void nvt_set_touch_direction(void *chip_data, uint8_t dir)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    chip_info->touch_direction = dir;
}

static uint8_t nvt_get_touch_direction(void *chip_data)
{
    struct chip_data_nt36525b *chip_info = (struct chip_data_nt36525b *)chip_data;

    return chip_info->touch_direction;
}

static struct oppo_touchpanel_operations nvt_ops = {
    .ftm_process                = nvt_ftm_process,
    .reset                      = nvt_reset,
    .power_control              = nvt_power_control,
    .get_chip_info              = nvt_get_chip_info,
    .trigger_reason             = nvt_trigger_reason,
    .get_touch_points           = nvt_get_touch_points,
    .get_gesture_info           = nvt_get_gesture_info,
    .mode_switch                = nvt_mode_switch,
    .fw_check                   = nvt_fw_check,
    .fw_update                  = nvt_fw_update,
    .get_vendor                 = nvt_get_vendor,
    .get_usb_state              = nvt_get_usb_state,
    .esd_handle                 = nvt_esd_handle,
    .reset_gpio_control         = nvt_reset_gpio_control,
    .set_touch_direction        = nvt_set_touch_direction,
    .get_touch_direction        = nvt_get_touch_direction,
};

static struct debug_info_proc_operations debug_info_proc_ops = {
};


/*********** Start of SPI Driver and Implementation of it's callbacks*************************/
int __maybe_unused nvt_tp_probe(struct spi_device *client)
{
    struct chip_data_nt36525b *chip_info = NULL;
    struct touchpanel_data *ts = NULL;
    int ret = -1;

    TPD_INFO("%s  is called\n", __func__);

    /* 1. alloc chip info */
    chip_info = kzalloc(sizeof(struct chip_data_nt36525b), GFP_KERNEL);
    if (chip_info == NULL) {
        TPD_INFO("chip info kzalloc error\n");
        ret = -ENOMEM;
        return ret;
    }
    memset(chip_info, 0, sizeof(*chip_info));
    g_chip_info = chip_info;
    chip_info->probe_done = 0;
    /* 2. Alloc common ts */
    ts = common_touch_data_alloc();
    if (ts == NULL) {
        TPD_INFO("ts kzalloc error\n");
        goto ts_malloc_failed;
    }
    memset(ts, 0, sizeof(*ts));

    chip_info->g_fw_buf = vmalloc(128 * 1024);

    if (chip_info->g_fw_buf == NULL) {
        TPD_INFO("fw buf vmalloc error\n");
        //ret = -ENOMEM;
        goto err_g_fw_buf;
    }
    chip_info->g_fw_sta = false;

    chip_info->fw_buf_dma = kzalloc(128 * 1024, GFP_KERNEL | GFP_DMA);
    if (chip_info->fw_buf_dma == NULL) {
        TPD_INFO("fw kzalloc error\n");
        //ret = -ENOMEM;
        goto err_fw_dma;
    }

    /* 3. bind client and dev for easy operate */
    chip_info->s_client = client;
    ts->debug_info_ops = &debug_info_proc_ops;
    ts->s_client = client;
    ts->irq = client->irq;
    spi_set_drvdata(client, ts);
    ts->dev = &client->dev;
    ts->chip_data = chip_info;
    chip_info->irq_num = ts->irq;
    chip_info->hw_res = &ts->hw_res;
    chip_info->ENG_RST_ADDR = 0x7FFF80;
    chip_info->recovery_cnt = 0;
    chip_info->partition = 0;
    chip_info->ilm_dlm_num = 2;
    chip_info->p_firmware_headfile = &ts->panel_data.firmware_headfile;
    chip_info->touch_direction = VERTICAL_SCREEN;
    mutex_init(&chip_info->mutex_testing);
    chip_info->using_headfile = false;

    //---prepare for spi parameter---
    if (ts->s_client->master->flags & SPI_MASTER_HALF_DUPLEX) {
        TPD_INFO("Full duplex not supported by master\n");
        ret = -EIO;
        goto err_spi_setup;
    }
    ts->s_client->bits_per_word = 8;
    ts->s_client->mode = SPI_MODE_0;
    ts->s_client->chip_select = 0; //modify reg=0 for more tp vendor share same spi interface

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifdef CONFIG_SPI_MT65XX
    /* new usage of MTK spi API */
    memcpy(&chip_info->spi_ctrl, &spi_ctrdata, sizeof(struct mtk_chip_config));
    ts->s_client->controller_data = (void *)&chip_info->spi_ctrl;
#else
    /* old usage of MTK spi API */
    //memcpy(&chip_info->spi_ctrl, &spi_ctrdata, sizeof(struct mt_chip_conf));
    //ts->s_client->controller_data = (void *)&chip_info->spi_ctrl;

    ret = spi_setup(ts->s_client);
    if (ret < 0) {
        TPD_INFO("Failed to perform SPI setup\n");
        goto err_spi_setup;
    }
#endif	//CONFIG_SPI_MT65XX
#else // else of CONFIG_TOUCHPANEL_MTK_PLATFORM
    ret = spi_setup(ts->s_client);
    if (ret < 0) {
        TPD_INFO("Failed to perform SPI setup\n");
        goto err_spi_setup;
    }

#endif // end of CONFIG_TOUCHPANEL_MTK_PLATFORM

    TPD_INFO("mode=%d, max_speed_hz=%d\n", ts->s_client->mode, ts->s_client->max_speed_hz);

    /* 4. file_operations callbacks binding */
    ts->ts_ops = &nvt_ops;

    /* 5. register common touch device*/
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto err_register_driver;
    }
    nvt_lcd_reset_gpio_init(chip_info);
    ts->tp_suspend_order = TP_LCD_SUSPEND;
    ts->tp_resume_order = LCD_TP_RESUME;
    chip_info->is_sleep_writed = false;
    chip_info->fw_name = ts->panel_data.fw_name;
    chip_info->dev = ts->dev;
    chip_info->test_limit_name = ts->panel_data.test_limit_name;

    //reset esd handle time interval
    if (ts->esd_handle_support) {
        chip_info->esd_check_enabled = false;
        ts->esd_info.esd_work_time = msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD); // change esd check interval to 1.5s
        TPD_INFO("%s:change esd handle time to %d s\n", __func__, ts->esd_info.esd_work_time / HZ);
    }

    /*6. create nvt test files*/
    nvt_flash_proc_init(ts, "NVTSPI");

    chip_info->ts = ts;

    // update fw in probe
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
    if (ts->boot_mode == RECOVERY_BOOT || is_oem_unlocked() || ts->fw_update_in_probe_with_headfile)
#else
    if (ts->boot_mode == MSM_BOOT_MODE__RECOVERY || is_oem_unlocked() || ts->fw_update_in_probe_with_headfile)
#endif
    {
        TPD_INFO("In Recovery mode, no-flash download fw by headfile\n");
        ret = nvt_fw_update(chip_info, NULL, 0);
        if(ret > 0) {
            TPD_INFO("fw update failed!\n");
        }
    }

    chip_info->probe_done = 1;
    TPD_INFO("%s, probe normal end\n", __func__);
    return 0;

err_register_driver:
    common_touch_data_free(ts);
    ts = NULL;

err_spi_setup:
    if (chip_info->fw_buf_dma) {
        kfree(chip_info->fw_buf_dma);
    }

    spi_set_drvdata(client, NULL);
err_fw_dma:
    if (chip_info->g_fw_buf) {
        vfree(chip_info->g_fw_buf);
    }
err_g_fw_buf:
    if (ts) {
        kfree(ts);
    }

ts_malloc_failed:
    kfree(chip_info);
    chip_info = NULL;
    ret = -1;

    TPD_INFO("%s, probe error\n", __func__);
    return ret;
}

int __maybe_unused nvt_tp_remove(struct spi_device *client)
{
    struct touchpanel_data *ts = spi_get_drvdata(client);

    TPD_INFO("%s is called\n", __func__);
    kfree(ts);

    return 0;
}
