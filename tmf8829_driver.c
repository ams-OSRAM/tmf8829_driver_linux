/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/*! \file tmf8829_driver.c - TMF8829 linux driver
 * \brief Device driver for measuring distance in mm.
 */

/* -------------------------------- includes -------------------------------- */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/printk.h>
#include <linux/poll.h>

#include "tmf8829_driver.h"
#include "tmf8829_hex_interpreter.h"

/* -------------------------------- defines --------------------------------- */
#define APP_LOG_LEVEL          TMF8829_LOG_LEVEL_ERROR // initialization log level
#define TMF8829_IMAGE_START    0x00010000

/* -------------------------------- variables ------------------------------- */
static struct tmf8829_platform_data tmf8829_pdata = {
    .tof_name = "tmf8829",
    .ram_patch_fname = { "tmf8829_application.hex", },
};

/**************************************************************************/
/*  TMF8829 Common Functions                                              */
/**************************************************************************/
/**
 * firmware_download - firmware download and start of RAM application
 *
 * @dev: device pointer
 *
 * Returns 0 for success, otherwise < 0 for an error.
 */
static int firmware_download(struct device *dev) {

    tmf8829_chip *chip = dev_get_drvdata(dev);
    int error = -1;
    const struct firmware *cfg = NULL;
    const u8 *line;
    const u8 *line_end;
    u32 patch_size = 0;

    /* Read in Hex Data */
    dev_info(dev, "Trying firmware: \'%s\'...\n", chip->pdata->ram_patch_fname[0]);
    error = request_firmware_direct(&cfg,chip->pdata->ram_patch_fname[0], dev);
    if (error || !cfg) {
        dev_err(dev," FW not available: %d\n", error);
        return -1;
    }

    intelHexInterpreterInitialise( );
    line = cfg->data;
    line_end = line;
    while ((line_end - cfg->data) < cfg->size) {
        line_end = strchrnul(line, '\n');
        patch_size += ((line_end - line) > INTEL_HEX_MIN_RECORD_SIZE) ?
                        ((line_end - line - INTEL_HEX_MIN_RECORD_SIZE) / 2) : 0;
        error = intelHexHandleRecord(chip, line_end - line, line);
        if (error) {
            dev_err(dev, "Ram patch failed: %d\n", error);
            return -2;
        }
        line = ++line_end;
    }

    /*  Download Data */
    error = tmf8829DownloadFirmware(&chip->tof_core, TMF8829_IMAGE_START , patchImage, imageSize, 1 /* use Fifo */);
    if ( error != BL_SUCCESS_OK) {
        dev_err(dev, "Download Error %d\n", error);
        return -3;
    }

    return 0;
}
/**************************************************************************/
/* Sysfs callbacks                                                        */
/**************************************************************************/

/******** Common show/store functions ********/

static ssize_t program_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int error;
    char prog;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);
    error = rxReg(chip, 0, TMF8829_COM_REG_APP_ID, 1, &prog);
    AMS_MUTEX_UNLOCK(&chip->lock);
    
    if (error) { 
        return -EIO;
    }

    return scnprintf(buf, PAGE_SIZE, "%#hhx\n", prog);
}

static ssize_t chip_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int state;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);
    if (!chip->pdata->gpiod_enable) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    state = gpiod_get_value(chip->pdata->gpiod_enable) ? 1 : 0;
    AMS_MUTEX_UNLOCK(&chip->lock);

    return scnprintf(buf, PAGE_SIZE, "%d\n", state);
}

static ssize_t chip_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    char req_state;
    int len;

    dev_info(dev, "%s\n", __func__);

    len = sscanf(buf, "%hhx", &req_state);
    if (len != 1)
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);
    if (!chip->pdata->gpiod_enable) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
     
    if (req_state == 0) {
        if (chip->tof_core.device.appVersion[0] == TMF8829_APP_ID) {
            if (tmf8829StopMeasurement(&chip->tof_core) != APP_SUCCESS_OK) {
                dev_err(dev, "Stop Measurement");
            }
        }
        enablePinLow(chip);
    }
    else {
        tmf8829Enable(&chip->tof_core);
        delayInMicroseconds(ENABLE_TIME_MS * 1000);
        tmf8829PowerUp(&chip->tof_core);

        if (tmf8829IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS) == 0) {
            dev_err(dev, "CPU not ready");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        if (req_state == TMF8829_COM_APP_ID__bootloader) {
            dev_info(dev, "Bootloader mode\n");
        }
        else {
            ssize_t patch_ret = firmware_download(dev);
        
            if (patch_ret != 0) {
                AMS_MUTEX_UNLOCK(&chip->lock);
                return -EIO;
            }
            else {
                dev_info(dev, "Firmware download done.\n");
            }


            if (tmf8829GetConfiguration(&chip->tof_core) != APP_SUCCESS_OK) {
                dev_err(dev, "Read device configuration error.\n");
                AMS_MUTEX_UNLOCK(&chip->lock);
                return -EIO;
            }
            if (tmf8829ReadDeviceInfo( &chip->tof_core) != APP_SUCCESS_OK) {
                dev_err(dev, "Read device Information.\n");
                AMS_MUTEX_UNLOCK(&chip->lock);
                return -EIO;
            }
        }
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return count;
}

static ssize_t driver_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);

    return scnprintf(buf, PAGE_SIZE, "%#x\n", chip->tof_core.logLevel);
}

static ssize_t driver_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    char logLev;

    dev_info(dev, "%s\n", __func__);

    sscanf(buf, "%hhx", &logLev);
    tmf8829SetLogLevel(&chip->tof_core,logLev);

    return count;
}

static ssize_t program_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int len = -1;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);
    if (tmf8829ReadDeviceInfo(&chip->tof_core) == APP_SUCCESS_OK) {
        len = scnprintf(buf, PAGE_SIZE, "%#x %#x %#x %#x %#x %#x\n",
                        chip->tof_core.device.appVersion[0], chip->tof_core.device.appVersion[1],
                        chip->tof_core.device.appVersion[2], chip->tof_core.device.appVersion[3],
                        chip->tof_core.device.chipVersion[0], chip->tof_core.device.chipVersion[1]);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return len;
}

static ssize_t serial_number_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int len = -1;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);
    if (tmf8829ReadDeviceInfo(&chip->tof_core) == APP_SUCCESS_OK) {
        len = scnprintf(buf, PAGE_SIZE, "%#lx\n", (long unsigned int)chip->tof_core.device.deviceSerialNumber);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return len;
}


static ssize_t register_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    char preg;
    char pval;
    char pmask = -1;
    int numparams;
    int rc = 0;
      
    dev_info(dev, "%s\n", __func__);

    numparams = sscanf(buf, "%hhx:%hhx:%hhx", &preg, &pval, &pmask);
    if ((numparams < 2) || (numparams > 3))
        return -EINVAL;
    if ((numparams >= 1) && (preg < 0))
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);
    if (numparams == 2) {
        rc = txReg(chip, 0, preg,1, &pval);
    } else {
        rc = regWriteMask(chip, 0, preg, pval, pmask);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return rc ? rc : count;
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int per_line = 4; // dumped register values per line
    int len = 0;
    int idx, per_line_idx;
    int bufsize = PAGE_SIZE;
    int error;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);
    memset(chip->shadow, 0, MAX_REGS);
    error = rxReg(chip, 0, 0x00, MAX_REGS, chip->shadow);

    if (error) {
        dev_err(dev, "Read all registers failed: %d\n", error);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return error;
    }

    for (idx = 0; idx < MAX_REGS; idx += per_line) {
        len += scnprintf(buf + len, bufsize - len, "%#02x:", idx);
        for (per_line_idx = 0; per_line_idx < per_line; per_line_idx++) {
            len += scnprintf(buf + len, bufsize - len, " ");
            len += scnprintf(buf + len, bufsize - len, "%#02x", chip->shadow[idx+per_line_idx]);
        }
        len += scnprintf(buf + len, bufsize - len, "\n");
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return len;
}

static ssize_t request_ram_patch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int error = -1;
    char prog;
    ssize_t patch_ret = -1 ;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);

    /* Check if bootloader mode */
    error = rxReg(chip, 0, TMF8829_COM_REG_APP_ID, 1, &prog);
    if (error || (prog != TMF8829_COM_APP_ID__bootloader)) {
        dev_info(dev, "%s Download only in Bootloader mode\n", __func__);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }

    patch_ret = firmware_download(dev);

    dev_info(dev, "Firmware download done.\n");
    if (patch_ret != 0) {
        goto err_fmwdwnl;
    }

    /* Get Configuration */
    error = tmf8829GetConfiguration(&chip->tof_core);
    if ( error != APP_SUCCESS_OK) {
        dev_err(dev, "Read device configuration error.\n");
        goto err_fmwdwnl;
    }

    /* Read Device Info */
    error = tmf8829ReadDeviceInfo(&chip->tof_core);
    if ( error != APP_SUCCESS_OK) {
        dev_err(dev, "Read Device Info Error %d\n", error);
        goto err_fmwdwnl;
    }
    tmf8829ClrAndEnableInterrupts(&chip->tof_core, TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_HISTOGRAMS);
    dev_info(dev, "Download done\n");
    
err_fmwdwnl:
    AMS_MUTEX_UNLOCK(&chip->lock);

    return error ? -EIO : count;
}

/****** Application cmd show/store functions  ******/

static ssize_t start_measurement_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);

    if (chip->tof_core.logLevel & TMF8829_LOG_LEVEL_VERBOSE) {
        dev_info(dev, "%s\n", __func__);
    }

    return scnprintf(buf, PAGE_SIZE, "%#x\n", chip->tof_core.cyclicRunning);
}

static ssize_t start_measurement_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int len = 0;
    int error = 0;
    char start;

    dev_info(dev, "%s\n", __func__);

    len = sscanf(buf, "%hhx",&start);
 
    if (len != 1)
        return -EINVAL;
    
    AMS_MUTEX_LOCK(&chip->lock);  
    if (start) {
        error = tmf8829StartMeasurement(&chip->tof_core);
    }
    else {
        error = tmf8829StopMeasurement(&chip->tof_core);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return (error) ? -EIO : count;
}

static ssize_t configuration_setting_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    ssize_t ret = 0;
    int i;
    int bufsize = PAGE_SIZE;
    int val;

    if (chip->tof_core.logLevel & TMF8829_LOG_LEVEL_VERBOSE) {
         dev_info(dev, "%s cmd: %s buf %s \n", __func__, attr->attr.name, buf);
    }

    AMS_MUTEX_LOCK(&chip->lock);
    tmf8829GetConfiguration(&chip->tof_core);
    if (!strncmp(attr->attr.name,"config_custom", strlen(attr->attr.name))) {
       for (i = 0; i < TMF8829_CFG_PAGE_SIZE; i++) {
           ret += scnprintf(buf + ret, bufsize - ret, "%hhx ", chip->tof_core.config[i]);
       }
       ret += scnprintf(buf + ret, bufsize - ret, "\n");
    }
    else if (!strncmp(attr->attr.name,"period", strlen(attr->attr.name))) {
        val = chip->tof_core.config[TMF8829_CFG_PERIOD_MS_MSB-TMF8829_CFG_PERIOD_MS_LSB] * 256 + chip->tof_core.config[0];
        ret += scnprintf(buf, PAGE_SIZE, "%hu\n", val);
    }
    else if (!strncmp(attr->attr.name,"iterations", strlen(attr->attr.name))) {
        val = chip->tof_core.config[TMF8829_CFG_KILO_ITERATIONS_MSB-TMF8829_CFG_PERIOD_MS_LSB] * 256;
        val += chip->tof_core.config[TMF8829_CFG_KILO_ITERATIONS_LSB- TMF8829_CFG_PERIOD_MS_LSB];
        ret += scnprintf(buf, PAGE_SIZE, "%hu\n", val);
    }
    else if (!strncmp(attr->attr.name,"fp_mode", strlen(attr->attr.name))) {
        ret += scnprintf(buf, PAGE_SIZE, "%hhu\n", chip->tof_core.config[TMF8829_CFG_FP_MODE-TMF8829_CFG_PERIOD_MS_LSB]);
    }
    else if (!strncmp(attr->attr.name,"result_format", strlen(attr->attr.name))) {
        ret += scnprintf(buf, PAGE_SIZE, "%hhu\n", chip->tof_core.config[TMF8829_CFG_RESULT_FORMAT-TMF8829_CFG_PERIOD_MS_LSB]); 
    }
    else if (!strncmp(attr->attr.name,"histograms", strlen(attr->attr.name))) {
        ret += scnprintf(buf, PAGE_SIZE, "%hhu\n", chip->tof_core.config[TMF8829_CFG_DUMP_HISTOGRAMS-TMF8829_CFG_PERIOD_MS_LSB]);  
    }
    else {
        ret += scnprintf(buf, PAGE_SIZE, "wrong cmd \n");
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return ret;
}

static ssize_t config_custom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return configuration_setting_show(dev, attr, buf);
}

static ssize_t period_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return configuration_setting_show(dev, attr, buf);
}

static ssize_t iterations_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return configuration_setting_show(dev, attr, buf);
}

static ssize_t fp_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return configuration_setting_show(dev, attr, buf);
}

static ssize_t result_format_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return configuration_setting_show(dev, attr, buf);
}

static ssize_t histograms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return configuration_setting_show(dev, attr, buf);
}

static ssize_t configuration_setting_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    ssize_t ret = count;
    char *sub_string = NULL;
    int8_t error;
    unsigned short val;
    int i;
    
    if (chip->tof_core.logLevel & TMF8829_LOG_LEVEL_VERBOSE) {
        dev_info(dev, "%s cmd: %s buf: %s \n", __func__, attr->attr.name, buf); 
    }

    AMS_MUTEX_LOCK(&chip->lock);

    error = tmf8829GetConfiguration(&chip->tof_core);

    if (error == APP_SUCCESS_OK) {

        if (!strncmp(attr->attr.name, "config_custom", strlen(attr->attr.name))) {
            for (i = 0; (i <= TMF8829_CFG_PAGE_SIZE); i++) {
                sub_string = strsep((char **)&buf, " ");
                if (sub_string) {
                    ret = sscanf(sub_string, "%hhx", &chip->tof_core.config[i]);
                    if (ret == 0) {
                        break;
                    }
                }
            }
        }
        else if (!strncmp(attr->attr.name, "period", strlen(attr->attr.name))) {
            ret = sscanf(buf, "%hu",  &val);
            chip->tof_core.config[0] = val & 0xFF;
            chip->tof_core.config[TMF8829_CFG_PERIOD_MS_MSB-TMF8829_CFG_PERIOD_MS_LSB] = (val >> 8) & 0xFF;
        }
        else if (!strncmp(attr->attr.name, "iterations", strlen(attr->attr.name))) {
            ret = sscanf(buf, "%hu", &val);
            chip->tof_core.config[TMF8829_CFG_KILO_ITERATIONS_LSB-TMF8829_CFG_PERIOD_MS_LSB] = val & 0xFF;
            chip->tof_core.config[TMF8829_CFG_KILO_ITERATIONS_MSB-TMF8829_CFG_PERIOD_MS_LSB] = (val >> 8) & 0xFF;
        }
        else if (!strncmp(attr->attr.name,"fp_mode", strlen(attr->attr.name))) {
            ret = sscanf(buf, "%hhu", &chip->tof_core.config[TMF8829_CFG_FP_MODE-TMF8829_CFG_PERIOD_MS_LSB]);
        }
        else if (!strncmp(attr->attr.name,"result_format", strlen(attr->attr.name))) {
            ret = sscanf(buf, "%hhu", &chip->tof_core.config[TMF8829_CFG_RESULT_FORMAT-TMF8829_CFG_PERIOD_MS_LSB]);
        }
        else if (!strncmp(attr->attr.name,"histograms", strlen(attr->attr.name))) {
            ret = sscanf(buf, "%hhu", &chip->tof_core.config[TMF8829_CFG_DUMP_HISTOGRAMS-TMF8829_CFG_PERIOD_MS_LSB]);
        }
        else {
            dev_info(dev, " cmd not found\n"); 
        }
        error = tmf8829SetConfiguration(&chip->tof_core);
    }
    
    AMS_MUTEX_UNLOCK(&chip->lock);

    if (error)
       return -EIO;

    return (ret != 1) ? -EINVAL : count;
}

static ssize_t config_custom_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return configuration_setting_store(dev, attr, buf,count);
}

static ssize_t period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return configuration_setting_store(dev, attr, buf,count);
}

static ssize_t iterations_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return configuration_setting_store(dev, attr, buf,count);
}

static ssize_t fp_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return configuration_setting_store(dev, attr, buf,count);
}

static ssize_t result_format_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return configuration_setting_store(dev, attr, buf,count);
}

static ssize_t histograms_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    return configuration_setting_store(dev, attr, buf,count);
}

static ssize_t config_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{ 
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int len = 0;
    int error = 0;
    char cmd;

    dev_info(dev, "%s\n", __func__);

    len = sscanf(buf, "%hhx",&cmd);
 
    if (len != 1)
        return -EINVAL;
    
    if ((cmd < TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8) || 
        (cmd > TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32_HIGH_ACCURACY)) {
        return -EINVAL;
    }
    AMS_MUTEX_LOCK(&chip->lock);  
    error = tmf8829Command(&chip->tof_core, cmd);
    AMS_MUTEX_UNLOCK(&chip->lock);

    return (error) ? -EIO : count;
}

static ssize_t clk_correction_store(struct device * dev, struct device_attribute * attr, const char * buf, size_t count)
{
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int ret;
    uint8_t clkenable = 0;

    dev_info(dev, "%s\n", __func__);

    ret = sscanf(buf, "%hhx ", &clkenable);
    if (ret != 1)
        return -EINVAL;
          
    AMS_MUTEX_LOCK(&chip->lock);
    tmf8829ClkCorrection(&chip->tof_core, clkenable);
    AMS_MUTEX_UNLOCK(&chip->lock);

    return count;
}


static ssize_t app_tof_output_read(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
                                   char *buf, loff_t off, size_t size)
{
    struct device *dev = kobj_to_dev(kobj);
    tmf8829_chip *chip = dev_get_drvdata(dev);
    int read;
    ssize_t elem_len;

    AMS_MUTEX_LOCK(&chip->fifo_lock);
    elem_len = kfifo_peek_len(&chip->tof_output_fifo);
    
    if (elem_len > PAGE_SIZE) {
        if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
            dev_info(dev, "frames to large: %u\n", elem_len); 
        }
        AMS_MUTEX_UNLOCK(&chip->fifo_lock);
        return -EBADE;
    }

    if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
        dev_info(dev, "%s size: %u\n", __func__, (unsigned int) size);
    }
    if (kfifo_len(&chip->tof_output_fifo)) {
        if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
            dev_info(dev, "fifo read elem_len: %u\n", elem_len);
        }
        read = kfifo_out(&chip->tof_output_fifo, buf, elem_len);
        if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
            dev_info(dev, "fifo_len: %u\n", kfifo_len(&chip->tof_output_fifo));
        }
        AMS_MUTEX_UNLOCK(&chip->fifo_lock);
        return elem_len;
    }
    AMS_MUTEX_UNLOCK(&chip->fifo_lock);
    return 0;
}

/**************************************************************************/
/* System File System                                                     */
/**************************************************************************/
/* Common Sysfs */
static DEVICE_ATTR_RO(program);
static DEVICE_ATTR_RW(chip_enable);
static DEVICE_ATTR_RW(driver_debug);
static DEVICE_ATTR_RO(program_version);
static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_WO(register_write);
static DEVICE_ATTR_RO(registers);
static DEVICE_ATTR_WO(request_ram_patch);

static struct attribute *tmf8829_common_attrs[] = {
    &dev_attr_program.attr,
    &dev_attr_chip_enable.attr,
    &dev_attr_driver_debug.attr,
    &dev_attr_program_version.attr,
    &dev_attr_serial_number.attr,
    &dev_attr_register_write.attr,
    &dev_attr_registers.attr,
    &dev_attr_request_ram_patch.attr,
    NULL,
};
static const struct attribute_group tmf8829_common_attr_group = {
  .name = "tmf8829_common",
  .attrs = tmf8829_common_attrs,
};

/* Application Sysfs */
static DEVICE_ATTR_RW(start_measurement);
static DEVICE_ATTR_RW(config_custom);
static DEVICE_ATTR_RW(period);
static DEVICE_ATTR_RW(iterations);
static DEVICE_ATTR_RW(fp_mode);
static DEVICE_ATTR_RW(result_format);
static DEVICE_ATTR_RW(histograms);
static DEVICE_ATTR_WO(config_mode);
static DEVICE_ATTR_WO(clk_correction);
static BIN_ATTR_RO(app_tof_output, 0);

static struct bin_attribute *tof_app_bin_attrs[] = {
  &bin_attr_app_tof_output,
  NULL,
};
static struct attribute *tmf8829_app_attrs[] = {
    &dev_attr_start_measurement.attr,
    &dev_attr_config_custom.attr,
    &dev_attr_period.attr,
    &dev_attr_iterations.attr,
    &dev_attr_fp_mode.attr,
    &dev_attr_result_format.attr,
    &dev_attr_histograms.attr,
    &dev_attr_config_mode.attr,
    &dev_attr_clk_correction.attr,
    NULL,
};
static const struct attribute_group tmf8829_app_attr_group = {
    .name = "tmf8829_app",
    .attrs = tmf8829_app_attrs,
    .bin_attrs = tof_app_bin_attrs,
};
/*All groups*/
static const struct attribute_group *tmf8829_attr_groups[] = {
    &tmf8829_common_attr_group,
    &tmf8829_app_attr_group,
    NULL,
};

/**************************************************************************/
/* Output Data Handling                                                   */
/**************************************************************************/
int tof_queue_frame(tmf8829_chip *chip)
{
    int result = 0;
    int size = (chip->tof_output_frame.frame.payload_msb << 8) + chip->tof_output_frame.frame.payload_lsb
               + DRIVER_HEADER_SIZE;

    AMS_MUTEX_LOCK(&chip->fifo_lock);

    result = kfifo_in(&chip->tof_output_fifo, chip->tof_output_frame.buf, size);
    if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_INFO) {
        
        dev_info(&chip->client->dev, "Size %#x\n",size);
        dev_info(&chip->client->dev, "Fr Num %#x\n",chip->tof_output_frame.frame.frameNumber);
    }
    if (result == 0) {
        if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_INFO) {
            dev_info(&chip->client->dev, "fifo_len: %u\n", kfifo_len(&chip->tof_output_fifo));
            dev_info(&chip->client->dev, "Reset output frame.\n"); 
        }
        kfifo_reset(&chip->tof_output_fifo);
        result = kfifo_in(&chip->tof_output_fifo, chip->tof_output_frame.buf, size);
        if (result == 0) {
            dev_err(&chip->client->dev, "Error: queueing ToF output frame.\n");
        }
        if (result != size) {
            dev_err(&chip->client->dev, "Error: queueing ToF output frame Size.\n");
        }
    }
    AMS_MUTEX_UNLOCK(&chip->fifo_lock);
   
    chip->tof_output_frame.frame.frameId = TMF8829_COM_RESULT__no_frame;
    chip->tof_output_frame.frame.frameNumber++;
    chip->tof_output_frame.frame.clkcorr_lsb = 0x66;
    chip->tof_output_frame.frame.clkcorr_msb = 0x66;
    chip->tof_output_frame.frame.payload_lsb = 0;
    chip->tof_output_frame.frame.payload_msb = 0;

    return (result == size) ? 0 : -1;
}

/**************************************************************************/
/* IRQ Handling and GPIO Configuration                                    */
/**************************************************************************/
/**
 * tof_get_gpio_config - Get GPIO config from Device-Managed API
 *
 * @tmf8829_chip: tmf8829_chip pointer
 *
 * Returns 0
 */

static int tof_get_gpio_config(tmf8829_chip *tof_chip)
{
    int error;
    struct device *dev;
    struct gpio_desc *gpiod;

    if (!tof_chip->client) {
        return -EINVAL;
    }
    dev = &tof_chip->client->dev;

    /* Get the enable line GPIO pin number */
    gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_ENABLE_NAME, GPIOD_OUT_HIGH);
    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        return error;
    }
    tof_chip->pdata->gpiod_enable = gpiod;

    /* Get the interrupt GPIO pin number */
    gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_INT_NAME, GPIOD_IN);

    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        dev_info(&tof_chip->client->dev, "Error: Irq. %d \n", error);
        if (PTR_ERR(gpiod) != -EBUSY) { //for ICAM on this pin there is an Error (from ACPI), but it is working
            return error;
        }
    }
    tof_chip->pdata->gpiod_interrupt = gpiod;

    return 0;
}

/**
 * tof_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 * 
 * Returns IRQ_HANDLED
 */

static irqreturn_t tof_irq_handler(int irq, void *dev_id)
{
    tmf8829_chip *chip = (tmf8829_chip *)dev_id;
    int handled;
    AMS_MUTEX_LOCK(&chip->lock);

    if (chip->tof_core.logLevel & TMF8829_LOG_LEVEL_VERBOSE) {
        dev_info(&chip->client->dev, "irq_handler");
    }
    
    handled = tmf8829_app_process_irq(&chip->tof_core);

    if (handled == 0) {
        if (chip->tof_core.cyclicRunning) {
            dev_err(&chip->client->dev, "unknown irq");
        }
    }

    AMS_MUTEX_UNLOCK(&chip->lock);
    
    return IRQ_HANDLED;
}

/**
 * tof_request_irq - request IRQ for given gpio
 *
 * @tof_chip: tmf8829_chip pointer
 * 
 * Returns status of function devm_request_threaded_irq
 */

static int tof_request_irq(tmf8829_chip *tof_chip)
{
    int irq = tof_chip->client->irq;
    unsigned long default_trigger = irqd_get_trigger_type(irq_get_irq_data(irq));

    dev_info(&tof_chip->client->dev, "irq: %d, trigger_type: %lu", irq, default_trigger);

    return devm_request_threaded_irq(&tof_chip->client->dev, tof_chip->client->irq, NULL, tof_irq_handler,
                                    default_trigger | IRQF_SHARED | IRQF_ONESHOT, tof_chip->client->name, tof_chip);
}

/**
 * tmf8829_app_poll_irq_thread - 
 *
 * @tof_chip: tmf8829_chip pointer
 * 
 * Returns 0
 */

static int tmf8829_app_poll_irq_thread(void *tof_chip)
{
    tmf8829_chip *chip = (tmf8829_chip *)tof_chip;
    int us_sleep = 0;
    int period = chip->tof_core.config[TMF8829_CFG_PERIOD_MS_MSB-TMF8829_CFG_PERIOD_MS_LSB] * 256;
    period += chip->tof_core.config[TMF8829_CFG_PERIOD_MS_LSB];
    AMS_MUTEX_LOCK(&chip->lock);

    us_sleep = period * 1000;// Poll period is interpreted in units of 100 usec
    if (us_sleep == 0 ) {
        us_sleep = 10000;
    }
    dev_info(&chip->client->dev, "Starting ToF irq polling thread, period: %u us\n", us_sleep);
    AMS_MUTEX_UNLOCK(&chip->lock);
    while (!kthread_should_stop()) {
        (void) tof_irq_handler(0, tof_chip);
        delayInMicroseconds(us_sleep);
    }
    return 0;
}

/**
 * misc device functions
 */

static int tof_misc_release(struct inode *inode, struct file *f)
{
    return 0;
}

static int tof_misc_open(struct inode *inode, struct file *f)
{
    return 0;
}

static ssize_t tof_misc_read(struct file *f, char *buf, size_t len, loff_t *off)
{
    struct miscdevice *misc = (struct miscdevice *)f->private_data;
    struct _tmf8829_chip *chip =  container_of(misc, struct _tmf8829_chip, tof_mdev);
    unsigned int copied = 0;
    unsigned int sum_copied = 0;
    ssize_t elem_len;

    int ret;

    dev_dbg(&chip->client->dev, "%s\n", __func__);

    if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
        dev_info(&chip->client->dev, "read: %u\n", len);
    }

    AMS_MUTEX_LOCK(&chip->fifo_lock);
    while (kfifo_len(&chip->tof_output_fifo)) {
        elem_len = kfifo_peek_len(&chip->tof_output_fifo);

        if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
            dev_info(&chip->client->dev, "fifo read elem_len: %u\n", elem_len);
        }
        ret = kfifo_to_user(&chip->tof_output_fifo, &buf[sum_copied], elem_len, &copied);
        sum_copied +=copied;
        if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
            dev_info(&chip->client->dev, "fifo_len: %u\n", kfifo_len(&chip->tof_output_fifo));
        }

        if(ret) {
            dev_err(&chip->client->dev, "copy to user space");
        }
    }
    
    AMS_MUTEX_UNLOCK(&chip->fifo_lock);

    if (chip->tof_core.logLevel >= TMF8829_LOG_LEVEL_VERBOSE) {
        dev_info(&chip->client->dev, "copied: %u\n", sum_copied);
    }

    return sum_copied;
}

static const struct file_operations tof_miscdev_fops = {
    .owner          = THIS_MODULE,
    .read           = tof_misc_read,
    .open           = tof_misc_open,
    .release        = tof_misc_release,
    .llseek         = no_llseek,
};

/**
* I2C Device 
*/

static int tmf8829_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
    tmf8829_chip *chip;
    int error = 0;
    void *poll_prop_ptr = NULL;

    /* Check I2C functionality */
    dev_info(&client->dev, "I2C Address: %#04x\n", client->addr);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C check functionality failed.\n");
        return -ENXIO;
    }
    
    /* Memory Alloc */
    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        dev_err(&client->dev, "Mem kzalloc failed. \n");
        return -ENOMEM;
    }

    /* Platform Setup */
    mutex_init(&chip->lock);
    mutex_init(&chip->fifo_lock);
    
    chip->client = client;
    i2c_set_clientdata(client, chip);
    chip->pdata = &tmf8829_pdata;
    error = tof_get_gpio_config(chip);
    if (error) {
        dev_err(&client->dev, "Error gpio config.\n");
        goto gpio_err;
    }
    
    if (writePin( chip->pdata->gpiod_enable, 1)) {
        dev_err(&client->dev, "Chip enable failed.\n");
        goto gpio_err;
    }

    /* Setup IRQ Handling */
    poll_prop_ptr = (void *)of_get_property(chip->client->dev.of_node, TOF_PROP_NAME_POLLIO, NULL);
    chip->poll_period = poll_prop_ptr ? be32_to_cpup(poll_prop_ptr) : 0;

    if(chip->poll_period == 0) { /* Use Interrupt I/O */
        if (chip->pdata->gpiod_interrupt) {
            error = tof_request_irq(chip);
            if (error) {
                dev_err(&client->dev, "Interrupt request failed.\n");
                goto gpio_err;
            }
        }
    }
    else { /* Polled I/O */
        chip->app_poll_irq = kthread_run(tmf8829_app_poll_irq_thread, (void *)chip, "tof-irq_poll");
        if (IS_ERR(chip->app_poll_irq)) {
            dev_err(&client->dev, "Error starting IRQ polling thread.\n");
            error = PTR_ERR(chip->app_poll_irq);
            goto gpio_err;
        }
    }

    /* initialize kfifo for frame output */
    INIT_KFIFO(chip->tof_output_fifo);
    chip->tof_output_frame.frame.frameId = TMF8829_COM_RESULT__no_frame;
    chip->tof_output_frame.frame.frameNumber = 0;
    chip->tof_output_frame.frame.clkcorr_lsb = 0;
    chip->tof_output_frame.frame.clkcorr_msb = 0;
    chip->tof_output_frame.frame.reserved_0  = 0x77;
    chip->tof_output_frame.frame.reserved_1  = 0x77;
    chip->tof_output_frame.frame.payload_lsb = 0;
    chip->tof_output_frame.frame.payload_msb = 0;

    /* set bustype; default is I2C, if SPI overlay file is found, SPI is used */
    chip->bustype = BUS_I2C;

    error = tof_register_spi_driver(chip);
    if (error) {
        dev_err(&client->dev, "Error tof_register_spi_driver");
        goto spi_reg_err;
    }

    /* TMF8829 Setup */
    AMS_MUTEX_LOCK(&chip->lock);

    tmf8829Initialise(&chip->tof_core);
    tmf8829SetLogLevel(&chip->tof_core, APP_LOG_LEVEL);
    delayInMicroseconds(ENABLE_TIME_MS * 1000);
    tmf8829PowerUp(&chip->tof_core);

    if (tmf8829IsCpuReady(&chip->tof_core, CPU_READY_TIME_MS) == 0) {
        dev_err(&client->dev, "CPU is not ready.\n");
        error = 5;
        goto gen_err;
    }

    /* switch off unused communication interface */
    if (chip->bustype == BUS_I2C) {
        error = tmf8829BootloaderCmdSpiOff(&chip->tof_core);
    }
    else if  (chip->bustype == BUS_SPI) {
        error = tmf8829BootloaderCmdI2cOff(&chip->tof_core);
    }
    else {
        dev_err(&client->dev, "Bus not supported.\n");
        error = 1;
    }
    if (error != BL_SUCCESS_OK) {
        dev_err(&client->dev, "error disable communication interface .\n");
        goto gen_err;
    }

    error = firmware_download(&client->dev);
    
    if (error != 0) {
        goto gen_err;
    }
    else {
        dev_info(&client->dev, "Firmware download done.\n");
    }

    if (tmf8829GetConfiguration(&chip->tof_core) != APP_SUCCESS_OK) {
        dev_err(&client->dev, "Read device configuration error.\n");
        goto gen_err;
    }

    if (tmf8829ReadDeviceInfo( &chip->tof_core) != APP_SUCCESS_OK) {
        dev_err(&client->dev, "Read device information.\n");
        goto gen_err;
    }
    chip->tof_core.cyclicRunning = 0;
    tmf8829ClrAndEnableInterrupts( &chip->tof_core, TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_HISTOGRAMS );

    AMS_MUTEX_UNLOCK(&chip->lock);

    /* Sysfs Setup */
    error = sysfs_create_groups(&client->dev.kobj, tmf8829_attr_groups);
    if (error) {
        dev_err(&client->dev, "Error creating sysfs attribute group.\n");
        goto sysfs_err;
    }

	/* setup misc char device */
    chip->tof_mdev.fops = &tof_miscdev_fops;
    chip->tof_mdev.name = "tof_tmf8829";
    chip->tof_mdev.minor = MISC_DYNAMIC_MINOR;
	error = misc_register(&chip->tof_mdev);

    if (error) {
        dev_err(&client->dev, "Error registering misc_dev.\n");
        goto misc_reg_err;
    }

    dev_info(&client->dev, "Probe pass.\n");
    return 0;

    /* Probe error handling */
misc_reg_err:
sysfs_err:
    sysfs_remove_groups(&client->dev.kobj, tmf8829_attr_groups);
gen_err:
spi_reg_err:
    spi_unregister_driver(&chip->spi_drv);
    if (chip->poll_period != 0) {
        (void)kthread_stop(chip->app_poll_irq);
    }
gpio_err:
    dev_err(&client->dev, "Probe failed.\n");
    enablePinLow(chip);
    i2c_set_clientdata(client, NULL);
    AMS_MUTEX_UNLOCK(&chip->lock);

    return error;
}

static void tmf8829_remove(struct i2c_client *client)
{
    tmf8829_chip *chip = i2c_get_clientdata(client);

    tmf8829StopMeasurement(&chip->tof_core);

    if (chip->pdata->gpiod_interrupt != 0 && (PTR_ERR(chip->pdata->gpiod_interrupt) != -EBUSY)) {
        dev_info(&client->dev, "clear gpio irqdata %s\n", __func__);
        devm_free_irq(&client->dev, client->irq, chip);
        dev_info(&client->dev, "put %s\n", __func__);
        devm_gpiod_put(&client->dev, chip->pdata->gpiod_interrupt);
    }
    if (chip->poll_period != 0) {
        (void)kthread_stop(chip->app_poll_irq);
    }
    if (chip->pdata->gpiod_enable) {
        dev_info(&client->dev, "clear gpio enable %s\n", __func__);
        gpiod_direction_output(chip->pdata->gpiod_enable, 0);
        devm_gpiod_put(&client->dev, chip->pdata->gpiod_enable);
    }
    misc_deregister(&chip->tof_mdev);
    dev_info(&client->dev, "clear sys attr %s\n", __func__);
    sysfs_remove_groups(&client->dev.kobj, tmf8829_attr_groups);
    dev_info(&client->dev, "%s\n", __func__);
    i2c_set_clientdata(client, NULL);

    spi_unregister_driver(&chip->spi_drv);
    return;
}

/**************************************************************************/
/* Linux Driver Specific Code                                             */
/**************************************************************************/
/** I2C **/
static struct i2c_device_id tmf8829_idtable[] = {
    { "tmf8829", 0 },
    { }
};

static const struct of_device_id tmf8829_of_match[] = {
    { .compatible = "ams,tmf8829" },
    { }
};

MODULE_DEVICE_TABLE(i2c, tmf8829_idtable);
MODULE_DEVICE_TABLE(of, tmf8829_of_match);

static struct i2c_driver tmf8829_driver = {
    .driver = {
        .name = "ams-OSRAM tmf8829",
        .of_match_table = of_match_ptr(tmf8829_of_match),
    },
    .id_table = tmf8829_idtable,
    .probe = tmf8829_probe,
    .remove = tmf8829_remove,
};

module_i2c_driver(tmf8829_driver);

/** SPI **/
static const struct of_device_id tof_spi_ids[] = {
    { .compatible = "ams,tmf8829_spi" },
    { }
};

MODULE_DEVICE_TABLE(of, tof_spi_ids);

static int tof_spi_probe(struct spi_device *spi)
{
    int ret;
    tmf8829_chip* chip = container_of(spi->dev.driver, tmf8829_chip, spi_drv.driver);
    spi->bits_per_word = 8;
    spi->mode = 0;

    ret = spi_setup(spi);
    if (ret < 0) {
        return ret;
    }
    if (spi == NULL) {
        dev_err(&chip->client->dev, "Error tof_spi_probe failed");
    } else {
        chip->spi_dev = spi;
        dev_info(&chip->client->dev, "tof_spi_probe SUCCESS");
    }
    
    chip->bustype = BUS_SPI;
        
    return 0;
}

int tof_register_spi_driver(tmf8829_chip *tof_chip)
{
    struct spi_driver *spidrv = (struct spi_driver *)&tof_chip->spi_drv;
    spidrv->driver.name = "tmf8829_spi";
    spidrv->probe = tof_spi_probe;
    spidrv->driver.of_match_table = tof_spi_ids;
    return spi_register_driver(spidrv);
};

/** MODULE **/

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ams-OSRAM AG TMF8829 ToF sensor driver");
MODULE_VERSION("2.1");
