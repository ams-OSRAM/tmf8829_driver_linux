/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/*
 * History of Versions:
 * 1.0 ... first release version
 * 2.0 ... release for ROM 1v1 Parts
 *     ... update to new core driver
 *     ... some clean up
 *     ... chip_enable_store: start ROM or RAM application(with patch)
 *     ... request_ram_patch_store, execute a Get configuration
 *     ... switch off unused communication interface
 * 2.1 ... clock correction bugfix 125kHz instead of 128kHz
 *     ... remove unused ROM start functionality
 *     ... clock correction
 *     ... probe error handling changed
 */

/*! \file tmf8829_driver.h - TMF8829 linux driver
 * \brief Device driver for measuring distance in mm.
 */

#ifndef TMF8829_DRIVER_H
#define TMF8829_DRIVER_H

/* -------------------------------- includes -------------------------------- */
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

#include "tmf8829.h"
#include "tmf8829_shim.h"

/* -------------------------------- defines --------------------------------- */
#define MAX_REGS              256                    // i2c max accessible registers
#define TOF_GPIO_INT_NAME     "irq"                  // pin from overlay file
#define TOF_GPIO_ENABLE_NAME  "enable"               // pin from overlay file
#define TOF_PROP_NAME_POLLIO  "tof,tof_poll_period"  // property in overlay file
#define MAX_PAYLOAD_SIZE      (PAGE_SIZE * 8 - 2000) // enough for 2 histogram frames 
#define DRIVER_HEADER_SIZE    8                      // header for every output frame

#define BUS_I2C               1
#define BUS_SPI               2

/* -------------------------------- macros ---------------------------------- */
#define AMS_MUTEX_LOCK(m) { \
    mutex_lock(m); \
  }
#define AMS_MUTEX_UNLOCK(m) { \
    mutex_unlock(m); \
  }
/* -------------------------------- structures ------------------------------ */
struct tmf8829_output {
    char frameId;
    char frameNumber;
    char clkcorr_lsb;
    char clkcorr_msb;
    char reserved_0;
    char reserved_1;
    char payload_lsb;
    char payload_msb;
    char payload[MAX_PAYLOAD_SIZE];
};

union tmf8829_output_frame {
    struct tmf8829_output frame;
    char buf[sizeof(struct tmf8829_output)];
}__attribute__((packed));

struct tmf8829_platform_data {
    const char *tof_name;
    struct gpio_desc *gpiod_interrupt;
    struct gpio_desc *gpiod_enable;
    const char *ram_patch_fname[];
};

typedef struct _tmf8829_chip
{
    tmf8829Driver tof_core; // first item in the structure!! pointer to this stucture is pointer to tmf8829_chip structure

    struct i2c_client *client;
    struct spi_device *spi_dev;
    struct spi_driver spi_drv;
    struct miscdevice tof_mdev;
    struct mutex lock;
    struct mutex fifo_lock;
    struct tmf8829_platform_data *pdata;
    uint8_t bustype;
    uint8_t shadow[256];
    STRUCT_KFIFO_REC_2(PAGE_SIZE * 32) tof_output_fifo;
    struct task_struct *app_poll_irq;
    int poll_period;
    union tmf8829_output_frame tof_output_frame;
}tmf8829_chip;

/* -------------------------------- functions ------------------------------- */
extern int tof_register_spi_driver(tmf8829_chip *tof_chip);

/**
 * tof_queue_frame - queue data of buffer into output fifo
 *
 * @tmf8829_chip: tmf8829_chip pointer
 *
 * Returns 0 for no Error, -1 for Error
 */
int tof_queue_frame(tmf8829_chip *chip);

#endif
