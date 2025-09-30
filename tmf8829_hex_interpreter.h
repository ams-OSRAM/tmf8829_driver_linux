/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/** @file This is the hex interpreter for FW Download
 */

#ifndef HEX_INTERPRETER_H
#define HEX_INTERPRETER_H
// ---------------------------------------------- includes ----------------------------------------
#include <linux/types.h>
#include "tmf8829_driver.h"
// ---------------------------------------------- defines -----------------------------------------

/* supported intel hex record types: */
#define INTEL_HEX_TYPE_DATA                     0
#define INTEL_HEX_TYPE_EOF                      1
#define INTEL_HEX_TYPE_EXT_LIN_ADDR             4
#define INTEL_HEX_TYPE_START_LIN_ADDR           5

/* return codes: negative numbers are errors */
#define INTEL_HEX_EOF                   1       /* end of file -> reset */
#define INTEL_HEX_CONTINUE              0       /* continue reading in */
#define INTEL_HEX_ERR_NOT_A_NUMBER      -1
#define INTEL_HEX_ERR_TOO_SHORT         -2
#define INTEL_HEX_ERR_CRC_ERR           -3
#define INTEL_HEX_ERR_UNKNOWN_TYPE      -4
#define INTEL_HEX_WRITE_FAILED          -5
#define READ_ERROR                      -6

/* get the ULBA from a 32-bit address */
#define INTEL_HEX_ULBA( adr )         ( (adr) & 0xFFFF0000UL )

/* an intel hex record always has at least:
    :llaaaattcc
   l= length, a= address, t= type, c= crc
   so we need at least 11 ascii uint8_tacters for 1 regular record */
#define INTEL_HEX_MIN_RECORD_SIZE   11
/* the lineLength is at the beginning not the length but the last written
   address. To make out of this the length we substract instead
   of the min_record_size the min_last_address. */
#define INTEL_HEX_MIN_LAST_ADDRESS  ((INTEL_HEX_MIN_RECORD_SIZE) - 1 )

/* an intel hex record has a length field in that only 256 = an 8-bit number
 * can be represented, so we can limit the data buffer to half the size,
 * as it converts from ascii to binary */
#define INTEL_HEX_MAX_RECORD_DATA_SIZE  (128)

// ---------------------------------------------- image  ---------------------------------------
#define MAX_PATCH_IMAGE_SIZE 0x4000
extern unsigned char patchImage[MAX_PATCH_IMAGE_SIZE];
extern int imageSize;
// ---------------------------------------------- functions ---------------------------------------
/**
 * intelHexInterpreterInitialise
 * initialise before hex file is read
 */
void intelHexInterpreterInitialise ( void );

/**
 * intelHexHandleRecord
 * hand in the line of intel hex record, returns any of the above
 * return codes (1 == reset of tof done)
 * fills the patchImage with data and updates the imageSize
 * @tmf8829_chip: tmf8829_chip pointer
 * @lineLength: lineLength of hex file
 * @line: line of hex file
 * Returns 0 for no Error, otherwise Error
 */

uint8_t intelHexHandleRecord ( tmf8829_chip *chip, uint8_t lineLength, const uint8_t * line );

#endif /* HEX_INTERPRETER_H */
