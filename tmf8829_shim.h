/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/** @file This is the shim for raspberry pi
 * Defines, macro and functions to match the target platform.
 */

#ifndef TMF8829_SHIM_EVM_H
#define TMF8829_SHIM_EVM_H

// ---------------------------------------------- includes ----------------------------------------

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include "ams_i2c.h"

// ---------------------------------------------- defines -----------------------------------------

#define ENABLE_PIN                     2
#define INTERRUPT_PIN                  1
#define DATA_BUFFER_SIZE               30000

// ---------------------------------------------- macros ------------------------------------------

#define PROGMEM 

/** @brief macros to cast a pointer to an address - adapt for your machine-word size
 */ 
#define PTR_TO_UINT(ptr)               ( (uint32_t)(ptr) )

/** @brief macros to replace the platform specific printing
 */ 
#define PRINT_CHAR(c)                  pr_cont( "%c", c )
#define PRINT_INT(i)                   pr_cont( "%ld", (long)i )
#define PRINT_UINT(i)                  pr_cont( "%lu", (unsigned long)i )
#define PRINT_UINT_HEX(i)              pr_cont( "%lX", (unsigned long)i )
#define PRINT_STR(str)                 pr_cont( "%s", str )
#define PRINT_LN()                     pr_cont( "\n" )

/** Which character to use to separate the entries in printing */
#define SEPARATOR                      ','

// for clock correction insert here the number in relation to your host
#define HOST_TICKS_PER_1000_US         1000     /**< number of host ticks every 1000 microseconds */  
#define TMF8829_TICKS_PER_1000_US      125      /**< number of tmf8829 ticks every 1000 microseconds  125kHz */ 

// ---------------------------------------------- functions ---------------------------------------

int readPin( struct gpio_desc *gpiod );

int writePin( struct gpio_desc *gpiod, uint8_t value );

// ---------------------------------------------- functions ---------------------------------------

/** @brief Function to allow to wait for some time in microseconds
 *  @param[in] wait number of microseconds to wait before this function returns
 */
void delayInMicroseconds( uint32_t wait );

/** @brief Function returns the current sys-tick.
 * \return current system tick
 */
uint32_t getSysTick( void );

/** @brief Function reads a single byte from the given address. This is only needed on 
 * systems that have special memory access methods for constant segments. Like e.g. Arduino Uno
 *  @param[in] address absolute memory address to read from
 * \return single byte from the given address 
 */
uint8_t readProgramMemoryByte( uint32_t address );

/** @brief Function sets the enable pin HIGH. Note that the enable pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the enable pin, can
 * be 0-pointer if the function does not need it
 */
int enablePinHigh( void * dptr );

/** @brief Function sets the enable pin LOW. Note that the enable pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the enable pin, can
 * be 0-pointer if the function does not need it
 */
int enablePinLow( void * dptr );

// ---------------------------------- write / read functions ------------------------------------
#define UNSUPPORTED_BUS_ERROR       -EIO      /**< device communication error, handled as linux I/O error */

/** @brief Transmit only function.
 * The used communication interface is stored in the driver structure.
 * @param[in] dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param[in] slaveAddr the i2c slave address to be used (7-bit unshifted)
 * @param[in] regAddr the register to start writing to
 * @param[in] toTx number of bytes in the buffer to transmit
 * @param[in] txData pointer to the buffer to transmit
 * \return 0 when successfully transmitted, else an error code
 */ 
int8_t txReg( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData );

/** @brief Transmit register address and receive function.
 * The used communication interface is stored in the driver structure.
 * @param[in] dptr a pointer to a data structure the function needs for receiving, can
 * be 0-pointer if the function does not need it
 * @param slaveAddr the i2c slave address to be used (7-bit)
 * @param regAddr the register address to start reading from
 * @param toRx number of bytes in the buffer to receive
 * @param rxData pointer to the buffer to be filled with received bytes
 * \return 0 when successfully received, else an error code
 */ 
int8_t rxReg( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData );

/** @brief Modify a Register function.
 * The used communication interface is stored in the driver structure.
 * @param[in] dptr a pointer to a data structure the function needs for receiving, can
 * be 0-pointer if the function does not need it
 * @param slaveAddr the i2c slave address to be used (7-bit)
 * @param reg the register address
 * @param val the value
 * @param mask the mask
 * \return 0 when successfully received, else an error code
 */ 
int8_t regWriteMask( void * dptr, uint8_t slaveAddr, char reg, char val, char mask);

// ---------------------------------- SPI functions ---------------------------------------------
/**  Return codes for spi functions: 
 */
#define SPI_SUCCESS             0       /**< successful execution no error */
#define SPI_ERROR              -1       /**< spi error */

/** @brief SPI transmit only function.
 * @param[in] dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param[in] regAddr the register to start writing to
 * @param[in] data pointer to the buffer to transmit
 * @param[in] len number of bytes in the buffer to transmit
 * \return 0 when successfully transmitted, else an error code
 */ 
int spiTxReg( void * dptr, uint8_t regAddr , const char *data, size_t len );

/** @brief SPI transmit register address and receive function.
 * @param[in] dptr a pointer to a data structure the function needs for receiving, can
 * be 0-pointer if the function does not need it
 * @param regAddr the register address to start reading from
 * @param rxData pointer to the buffer to be filled with received bytes
 * @param len number of bytes in the buffer to receive
 * \return 0 when successfully received, else an error code
 */ 
int spiRxReg( void * dptr, uint8_t regAddr, void *rxData, size_t len );

/** @brief  SPI - Write a byte to the specified address with a given bitmask.
 * @param[in] dptr a pointer to a data structure the function needs for receiving, can
 * be 0-pointer if the function does not need it
 * @param reg the register address
 * @param val the value
 * @param mask the mask
 * \return 0 when successfully received, else an error code
 */ 
int spi_write_mask( void * dptr, char reg, char val, char mask );

// ---------------------------------- I2C functions ---------------------------------------------

/**  Return codes for i2c functions: 
 */
#define I2C_SUCCESS             0       /**< successful execution no error */
#define I2C_ERROR              -1       /**< i2c error */

/** There are 2 styles of functions available:
 * 1. those that always require a register address to be specified: i2cTxReg, i2cRxReg
 * 2. the more generic (more I2C standard like) that can transmit and/or receive (here the
 *  register address if needed is the first transmitted byte): i2cTxRx
 * Only one set of those two *have to be* available. Both can be available.
 */

/** @brief I2C transmit only function.
 * @param[in] dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param[in] slaveAddr the i2c slave address to be used (7-bit unshifted)
 * @param[in] regAddr the register to start writing to
 * @param[in] toTx number of bytes in the buffer to transmit
 * @param[in] txData pointer to the buffer to transmit
 * \return 0 when successfully transmitted, else an error code
 */ 
int8_t i2cTxReg( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData );

/** @brief I2C transmit register address and receive function.
 * @param[in] dptr a pointer to a data structure the function needs for receiving, can
 * be 0-pointer if the function does not need it
 * @param slaveAddr the i2c slave address to be used (7-bit)
 * @param regAddr the register address to start reading from
 * @param toRx number of bytes in the buffer to receive
 * @param rxData pointer to the buffer to be filled with received bytes
 * \return 0 when successfully received, else an error code
 */ 
int8_t i2cRxReg( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData );

/** @brief I2C transmit and receive function.
 * @param dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param slaveAddr the i2c slave address to be used (7-bit)
 * @param toTx number of bytes in the buffer to transmit (set to 0 if receive only)
 * @param txData pointer to the buffer to transmit
 * @param toRx number of bytes in the buffer to receive (set to 0 if transmit only)
 * @param rxData pointer to the buffer to be filled with received bytes
 * \return 0 when successfully transmitted and received, else an error code
 */ 
int8_t i2cTxRx( void * dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t * txData, uint16_t toRx, uint8_t * rxData );

/* --------------------- functions used by the application only (not driver) -------------------------------- */

/** @brief Process the irq, readout of data
 * @param dptr ... a pointer to a data structure the function needs for transmitting
 * @return 0.. no data read, 1.. data read
 */ 
int tmf8829_app_process_irq( void * dptr );

/** @brief Function to handle the received frame header data 
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 *  @param[in] data ... pointer to the result header structure as defined for tmf8829
 */
void handleReceivedFrameHeaderData ( void * dptr, uint8_t * data );

/** @brief Function to handle the received result frame data
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 *  @param[in] data ... pointer to the result structure as defined for tmf8829
 *  @param[in] size ... number of bytes the data pointer points to
 */
void handleReceivedResultData( void * dptr, uint8_t * data, uint16_t size );

/** @brief Function to handle the received histogram frame data 
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 *  @param[in] data ... pointer to the result structure as defined for tmf8829
 *  @param[in] size ... number of bytes the data pointer points to
 */
void handleReceivedHistogramData( void * dptr, uint8_t * data, uint16_t size );

/** @brief Function to handle the end of a result frame
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 */
void handleReceivedResultDataEnd( void * dptr );

/** @brief Function to handle the end of a histogram frame
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 */
void handleReceivedHistogramDataEnd( void * dptr );

/** @brief Function to print the frame header in a kind of CSV like format
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 *  @param[in] data ... pointer to the result header structure as defined for tmf8829
 *  @param[in] len ... number of bytes the data pointer points to
 */
void printResultHeader( void * dptr, uint8_t * data, uint8_t len );

/** @brief Function to print the results in a kind of CSV like format
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 *  @param[in] data ... pointer to the result structure as defined for tmf8829
 *  @param[in] len ... number of bytes the data pointer points to
 */
void printResults( void * dptr, uint8_t * data, uint16_t len );

/** @brief Function to print a histogram part in a kind of CSV like format 
 * @param[in] dptr a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 *  @param[in] data ... pointer to the result structure as defined for tmf8829
 *  @param[in] len ... number of bytes the data pointer points to
 */
void printHistogram( void * dptr, uint8_t * data, uint16_t len );

#endif /* TMF8829_SHIM_EVM_H */
