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

#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/math64.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>

#include "tmf8829_driver.h"
#include "tmf8829_shim.h"

int readPin ( struct gpio_desc *gpiod )
{
    return(gpiod_get_value(gpiod) ? 1 : 0);
}

int writePin ( struct gpio_desc *gpiod, uint8_t value )
{
    return gpiod_direction_output(gpiod, value);
}

void delayInMicroseconds ( uint32_t wait )
{
    usleep_range(wait, wait + wait/10);
}

uint32_t getSysTick ( void ) //Note: is only for 70 minutes
{
    u64 ktime_us = div_u64(ktime_get_ns(), NSEC_PER_USEC);
    return (uint32_t)ktime_us;
}

uint8_t readProgramMemoryByte ( uint32_t address )
{
    uint32_t *ptr = (void *) address;
    uint8_t byte = *ptr;
    return byte;
}

int8_t txReg ( void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t *txData )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
    {
      return i2cTxReg(driver, 0, regAddr, toTx, txData);
    }
    else if (driver->bustype == BUS_SPI)
    {
      return spiTxReg(driver, regAddr , txData, toTx);
    }
    else
    {
        return UNSUPPORTED_BUS_ERROR;
    }
}

int8_t rxReg ( void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t *rxData )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
    {
      return i2cRxReg(driver, 0, regAddr, toRx, rxData);
    }
    else if (driver->bustype == BUS_SPI)
    {
      return spiRxReg(driver, regAddr, rxData, toRx);
    }
    else
    {
        return UNSUPPORTED_BUS_ERROR;
    }
}

int8_t regWriteMask ( void * dptr, uint8_t slaveAddr, char reg, char val, char mask)
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    if (driver->bustype == BUS_I2C)
    {
      return i2c_write_mask(driver->client, reg, val, mask);
    }
    else if (driver->bustype == BUS_SPI)
    {
      return spi_write_mask(driver, reg, val, mask);
    }
    else
    {
        return UNSUPPORTED_BUS_ERROR;
    }
}

int spiTxReg ( void *dptr, uint8_t regAddr, const char *data, size_t len )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    unsigned char *wrbuf;
    int ret = 0;

    wrbuf = kmalloc(len + 1, GFP_KERNEL);
    if (!wrbuf)
        return -ENOMEM;

    wrbuf[0] = SPI_WR_CMD;
    wrbuf[1] = regAddr;

    memcpy(&wrbuf[2], data, len);

    ret = spi_write(driver->spi_dev, wrbuf, len + 2);
    if (ret < 0)
    {
        dev_err(&driver->client->dev, "Error: %d\n", ret);
    }

    kfree(wrbuf);

    return ret;
}

int spiRxReg ( void *dptr, uint8_t regAddr, void *rxData, size_t len )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;

    uint8_t *buf = (uint8_t*)rxData;
    unsigned char wrbuf[3];
    int ret = 0;

    wrbuf[0] = SPI_RD_CMD;
    wrbuf[1] = regAddr;
    wrbuf[2] = 0;          // dummy

    ret = spi_write_then_read(driver->spi_dev, &wrbuf, 3, buf, len);

    if (ret < 0)
    {
        dev_err(&driver->client->dev, "Error, tof_spi_read_func failed\n");
        return ret;
    }
    return ret;
}

int spi_write_mask ( void *dptr, char reg, char val, char mask )
{
  int ret;
  unsigned char temp;

  ret = spiRxReg(dptr, reg, &temp, 1);

  if (ret == 0 )
  { 
    temp &= ~mask;
    val &= mask;
    temp |= val;
    ret = spiTxReg(dptr, reg, &temp, 1);
  }
  
  return ret;
}

int8_t i2cTxReg ( void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t *txData )
{  
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    int error;
    char debug[120]; //max for debug
    int idx;
    u32 strsize = 0;

    error = i2c_write(driver->client, regAddr, txData, toTx);
    if ( error)
    {
        return I2C_ERROR;
    }

    if ( driver->tof_core.logLevel & TMF8829_LOG_LEVEL_I2C ) 
    { 
        strsize = scnprintf(debug, sizeof(debug), "i2cTx: Reg:%02x Len:%hx Dat: ", regAddr, toTx);
        for(idx = 0; idx < toTx; idx++)
        {
            strsize += scnprintf(debug + strsize, sizeof(debug) - strsize, "%02x ", txData[idx]);
        }
        dev_info(&driver->client->dev, "%s", debug);
    }

    return I2C_SUCCESS;
}

int8_t i2cRxReg ( void *dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t *rxData )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    int error;
    char debug[120]; //max for debug
    int idx;
    u32 strsize = 0;

    error = i2c_read(driver->client, regAddr, rxData, toRx);
    
    if ( error)
    {
        return I2C_ERROR;
    }
    
    if ( driver->tof_core.logLevel & TMF8829_LOG_LEVEL_I2C ) 
    {
        strsize = scnprintf(debug, sizeof(debug), "i2cRx: Reg:%02x Len:%hx Dat:", regAddr, toRx);
        for(idx = 0; idx < toRx; idx++)
        {
            strsize += scnprintf(debug + strsize, sizeof(debug) - strsize, "%02x ", rxData[idx]);
        }
        dev_info(&driver->client->dev, " %s", debug);
    }

    return I2C_SUCCESS;
}

int8_t i2cTxRx( void *dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t *txData, uint16_t toRx, uint8_t *rxData )
{
  return I2C_ERROR;  //not implemented
}

int enablePinHigh ( void *dptr )
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    return writePin( driver->pdata->gpiod_enable, 1);
}

int enablePinLow ( void *dptr ) 
{
    tmf8829_chip *driver = (tmf8829_chip *)dptr;
    return writePin( driver->pdata->gpiod_enable, 0);
}

int tmf8829_app_process_irq( void *dptr )
{
    uint8_t intStatus = 0;
    int8_t res = APP_SUCCESS_OK;
    int inthandled = 0;
    tmf8829_chip *driver = (tmf8829_chip *)dptr ;

    intStatus = tmf8829GetAndClrInterrupts( &driver->tof_core, TMF8829_APP_INT_ALL );

    if ( driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_RESULTS )
    {
        PRINT_STR( "Int:" );
        PRINT_INT(intStatus);
        PRINT_CHAR( SEPARATOR );
    }

    if ( intStatus & TMF8829_APP_INT_RESULTS )   // check if a result is available
    {  
        res = tmf8829ReadResults( &driver->tof_core );
        if (res == APP_SUCCESS_OK)
        { 
            tof_queue_frame(driver);
        }
      inthandled = 1;
    }

    if ( intStatus & TMF8829_APP_INT_MOTION )
    { 
      /*********************************************************************************************/
      /* This MOTION DETECTION interrupt needs to be implemented by the customer if required !!!   */
      /*********************************************************************************************/
      if ( driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_DEBUG )
      {
        PRINT_STR( "MOTION" );
        PRINT_LN();
      }
      inthandled = 1;
    }

    if ( intStatus & TMF8829_APP_INT_PROXIMITY )
    {
      /*********************************************************************************************/
      /* This PROXIMITY DETECTION interrupt needs to be implemented by the customer if required !!!*/
      /*********************************************************************************************/
      if ( driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_DEBUG )
      {
        PRINT_STR( "PROX" );
        PRINT_LN();
        
      }
      inthandled = 1;
    }

    if ( intStatus & TMF8829_APP_INT_HISTOGRAMS )
    {
        res = tmf8829ReadHistogram( &driver->tof_core );
        if (res == APP_SUCCESS_OK)
        { 
            tof_queue_frame(driver);
        }

      inthandled = 1;
    }

    if (res != APP_SUCCESS_OK)
    { 
        driver->tof_output_frame.frame.frameId = TMF8829_COM_RESULT__error_frame;
        driver->tof_output_frame.frame.frameNumber++;
        driver->tof_output_frame.frame.payload_lsb = 1;
        driver->tof_output_frame.frame.payload_msb = 0;
        driver->tof_output_frame.frame.clkcorr_lsb = 0xCC;
        driver->tof_output_frame.frame.clkcorr_msb = 0xCC;
        driver->tof_output_frame.frame.payload[0] = TMF8829_COM_ERROR_eof;
    }
    return inthandled;
}

void handleReceivedFrameHeaderData ( void * dptr, uint8_t *data )
{
  tmf8829_chip *driver = (tmf8829_chip *)dptr;  
  if ( driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_RESULTS_HEADER )
  {
    printResultHeader( &driver->tof_core, data, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE );
  }
  driver->tof_output_frame.frame.frameId = TMF8829_COM_RESULT__measurement_header;
  driver->tof_output_frame.frame.payload_lsb = TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE ;
  driver->tof_output_frame.frame.payload_msb = 0;
  driver->tof_output_frame.frame.clkcorr_lsb = 0xEE;
  driver->tof_output_frame.frame.clkcorr_msb = 0xEE;
  memcpy(&driver->tof_output_frame.frame.payload[0], data, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE );

}

void handleReceivedResultData ( void *dptr, uint8_t *data, uint16_t size )
{
  tmf8829_chip *driver = (tmf8829_chip *)dptr;  
  uint16_t payload_size = driver->tof_output_frame.frame.payload_msb << 8;
  payload_size += driver->tof_output_frame.frame.payload_lsb;

  if ( driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_RESULTS )
  {
    printResults( &driver->tof_core, data, 32 ); // limited length do not print all data
  }
  if (size <= MAX_PAYLOAD_SIZE)
  {
    memcpy(&driver->tof_output_frame.frame.payload[payload_size], data, size );
    payload_size += size;
    driver->tof_output_frame.frame.frameId = TMF8829_COM_RESULT__measurement_res_frame;
    driver->tof_output_frame.frame.payload_lsb = payload_size & 0x00FF;
    driver->tof_output_frame.frame.payload_msb = (payload_size & 0xFF00) >> 8;
    driver->tof_output_frame.frame.clkcorr_lsb = driver->tof_core.clkCorrRatioUQ & 0xFF;
    driver->tof_output_frame.frame.clkcorr_msb = (driver->tof_core.clkCorrRatioUQ >> 8 ) & 0xFF;
  }
  else
  {
    PRINT_STR( "Payload to big" );
    PRINT_INT( payload_size );
    PRINT_CHAR( SEPARATOR );
  }
}

void handleReceivedHistogramData( void *dptr, uint8_t *data, uint16_t size )
{
  tmf8829_chip *driver = (tmf8829_chip *)dptr;
  uint16_t payload_size = driver->tof_output_frame.frame.payload_msb << 8;
  payload_size += driver->tof_output_frame.frame.payload_lsb;

  if ( driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_RESULTS )
  {
    printHistogram( &driver->tof_core, data, 32 ); // limited length do not print all data
  }
  if (size <= MAX_PAYLOAD_SIZE)
  {
    memcpy(&driver->tof_output_frame.frame.payload[payload_size], data, size );
    payload_size += size;
    driver->tof_output_frame.frame.frameId = TMF8829_COM_RESULT__measurement_hist_frame;
    driver->tof_output_frame.frame.payload_lsb = payload_size & 0x00FF;
    driver->tof_output_frame.frame.payload_msb = (payload_size & 0xFF00) >> 8;
    driver->tof_output_frame.frame.clkcorr_lsb = 0xFF;
    driver->tof_output_frame.frame.clkcorr_msb = 0xFF;
  }
  else
  {
    PRINT_STR( "Payload to big" );
    PRINT_INT( payload_size );
    PRINT_CHAR( SEPARATOR );
  }
}

void handleReceivedResultDataEnd( void *dptr )
{
  tmf8829_chip *driver = (tmf8829_chip *)dptr;

  if (driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_RESULTS_HEADER )
  {
    PRINT_LN( );
  }

}

void handleReceivedHistogramDataEnd( void *dptr )
{
  tmf8829_chip *driver = (tmf8829_chip *)dptr;

  if (driver->tof_core.logLevel >= TMF8829_LOG_LEVEL_RESULTS_HEADER )
  {
    PRINT_LN( );
  }

}

// Result Header printing:
// #Obj,<i2c_slave_address>,<fifostatus>,<systick>,
//      <frame_identifier>,<result_layout>,<payload>,<frameNumber>,
//      <temperature0>,<temperature1>,<temperature2>,<bdv_value>,<ref_peak_position1>, <ref_peak_position2>
void printResultHeader ( void *dptr, uint8_t *data, uint8_t len )
{
  tmf8829_chip *driver = (tmf8829_chip *)dptr;

  if ( len == (TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE) )
  {
    uint32_t sysTick = tmf8829GetUint32( data + 1 );
    uint16_t payload = tmf8829GetUint16( data + TMF8829_PRE_HEADER_SIZE + 2 );
    uint32_t frameNum = tmf8829GetUint32( data + TMF8829_PRE_HEADER_SIZE + 4 );
    uint16_t refPos1 = tmf8829GetUint16( data + TMF8829_PRE_HEADER_SIZE + 12 );
    uint16_t refPos2 = tmf8829GetUint16( data + TMF8829_PRE_HEADER_SIZE + 14 );
    PRINT_STR( "#Obj" );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( driver->tof_core.i2cSlaveAddress );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ 0 ] );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( sysTick );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ TMF8829_PRE_HEADER_SIZE ] ); //ID
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ TMF8829_PRE_HEADER_SIZE + 1 ] ); // Layout
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( payload ); // Payload
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( frameNum ); // Frame Number
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ TMF8829_PRE_HEADER_SIZE + 8 ] ); // Temp0
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ TMF8829_PRE_HEADER_SIZE + 9 ] ); // Temp1
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ TMF8829_PRE_HEADER_SIZE + 10 ] ); // Temp2
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ TMF8829_PRE_HEADER_SIZE + 11 ] ); // BDV
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( refPos1 ); // Ref Pos 1
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( refPos2 ); // Ref Pos 2
  }
  else // result structure too short
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPARATOR );
    PRINT_STR( "header size length wrong" );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( len );
    PRINT_LN( );
  }
}

void printResults ( void *dptr, uint8_t *data, uint16_t len )
{
  uint16_t cnt;

  for ( cnt = 0 ; cnt < len ; cnt ++ )
  {
    PRINT_INT( data[cnt] );
    PRINT_CHAR( SEPARATOR );
  }
}

void printHistogram ( void *dptr, uint8_t *data, uint16_t len )
{
  uint16_t cnt;
  
  for ( cnt = 0 ; cnt < len ; cnt ++ )
  {
    PRINT_INT( data[cnt] );
    PRINT_CHAR( SEPARATOR );
  }
  PRINT_LN( );
}
