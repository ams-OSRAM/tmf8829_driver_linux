/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

/* simple TMF8829 driver */

// --------------------------------------------------- includes ---------------------------------------------------
#include "tmf8829_shim.h"
#include "tmf8829.h"

// --------------------------------------------------- defines for HW registers   ---------------------------------
#define I2C_DEVADDR 0xe0
#define I2C_DEVADDR__i2c_devaddr__SHIFT 1
#define I2C_DEVADDR__i2c_devaddr__WIDTH 7
#define I2C_DEVADDR__i2c_devaddr__RESET 65
#define I2C_DEVADDR__enab_ack_hdr_write__SHIFT 0
#define I2C_DEVADDR__enab_ack_hdr_write__WIDTH 1
#define I2C_DEVADDR__enab_ack_hdr_write__RESET 0
#define INT_STATUS 0xe1
#define INT_STATUS__int7__SHIFT 7
#define INT_STATUS__int7__WIDTH 1
#define INT_STATUS__int7__RESET 0
#define INT_STATUS__int6__SHIFT 6
#define INT_STATUS__int6__WIDTH 1
#define INT_STATUS__int6__RESET 0
#define INT_STATUS__int5__SHIFT 5
#define INT_STATUS__int5__WIDTH 1
#define INT_STATUS__int5__RESET 0
#define INT_STATUS__int4__SHIFT 4
#define INT_STATUS__int4__WIDTH 1
#define INT_STATUS__int4__RESET 0
#define INT_STATUS__int3__SHIFT 3
#define INT_STATUS__int3__WIDTH 1
#define INT_STATUS__int3__RESET 0
#define INT_STATUS__int2__SHIFT 2
#define INT_STATUS__int2__WIDTH 1
#define INT_STATUS__int2__RESET 0
#define INT_STATUS__int1__SHIFT 1
#define INT_STATUS__int1__WIDTH 1
#define INT_STATUS__int1__RESET 0
#define INT_STATUS__int0__SHIFT 0
#define INT_STATUS__int0__WIDTH 1
#define INT_STATUS__int0__RESET 0
#define INT_ENAB 0xe2
#define INT_ENAB__int7_enab__SHIFT 7
#define INT_ENAB__int7_enab__WIDTH 1
#define INT_ENAB__int7_enab__RESET 0
#define INT_ENAB__int6_enab__SHIFT 6
#define INT_ENAB__int6_enab__WIDTH 1
#define INT_ENAB__int6_enab__RESET 0
#define INT_ENAB__int5_enab__SHIFT 5
#define INT_ENAB__int5_enab__WIDTH 1
#define INT_ENAB__int5_enab__RESET 0
#define INT_ENAB__int4_enab__SHIFT 4
#define INT_ENAB__int4_enab__WIDTH 1
#define INT_ENAB__int4_enab__RESET 0
#define INT_ENAB__int3_enab__SHIFT 3
#define INT_ENAB__int3_enab__WIDTH 1
#define INT_ENAB__int3_enab__RESET 0
#define INT_ENAB__int2_enab__SHIFT 2
#define INT_ENAB__int2_enab__WIDTH 1
#define INT_ENAB__int2_enab__RESET 0
#define INT_ENAB__int1_enab__SHIFT 1
#define INT_ENAB__int1_enab__WIDTH 1
#define INT_ENAB__int1_enab__RESET 0
#define INT_ENAB__int0_enab__SHIFT 0
#define INT_ENAB__int0_enab__WIDTH 1
#define INT_ENAB__int0_enab__RESET 0
#define ID 0xe3
#define ID__device_id__SHIFT 0
#define ID__device_id__WIDTH 8
#define ID__device_id__RESET 158
#define REVID 0xe4
#define REVID__rev_id__SHIFT 0
#define REVID__rev_id__WIDTH 3
#define REVID__rev_id__RESET 0
#define RESET 0xf7
#define RESET__hard_reset__SHIFT 7
#define RESET__hard_reset__WIDTH 1
#define RESET__hard_reset__RESET 0
#define RESET__soft_reset__SHIFT 6
#define RESET__soft_reset__WIDTH 1
#define RESET__soft_reset__RESET 0
#define RESET__reset_reason_coldstart__SHIFT 5
#define RESET__reset_reason_coldstart__WIDTH 1
#define RESET__reset_reason_coldstart__RESET 1
#define RESET__reset_reason_hard_reset__SHIFT 4
#define RESET__reset_reason_hard_reset__WIDTH 1
#define RESET__reset_reason_hard_reset__RESET 0
#define RESET__reset_reason_soft_reset__SHIFT 3
#define RESET__reset_reason_soft_reset__WIDTH 1
#define RESET__reset_reason_soft_reset__RESET 0
#define RESET__reset_reason_host__SHIFT 1
#define RESET__reset_reason_host__WIDTH 1
#define RESET__reset_reason_host__RESET 0
#define RESET__reset_reason_timer__SHIFT 0
#define RESET__reset_reason_timer__WIDTH 1
#define RESET__reset_reason_timer__RESET 0
#define ENABLE 0xf8
#define ENABLE__cpu_ready__SHIFT 7
#define ENABLE__cpu_ready__WIDTH 1
#define ENABLE__cpu_ready__RESET 0
#define ENABLE__bootwithoutpll__SHIFT 6
#define ENABLE__bootwithoutpll__WIDTH 1
#define ENABLE__bootwithoutpll__RESET 0
#define ENABLE__powerup_select__SHIFT 4
#define ENABLE__powerup_select__WIDTH 2
#define ENABLE__powerup_select__RESET 0
// Enumeration for powerup_select
#define ENABLE__powerup_select__NO_OVERRIDE 0 // No override - use content of fuses to select - see boot-matrix above (default)
#define ENABLE__powerup_select__FORCE_BOOTMONITOR 1 // Force bootmonitor - ignore fuses and wait for host commands
#define ENABLE__powerup_select__RAM 2 // execute AORAM bootrecords and then set IVT to 0x10000 (base of RAM)to RAM and start whatever application is there - no checks are done!! - application in RAM has to check for reset-reason to know how to proceed
#define ENABLE__poff__SHIFT 3
#define ENABLE__poff__WIDTH 1
#define ENABLE__poff__RESET 0
#define ENABLE__pon__SHIFT 2
#define ENABLE__pon__WIDTH 1
#define ENABLE__pon__RESET 1
#define ENABLE__timed_standby_mode__SHIFT 1
#define ENABLE__timed_standby_mode__WIDTH 1
#define ENABLE__timed_standby_mode__RESET 0
#define ENABLE__standby_mode__SHIFT 0
#define ENABLE__standby_mode__WIDTH 1
#define ENABLE__standby_mode__RESET 0
#define FIFOSTATUS 0xfa
#define FIFOSTATUS__fifo_direction__SHIFT 0
#define FIFOSTATUS__fifo_direction__WIDTH 1
#define FIFOSTATUS__fifo_direction__RESET 0
#define FIFOSTATUS__fifo_dma_busy__SHIFT 1
#define FIFOSTATUS__fifo_dma_busy__WIDTH 1
#define FIFOSTATUS__fifo_dma_busy__RESET 0
#define FIFOSTATUS__txfifo_empty__SHIFT 2
#define FIFOSTATUS__txfifo_empty__WIDTH 1
#define FIFOSTATUS__txfifo_empty__RESET 1
#define FIFOSTATUS__rxfifo_full__SHIFT 3
#define FIFOSTATUS__rxfifo_full__WIDTH 1
#define FIFOSTATUS__rxfifo_full__RESET 0
#define FIFOSTATUS__txfifo_underrun__SHIFT 4
#define FIFOSTATUS__txfifo_underrun__WIDTH 1
#define FIFOSTATUS__txfifo_underrun__RESET 0
#define FIFOSTATUS__rxfifo_overrun__SHIFT 5
#define FIFOSTATUS__rxfifo_overrun__WIDTH 1
#define FIFOSTATUS__rxfifo_overrun__RESET 0
#define SYSTICK_0 0xfb
#define SYSTICK_0__hif_systick__SHIFT 0
#define SYSTICK_0__hif_systick__WIDTH 8
#define SYSTICK_0__hif_systick__RESET 0
#define SYSTICK_1 0xfc
#define SYSTICK_1__hif_systick__SHIFT 0
#define SYSTICK_1__hif_systick__WIDTH 8
#define SYSTICK_1__hif_systick__RESET 0
#define SYSTICK_2 0xfd
#define SYSTICK_2__hif_systick__SHIFT 0
#define SYSTICK_2__hif_systick__WIDTH 8
#define SYSTICK_2__hif_systick__RESET 0
#define SYSTICK_3 0xfe
#define SYSTICK_3__hif_systick__SHIFT 0
#define SYSTICK_3__hif_systick__WIDTH 8
#define SYSTICK_3__hif_systick__RESET 0
#define FIFO 0xff
#define FIFO__fifo_data__SHIFT 0
#define FIFO__fifo_data__WIDTH 8
#define FIFO__fifo_data__RESET 0

#define TMF8829_ENABLE__cpu_ready__MASK          0x80
#define TMF8829_ENABLE__poff__MASK               0x08
#define TMF8829_ENABLE__pon__MASK                0x04
#define TMF8829_RESET__soft_reset__MASK          0x40

// --------------------------------------------------- defines     ------------------------------------------------
#define TMF8829_COM_BL_REG_VERSION               0x01
#define TMF8829_COM_BL_REG_SIZE                  0x09
#define TMF8829_COM_BL_REG_DATA0                 0x0A

#define TMF8829_BL_MAX_DATA_SIZE                 0x80  // Number of bytes that can be written or read with one BL command

// Bootloader commands
#define TMF8829_COM_BL_CMD_STAT_START_RAM_APP    0x16  // Start RAM application 
#define TMF8829_COM_BL_CMD_STAT_START_ROM_APP    0x17  // Start ROM application 
#define TMF8829_COM_BL_CMD_STAT_SPI_OFF          0x20  // SPI off
#define TMF8829_COM_BL_CMD_STAT_I2C_OFF          0x22  // I2C off
#define TMF8829_COM_BL_CMD_STAT_R_RAM            0x40  // Read RAM
#define TMF8829_COM_BL_CMD_STAT_W_RAM            0x41  // Write Ram
#define TMF8829_COM_BL_CMD_STAT_W_RAM_BOTH       0x42  // Write RAM both CPUs
#define TMF8829_COM_BL_CMD_STAT_ADDR_RAM         0x43  // Set address pointer in RAM for Read/Write to BL RAM.
#define TMF8829_COM_BL_CMD_STAT_W_FIFO           0x44  // setup size for Fifo write
#define TMF8829_COM_BL_CMD_STAT_FIFO_BOTH        0x45  // setup size for Fifo write to both CPU Rams

#define BL_HEADER                                2     // bootloader header is 2 bytes
#define BL_MAX_DATA_PAYLOAD                      128   // bootloader data payload can be up to 128
#define BL_FOOTER                                1     // bootloader footer is 1 byte

// Bootloader status codes
#define TMF8829_COM_BL_STAT_READY                0  // status success, ready to receive and execute next command
#define TMF8829_COM_BL_STAT_ERR_PARAM            1  // last command had a parameter error, ready to receive and execute next command
#define TMF8829_COM_BL_STAT_ERR_ADDR             2  // last command tried to access RAM out of range, ready to receive and execute next command
#define TMF8829_COM_BL_STAT_ERR_FIFO             4  // the previous FIFO upload was not completed, command ignored, write more data to fifo

// Bootloader maximum wait sequences
#define BL_CMD_START_RAM_APP_MS                  3  // wait times
#define BL_CMD_SET_ADDR_TIMEOUT_MS               3
#define BL_CMD_W_RAM_TIMEOUT_MS                  3 
#define BL_CMD_W_FIFO_TIMEOUT_MS                 3 
#define BL_CMD_TIMEOUT_MS                        3 

// Application maximum wait sequences
#define APP_CMD_LOAD_CONFIG_TIMEOUT_MS           3
#define APP_CMD_WRITE_CONFIG_TIMEOUT_MS          3
#define APP_CMD_MEASURE_TIMEOUT_MS               5
#define APP_CMD_STOP_TIMEOUT_MS                  25
#define APP_CMD_SWITCH_MODE_CMD_TIMEOUT_MS       1  // timeout until command is accepted
#define APP_CMD_SWITCH_MODE_TIMEOUT_MS           10

// ---------------------- some checks ----------------------

// clock correction pairs index calculation
#define CLK_CORRECTION_IDX_MODULO( x )    ( (x) & ( (CLK_CORRECTION_PAIRS)-1 ) )

// Saturation macro for 16-bit
#define SATURATE16( v )                                                 ( (v) > 0xFFFF ? 0xFFFF : (v) )

// --------------------------------------------------- constants ---------------------------------------------------
// Driver Version
const tmf8829DriverInfo tmf8829DriverInfoReset = 
{ .version = { TMF8829_DRIVER_MAJOR_VERSION , TMF8829_DRIVER_MINOR_VERSION }
};

const tmf8829DeviceInfo tmf8829DeviceInfoReset =
{ .deviceSerialNumber = 0
, .appVersion = { 0, 0, 0, 0 }
, .chipVersion = { 0, 0}
};

// --------------------------------------------------- functions ---------------------------------------------------
static void tmf8829ResetClockCorrection ( tmf8829Driver * driver );

void tmf8829Initialise ( tmf8829Driver * driver )
{
  tmf8829ResetClockCorrection( driver );
  driver->device = tmf8829DeviceInfoReset;
  driver->info = tmf8829DriverInfoReset;
  driver->i2cSlaveAddress = TMF8829_SLAVE_ADDR;
  driver->spiWriteCommand = SPI_WR_CMD;
  driver->spiWriteCommand = SPI_RD_CMD;
  driver->clkCorrectionEnable = 1;
  driver->logLevel =TMF8829_LOG_LEVEL_ERROR;
  memset(driver->dataBuffer, 0, DATA_BUFFER_SIZE);
}

void tmf8829SetLogLevel ( tmf8829Driver * driver, uint8_t level )
{
  driver->logLevel = level;
}

void tmf8829ClkCorrection ( tmf8829Driver * driver, uint8_t enable )
{
  driver->clkCorrectionEnable = !!enable;
  tmf8829ResetClockCorrection( driver );
}

void tmf8829Reset ( tmf8829Driver * driver )
{
  driver->dataBuffer[0] = TMF8829_RESET__soft_reset__MASK;
  txReg( driver, driver->i2cSlaveAddress, RESET, 1, driver->dataBuffer );
  tmf8829ResetClockCorrection( driver );
  if ( driver->logLevel >= TMF8829_LOG_LEVEL_VERBOSE ) 
  {
    PRINT_STR( "reset" );
    PRINT_LN( );
  }
}

void tmf8829Enable ( tmf8829Driver * driver ) 
{
  enablePinHigh( driver );     // when enable gets high, the HW resets to default slave addr
  tmf8829Initialise( driver );           
}

void tmf8829Disable ( tmf8829Driver * driver ) 
{
  enablePinLow( driver );
}

void tmf8829Standby ( tmf8829Driver * driver )
{ 
  driver->dataBuffer[0] = 0; // clear before reading
  rxReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer );   // read the enable register to determine power state
  if ( ( driver->dataBuffer[0] & TMF8829_ENABLE__cpu_ready__MASK ) != 0 )                  
  {
    driver->dataBuffer[0] = driver->dataBuffer[0] | TMF8829_ENABLE__poff__MASK;         // poff to power down
    txReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer ); // clear PON bit in enable register
    if ( driver->logLevel >=TMF8829_LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "POFF=1" );
      PRINT_LN( );
    }
  }
  else
  {
    if ( driver->logLevel >=TMF8829_LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "standby TMF8829_ENABLE=0x" );
      PRINT_UINT_HEX( driver->dataBuffer[0] );
      PRINT_LN( );
    }  
  }
}

void tmf8829PowerUp ( tmf8829Driver * driver ) 
{
  driver->dataBuffer[0] = TMF8829_ENABLE__pon__MASK; //  set PON
  txReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer ); // set PON bit in enable register
  delayInMicroseconds( ENABLE_TIME_MS * 1000 );

  if ( driver->logLevel >= TMF8829_LOG_LEVEL_INFO ) 
  {
      rxReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer ); //  for logging if desired
      PRINT_STR( "pwup ENABLE=0x" );
      PRINT_UINT_HEX( driver->dataBuffer[0] );
      PRINT_LN( );
  }
}

void tmf8829Wakeup ( tmf8829Driver * driver )
{ 
  driver->dataBuffer[0] = 0; // clear before reading
  rxReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer ); // read the enable register to determine power state

  if ( ( driver->dataBuffer[0] & TMF8829_ENABLE__cpu_ready__MASK ) == 0 )
  {
    driver->dataBuffer[0] = driver->dataBuffer[0] | ( ENABLE__powerup_select__RAM << ENABLE__powerup_select__SHIFT ); // select RAM
    driver->dataBuffer[0] = driver->dataBuffer[0] | TMF8829_ENABLE__pon__MASK;          // make sure to keep the remap bits
    txReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer ); // set PON bit in enable register
    if ( driver->logLevel >= TMF8829_LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "PON=1" );
      PRINT_LN( );
    }
  }
  else
  {
    if ( driver->logLevel >= TMF8829_LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "awake ENABLE=0x" );
      PRINT_UINT_HEX( driver->dataBuffer[0] );
      PRINT_LN( );
    }  
  }
}

int8_t tmf8829IsCpuReady ( tmf8829Driver * driver, uint8_t waitInMs )
{
  int8_t status;
  do 
  {
    status = rxReg( driver, driver->i2cSlaveAddress, ENABLE, 1, driver->dataBuffer ); // read the enable register to determine power state

    if ( !!( driver->dataBuffer[0] & TMF8829_ENABLE__cpu_ready__MASK ) && ( status == I2C_SUCCESS) )
    {
      if ( driver->logLevel >=TMF8829_LOG_LEVEL_VERBOSE )
      {
        PRINT_STR( "CPU ready" );
        PRINT_LN( );
      }
      return 1; // done                  
    }
    else if ( waitInMs )   // only wait until it is the last time we go through the loop, that would be a waste of time to wait again
    {
      delayInMicroseconds( 1000 );
    }
  } while ( waitInMs-- > 0);
  if ( driver->logLevel >=TMF8829_LOG_LEVEL_ERROR )
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPARATOR );
    PRINT_STR( "CPU not ready" );
    PRINT_LN( );
  }
  return 0; // cpu did not get ready
}

// function to check if a register has a specific value, with timeout
static int8_t tmf8829CheckRegister ( tmf8829Driver * driver, uint8_t regAddr, uint8_t expected, uint8_t len, uint16_t timeoutInMs )
{
  uint8_t i;
  uint32_t t = getSysTick();
  do 
  {
    driver->dataBuffer[0] = ~expected;
    rxReg( driver, driver->i2cSlaveAddress, regAddr, len, driver->dataBuffer );
    if ( driver->dataBuffer[0] == expected )
    {
      return APP_SUCCESS_OK; 
    }
    else if ( timeoutInMs ) // do not wait if timeout is 0
    {
      delayInMicroseconds(1000);  
    }
  } while ( timeoutInMs-- > 0 );
  if ( driver->logLevel >=TMF8829_LOG_LEVEL_ERROR ) 
  {
    t = getSysTick() - t;
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPARATOR );
    PRINT_STR( "timeout " );
    PRINT_INT( t );
    PRINT_STR( " reg=0x" );
    PRINT_UINT_HEX( regAddr );
    PRINT_STR( " exp=0x" );
    PRINT_UINT_HEX( expected );
    for ( i = 0; i < len; i++ )
    {
      PRINT_STR( " 0x" );
      PRINT_UINT_HEX( driver->dataBuffer[i] );
    }
    PRINT_LN( );
  }      
  return APP_ERROR_TIMEOUT;
}

// ------------------------------- convenience functions --------------------------------------------
uint16_t tmf8829GetUint16 ( const uint8_t * data ) 
{
  uint16_t t =    data[ 1 ];
  t = (t << 8 ) + data[ 0 ];
  return t;
}

uint32_t tmf8829GetUint32 ( const uint8_t * data ) 
{
  uint32_t t =    data[ 3 ];
  t = (t << 8 ) + data[ 2 ];
  t = (t << 8 ) + data[ 1 ];
  t = (t << 8 ) + data[ 0 ];
  return t;
}

// Convert uint16_t in little endian format to 2 bytes
void tmf8829SetUint16 ( uint16_t value, uint8_t * data ) 
{
  data[ 0 ] = value;
  data[ 1 ] = value >> 8;
}

// --------------------------------------- bootloader ------------------------------------------

int8_t tmf8829BootloaderCmdSpiOff ( tmf8829Driver * driver )
{ 
  driver->dataBuffer[0] = TMF8829_COM_BL_CMD_STAT_SPI_OFF;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer );
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_COM_BL_STAT_READY, 1, BL_CMD_TIMEOUT_MS );
}

int8_t tmf8829BootloaderCmdI2cOff ( tmf8829Driver * driver )
{
  driver->dataBuffer[0] = TMF8829_COM_BL_CMD_STAT_I2C_OFF;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer );
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_COM_BL_STAT_READY, 1, BL_CMD_TIMEOUT_MS );
}

// execute command to set the RAM address pointer for RAM read/writes
static int8_t tmf8829BootloaderSetRamAddr ( tmf8829Driver * driver, uint32_t addr )
{
  driver->dataBuffer[0] = TMF8829_COM_BL_CMD_STAT_ADDR_RAM;
  driver->dataBuffer[1] = 4; // payload
  driver->dataBuffer[2] = addr & 0xff;
  driver->dataBuffer[3] = ( addr >> 8 ) & 0xff;
  driver->dataBuffer[4] = ( addr >> 16 ) & 0xff;
  driver->dataBuffer[5] = ( addr >> 24 ) & 0xff;

  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 6, driver->dataBuffer );
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_COM_BL_STAT_READY, 3, BL_CMD_SET_ADDR_TIMEOUT_MS );
}

// writes to both CPU RAMs in parallel
static int8_t tmf8829BootloaderWriteRamBoth ( tmf8829Driver * driver, uint8_t len )
{
  driver->dataBuffer[0] = TMF8829_COM_BL_CMD_STAT_W_RAM_BOTH;
  driver->dataBuffer[1] = len; // payload
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 2 + len, driver->dataBuffer );
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_COM_BL_STAT_READY, 1, BL_CMD_W_RAM_TIMEOUT_MS );
}

// setup size for FIFO writes to both CPU RAMs in parallel
static int8_t tmf8829BootloaderWriteFifoBoth ( tmf8829Driver * driver, uint32_t addr, uint32_t len )
{
  uint16_t wsize = ( len + ( 4 - 1 ) ) / 4; // round value to a word size
  driver->dataBuffer[0] = TMF8829_COM_BL_CMD_STAT_FIFO_BOTH;
  driver->dataBuffer[1] = 6; // payload
  driver->dataBuffer[2] = addr & 0xff;
  driver->dataBuffer[3] = ( addr >> 8 ) & 0xff;
  driver->dataBuffer[4] = ( addr >> 16 ) & 0xff;
  driver->dataBuffer[5] = ( addr >> 24 ) & 0xff;
  driver->dataBuffer[6] = wsize & 0xFF;
  driver->dataBuffer[7] = ( wsize >> 8 ) & 0xff;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 8, driver->dataBuffer );
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_COM_BL_STAT_READY, 1, BL_CMD_W_FIFO_TIMEOUT_MS );
}

//to start the down-loaded measurement application
static int8_t tmf8829BootloaderStartRamApp ( tmf8829Driver * driver )
{ 
  driver->dataBuffer[0] = TMF8829_COM_BL_CMD_STAT_START_RAM_APP;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer );
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_APP_ID, TMF8829_COM_APP_ID__application, 1, BL_CMD_START_RAM_APP_MS );
}

int8_t tmf8829DownloadFirmware ( tmf8829Driver * driver, uint32_t imageStartAddress, const uint8_t * image, int32_t imageSizeInBytes, uint8_t useFifo )
{
  uint32_t idx = 0;
  int8_t stat = BL_SUCCESS_OK;
  uint16_t chunkLen;
  if ( driver->logLevel >= TMF8829_LOG_LEVEL_VERBOSE ) 
  {
    PRINT_STR( "Image addr=0x" );
    PRINT_UINT_HEX( PTR_TO_UINT( image ) );
    PRINT_STR( " len=" );
    PRINT_INT( imageSizeInBytes );
    for ( idx = 0; idx < 16; idx++ )
    {
      uint8_t d = readProgramMemoryByte( ( PTR_TO_UINT( image ) + idx ) );
      PRINT_STR( " 0x" );
      PRINT_UINT_HEX( d );      // read from program memory space
    }
    PRINT_LN( );
  }

  if ( useFifo )
  {
    stat = tmf8829BootloaderWriteFifoBoth( driver, imageStartAddress, imageSizeInBytes );
    idx = 0;  // start again at the image begin
    while ( stat == BL_SUCCESS_OK && ( idx < imageSizeInBytes ) )
    {
        for( chunkLen=0; chunkLen < DATA_BUFFER_SIZE && idx < imageSizeInBytes; chunkLen++, idx++ )
        {
          driver->dataBuffer[chunkLen] = readProgramMemoryByte( PTR_TO_UINT( image ) + idx ); // read from code memory into local ram buffer
        }

        stat = txReg( driver, driver->i2cSlaveAddress, FIFO, chunkLen, driver->dataBuffer );
    }

  }
  else
 {
    stat = tmf8829BootloaderSetRamAddr( driver, imageStartAddress );
    idx = 0;  // start again at the image begin
    while ( stat == BL_SUCCESS_OK && ( idx < imageSizeInBytes ) )
    {
        if ( driver->logLevel >=TMF8829_LOG_LEVEL_VERBOSE )
        {
          PRINT_STR( "Download addr=0x" );
          PRINT_UINT_HEX( (uint32_t)idx );
          PRINT_LN( );
        }
        for( chunkLen=0; chunkLen < BL_MAX_DATA_PAYLOAD && idx < imageSizeInBytes; chunkLen++, idx++ )
        {
          driver->dataBuffer[BL_HEADER  + chunkLen] = readProgramMemoryByte( PTR_TO_UINT( image ) + idx ); // read from code memory into local ram buffer
        }
        stat = tmf8829BootloaderWriteRamBoth( driver, chunkLen );
    }

  }

  if ( stat == BL_SUCCESS_OK )
  {
    stat = tmf8829BootloaderStartRamApp( driver );
    if ( stat == BL_SUCCESS_OK )
    {
      if ( driver->logLevel >=TMF8829_LOG_LEVEL_INFO )
      {
        PRINT_STR( "FW downloaded" );
        PRINT_LN( );
      }
      return stat;
    }
  }

  if ( driver->logLevel >= TMF8829_LOG_LEVEL_ERROR ) 
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPARATOR );
    PRINT_STR( "FW downl or REMAP" );
    PRINT_LN( );
  }
  return stat;
}

// --------------------------------------- application -----------------------------------------

// Reset clock correction calculation
static void tmf8829ResetClockCorrection ( tmf8829Driver * driver )
{
  uint8_t i;
  driver->clkCorrectionIdx = 0;                // reset clock correction
  driver->clkCorrRatioUQ = (1<<15);            // this is 1.0 in UQ1.15 
  for ( i = 0; i < CLK_CORRECTION_PAIRS; i++ )
  {
    driver->hostTicks[ i ] = 0;
    driver->tmf8829Ticks[ i ] = 0;             // initialise the tmf8829Ticks to a value that has the LSB cleared -> can identify that these are no real ticks
  }
  if ( driver->logLevel &TMF8829_LOG_LEVEL_CLK_CORRECTION )
  {
    PRINT_STR( "ClkCorr reset" );
    PRINT_LN( );
  }
}

#define MAX_UINT_VALUE                  (0x80000000)
// function calculates the ratio between hDiff (host) and tDiff (tmf8829) in Uq1.15
static uint16_t tmf8829CalcClkRatioUQ16 ( uint32_t hDiff, uint32_t tDiff, uint16_t prevRatioUQ )
{
  uint32_t ratioUQ = prevRatioUQ;
  if ((hDiff >= HOST_TICKS_PER_1000_US) && (tDiff >= TMF8829_TICKS_PER_1000_US)) 
  { /* move both values to be as big as possible to increase precision to max. possible */
    while ( hDiff < MAX_UINT_VALUE && tDiff < MAX_UINT_VALUE ) 
    {
        hDiff <<= 1;            
        tDiff <<= 1;
    }
    tDiff = tDiff / TMF8829_TICKS_PER_1000_US;
    hDiff = hDiff / HOST_TICKS_PER_1000_US;
    while ( hDiff < MAX_UINT_VALUE && tDiff < MAX_UINT_VALUE )      /* scale up again */ 
    {
        hDiff <<= 1;            
        tDiff <<= 1;
    }
    tDiff = ( tDiff + (1<<14)) >> 15;    /* The number of shifts defines the number of "digits" after the decimal point. UQ1.15 range is [0..2), 1<<15=32768==1.0 */
    if ( tDiff )                         /* this can get 0 if the value was close to 2^32 because of the adding of 2^14 */
    {
      ratioUQ = (hDiff + (tDiff>>1)) / tDiff;                                       /* round the ratio */

      /* ratioUQ16 is the range of [0..2), restrict the ratioUQ to 0.5..1.5 */    
      if ( ( ratioUQ > (1<<15)+(1<<14) ) || ( ratioUQ < (1<<15)-(1<<14) ) ) // this check ensures that the new value fits in 16-bit also.
      {
        ratioUQ = prevRatioUQ; 
      }
    }  
  }
  return (uint16_t)ratioUQ;           // return an UQ1.15 = [0..2)
}

// Add a host tick and a tmf8829 tick to the clock correction list and update ratioUQ
static void tmf8829ClockCorrectionAddPair ( tmf8829Driver * driver, uint32_t hostTick, uint32_t tmf8829Tick )
{
  uint8_t idx;
  uint8_t idx2;
  uint32_t hDiff;
  uint32_t tDiff;
  driver->clkCorrectionIdx = CLK_CORRECTION_IDX_MODULO( driver->clkCorrectionIdx + 1 );     // increment and take care of wrap-over
  driver->hostTicks[ driver->clkCorrectionIdx ] = hostTick;
  driver->tmf8829Ticks[ driver->clkCorrectionIdx ] = tmf8829Tick;
  /* check if we have enough entries to re-calculate the ratioUQ */
  idx = driver->clkCorrectionIdx;                                         // last inserted
  idx2 = CLK_CORRECTION_IDX_MODULO( idx + CLK_CORRECTION_PAIRS - 1 );     // oldest available

  if ((driver->hostTicks[ idx ]) < driver->hostTicks[ idx2 ]) //prevent rollover
  {
    hDiff = 0;
  }
  else
  {
    hDiff = driver->hostTicks[ idx ] - driver->hostTicks[ idx2 ];
  }

  tDiff = driver->tmf8829Ticks[ idx ] - driver->tmf8829Ticks[ idx2 ];
  driver->clkCorrRatioUQ = tmf8829CalcClkRatioUQ16( hDiff, tDiff, driver->clkCorrRatioUQ );
  if ( driver->logLevel & TMF8829_LOG_LEVEL_CLK_CORRECTION )
  {
    PRINT_LN( );
    PRINT_STR( "ClkCorr ratio " );  
    PRINT_UINT( hDiff );
    PRINT_CHAR( ' ' );
    PRINT_UINT( tDiff );
    PRINT_CHAR( ' ' );
    PRINT_STR( "Per ms " );  
    PRINT_UINT( hDiff  / HOST_TICKS_PER_1000_US );
    PRINT_CHAR( ' ' );
    PRINT_UINT( tDiff  / TMF8829_TICKS_PER_1000_US );
    PRINT_CHAR( ' ' );
    PRINT_STR( "Factor " );
    PRINT_UINT( driver->clkCorrRatioUQ ); 
    PRINT_LN( );                              
  }                                             
    
}  

int8_t tmf8829ReadDeviceInfo ( tmf8829Driver * driver )
{
  driver->device = tmf8829DeviceInfoReset; 
  rxReg( driver, driver->i2cSlaveAddress, ID, 2, driver->device.chipVersion );
  rxReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_APP_ID, 4, driver->device.appVersion );  // tmf8829 application has 4 version bytes
  if ( driver->device.appVersion[0] == TMF8829_COM_APP_ID__application )
  {
    rxReg( driver, driver->i2cSlaveAddress, TMF8829_SERIAL_NUMBER_0, 4, driver->dataBuffer );
    driver->device.deviceSerialNumber = tmf8829GetUint32( &(driver->dataBuffer[0]) );
    return APP_SUCCESS_OK;
  }
  return APP_ERROR_CMD;
}

int8_t tmf8829Command ( tmf8829Driver * driver, uint8_t cmd )
{
  int8_t stat;
  driver->dataBuffer[0] = cmd;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer );
  stat = tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_CMD_STAT__cmd_stat__STAT_OK, 1, APP_CMD_LOAD_CONFIG_TIMEOUT_MS );  // check that load command is completed

  return stat;
}

int8_t tmf8829CmdLoadConfigPage ( tmf8829Driver * driver )
{
  int8_t stat;
  driver->dataBuffer[0] = TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CONFIG_PAGE;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer ); // instruct device to load page
  stat = tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_CMD_STAT__cmd_stat__STAT_OK, 1, APP_CMD_LOAD_CONFIG_TIMEOUT_MS );  // check that load command is completed
  if ( stat == APP_SUCCESS_OK )
  {
    stat = tmf8829CheckRegister( driver, TMF8829_CID_RID, TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CONFIG_PAGE, 1, APP_CMD_LOAD_CONFIG_TIMEOUT_MS );     // check that correct config page is loaded
  }
  return stat;
}

int8_t tmf8829CmdWritePage ( tmf8829Driver * driver )
{
  int8_t stat;
  driver->dataBuffer[0] = TMF8829_CMD_STAT__cmd_stat__CMD_WRITE_PAGE;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer ); // instruct device to write page
  stat = tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_CMD_STAT__cmd_stat__STAT_OK, 1, APP_CMD_WRITE_CONFIG_TIMEOUT_MS ); // check that write command is completed
  return stat;
}

int8_t tmf8829GetConfiguration (tmf8829Driver * driver)
{
  int8_t stat = tmf8829CmdLoadConfigPage( driver ); // first load the page, then only overwrite the registers you want to change 

  if ( stat == APP_SUCCESS_OK )
  {
    rxReg( driver, driver->i2cSlaveAddress, TMF8829_CFG_PERIOD_MS_LSB, TMF8829_CFG_PAGE_SIZE, driver->config );
  }
  if ( stat != APP_SUCCESS_OK )
  {
    if ( driver->logLevel >=TMF8829_LOG_LEVEL_ERROR ) 
    {
      PRINT_STR( "#Err" );
      PRINT_CHAR( SEPARATOR );
      PRINT_STR( " Get Configuration " );
      PRINT_INT( stat );
      PRINT_LN( );
    }
  }
  return stat;
}

int8_t tmf8829SetConfiguration ( tmf8829Driver * driver )
{
  int8_t stat = APP_ERROR_PARAM;

  txReg( driver, driver->i2cSlaveAddress, TMF8829_CFG_PERIOD_MS_LSB, TMF8829_CFG_PAGE_SIZE, driver->config );
  stat = tmf8829CmdWritePage( driver );  // as a last step write the config page back

  if ( stat != APP_SUCCESS_OK )
  {
    if ( driver->logLevel >=TMF8829_LOG_LEVEL_ERROR ) 
    {
      PRINT_STR( "#Err" );
      PRINT_CHAR( SEPARATOR );
      PRINT_STR( "Config " );
      PRINT_INT( stat );
      PRINT_LN( );
    }
  }
  return stat;
}

int8_t tmf8829StartMeasurement ( tmf8829Driver * driver ) 
{
  int8_t accepted;
  int16_t period = (driver->config[0]) + (driver->config[TMF8829_CFG_PERIOD_MS_MSB-TMF8829_CFG_PERIOD_MS_LSB]*256);
  tmf8829ResetClockCorrection( driver ); // clock correction only works during measurements
  driver->dataBuffer[0] = TMF8829_CMD_STAT__cmd_stat__CMD_MEASURE;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer );
  accepted = tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_CMD_STAT__cmd_stat__STAT_ACCEPTED, 1, APP_CMD_MEASURE_TIMEOUT_MS );

  if ((accepted == APP_SUCCESS_OK) && period)
  {
    driver->cyclicRunning = 1;
  }
  return accepted;
}

int8_t tmf8829StopMeasurement ( tmf8829Driver * driver ) 
{
  driver->dataBuffer[0] = TMF8829_CMD_STAT__cmd_stat__CMD_STOP;
  txReg( driver, driver->i2cSlaveAddress, TMF8829_COM_REG_CMD_STAT, 1, driver->dataBuffer );
  driver->cyclicRunning = 0;
  return tmf8829CheckRegister( driver, TMF8829_COM_REG_CMD_STAT, TMF8829_CMD_STAT__cmd_stat__STAT_OK, 1, APP_CMD_STOP_TIMEOUT_MS );
}

uint8_t tmf8829GetAndClrInterrupts ( tmf8829Driver * driver, uint8_t mask )
{
  uint8_t setInterrupts;
  driver->dataBuffer[0] = 0;
  rxReg( driver, driver->i2cSlaveAddress, INT_STATUS, 1, driver->dataBuffer ); // read interrupt status register
  setInterrupts = driver->dataBuffer[0] & mask;
  if ( setInterrupts )
  {
    driver->dataBuffer[0] = setInterrupts;                                   // clear only those that were set when we read the register, and only those we want to know
    txReg( driver, driver->i2cSlaveAddress, INT_STATUS, 1, driver->dataBuffer ); // clear interrupts by pushing a 1 to status register
  }
  return setInterrupts;
}

void tmf8829ClrAndEnableInterrupts ( tmf8829Driver * driver, uint8_t mask )
{
  driver->dataBuffer[0] = 0xFF;                                                   // clear all interrupts  
  txReg( driver, driver->i2cSlaveAddress, INT_STATUS, 1, driver->dataBuffer ); // clear interrupts by pushing a 1 to status register
  driver->dataBuffer[0] = 0;
  rxReg( driver, driver->i2cSlaveAddress, INT_ENAB, 1, driver->dataBuffer );   // read current enabled interrupts 
  driver->dataBuffer[0] = driver->dataBuffer[0] | mask;                                   // enable those in the mask, keep the others if they were enabled
  txReg( driver, driver->i2cSlaveAddress, INT_ENAB, 1, driver->dataBuffer );
}

void tmf8829DisableInterrupts ( tmf8829Driver * driver, uint8_t mask )
{ 
  driver->dataBuffer[0] = 0;
  txReg( driver, driver->i2cSlaveAddress, INT_ENAB, 1, driver->dataBuffer );   // read current enabled interrupts 
  driver->dataBuffer[0] = driver->dataBuffer[0] & ~mask;                                  // clear only those in the mask, keep the others if they were enabled
  txReg( driver, driver->i2cSlaveAddress, INT_ENAB, 1, driver->dataBuffer );
  driver->dataBuffer[0] = mask; 
  txReg( driver, driver->i2cSlaveAddress, INT_STATUS, 1, driver->dataBuffer ); // clear interrupts by pushing a 1 to status register
}

int8_t tmf8829ReadResults ( tmf8829Driver * driver )
{
  uint32_t hTick = 0;
  uint32_t tTick = 0;
  uint8_t pixelSize = 0;
  uint8_t layout = 0;
  //Read Results PRE_HEADER (0xFA + 0xFB..0xFE) + Frame header
  rxReg( driver, driver->i2cSlaveAddress, FIFOSTATUS, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE, driver->dataBuffer );

  hTick = getSysTick( );
  tTick = tmf8829GetUint32( driver->dataBuffer + 1 );
  layout = driver->dataBuffer[TMF8829_PRE_HEADER_SIZE + 1];
  pixelSize = tmf8829GetPixelSize(layout);

  handleReceivedFrameHeaderData( driver, driver->dataBuffer);

  tmf8829ClockCorrectionAddPair( driver, hTick, tTick );

  if ( (driver->dataBuffer[TMF8829_PRE_HEADER_SIZE] & TMF8829_FID_MASK ) == TMF8829_FID_RESULTS )
  {
    uint16_t eofMarker;
    uint16_t payload = tmf8829GetUint16( driver->dataBuffer + 7 ); // payload in dataBuffer in Frameheader Byte 2 + 3
    uint16_t sizeToRead = payload - ( TMF8829_FRAME_HEADER_SIZE - TMF8829_FRAME_HEADER_OFFSET );
    uint16_t rxSize = 0;
    uint8_t rxStatus;
    
    do        // data reading is done in 1 or multiple junks
    {
      if (sizeToRead > DATA_BUFFER_SIZE)
      {
        // rxSize = DATA_BUFFER_SIZE; 
        rxSize = ((uint16_t)( DATA_BUFFER_SIZE / pixelSize)) * pixelSize; // a pixel result not spreaded over two data junks (for Distance Correction)
      }
      else
      {
        rxSize = sizeToRead;
      }
      sizeToRead -= rxSize;        
      rxStatus = rxReg( driver, driver->i2cSlaveAddress, FIFO,  rxSize, driver->dataBuffer ); 
  
      if (driver->clkCorrectionEnable)
      {
        uint16_t sizeToCorrect = rxSize;
        if (sizeToRead < TMF8829_FRAME_FOOTER_SIZE) // do not modifiy footer
        {
          sizeToCorrect = rxSize - (TMF8829_FRAME_FOOTER_SIZE - sizeToRead);
        }
        tmf8829CorrectDistanceDataSegment( driver, sizeToCorrect, layout);
      }

      handleReceivedResultData( driver, driver->dataBuffer, rxSize );

    } while ( ( sizeToRead > 0 ) && ( rxStatus == APP_SUCCESS_OK ) );

    eofMarker = tmf8829GetUint16( driver->dataBuffer +  rxSize - TMF8829_FRAME_EOF_SIZE );
    handleReceivedResultDataEnd( driver );

    if ( eofMarker == TMF8829_FRAME_EOF )
    {
      return APP_SUCCESS_OK;
    }
  }

  return APP_ERROR_NO_RESULT;
}

// Function to read histograms and print them on UART.
int8_t tmf8829ReadHistogram ( tmf8829Driver * driver )
{
  //Read Results PRE_HEADER (0xFA + 0xFB..0xFE) + Frame header
  rxReg( driver, driver->i2cSlaveAddress, FIFOSTATUS, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE, driver->dataBuffer );
  handleReceivedFrameHeaderData( driver, driver->dataBuffer);

  if ( (driver->dataBuffer[TMF8829_PRE_HEADER_SIZE] & TMF8829_FID_MASK ) == TMF8829_FID_HISTOGRAMS )
  {
    uint16_t eofMarker;
    uint16_t payload = tmf8829GetUint16( driver->dataBuffer + 7 ); // payload in dataBuffer in Frameheader Byte 2 + 3
    uint16_t sizeToRead = payload - ( TMF8829_FRAME_HEADER_SIZE - TMF8829_FRAME_HEADER_OFFSET );
    uint16_t rxSize = 0;
    uint8_t rxStatus;

    do        // data reading is done in 1 or multiple junks
    {
      if (sizeToRead > DATA_BUFFER_SIZE)
      {
        rxSize = DATA_BUFFER_SIZE;
      }
      else
      {
        rxSize = sizeToRead;
      }
      sizeToRead -= rxSize;        

      rxStatus = rxReg( driver, driver->i2cSlaveAddress, FIFO,  rxSize, driver->dataBuffer ); 

      handleReceivedHistogramData( driver, driver->dataBuffer, rxSize );

    } while ( ( sizeToRead > 0 ) && ( rxStatus == APP_SUCCESS_OK ) );

    eofMarker = tmf8829GetUint16( driver->dataBuffer +  rxSize - TMF8829_FRAME_EOF_SIZE );
    handleReceivedHistogramDataEnd( driver );

    if ( eofMarker == TMF8829_FRAME_EOF )
    {
      return APP_SUCCESS_OK;
    }
  }
  return APP_ERROR_NO_RESULT;
}

void tmf8829CorrectDistanceDataSegment ( tmf8829Driver * driver, uint16_t size, uint8_t layout )
{
  uint8_t pixelSize = tmf8829GetPixelSize(layout);
  uint8_t numPeaks = layout & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
  uint8_t idx = 0;
  
  for (uint16_t idxpixel = 0; idxpixel < size; idxpixel += pixelSize)
  {
    idx = 0;
    if (layout & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK)
    {
      idx += 2;
    }
    if (layout & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK)
    {
      idx += 2;
    }
     
    for (uint8_t idxPeak= 0; idxPeak < numPeaks; idxPeak++)
    {
      uint16_t corrDist = tmf8829CorrectDistance(driver, tmf8829GetUint16(&driver->dataBuffer[idxpixel+idx]));
      tmf8829SetUint16(corrDist, &driver->dataBuffer[idxpixel+idx]);

      idx += 3;
      if (layout & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK)
      {
        idx += 2;
      }
     }
  }

  return;
}

uint16_t tmf8829CorrectDistance( tmf8829Driver * driver, uint16_t distance)
{
  uint32_t corrdist = distance;
  corrdist = ( driver->clkCorrRatioUQ * corrdist + (1<<14) ) >> 15;

  return SATURATE16( corrdist );
}

uint8_t tmf8829GetPixelSize(uint8_t layout)
{
  uint8_t size = 0;
  uint8_t numPeak = layout & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
  uint8_t useSignal = ( (layout & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) > 0) ? 1 : 0;
  uint8_t useNoise =  ( (layout & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) > 0) ? 1 : 0;
  uint8_t useXtalk =  ( (layout & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) > 0) ? 1 : 0;

  size = ( numPeak * ( 3 + 2 * useSignal) ) + ( 2 * useNoise ) + ( 2 * useXtalk );

  return size;
}
