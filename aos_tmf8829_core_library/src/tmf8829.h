/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#ifndef TMF8829_H
#define TMF8829_H

// ---------------------------------------------- includes ----------------------------------------

#include "tmf8829_shim.h"

#ifdef __cplusplus
extern "C" {
#endif  

/** Driver version number
 * 0.1 .. initial Version; (based on tmf882x arduino driver )
 *     .. first release ROM 1v0
 * 1.0 .. tmf8829BootloaderCmdSpiOff, tmf8829BootloaderCmdI2cOff added
 *     .. load/write configuration size changed
 * 1.1 .. Standby bugfix, clear databuffer in Initialise
 *     .. remove start rom app, wait times for BL commands
 * 1.2 .. Wakeup changed, use RAM option for powerup_select
 *     .. clock correction bugfix 128kHz to 125kHz for Systick
 *     .. some cleanup
 *     .. handleReceivedHistogramDataEnd added (not used in linux driver)
 *     .. clock correction could be done in driver
 * 1.3 .. Motion and proximity interrupt added
*/

#define TMF8829_DRIVER_MAJOR_VERSION  1
#define TMF8829_DRIVER_MINOR_VERSION  3

// ---------------------------------------------- defines -----------------------------------------

/* tmf8829 has as default i2c slave address */
#define TMF8829_SLAVE_ADDR         0x41

/* tmf8829 has default spi write and read commands */
#define SPI_WR_CMD                 0x02
#define SPI_RD_CMD                 0x03

/* tmf8829 has a max fifo size */
#define  TMF8829_FIFO_SIZE         (8192)

/* important wait timings  */
#define CAP_DISCHARGE_TIME_MS      3                   /**< wait time until we are sure the PCB's CAP has discharged properly */
#define ENABLE_TIME_MS             3                   /**< wait time after enable pin is high */
#define CPU_READY_TIME_MS          3                   /**< wait time for CPU ready */

/* return codes from the bootloader */
#define BL_SUCCESS_OK              0                   /**< success */
#define BL_ERROR_CMD              -1                   /**< command/communication failed */
#define BL_ERROR_TIMEOUT          -2                   /**< communication timeout */

/* return codes from the measurement application */
#define APP_SUCCESS_OK            (BL_SUCCESS_OK)     /**< success */
#define APP_ERROR_CMD             (BL_ERROR_CMD)      /**< command/communication failed */
#define APP_ERROR_TIMEOUT         (BL_ERROR_TIMEOUT)  /**< timeout */
#define APP_ERROR_PARAM           -3                  /**< invalid parameter (e.g. spad map id wrong) */
#define APP_ERROR_NO_RESULT       -4                  /**<  did not receive a measurement result */

/* Interrupt macros */
#define TMF8829_APP_INT_RESULTS                 0x01  /**< a measurement result is ready for readout */
#define TMF8829_APP_INT_MOTION                  0x02  /**< interrupt for motion detection            */
#define TMF8829_APP_INT_PROXIMITY               0x04  /**< interrupt for proximity results           */
#define TMF8829_APP_INT_HISTOGRAMS              0x08  /**< histogram results are ready for readout   */

#define TMF8829_APP_INT_ALL       ( TMF8829_APP_INT_RESULTS | TMF8829_APP_INT_MOTION  | TMF8829_APP_INT_PROXIMITY | TMF8829_APP_INT_HISTOGRAMS )

/* application registers */
#define TMF8829_APP_ID                          0x01  /**<  major app version */
#define TMF8829_MAJOR                           0x02  /**<  minor app version */
#define TMF8829_MINOR                           0x04  /**<  status of application */
#define TMF8829_MEASURE_STATUS                  0x05  /**<  status of measurement */
#define TMF8829_SERIAL_NUMBER_0                 0x1c  /**<  register with serial number */
#define TMF8829_CID_RID                         0x20  /**<  loaded page */

/* configuration page addresses */
#define TMF8829_CFG_PERIOD_MS_LSB               0x22  /**<  period in milliseconds */
#define TMF8829_CFG_PERIOD_MS_MSB               0x23
#define TMF8829_CFG_KILO_ITERATIONS_LSB         0x24  /**<  Kilo (1024) iterations */
#define TMF8829_CFG_KILO_ITERATIONS_MSB         0x25
#define TMF8829_CFG_FP_MODE                     0x26
#define TMF8829_CFG_SPAD_DEADTIME               0x29
#define TMF8829_CFG_RESULT_FORMAT               0x2A
#define TMF8829_CFG_DUMP_HISTOGRAMS             0x2B  /**<  0 ... off, 1 ... raw histograms in blocking mode */
#define TMF8829_CFG_VCSEL_ON                    0x30
#define TMF8829_CFG_VCDRV_2                     0x33
#define TMF8829_CFG_VCDRV_3                     0x34
#define TMF8829_CFG_ALG_DISTANCE                0x52
#define TMF8829_CFG_INT_THRESHOLD_LOW_LSB       0x68
#define TMF8829_CFG_INT_THRESHOLD_LOW_MSB       0x69
#define TMF8829_CFG_INT_THRESHOLD_HIGH_LSB      0x6A
#define TMF8829_CFG_INT_THRESHOLD_HIGH_MSB      0x6B
#define TMF8829_CFG_INT_PERSISTANCE             0x6C
#define TMF8829_CFG_LAST_AVAILABLE              0xDF
#define TMF8829_CFG_PAGE_SIZE                   (TMF8829_CFG_LAST_AVAILABLE - TMF8829_CFG_PERIOD_MS_LSB + 1)

/* Interrupt Masks  */

#define TMF8829_FID_RESULTS         0x10
#define TMF8829_FID_HISTOGRAMS      0x20

#define TMF8829_FID_MASK            0xF0
#define TMF8829_FPM_MASK            0x0F

/* Focal plane modes */
#define TMF8829_CFG_FP_MODE_8x8A    0
#define TMF8829_CFG_FP_MODE_8x8B    1
#define TMF8829_CFG_FP_MODE_16x16   2
#define TMF8829_CFG_FP_MODE_32x32   3
#define TMF8829_CFG_FP_MODE_32x32s  4
#define TMF8829_CFG_FP_MODE_48x32   5

/* result page addresses and defines */
#define TMF8829_PRE_HEADER_SIZE      5
#define TMF8829_FRAME_HEADER_SIZE   16
#define TMF8829_FRAME_FOOTER_SIZE   12
#define TMF8829_FRAME_EOF_SIZE       2

#define TMF8829_FRAME_HEADER_OFFSET  4        /**< the first bytes are not part of the payload value in the frame */
#define TMF8829_FRAME_EOF            0xE0F7   /**< End of Frame Marker */

#define TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK         0x07
#define TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK  0x08
#define TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK   0x10
#define TMF8829_CFG_RESULT_FORMAT_XTALK_MASK            0x20

#define TMF8829_RESULT_FRAME_SUBIDX_MASK                0x40 /**< bitmask to read the sub-idx */

// some more info registers from the results page
#define TMF8829_COM_FIFOSTATUS             0xFA
#define TMF8829_COM_SYS_TICK_0             0xFB

/* Use this macro like this: data[ RESULT_REG( RESULT_NUMBER ) ], it calculates the offset into the data buffer */
#define RESULT_REG( regName )             ( (TMF8829_COM_##regName) - (TMF8829_COM_FIFOSTATUS) )

/* Clock correction pairs must be a power of 2 value. */
#define CLK_CORRECTION_PAIRS               4   /**< how many clock correction pairs are stored */

/* ---------------------- Application ID ---------------------- */
#define TMF8829_COM_REG_APP_ID              0x00   /**< register address, same for BL and application */
#define TMF8829_COM_REG_CMD_STAT            0x08   /**< register to write commands, same for BL and application */

#define TMF8829_COM_APP_ID__application     0x01   /**< measurement application id */
#define TMF8829_COM_APP_ID__bootloader      0x80   /**< bootloader application id */

// application commands
#define TMF8829_CMD_STAT__cmd_stat__CMD_MEASURE                       0x10  // Start a measurement
#define TMF8829_CMD_STAT__cmd_stat__CMD_CLEAR_STATUS                  0x11  // clear all status registers
#define TMF8829_CMD_STAT__cmd_stat__CMD_WRITE_PAGE                    0x15  // Write the active config page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CONFIG_PAGE              0x16  // Load the common config page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8                  0x40  // Preconfigure for 8x8 default mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_LONG_RANGE       0x41  // Preconfigure for 8x8 12m mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_8X8_HIGH_ACCURACY    0x42  // Preconfigure for 8x8 (short range mode) higher resolution
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16                0x43  // Preconfigure for 16x16 default mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_16X16_HIGH_ACCURACY  0x44  // Preconfigure for 16x16 short range mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32                0x45  // Preconfigure for 32x32 default mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_32X32_HIGH_ACCURACY  0x46  // Preconfigure for 32x32 short range mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32                0x47  // Preconfigure for 48x32 default mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_LOAD_CFG_48X32_HIGH_ACCURACY  0x48  // Preconfigure for 48x32 short range mode and load configuration page
#define TMF8829_CMD_STAT__cmd_stat__CMD_STOP                          0xff  // Stop a measurement

// application status, we check only for ok or accepted, otherwise it is an error
#define TMF8829_CMD_STAT__cmd_stat__STAT_OK               0x00  // Everything is okay
#define TMF8829_CMD_STAT__cmd_stat__STAT_ACCEPTED         0x01  // Everything is okay too, send stop to halt ongoing command

/* ---------------------- Result Frame IDs -------------------- */

#define TMF8829_COM_RESULT__measurement_res_frame  0xAA /**< measurement result frame */
#define TMF8829_COM_RESULT__measurement_hist_frame 0xBB /**< measurement histogram frame */
#define TMF8829_COM_RESULT__measurement_header     0xFD /**< measurement frame header only; for debug only should never occur in result frame */
#define TMF8829_COM_RESULT__error_frame            0xFE /**< error frame */
#define TMF8829_COM_RESULT__no_frame               0xFF /**< no frame / unhandled interrupt */

#define TMF8829_COM_ERROR_eof                      0x01 /**< error frame  */
#define TMF8829_COM_ERROR_unkown                   0x00 /**< unkown error */

/* ---------------------- logging ------------------------------ */

#define TMF8829_LOG_LEVEL_NONE              0x00   /**< no logging */
#define TMF8829_LOG_LEVEL_ERROR             0x01   /**< only error logging - recommended */
#define TMF8829_LOG_LEVEL_RESULTS_HEADER    0x02   /**< only result header */
#define TMF8829_LOG_LEVEL_RESULTS           0x04   /**< results */
#define TMF8829_LOG_LEVEL_CLK_CORRECTION    0x08   /**< this is a bit-mask check for clock correction logging */
#define TMF8829_LOG_LEVEL_INFO              0x10   /**< some information */
#define TMF8829_LOG_LEVEL_VERBOSE           0x20   /**< very chatty firmware */
#define TMF8829_LOG_LEVEL_I2C               0x80   /**< this is a bit-mask check for i2c logging */
#define TMF8829_LOG_LEVEL_DEBUG             0xFF   /**< everything */

/* ---------------------- types   ------------------------------ */

/** @brief driver info: compile time info of this driver
 */ 
typedef struct _tmf8829DriverInfo
{
  uint8_t version[2];              /**< this driver version number major.minor*/ 
} tmf8829DriverInfo;

/** @brief device information: read from the device
 */
typedef struct _tmf8829DeviceInfo
{
  uint32_t deviceSerialNumber;     /**< serial number of device, if 0 not read */
  uint8_t appVersion[4];           /**< application version number (app id, major, minor, patch) */
  uint8_t chipVersion[2];          /**< chip version (Id, revId) */
} tmf8829DeviceInfo;

/** @brief Each tmf8829 driver instance needs a data structure like this
 */
typedef struct _tmf8829Driver
{
  tmf8829DeviceInfo device;                      /**< information record of device */
  tmf8829DriverInfo info;                        /**< information record of driver */  
  uint32_t hostTicks[ CLK_CORRECTION_PAIRS ];    /**< host ticks for clock correction */
  uint32_t tmf8829Ticks[ CLK_CORRECTION_PAIRS ]; /**< device ticks for clock correction */
  uint16_t clkCorrRatioUQ;                       /**< clock ratio in UQ1.15 [0..2) */
  uint8_t clkCorrectionIdx;                      /**< index of the last inserted pair */
  uint8_t i2cSlaveAddress;                       /**< i2c slave address to talk to device */
  uint8_t spiWriteCommand;                       /**< spi write address to talk to device */
  uint8_t spiReadCommand;                        /**< spi read address to talk to device */
  uint8_t clkCorrectionEnable;                   /**< default is clock correction on  */
  uint8_t cyclicRunning;                         /**< 1 for ongoing cyclic measurement, otherwise 0 */
  uint8_t logLevel;                              /**< how chatty the program is */
  uint8_t config[TMF8829_CFG_PAGE_SIZE];         /**< holds the configuration parameters */
  uint8_t dataBuffer[ DATA_BUFFER_SIZE ];        /**< transfer/receive buffer */
} tmf8829Driver;

// ---------------------------------------------- functions ---------------------------------------
// Power and bootloader functions are available with ROM code.
// ------------------------------------------------------------------------------------------------

/** @brief  Function to initialise the driver data structure, call this as the first function
 * of your program, before using any other function of this driver
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829Initialise( tmf8829Driver * driver );

/** @brief  Function to configure the driver how chatty it should be. This is only the driver itself.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @param level ... the log level for printing
 */ 
void tmf8829SetLogLevel( tmf8829Driver * driver, uint8_t level );

/** @brief  Function to set clock correction on or off.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @param enable ... if <>0 clock correction is enabled (default), if ==0 clock correction is disabled
 */ 
void tmf8829ClkCorrection( tmf8829Driver * driver, uint8_t enable );

/** @brief  Function to reset the HW and SW on the device
 * driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829Reset( tmf8829Driver * driver );

/** @brief  Function to set the enable pin high
 * it also does driver state initialization
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829Enable( tmf8829Driver * driver );

/** @brief  Function to set the enable pin low
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829Disable( tmf8829Driver * driver );

/** @brief  Function to put the device in standby mode
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829Standby( tmf8829Driver * driver );

/** @brief  Function to SW-powerup the enabled device via the PON bit
 * driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829PowerUp( tmf8829Driver * driver );

/** @brief  Function to wake the device up from standby mode
 * driver ... pointer to an instance of the tmf8829 driver data structure
 */ 
void tmf8829Wakeup( tmf8829Driver * driver );

/** @brief  Function returns true if CPU is ready within the given time, else false.
 * If CPU is not ready, device cannot be used.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @param waitInMs ... time in ms to get ready
 * @return Returns !=0 if CPU is ready (device can be used), else it returns 0 (device cannot be accessed).
 */ 
int8_t tmf8829IsCpuReady( tmf8829Driver * driver, uint8_t waitInMs );

/** @brief  Function executes command BL_CMD_SPI_OFF.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Returns 0 if execution was successful
 */ 
int8_t tmf8829BootloaderCmdSpiOff( tmf8829Driver * driver );

/** @brief  Function executes command BL_CMD_I2C_OFF.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Returns 0 if execution was successful
 */ 
int8_t tmf8829BootloaderCmdI2cOff( tmf8829Driver * driver );

// ------------------------------- convenience functions --------------------------------------------

/** @brief Convert 4 bytes in little endian format into an uint32_t
 * @param data ... pointer to the 4 bytes to be converted
 * @return returns the 32-bit value  
 */ 

uint32_t tmf8829GetUint32( const uint8_t * data );
/** @brief Convert 2 bytes in little endian format into an uint16_t
 * @param data ... pointer to the 2 bytes to be converted
 * @return the converted 16-bit value  
 */
uint16_t tmf8829GetUint16( const uint8_t * data );

/** @brief Convert uint16_t in little endian format to 2 bytes
 * @param value ... the 16-bit value to be converted
 * @param data ... pointer to destinationw here the converted value shall be stored
 */ 
void tmf8829SetUint16( uint16_t value, uint8_t * data );

// ------------------------------- bootloader functions --------------------------------------------

/** @brief  Function to download the firmware image that was linked against the firmware (tmf8829_image.{h,c} files)
 * The function tmf8829BootloaderStartRamApp is executed after successful download.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @param imageStartAddress ... destination in RAM to which the image shall be downloaded
 * @param image ... pointer to the character array that represents the downloadable image
 * @param imageSizeInBytes ... the number of bytes to be downloaded
 * @param  useFifo ... the firmware is downloaded via the fifo, otherwise with writing to the Ram
 * @return Function returns BL_SUCCESS_OK if successfully downloaded the FW, else it returns an error BL_ERROR_*
 */
int8_t tmf8829DownloadFirmware( tmf8829Driver * driver, uint32_t imageStartAddress, const uint8_t * image, int32_t imageSizeInBytes, uint8_t useFifo );

// ------------------------------- application functions --------------------------------------------

// Application functions are only available after a successful firmware download and a successful start of the ram app.

/** @brief Function reads complete device information
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully read the complete device information, else  it returns an error APP_ERROR_* 
 */ 
int8_t tmf8829ReadDeviceInfo( tmf8829Driver * driver );

/** @brief  Function to execute a application command via I2C
 * The command is written to the register TMF8829_CMD_STAT.
 * @param  driver ... pointer to an instance of the tmf8829 driver data structure
 * @param  cmd ... command 
 * @return Function returns APP_SUCCESS_OK if successfully loaded, else it returns an error APP_ERROR_*
 */
int8_t tmf8829Command( tmf8829Driver * driver, uint8_t cmd);

/** @brief  Function to load the config page into I2C ram that the host can read/write it via I2C
 * @param  driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully loaded, else it returns an error APP_ERROR_*
 */
int8_t tmf8829CmdLoadConfigPage( tmf8829Driver * driver );

/** @brief  Function writes the current configuration page.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully written, else it returns an error APP_ERROR_*
 */
int8_t tmf8829CmdWritePage( tmf8829Driver * driver );

/** @brief  Loads the Command Page and stores the data in the driver parameter config.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully executed, else it returns an error
 */
int8_t tmf8829GetConfiguration(tmf8829Driver * driver);

/** @brief  The config parameters of the driver are written to the device.
 * The function tmf8829GetConfiguration should be called before to have the right page loaded
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully executed, else it returns an error
 */
int8_t tmf8829SetConfiguration(tmf8829Driver * driver);

/** @brief  Function to start a measurement
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully started a measurement, else it returns an error APP_ERROR_*
 */
int8_t tmf8829StartMeasurement( tmf8829Driver * driver );

/** @brief  Function to stop a measurement 
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if successfully stopped the application, else it returns an error APP_ERROR_*
 */
int8_t tmf8829StopMeasurement( tmf8829Driver * driver );

/** @brief  Function reads the interrupts that are set and clears those. 
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns the bit-mask of the interrupts that were set.
 */
uint8_t tmf8829GetAndClrInterrupts( tmf8829Driver * driver, uint8_t mask );

/** @brief  Function clears the given interrupts and enables them.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 */
void tmf8829ClrAndEnableInterrupts( tmf8829Driver * driver, uint8_t mask );

/** @brief  Function disable the given interrupts.
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 */
void tmf8829DisableInterrupts( tmf8829Driver * driver, uint8_t mask );

/** @brief  Function to read results and print them on UART. This function should only be called when there was a 
 * result interrupt (use function tmf8829GetAndClrInterrupts to find this out). 
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if there was a result page, else APP_ERROR_NO_RESULT.
 */
int8_t tmf8829ReadResults( tmf8829Driver * driver );

/** @brief  Function to read histograms and print them on UART. This function should only be called when there was a 
 * raw histogram interrupt (use function tmf8829GetAndClrInterrupts to find this out).
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @return Function returns APP_SUCCESS_OK if there was a histogram page, else APP_ERROR_NO_RESULT.
 */
int8_t tmf8829ReadHistogram( tmf8829Driver * driver );

/** @brief  Function to correct the distance of result data in the dataBuffer.
 * only data segments with whole pixel data is allowed
 * @param driver ... pointer to an instance of the tmf8829 driver data structure
 * @param size    ... number of bytes the data pointer points to, or the data fragment holds
 * @param layout  ... layout of the result structure
 */
void tmf8829CorrectDistanceDataSegment( tmf8829Driver * driver, uint16_t size, uint8_t layout );

/** @brief  Function to correct the distance of result data.
 * @param driver   ... pointer to an instance of the tmf8829 driver data structure
 * @param distance ... distance Value
 */
uint16_t tmf8829CorrectDistance( tmf8829Driver * driver, uint16_t distance);

/** @brief  Calculate the data pixelSize from the layout
 * @param layout ... layout of the result structure
 * @return size of pixel data
 */
uint8_t tmf8829GetPixelSize(uint8_t layout);

#ifdef __cplusplus
}
#endif  

#endif // TMF8829_H 
