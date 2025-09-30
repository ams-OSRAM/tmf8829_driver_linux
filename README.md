# TMF8829 Linux Reference Driver README.md

## Description

This file describes the installation of the tmf8829 linux reference driver, and how to use it.

### Compilation

The compilation was done with the ams-Osram provided image bookworm_4v0.
Linux version 6.1.73-ams #1 Mon Apr 22 19:32:02 BST 2024 2024 armv6l

## Files
    
Overlay Files
The device tree overlay files for the tmf8829 prototyped on the Raspberry Pi zero.
    - tmf8829-overlay-fpc.dts
        evm configuration with flex printed circuit, EN gpio and INT gpio, I2C
    - tmf8829-overlay-fpc-polled.dts
        evm configuration with flex printed circuit, EN gpio and INT register polled, I2C
    - tmf8829-overlay-fpc-spi.dts
        evm configuration with flex printed circuit, EN gpio and INT gpio, SPI
    - tmf8829-overlay-fpc-spi-polled.dts
        evm configuration with flex printed circuit, EN gpio and INT register polled, SPI

Module File
    - tmf8829.ko

ams Raspberry USB Sensorbridge
    - ams-usb-sensorbridge.service

TMF8829 Application Hex file
    if an application patch file is required
    - tmf8829_application.hex

## Installation 

Configuration of Raspberry Pi zero:
1. Use the right image

   bookworm_4v0 (provided by ams-Osram)

2. connect to Raspberry Pi

   (e.g. using a windows powershell)

    ssh ams@169.254.0.2
    Pw: ams
	
3. copy the right files on the Raspberry Pi /home/ams directory

   (this could be done with WinScp or with a shell and scp)
   
   - Module tmf8829.ko
   
   - Install script
     - tmf8829_install_i2c.sh for i2c communication
     <br>or
     - tmf8829_install_spi.sh for spi communication
     <br>for the polling int version
     - tmf8829_install_i2c_polled_int.sh for i2c communication
     <br>or
     - tmf8829_install_spi_polled_int.sh for spi communication
     
   - Overlay file
     - tmf8829-overlay-fpc.dtbo for i2c communication 
     - tmf8829-overlay-fpc-spi.dtbo for spi communication
     - tmf8829-overlay-fpc-polled.dtbo for i2c communication and int register polling
     - tmf8829-overlay-fpc-spi-polled.dtbo for spi communication and int register polling

   - Hex File ( if required )

   - ams-usb-sensorbridge.service

   - tmf8829_zmq_server.service
   
4. - chmod 777 tmf8829_install_i2c.sh
   <br>or
   - chmod 777 tmf8829_install_spi.sh
   <br>for the polling int version
   - chmod 777 tmf8829_install_i2c_polled_int.sh
   <br>or
   -chmod 777 tmf8829_install_spi_polled_int.sh
   
5. sudo su (Super User mode)
   
6. - ./tmf8829_install_i2c.sh
   <br>or
   - ./tmf8829_install_spi.sh
   <br>for the polling int version
   - ./tmf8829_install_i2c_polled_int.sh
   <br>or
   - ./tmf8829_install_spi_polled_int.sh

7. Optional:<br> If the python zmq server sources will be installed
   and the server should be started at boot time, enable the zmq service.
      
   - systemctl enable tmf8829_zmq_server.service

8. sync

9. reboot (reboot system)

## General commands on the Raspberry Pi

1. Connect to Raspberry Pi

    - ssh ams@169.254.0.2
    - Pw: ams

2. See pinout on Raspberry Pi

    - pinout

3. CPU information

    - cat /proc/cpuinfo
    
    Additional information
    - lscpu

    Number of cores

    - cat /proc/cpuinfo | grep processor | wc -l

4. Debug messages

    - list: dmesg
    - list and clear: sudo dmesg -c
   
5. Change Raspberry I2C frequency

    - in file: /boot/firmware/config.txt
    - reboot

6. Module Functions

    - list all modules: lsmod
    - insert module:    sudo insmod tmf8829.ko
    - remove module:    sudo rmmod tmf8829.ko
    - module info :     modinfo tmf8829.ko

7. See running tmf8829 thread for interrupt handling

    - ps -ef

8. Makefile command for TMF8829

    - make CONFIG_SENSORS_TMF8829=m

9. The overlay file could be changed

    - copy overlay file to boot directory
    
    - if overlay files are generated

        - cp /home/ams/arch/arm/boot/dts/tmf8829-overlay-fpc.dtbo /boot/overlays
        - cp /home/ams/arch/arm/boot/dts/tmf8829-overlay-fpc_polled.dtbo /boot/overlays
        - cp /home/ams/arch/arm/boot/dts/tmf8829-overlay-fpc-spi.dtbo /boot/overlays
        - cp /home/ams/arch/arm/boot/dts/tmf8829-overlay-fpc-spi-polled.dtbo /boot/overlays

    - in /boot/config.txt add the desired overlay file:

        - dtoverlay=tmf8829-overlay-fpc
        - dtoverlay=tmf8829-overlay-fpc-polled
        - dtoverlay=tmf8829-overlay-fpc-spi
        - dtoverlay=tmf8829-overlay-fpc-spi-polled

    - echo "dtoverlay=tmf8829-overlay" >> /boot/firmware/config.txt
   
10. Change and see log level
     
    - echo <level> > /proc/sys/kernel/printk
    - cat /proc/sys/kernel/printk

11. See temperature and cpu frequency

    - vcgencmd measure_temp
    - vcgencmd measure_clock arm

12. Change raspberry configuration

    - sudo raspi-config

13. see the running services

    - systemctl

## General Raspberry Information

Information for Broadcom BCM2835 GPIO (and pinmux) controller

https://www.kernel.org/doc/Documentation/devicetree/bindings/pinctrl/brcm,bcm2835-gpio.txt

Raspberry pi Documentation

https://www.raspberrypi.com/documentation/computers/raspberry-pi.html


## System File System

- I2C Bus 0 (fpc): /sys/class/i2c-adapter/i2c-0/0-0041/

### Common Attributes (tmf8829_common)

This section describes the common attributes with examples.

1. chip_enable (R/W)
    
    - See chip enabled status: cat chip_enable
    - Enable the device with the application including patch download: echo 0x01 > chip_enable
    - Disable the device:                       echo 0x00 > chip_enable
    - Enable the device in bootloader mode   :  echo 0x80 > chip_enable
    - Enable the device with ROM application :  echo 0x02 > chip_enable

2. driver_debug (R/W)
      
    - Debugging statement for driver
        - 0x00 ... no logging
        - 0x01 ... only error logging
        - 0x08 ... this is a bit-mask check for clock correction logging
        - 0x10 ... some information
        - 0x20 ... very chatty firmware
        - 0x80 ... this is a bit-mask check for i2c logging
        - 0xFF ... dump everything

    - Show debug state:  cat driver_debug (default 0x1)

    - Enable debugging:  echo 0x10 > driver_debug
    - Disable debugging: echo 0x00 > driver_debug

3. program (R)
    - Version: 
        - 0x80 ... Bootloader
        - 0x01 ... Application

    - See running program: cat program
    
4. program_version (R)
    
    - See program, major/minor/patch version, chip id and chip revision:
    cat program_version
      
5. registers (R)
    
    - Dump Registers 0x00-0xFF: cat registers
       
6. register_write (W)
    
    - example stop measurement:  echo "0x08:0xFF" > register_write
    - example irq enabling:      echo "0xe2:0x01" > register_write
                  
7. request_ram_patch (W)

    The firmware tmf8829_application.hex located in /usr/lib/firmware will be downloaded.
    Only possible if the device is in bootloader mode.

    - Download Firmware patch: echo 1 > request_ram_patch

8. serial_number (R)

    - Show serial number cat serial_number

### Application Attributes (tmf8829_app)

This section describes the application attributes with examples.
The configuration parameters are described in the Config Page tmf8829_config_page.
The application registers are described in tmf8829_application registers.
See also Datasheet.

Attributes:

1. start_measurement (R/W)

    - Show cyclic measurement is running:  cat start_measurement
         0 ... not running
         1 ... cyclic measurement running
         Note: not for single shoot measurements (period = 0)
    - Stop measurement:  echo 0 > start_measurement
    - Start measurement: echo 1 > start_measurement

2. config_custom (R/W)
    
    Do not use !!!
    Read/Write all configuration parameters from/to the device as bytestream (hex).

    - Show configuration (hex): cat config_custom
    - Set configuration (hex): echo ... > config_custom

3. period (R/W)
        
    - Show measurement period in ms (dec):  cat period
    - Set measurement period in ms(dec):    echo 33 > period

4. iterations (R/W)
  
    - Show measurement kilo iterations (dec): cat iterations
    - Set measurement kilo iterations (dec):  echo 500 > iterations
    
5. fp_mode (R/W)
    
    Should not be used !!!
    The config_mode attribute should be used!!!

    - Show the focal plane mode (dec): cat fp_mode
    - Set the focal plane mode (dec):  echo 2 > fp_mode

6. result_format (R/W)

    See description of register 0x2A TMF8829_CFG_RESULT_FORMAT.

    - Show the value of the register TMF8829_CFG_RESULT_FORMAT (dec): cat result_format
    - Set the value of the register TMF8829_CFG_RESULT_FORMAT (dec):  echo 1 > result_format

7. histograms (R/W)

    See description of register 0x2B TMF8829_CFG_DUMP_HISTOGRAMS.

    - Show histograms setting of the device (dec): cat histograms
    - Set histograms to 1 for dumping of histogram frames (dec): echo 1 > histograms
    - Set histograms to 0 for disabling the dumping of histogram frames(dec): echo 0 > histograms

8. config_mode (W)

    Command to pre-configure the device.

    - Set the configuration mode (hex):  echo 0x40 > config_mode
    
    - Options:
        - 0x40 ... CMD_LOAD_CFG_8X8
        - 0x41 ... CMD_LOAD_CFG_8X8_LONG_RANGE
        - 0x42 ... CMD_LOAD_CFG_8X8_HIGH_ACCURACY
        - 0x43 ... CMD_LOAD_CFG_16X16
        - 0x44 ... CMD_LOAD_CFG_16X16_HIGH_ACCURACY
        - 0x45 ... CMD_LOAD_CFG_32X32
        - 0x46 ... CMD_LOAD_CFG_32X32_HIGH_ACCURACY
        - 0x47 ... CMD_LOAD_CFG_48X32
        - 0x48 ... CMD_LOAD_CFG_48X32_HIGH_ACCURACY

9. clk_correction (W)

    Command to activate / deactivate the clock correction for distance data. (default activated)

    - Activate (dec):    echo 1 > clk_correction
    - Deactivate (dec):  echo 0 > clk_correction

10. app_tof_output (R)

    Reading of frames with a size greater then 4096 (PAGE SIZE of linux driver) like histogram frames are not supported.
    The misc driver needs to be used.
    Note: The output buffer is a fifo buffer with 64k (PAGE_SIZE * 16).

    Every data frame has 4 four byte header.
      Byte[0] = FrameID
      Byte[1] = frameNumber (free-running number, increments with each frame)
      Byte[2] = clk_corr_LSB
      Byte[3] = clk_corr_MSB
      Byte[4] = reserved_0 (0x77)
      Byte[5] = reserved_1 (0x77)
      Byte[6] = payload_size_LSB
      Byte[7] = payload_size_MSB

    - FrameID:
        - TMF8829_COM_RESULT__measurement_res_frame  0xAA /**< measurement result frame */
        - TMF8829_COM_RESULT__measurement_hist_frame 0xBB /**< measurement histogram frame */
        - TMF8829_COM_RESULT__measurement_header      0xFD /**< measurement frame header only; for debug only should never occur in result frame */
        - TMF8829_COM_RESULT__error_frame            0xFE /**< error frame */
        - TMF8829_COM_RESULT__no_frame               0xFF /**< no frame / unhandled interrupt */
    -  The clock correction factor is only available for result frames in Q1.15 numbering format.
       
       d = ( driver->clkCorrRatioUQ * d + (1<<14) ) >> 15;
       
    Result Data from I2C readout starting from register 0xFA. For content see datasheet.
        
    - Output data (binary) could be read: cat app_tof_output

## Misc Device Driver

The Misc device driver which is a simple character driver is used to read the measurement frames.
See ***Application Attributes app_tof_output*** .
    
- Get the measurement data: cat /dev/tof_tmf8829

## Python File 

After installation, the python folder /home/ams/tmf8829_zeromq_server_linux contains the python files for interacting with the driver.

All TMF8829 EVMs provide python sources with example files and zmq based scripts. 
The zmq-server interacts with the driver and over tcp-ports with the zmq-client.

### File Description 

See readme.md in /home/ams/tmf8829_zeromq_server_linux which has a file description for released python sources Linux driver EVM and Shield Board EVM. 

### Installation

1. Copy tmf8829_zeromq_server_linux.zip  on the Raspberry Pi /home/ams directory.
2. Unpack the tmf8829_zeromq_server_linux.zip file.
   - unzip tmf8829_zeromq_server_linux.zip -d tmf8829_zeromq_server_linux

### Example Files

The example file measurement.py could be used for a normal measurement.
Modifications could be done in the area "User Settings".

 ```bash
 python ./tmf8829_zeromq_server_linux/measurement.py
 ```

### zeromq Server

The zeromq Server is used to connect to the linux driver over tcp ports.

 ```bash
 python ./tmf8829_zeromq_server_linux.py
 ```


- See Raspberry connected with powershell:

    - Command ipconfig shows the IP

    - Command ss -tulw shows tcp ports

        | Netid | State | Recv-Q | Send-Q | Local Address:Port | Peer Address:Port | Process |
        ------- | ----- | -------| -------| -------------------| ------------------| --------|
         tcp    |LISTEN |  0     |    100 |  169.254.0.2:5557  |         0.0.0.0:* |         | 
         tcp    |LISTEN |  0     |    100 |  169.254.0.2:5558  |         0.0.0.0:* |         | 
         tcp    |LISTEN |  0     |    128 |      0.0.0.0:ssh   |         0.0.0.0:* |         | 

### zeromq Client

The logger description README.md in the folder /home/ams/tmf8829_zeromq_server_linux/client_info describes the usage of the logger and the output format. The zeromq client script could be used directly on the raspberry pi instead of the windows pc exe.

 ```bash
 python ./tmf8829_zeromq_client.py
 ```


### GUI

The TMF8829 EVM GUI could be used to see the results on a pc.
