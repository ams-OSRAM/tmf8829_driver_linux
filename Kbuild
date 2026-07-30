ccflags-y := -I$(src)/aos_tmf8829_core_library/src
ccflags-y += -I$(src)
ccflags-y += -Wno-unused-function # -Wextra
#ccflags-y += -DUSE_I2C
ccflags-y += -DUSE_SPI
obj-$(CONFIG_SENSORS_TMF8829) += tmf8829.o
tmf8829-y := tmf8829_driver.o aos_tmf8829_core_library/src/tmf8829.o ams_i2c.o tmf8829_shim.o tmf8829_hex_interpreter.o