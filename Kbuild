EXTRA_CFLAGS := -I$(src)/aos_tmf8829_core_library/src
EXTRA_CFLAGS += -I$(src)
EXTRA_CFLAGS += -Wno-unused-function # -Wextra
obj-$(CONFIG_SENSORS_TMF8829) += tmf8829.o
tmf8829-y = tmf8829_driver.o ./aos_tmf8829_core_library/src/tmf8829.o ams_i2c.o tmf8829_shim.o tmf8829_hex_interpreter.o