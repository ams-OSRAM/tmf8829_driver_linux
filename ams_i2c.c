/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#include <linux/delay.h>

#include "ams_i2c.h"

int i2c_read(struct i2c_client *client, char reg, char *buf, int len)
{
  struct i2c_msg msgs[2]; // send client address and read command
  int ret;

  msgs[0].flags = 0;
  msgs[0].addr  = client->addr;
  msgs[0].len   = 1;
  msgs[0].buf   = &reg;

  msgs[1].flags = I2C_M_RD;
  msgs[1].addr  = client->addr;
  msgs[1].len   = len;
  msgs[1].buf   = buf;

  ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

  if (ret < 0) {
    dev_err(&client->dev, "read: i2c_transfer failed: %d msg_len: %u", ret, len);
  }
  if (ret != ARRAY_SIZE(msgs)) {
    dev_err(&client->dev, "read: i2c_transfer failed: exp: %d msg_len: %d", ARRAY_SIZE(msgs), ret);
    ret = -EIO;
  }
  
  return (ret < 0) ? ret : 0;

}

int i2c_write(struct i2c_client *client, char reg, const char *buf, int len)
{
  unsigned char *addr_buf;
  struct i2c_msg msg;
  int ret;

  addr_buf = kmalloc(len + 1, GFP_KERNEL);
  if (!addr_buf)
    return -ENOMEM;

  addr_buf[0] = reg;
  memcpy(&addr_buf[1], buf, len);
  msg.flags = 0;
  msg.addr = client->addr;
  msg.buf = addr_buf;
  msg.len = len + 1;

  ret = i2c_transfer(client->adapter, &msg, 1);

  if (ret != 1) {
    dev_err(&client->dev, "write: i2c_transfer failed: %d msg_len: %u", ret, len);
    ret = -EIO;
  }

  kfree(addr_buf);
  return (ret < 0) ? ret : 0;
}

int i2c_write_mask(struct i2c_client *client, char reg, char val, char mask)
{
  int ret;
  unsigned char temp;

  ret = i2c_read(client, reg, &temp, 1);
  temp &= ~mask;
  val &= mask;
  temp |= val;
  ret = i2c_write(client, reg, &temp, 1);

  return ret;
}

int i2c_get_register(struct i2c_client *client, char reg, char *value)
{
  return i2c_read(client, reg, value, sizeof(char));
}

int i2c_set_register(struct i2c_client *client, char reg, const char value)
{
  return i2c_write(client, reg, &value, sizeof(char));
}
