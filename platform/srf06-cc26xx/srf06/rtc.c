/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sensortag-cc26xx-opt-sensor
 * @{
 *
 * \file
 *  Driver for the Sensortag-CC26xx Opt3001 light sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
//#include "lib/sensors.h"
#include "rtc.h"
#include "sys/ctimer.h"
#include "ti-lib.h"
#include "board-i2c.h"
//#include "sensor-common.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Slave address */
#define RTC_I2C_ADDRESS       0x69 

/*---------------------------------------------------------------------------*/
/* Register addresses */
#define REG_HUNDRETH           0x00
#define REG_SECOND             0x01
#define REG_MINUTE             0x02
#define REG_HOUR               0x03
#define REG_DATE               0x04
#define REG_MONTH              0x05
#define REG_YEAR               0x06
#define REG_DAY                0x07
#define REG_HUNDRETH_ALARM     0x08
#define REG_SECOND_ALARM       0x09
#define REG_MINUTE_ALARM       0x0A
#define REG_HOUR_ALARM         0x0B
#define REG_DATE_ALARM         0x0C
#define REG_MONTH_ALARM        0x0D
#define REG_DAY_ALARM          0x0E
#define REG_STATUS             0x0F

#define REG_CONTROL1           0x10
#define REG_CONTROL2           0x11
#define REG_INT_MASK           0x12
#define REG_SQW                0x13
#define REG_CAL_XT             0x14
#define REG_CAL_RC_HI          0x15
#define REG_CAL_RC_LOW         0x16
#define REG_INT_POLARITY       0x17
#define REG_TIMER_CONTROL      0x18
#define REG_TIMER              0x19
#define REG_TIMER_INITIAL      0x1A
#define REG_WDT                0x1B
#define REG_OSC_CONTROL        0x1C
#define REG_OSC_STATUS         0x1D

#define REG_CONFIG_KEY         0x1F
#define REG_TRICKLE            0x20
#define REG_BREF_CONTROL       0x21

#define REG_AFCTRL             0x26
#define REG_BATMODE            0x27

#define REG_ID0                0x28
#define REG_ID1                0x29
#define REG_ID2                0x2A
#define REG_ID3                0x2B
#define REG_ID4                0x2C
#define REG_ID5                0x2D
#define REG_ID6                0x2E
#define REG_ASTAT 	       0X2F
#define REG_OCTRL              0x30
#define REG_EXTENSION_ADDRESS  0x3F

#define TENTH_MASK                  0xF0
#define HUNDRETH_MASK               0x0F
#define SECOND_MASK                 0x7F
#define MINUTE_MASK                 0x7F
#define HOUR_24_MASK                0x3F
#define HOUR_12_MASK                0x1F
#define AM_PM_MASK                  0x20
#define DATE_MASK                   0x3F
#define MONTH_MASK                  0x1F
#define YEAR_MASK                   0xFF
#define DAY_MASK                    0x07

#define TENTH_ALARM_MASK            0xF0
#define HUNDRETH_ALARM_MASK         0xF0
#define SECOND_ALARM_MASK           0x7F
#define MINUTE_ALARM_MASK           0x7F
#define HOUR_24_ALARM_MASK          0x3F
#define HOUR_12_ALARM_MASK          0x1F
#define DATE_ALARM_MASK             0x3F
#define MONTH_ALARM_MASK            0x1F
#define WEEKDAY_ALARM_MASK          0x07
static uint8_t buffer[32];
/*---------------------------------------------------------------------------*/
/**
 * \brief Select the sensor on the I2C bus
 */

bool common_read_reg(uint8_t addr, uint8_t *buf, uint8_t len)
{
  return board_i2c_write_read(&addr, 1, buf, len);
}
bool common_write_reg(uint8_t addr, uint8_t *buf, uint8_t len)
{
  uint8_t i;
  uint8_t *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for(i = 0; i < len; i++) {
    *p++ = *buf++;
  }
  len++;

  /* Send data */
  return board_i2c_write(buffer, len);
}
bool rtc_read_ids( uint8_t *buf, uint8_t len)
{
  bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_ID0, buf,len);
  return success;
}
bool rtc_get_date(uint8_t *buf,uint8_t len)
{
    bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_DATE, buf,len);
  return success;
}
bool rtc_get_month(uint8_t *buf,uint8_t len)
{
    bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_MONTH, buf,len);
  return success;
}
bool rtc_get_year(uint8_t *buf,uint8_t len)
{
    bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_YEAR, buf,len);
  return success;
}
bool rtc_get_time(uint8_t *buf,uint8_t len)
{
    bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_SECOND, buf,len);
  return success;
}
//set
bool rtc_set_date(uint8_t date,uint8_t len)
{
  bool success;
  uint8_t val = date % 0x32;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
   success = common_write_reg(REG_DATE, &val,len);
return success;
}
bool rtc_set_month(uint8_t month,uint8_t len)
{
  bool success;
  uint8_t val = month % 0x13;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
   success = common_write_reg(REG_MONTH, &val,len);
return success;
}
bool rtc_set_year(uint8_t year,uint8_t len)
{
  bool success;
  uint8_t val = year % 0x100;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
   success = common_write_reg(REG_YEAR, &val,len);
return success;
}
bool rtc_set_seconds(uint8_t seconds,uint8_t len)
{
  bool success;
  uint8_t val = seconds % 0x60;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
   success = common_write_reg(REG_SECOND, &val,len);
return success;
}
bool rtc_set_minutes(uint8_t minutes,uint8_t len)
{
  bool success;
  uint8_t val = minutes % 0x60;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
   success = common_write_reg(REG_MINUTE, &val,len);
return success;
}
bool rtc_set_hours(uint8_t hours,uint8_t len)
{
  bool success;
  uint8_t val = hours % 0x24;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
   success = common_write_reg(REG_HOUR, &val,len);
 return success;
}
bool rtc_read_status_reg(uint8_t *status,uint8_t len)
{
  bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_STATUS, status,len);
  return success;
}
bool rtc_write_status_reg(uint8_t *status,uint8_t len)
{
  bool success;
  board_i2c_select(BOARD_I2C_INTERFACE_0, RTC_I2C_ADDRESS);
  success = common_read_reg(REG_STATUS, status,len);
  return success;
}
/** @} */
