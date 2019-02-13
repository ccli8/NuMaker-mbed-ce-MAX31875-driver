/*******************************************************************************
* Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include "max31875.h"
#include "max31875_cpp.h"
#include "mbed.h" 
// #include "USBSerial.h"

/******************************************************************************
 *  C++ version for MAX31875 driver                                           *
 ******************************************************************************
 */

/******************************************************************************/
MAX31875::MAX31875(I2C &i2c_bus, uint8_t slave_address):
m_i2c(i2c_bus), 
m_write_address(slave_address <<1),
m_read_address ((slave_address << 1) | 1)
{
    m_extended_format = 0;
}
 
/******************************************************************************/
MAX31875::~MAX31875(void) 
{
  /** empty block */
}

/******************************************************************************/
int MAX31875::read_reg(uint16_t *value, char reg) 
{
    int32_t ret;
    char data[2] = {0, 0};
    max31875_raw_data tmp;
     
    if (reg <= MAX31875_REG_MAX) {
        /* write to the Register Select */
        ret = m_i2c.write(m_write_address, &reg, 1, true);
        /* read the two bytes of data */
        if (ret == 0) {
            ret = m_i2c.read(m_read_address, data, 2, false);
            if (ret == 0) {
                tmp.msb = data[0];
                tmp.lsb = data[1];
                *value = tmp.uwrd;
                return MAX31875_NO_ERROR;
            } else {
                printf(
                    "%s: failed to read data: ret: %d\r\n", __func__, ret);
            }
        } else {                
            printf("%s: failed to write to Register Select: ret: %d\r\n",
                __func__, ret);
        }
    } else {
        printf("%s: register address is not correct: register: %d\r\n",
                __func__, reg);
    }                
    return MAX31875_ERROR;
}

/******************************************************************************/
float MAX31875::read_reg_as_temperature(uint8_t reg)
{
    max31875_raw_data tmp;
    float temperature;
    if (reg == MAX31875_REG_TEMPERATURE ||
        reg == MAX31875_REG_THYST_LOW_TRIP || reg == MAX31875_REG_TOS_HIGH_TRIP) {
        read_reg(&tmp.uwrd, reg);
        temperature = (float)tmp.magnitude_bits;
        if (m_extended_format)
            temperature *= MAX31875_CF_EXTENDED_FORMAT;
        else
            temperature *= MAX31875_CF_NORMAL_FORMAT;
        if (tmp.sign_bit)
            temperature = -temperature;
        return temperature;
        
    } else {
        printf("%s: register is invalid, %d r\n", __func__, reg);
        return 0;
    }
}

/******************************************************************************/
int MAX31875::write_reg(uint16_t value, char reg) 
{
    int32_t ret;
    char cmd[3];
    max31875_raw_data tmp;

    if (reg >= MAX31875_REG_CONFIGURATION && reg <= MAX31875_REG_MAX) {
        cmd[0] = reg;
        tmp.uwrd = value;
        cmd[1] = tmp.msb;
        cmd[2] = tmp.lsb;
        ret = m_i2c.write(m_write_address, cmd, 3, false);
        if (ret == 0) {
            if (MAX31875_REG_CONFIGURATION == reg) {
                m_extended_format = 0;
                if (tmp.uwrd & MAX31875_CFG_EXTENDED_FORMAT)
                    m_extended_format = 1;
            }
            return MAX31875_NO_ERROR;
        } else {
            printf("%s: I2C write error %d\r\n",__func__, ret);
            return MAX31875_ERROR;
        }
    } else {
        printf("%s: register value invalid %x\r\n",__func__, reg);
        return MAX31875_ERROR;
    }
}

/******************************************************************************/
int MAX31875::write_cfg(uint16_t cfg)
{
    return write_reg(cfg, MAX31875_REG_CONFIGURATION);
}

/******************************************************************************/
int MAX31875::write_trip_low(float temperature)
{
    max31875_raw_data raw;
    if (temperature < 0) {
        raw.sign_bit = 1;
        temperature = -temperature;
    }
    if (m_extended_format)
        temperature /= MAX31875_CF_EXTENDED_FORMAT;
    else
        temperature /= MAX31875_CF_NORMAL_FORMAT;
    raw.magnitude_bits= uint16_t(temperature);
    return write_reg(raw.uwrd, MAX31875_REG_THYST_LOW_TRIP);
}

/******************************************************************************/
int MAX31875::write_trip_high(float temperature)
{
    max31875_raw_data raw;
    if (temperature < 0) {
        raw.sign_bit = 1;
        temperature = -temperature;
    }
    if (m_extended_format)
        temperature /= MAX31875_CF_EXTENDED_FORMAT;
    else
        temperature /= MAX31875_CF_NORMAL_FORMAT;
    raw.magnitude_bits= uint16_t(temperature);
    return write_reg(raw.uwrd, MAX31875_REG_TOS_HIGH_TRIP);
}

/******************************************************************************/
float MAX31875::celsius_to_fahrenheit(float temp_c)
{
    float temp_f;
    temp_f = ((temp_c * 9)/5) + 32;
    return temp_f;
}