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
* @file MAX31875.h
*******************************************************************************
*/
#ifndef MAX31875_H
#define MAX31875_H
#include "mbed.h"

#define MAX31875_NO_ERROR   0
#define MAX31875_ERROR      -1

#define MAX31875_REG_TEMPERATURE   0X00
#define MAX31875_REG_CONFIGURATION 0X01
#define MAX31875_REG_THYST         0X02
#define MAX31875_REG_TOS           0X03
#define MAX31875_REG_MAX           0X03

#define MAX31875_CFG_ONE_SHOT_START   (1) /* Start one-shot measurement */

#define MAX31875_CFG_CONV_RATE_0_25   (0x00 << 1) /* 0.25 conversions/sec */
#define MAX31875_CFG_CONV_RATE_1      (0x01 << 1) /* 1.0 conversions/sec */
#define MAX31875_CFG_CONV_RATE_4      (0x02 << 1) /* 4.0 conversions/sec */
#define MAX31875_CFG_CONV_RATE_8      (0x03 << 1) /* 8.0 conversions/sec */
#define MAX31875_WAIT_CONV_RATE_0_25 (4.0)
#define MAX31875_WAIT_CONV_RATE_1    (1.0)
#define MAX31875_WAIT_CONV_RATE_4    (0.25)
#define MAX31875_WAIT_CONV_RATE_8    (0.125)

#define MAX31875_CFG_RESOLUTION_9BIT  (0x00 << 5)
#define MAX31875_CFG_RESOLUTION_10BIT (0x01 << 5)
#define MAX31875_CFG_RESOLUTION_11BIT (0x02 << 5)
#define MAX31875_CFG_RESOLUTION_12BIT (0x03 << 5)

#define MAX31875_CFG_NORMAL_FORMAT    (0X00 << 7)
#define MAX31875_CFG_EXTENDED_FORMAT  (0X01 << 7)

#define MAX31875_CFG_CONTINUOUS       (0X00 << 8)
#define MAX31875_CFG_SHUTDOWN         (0X01 << 8)

#define MAX31875_CFG_COMPARATOR_MODE  (0X00 << 9)
#define MAX31875_CFG_INTERRUPT_MODE   (0X01 << 9)


#define MAX31875_CFG_FAULT_FILTER_1   (0x00 << 11)
#define MAX31875_CFG_FAULT_FILTER_2   (0x01 << 11)
#define MAX31875_CFG_FAULT_FILTER_4   (0x02 << 11)
#define MAX31875_CFG_FAULT_FILTER_6   (0x03 << 11)

#define MAX31875_CFG_OVER_TEMP_MASK   (0x80)


#define MAX31875_I2C_SLAVE_ADR_R0 (0x90 >> 1) // code uses the 7 bit address
#define MAX31875_I2C_SLAVE_ADR_R1 (0x92 >> 1)
#define MAX31875_I2C_SLAVE_ADR_R2 (0x94 >> 1)
#define MAX31875_I2C_SLAVE_ADR_R3 (0x96 >> 1)
#define MAX31875_I2C_SLAVE_ADR_R4 (0x98 >> 1)
#define MAX31875_I2C_SLAVE_ADR_R5 (0x9A >> 1)
#define MAX31875_I2C_SLAVE_ADR_R6 (0x9C >> 1)
#define MAX31875_I2C_SLAVE_ADR_R7 (0x9E >> 1)

#define MAX31875_CF_NORMAL_FORMAT (0.00390625F)
#define MAX31875_CF_EXTENDED_FORMAT (0.0078125F)

/**
 * @brief Extremely small low-power temperature sensor.
 * @version 1.0000.0000
 *
 * @details The MAX31875 is a small WLP package temperature sensor.
 * It supports high, low triggers stored in EEPROM for hystersis
 * or limit alarms using comparator or interrupt mode.
 * The MAX31875 can operate in shutdown and one-shot mode for
 * extremely low power applications.
 * Tiny size of 0.84 x 0.84 x 0.35 mm.
 * Accuracy is +-1.5°C from +10°C to +45°C  (±0.5°C Typical),
 *             +-2.0°C from -10°C to +100°C (±0.6°C Typical),
 *             +-3.0°C from -20°C to +125°C (±1.0°C Typical),
 * I2C data rate of up to 1 MHz.
 *
 * @code 
 * #include "mbed.h"
 * #include "max32630fthr.h"
 * #include "max31875.h"
 * #include "USBSerial.h"
 * MAX32630FTHR pegasus(MAX32630FTHR::VIO_1V8); 
 * I2C i2cBus(P3_4, P3_5);
 * int main()
 * {
 *     uint16_t raw;
 *     float temperature;
 *     DigitalOut rLED(LED1, LED_OFF);
 *     DigitalOut gLED(LED2, LED_OFF);
 *     DigitalOut bLED(LED3, LED_OFF);
 *     gLED = LED_ON;
 *     MAX31875 temp_sensor(i2cBus, MAX31875_I2C_SLAVE_ADR_R0);
 *    i2cBus.frequency(1000000);
 *    temp_sensor.write_reg(uint16_t(MAX31875_CFG_CONV_RATE_8 | 
 *        MAX31875_CFG_RESOLUTION_12BIT), MAX31875_REG_CONFIGURATION);
 *    wait(MAX31875_WAIT_CONV_RATE_8);
 *     temperature = temp_sensor.read_reg_as_temperature(MAX31875_REG_TEMPERATURE);
 *     printf("Temperature = %3.4f Celsius, %3.4f Fahrenheit\r\n", 
 *             temperature, temp_sensor.celsius_to_fahrenheit(temperature));
 * }
 * @endcode
 */

class MAX31875
{
    public:

    /**********************************************************//**
     * @brief Constructor for MAX31875 Class.  
     * 
     * @details Allows user to use existing I2C object
     *
     * On Entry:
     *     @param[in] i2c_bus - pointer to existing I2C object
     *     @param[in] i2c_adrs - 7-bit slave address of MAX31875
     *
     * On Exit:
     *
     * @return None
     **************************************************************/
    MAX31875(I2C &i2c_bus, uint8_t slaveAddress);
 
    /**********************************************************//**
     * @brief Default destructor for MAX31875 Class.  
     *
     * @details Destroys I2C object if owner 
     *
     * On Entry:
     *
     * On Exit:
     *
     * @return None
     **************************************************************/
    ~MAX31875();

    /**
     * @brief  Read register of device at slave address
     * @param[out] value - Read data on success
     * @param reg - Register address
     * @return 0 on success, negative number on failure
     */
    int read_reg(uint16_t *value, char reg);

    /** 
     * @brief Reads the temperature registers
     * @param reg - the address of the temperature register
     * @return temprature in degrees Celsius
     */
    float read_reg_as_temperature(uint8_t reg);

    /** 
     * @brief Writes the configuration register
     * @return 0 on success, negative number on failure
     */
     int write_cfg(uint16_t cfg);

    /** 
     * @brief Writes the THYST register
     * @return 0 on success, negative number on failure
     */
     int write_trip_low(float temperature);

    /** 
     * @brief Writes the TOS register
     * @return 0 on success, negative number on failure
     */
     int write_trip_high(float temperature);

    /** 
     * @brief Converts Celcius degrees to Fahrenheit
     * @param temp_c - the temperature in Celsius degrees
     * @return 0 on success, negative number on failure
     */
     float celsius_to_fahrenheit(float temp_c);

protected: 
    /** union data structure for byte word manipulations */
    union max31875_raw_data {
        struct {
            uint8_t lsb;
            uint8_t msb;
        };
        uint16_t uwrd;
        int16_t swrd;
    };

    /** 
     * @brief Write a value to a register
     * @param value - value to write to the register
     * @param reg - register address
     * @return 0 on success, negative number on failure
     */
    int write_reg(uint16_t value, char reg);

private:
    /** I2C object */
    I2C &m_i2c;
    /** Device slave addresses */
    uint8_t m_writeAddress, m_readAddress;
    /** Extended Data Format flag. */
    uint32_t m_extended_format;
};
 
#endif/* MAX31875_H */