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
* @file max31875_cpp.h
*******************************************************************************
*/
#ifndef MAX31875_CPP_H
#define MAX31875_CPP_H
#include "mbed.h"


/**
 * @brief Extremely small low-power temperature sensor.
 * @version 1.0000.0003
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
    MAX31875(I2C &i2c_bus, uint8_t slave_address);
 
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
     * @return temperature in degrees Celsius
     */
    float read_reg_as_temperature(uint8_t reg);

    /** 
     * @brief Writes to the configuration register
     * @param cfg - configurate word
     * @return 0 on success, negative number on failure
     */
     int write_cfg(uint16_t cfg);

    /** 
     * @brief Writes to the THYST register
     * @param temperature - the temperature in Celsius degrees
     * @return 0 on success, negative number on failure
     */
     int write_trip_low(float temperature);

    /** 
     * @brief Writes to the TOS register
     * @param temperature - the temperature in Celsius degrees
     * @return 0 on success, negative number on failure
     */
     int write_trip_high(float temperature);

    /** 
     * @brief Converts Celsius degrees to Fahrenheit
     * @param temp_c - the temperature in Celsius degrees
     * @return temperature in Celsius degrees
     */
     float celsius_to_fahrenheit(float temp_c);

protected: 
    /** 
     * @brief Write a value to a register
     * @param value - value to write to the register
     * @param reg - register address
     * @return 0 on success, negative number on failure
     */
    int write_reg(uint16_t value, char reg);

private:
    /** @var m_i2c
     * @brief I2C object
     */
    I2C &m_i2c;
    /** @var m_write_address, m_read_address
     * @brief I2C address
     */
    uint8_t m_write_address, m_read_address;
    /** @var m_write_address, m_read_address
     * @brief m_extended_format
     */
    uint32_t m_extended_format;
};

#endif/* MAX31875_CPP_H */