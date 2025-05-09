#ifndef TMS_TMP117_HPP
#define TMS_TMP117_HPP

#include <core/io/I2C.hpp>

namespace io = core::io;

namespace TMS {

/**
 * Temp sensor for TMS
 * https://www.ti.com/lit/ds/symlink/tmp117.pdf
 */
class TMP117 {
public:
    /**
     * Temp sensor constructor
     *
     * @param i2c used to read temperature
     * @param i2cSlaveAddress address to ID the sensor on the I2C bus
     * */
    TMP117(io::I2C* i2c, uint8_t i2cSlaveAddress);

    /**
     * default constructor for instantiation of uninitialized instances
     */
    TMP117();

    /**
     * Reads the temperature
     *
     * @return temperature reading in degrees centi celsius
     */
    int16_t readTemp();

private:
    /**
     * Register for temperature values
     */
    static constexpr uint8_t TEMP_REG = 0x00;

    /**
     * Device ID
     */
    uint8_t i2cSlaveAddress = 0x48;

    /**
     * I2C instance
     */
    io::I2C* i2c;
};

}// namespace TMS

#endif//TMS_TMP117_HPP
