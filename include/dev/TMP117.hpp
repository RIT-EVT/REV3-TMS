#ifndef TMS_TMP117_HPP
#define TMS_TMP117_HPP

#include <core/io/I2C.hpp>
#include <dev/I2CDevice.hpp>

namespace io = core::io;

namespace TMS {

/**
 * Temp sensor for TMS
 * https://www.ti.com/lit/ds/symlink/tmp117.pdf
 */
class TMP117 : public I2CDevice {
public:
    /**
     * Temp sensor constructor
     *
     * @param[in] i2c used to read temperature
     * @param[in] i2cSlaveAddress address to ID the sensor on the I2C bus
     * */
    TMP117(io::I2C* i2c, uint8_t i2cSlaveAddress);

    /**
     * Temp sensor constructor
     *
     * @param[in] i2c used to read temperature
     * @param[in] i2cSlaveAddress address to ID the sensor on the I2C bus
     * @param[in] tempValue pointer to store the temp value in on action
     * */
    TMP117(io::I2C* i2c, uint8_t i2cSlaveAddress, int16_t* tempPtr);

    /**
     * default constructor for instantiation of uninitialized instances
     */
    TMP117();

    /**
     * Reads the temperature
     *
     * @param[out] temp the temperature reading in degrees centi-celsius
     * @return I2CStatus of the reading
     */
    io::I2C::I2CStatus readTemp(int16_t& temp);

    /**
     * Reads the sensor value and stores it in tempValue
     *
     * @return I2CStatus of the internal action
     */
    io::I2C::I2CStatus action(bool skip) override;

    /**
     * Gets the last read sensor value
     *
     * @return value of the last sensor reading
     */
    uint32_t value() override;

private:
    /**
     * Register for temperature values
     */
    static constexpr uint8_t TEMP_REG = 0x00;

    static constexpr int16_t ERROR_TEMP = -25600;

    /**
     * Device ID
     */
    uint8_t i2cSlaveAddress = 0x48;

    /**
     * I2C instance
     */
    io::I2C* i2c;

    /**
     * Address to write temperature values to
     */
    int16_t* tempPtr = nullptr;

    /**
     * Internal variable to store temperature values in
     */
    int16_t lastTempValue;
};

} // namespace TMS

#endif // TMS_TMP117_HPP
