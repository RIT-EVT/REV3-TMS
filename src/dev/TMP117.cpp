#include <cstdint>

#include <core/io/I2C.hpp>
#include <dev/TMP117.hpp>

namespace TMS {

TMP117::TMP117(io::I2C* i2c, uint8_t i2cSlaveAddress) : i2cSlaveAddress(i2cSlaveAddress), i2c(i2c) {}

TMP117::TMP117(io::I2C* i2c, uint8_t i2cSlaveAddress, int16_t* tempPtr)
    : i2cSlaveAddress(i2cSlaveAddress), i2c(i2c), tempPtr(tempPtr) {}

TMP117::TMP117() : i2c(nullptr) {}

io::I2C::I2CStatus TMP117::readTemp(int16_t& temp) {
    uint8_t tempBytes[2];

    io::I2C::I2CStatus status = i2c->readReg(i2cSlaveAddress, (uint8_t*) TEMP_REG, 1, tempBytes, 2);

    if (status == io::I2C::I2CStatus::OK) {
        temp = static_cast<int16_t>(((uint16_t) tempBytes[0]) << 8 | tempBytes[1]);
    } else {
        temp = INT16_MIN;
    }

    /**
     * degrees centi celsius
     * multiplied by 78125 because the sensor output increases by .0078125 degrees celsius - brings it to fixed point
     * within 32 bits divided by 100000 so it fits in a 16 bit int
     */
    temp = ((int64_t) temp) * 78125 / 100000;

    return status;
}

io::I2C::I2CStatus TMP117::action(bool skip = false) {
    io::I2C::I2CStatus status = io::I2C::I2CStatus::ERROR;
    if (skip) {
        lastTempValue = ERROR_TEMP;
    } else {
        status = readTemp(lastTempValue);
    }

    if (tempPtr) {
        *tempPtr = lastTempValue;
    }

    return status;
}

uint32_t TMP117::value() {
    return lastTempValue;
}

} // namespace TMS
