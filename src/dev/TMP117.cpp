#include <core/io/I2C.hpp>
#include <dev/TMP117.hpp>

namespace TMS {

TMP117::TMP117(io::I2C* i2c, uint8_t i2cSlaveAddress) : tempReg(TEMP_REG), i2cSlaveAddress(i2cSlaveAddress), i2c(i2c) {}

TMP117::TMP117() : tempReg(TEMP_REG), i2cSlaveAddress(0), i2c(nullptr) {}

uint16_t TMP117::readTemp() {
    uint16_t input;
    uint8_t tempBytes[2];
    uint16_t temp;

    io::I2C::I2CStatus status = i2c->readReg(i2cSlaveAddress, &tempReg, 1, tempBytes, 2);

    if (status == io::I2C::I2CStatus::OK) {
        input = ((uint16_t) tempBytes[0]) << 8 | tempBytes[1];
    } else {
        input = 0;
    }

    /**
     * degrees centi celsius
     * multiplied by 78125 because the sensor output increases by .0078125 degrees celsius - brings it to fixed point within 64 bits
     * divided by 100000 so it fits in a 16 bit int
     */
    temp = ((uint64_t) input) * 78125 / 100000;

    return temp;
}
}// namespace TMS
