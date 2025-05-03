#include <dev/TMP117I2CDevice.hpp>

namespace TMS {

TMP117I2CDevice::TMP117I2CDevice() {}

TMP117I2CDevice::TMP117I2CDevice(TMS::TMP117* tmp117, uint16_t* tempValue) : tmp117(tmp117),
                                                                             tempValue(tempValue) {}

io::I2C::I2CStatus TMP117I2CDevice::action() {
    *tempValue = tmp117->readTemp();
    return io::I2C::I2CStatus::OK;
}

}// namespace TMS
