#include <dev/TCA9545A.hpp>

namespace TMS {

TCA9545A::TCA9545A(io::I2C& i2c, uint8_t addr, I2CDevice** buses[4],
                   uint8_t numDevices[4]) : busDevices{buses[0], buses[1], buses[2], buses[3]},
                                            i2c(i2c),
                                            numDevices{numDevices[0], numDevices[1], numDevices[2], numDevices[3]},
                                            i2cSlaveAddress(addr){};

io::I2C::I2CStatus TCA9545A::setBus(uint8_t bus, bool toggled) {
    uint8_t val = static_cast<uint8_t>(toggled);
    switch (bus) {
    case 0:
        return writeRegister(TCA9545A_BUS::BUS_0, val);
    case 1:
        return writeRegister(TCA9545A_BUS::BUS_1, val);
    case 2:
        return writeRegister(TCA9545A_BUS::BUS_2, val);
    case 3:
        return writeRegister(TCA9545A_BUS::BUS_3, val);
    default:
        return io::I2C::I2CStatus::ERROR;
    }
}

io::I2C::I2CStatus TCA9545A::writeRegister(uint8_t reg, uint8_t val) {
    return i2c.write(i2cSlaveAddress, reg);
}

io::I2C::I2CStatus TCA9545A::readRegister(uint8_t reg, uint8_t* val) {
    return i2c.readReg(i2cSlaveAddress, reg, val);
}

void TCA9545A::pollDevices() {
    for (int i = 0; i < I2C_MUX_BUS_SIZE; i++) {
        if (setBus(i, true) == io::I2C::I2CStatus::ERROR) {
            continue;
        }

        for (int j = 0; j < numDevices[i]; j++) {
            busDevices[i][j]->action();
        }
    }
}
}// namespace TMS
