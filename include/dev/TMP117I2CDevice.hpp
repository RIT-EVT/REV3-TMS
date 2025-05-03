#ifndef TMS_TMP117I2CDEVICE_HPP
#define TMS_TMP117I2CDEVICE_HPP

#include <core/io/I2C.hpp>
#include <dev/I2CDevice.hpp>
#include <dev/TMP117.hpp>

namespace TMS {

/**
 * Implementation of I2CDevice for the TMP117
 */
class TMP117I2CDevice : public I2CDevice {

public:
    /**
     * Constructor for instantiation of uninitialized instances
     */
    TMP117I2CDevice();

    /**
     * TMP117 constructor
     *
     * @param[in] tmp117 Pointer to tmp object
     * @param[in] tempValue Where the temp value should be stored
     */
    TMP117I2CDevice(TMP117* tmp117, uint16_t* tempValue);

    /**
     * Reads the sensor value and stores it in tempValue
     *
     * @return I2CStatus of the internal action
     */
    io::I2C::I2CStatus action() override;

private:
    /**
     * TMP117 instance to read temperatures from
     */
    TMP117* tmp117;

    /**
     * Address to write temperature values to
     */
    uint16_t* tempValue;
};

}// namespace TMS

#endif//TMS_TMP117I2CDEVICE_HPP
