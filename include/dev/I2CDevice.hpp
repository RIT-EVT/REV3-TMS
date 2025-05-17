#ifndef TMS_I2CDEVICE_HPP
#define TMS_I2CDEVICE_HPP

#include <core/io/I2C.hpp>
#include <cstddef>

namespace io = core::io;
namespace TMS {

/**
 * Interface for devices using I2C to call a common function
 */
class I2CDevice {
public:
    /**
     * Performs an action utilizing I2C
     *
     * @return Status of I2C call made
     */
    virtual io::I2C::I2CStatus action() = 0;

    /**
     * Gets a previously retrieved value
     *
     * @return The last value
     */
    virtual uint32_t value() = 0;
};

}// namespace TMS

#endif//TMS_I2CDEVICE_HPP
