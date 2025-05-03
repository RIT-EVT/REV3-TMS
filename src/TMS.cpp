#include <TMS.hpp>

namespace TMS {

uint16_t TMS::sensorTemps[NUM_TEMP_SENSORS] = {};

TMS::TMS(TCA9545A& tca9545A, Pump pump) : tca9545A(tca9545A), pump(pump) {}

CO_OBJ_T* TMS::getObjectDictionary() {
    return &objectDictionary[0];
}
uint8_t TMS::getNumElements() {
    return OBJECT_DICTIONARY_SIZE;
}
uint8_t TMS::getNodeID() {
    return TMS::NODE_ID;
}

void TMS::process() {
    log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Updating Temps");
    tca9545A.pollDevices();

    for(int i = 0; i < NUM_TEMP_SENSORS; i++) {
        log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Temp #%d: %d", i, sensorTemps[i]);
    }

    switch (mode) {
    // Auxiliary Mode
    case CO_PREOP:
        // Turn the pump and fans off
        pump.stop();

        break;
    // Operational Mode
    case CO_OPERATIONAL:
        // Set the cooling controls
        applyThermalModel();

        break;
    default:
        log::LOGGER.log(log::Logger::LogLevel::ERROR, "Network Management state is not valid.");
    }
}

void TMS::setMode(CO_MODE newMode) {
    mode = newMode;
}

void TMS::applyThermalModel() {
    // Command devices to execute the control policy
    pump.setSpeed(pumpSpeed);
}

}// namespace TMS
