#include <TMS.hpp>

namespace TMS {

int16_t TMS::sensorTemps[NUM_TEMP_SENSORS] = {};

TMS::TMS(TCA9545A& tca9545A, Pump pumps[2]) : tca9545A(tca9545A), pumps{pumps[0], pumps[1]} {}

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
    static uint32_t lastUpdate = 0;

    tca9545A.pollDevices();
#ifdef EVT_CORE_LOG_ENABLE
    if (time::millis() - lastUpdate > 100) {
        lastUpdate = time::millis();
        log::LOGGER.log(log::Logger::LogLevel::DEBUG, "[%d] Updating!", lastUpdate);
        for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
            log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Temp #%d: %d", i, sensorTemps[i]);
        }
        for (int i = 0; i < 2; i++) {
            log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Pump #%d: %d", i, pumpSpeed[i]);
        }
        for (int i = 0; i < 2; i++) {
            log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Flow #%d: %d", i, flowRate[i]);
        }
    }
#endif

    switch (mode) {
    // Auxiliary Mode
    case CO_PREOP:
        // Turn the pump and fans off
        pumps[0].stop();
        pumps[1].stop();

        break;
    // Operational Mode
    case CO_OPERATIONAL:
        // Set the cooling controls
        pumps[0].setSpeed(pumpSpeed[0]);
        pumps[1].setSpeed(pumpSpeed[1]);

        flowRate[0] = pumpSpeed[0];
        flowRate[1] = pumpSpeed[1];

        break;
    default:
        log::LOGGER.log(log::Logger::LogLevel::ERROR, "Network Management state is not valid.");
    }
}

void TMS::setMode(CO_MODE newMode) {
    mode = newMode;
}

}// namespace TMS
