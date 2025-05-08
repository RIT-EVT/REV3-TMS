#ifndef TMS_HPP
#define TMS_HPP

#include <co_core.h>
#include <core/dev/Thermistor.hpp>
#include <core/io/CANDevice.hpp>
#include <core/io/CANOpenMacros.hpp>
#include <core/io/GPIO.hpp>
#include <core/io/pin.hpp>
#include <core/utils/log.hpp>
#include <dev/Pump.hpp>
#include <dev/TCA9545A.hpp>

#define NUM_TEMP_SENSORS 5

namespace dev = core::dev;
namespace log = core::log;
namespace io = core::io;

namespace TMS {

class TMS : public CANDevice {
public:

    static constexpr io::Pin TEMP_SCL = io::Pin::PB_8;
    static constexpr io::Pin TEMP_SDA = io::Pin::PB_9;

    static constexpr io::Pin CAN_RX = io::Pin::PA_11;
    static constexpr io::Pin CAN_TX = io::Pin::PA_12;

    static constexpr io::Pin PUMP1_PWM = io::Pin::PA_6;
    static constexpr io::Pin PUMP2_PWM = io::Pin::PA_7;

   static constexpr io::Pin FLOW1_PWM = io::Pin::PB_15;
   static constexpr io::Pin FLOW2_PWM = io::Pin::PB_14;

    /**
     * Construct a TMS instance
     *
     * @param tca9545A I2C MUX instance to use for getting temp sensor data
     */
    TMS(TCA9545A& tca9545A, Pump pump);

    /**
     * Array to store the thermistor values
     * Must be public, so they can be written to by the TMP117Device class
     * TODO: Refactor code to make this private
     */
    static uint16_t sensorTemps[NUM_TEMP_SENSORS];

    CO_OBJ_T* getObjectDictionary() override;

    uint8_t getNumElements() override;

    uint8_t getNodeID() override;

    /**
     * Update temperatures and apply cooling loop controls
     */
    void process();

    /**
     * Set current NMT mode
     *
     * @param newMode New NMT mode
     */
    void setMode(CO_MODE newMode);

private:
    /** The node ID used to identify the device on the CAN network */
    static constexpr uint8_t NODE_ID = 0x02;
    /** Current NMT Mode */
    CO_MODE mode = CO_PREOP;

    /** TCA9545A instance */
    TCA9545A& tca9545A;
    /** Heat pump instance */
    Pump pump;


    /** Current heat pump speed */
    uint8_t pumpSpeed = 0;
    /** Fan 1 speed */
    uint8_t fan1Speed = 0;
    /** Fan 2 speed */
    uint8_t fan2Speed = 0;

    /**
     * Update fan and pump speeds
     */
    void applyThermalModel();

    /**
     * Have to know the size of the object dictionary for initialization
     * process
     */
    static constexpr uint16_t OBJECT_DICTIONARY_SIZE = 41;

    CO_OBJ_T objectDictionary[OBJECT_DICTIONARY_SIZE + 1] = {
        MANDATORY_IDENTIFICATION_ENTRIES_1000_1014,
        HEARTBEAT_PRODUCER_1017(2000),
        IDENTITY_OBJECT_1018,
        SDO_CONFIGURATION_1200,// Mandatory Keys

        TRANSMIT_PDO_SETTINGS_OBJECT_18XX(0, TRANSMIT_PDO_TRIGGER_TIMER, TRANSMIT_PDO_INHIBIT_TIME_DISABLE, 2000),
        TRANSMIT_PDO_SETTINGS_OBJECT_18XX(1, TRANSMIT_PDO_TRIGGER_TIMER, TRANSMIT_PDO_INHIBIT_TIME_DISABLE, 2000),

        TRANSMIT_PDO_MAPPING_START_KEY_1AXX(0, 4),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0, 1, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0, 2, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0, 3, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0, 4, PDO_MAPPING_UNSIGNED16),

        TRANSMIT_PDO_MAPPING_START_KEY_1AXX(1, 3),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 1, PDO_MAPPING_UNSIGNED8),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 2, PDO_MAPPING_UNSIGNED8),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 3, PDO_MAPPING_UNSIGNED8),

        DATA_LINK_START_KEY_21XX(0, 4),
        DATA_LINK_21XX(0, 1, CO_TUNSIGNED16, &sensorTemps[0]),
        DATA_LINK_21XX(0, 2, CO_TUNSIGNED16, &sensorTemps[1]),
        DATA_LINK_21XX(0, 3, CO_TUNSIGNED16, &sensorTemps[2]),
        DATA_LINK_21XX(0, 4, CO_TUNSIGNED16, &sensorTemps[3]),

        DATA_LINK_START_KEY_21XX(1, 3),
        DATA_LINK_21XX(1, 1, CO_TUNSIGNED8, &pumpSpeed),
        DATA_LINK_21XX(1, 2, CO_TUNSIGNED8, &fan1Speed),
        DATA_LINK_21XX(1, 3, CO_TUNSIGNED8, &fan2Speed),

        // End of dictionary marker
        CO_OBJ_DICT_ENDMARK,
    };
};

}// namespace TMS

#endif//TMS_HPP
