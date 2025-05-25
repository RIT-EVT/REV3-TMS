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
#include <dev/TCA954MUX.hpp>

#ifndef NUM_TEMP_SENSORS
    #define NUM_TEMP_SENSORS 5
#endif

// Clang was removed because it adds unnecessary tabs in front of the following macros.
// clang-format off
//TODO: REMOVE ONCE SDOs ARE IN EVT-CORE!!!!
//Temporary fix for CAN SDO server to process requests!!!!
#define SDO_CONFIGURATION_1200                          \
    {                                                   \
        /* Communication Object SDO Server */           \
        .Key  = CO_KEY(0x1200, 0x00, CO_OBJ_D___R_),    \
        .Type = CO_TUNSIGNED32,                         \
        .Data = (CO_DATA) 0x02,                         \
    },                                                  \
    {                                                   \
        /* SDO Server Request COBID */                  \
        .Key  = CO_KEY(0x1200, 0x01, CO_OBJ_DN__R_),    \
        .Type = CO_TUNSIGNED32,                         \
        .Data = (CO_DATA) CO_COBID_SDO_REQUEST(),       \
    },                                                  \
    { /* SDO Server Response COBID */                   \
        .Key  = CO_KEY(0x1200, 0x02, CO_OBJ_DN__R_),    \
        .Type = CO_TUNSIGNED32,                         \
        .Data = (CO_DATA) CO_COBID_SDO_RESPONSE(),      \
    }
//TODO: REMOVE ONCE RPDO FIX IS IN EVT-CORE!!!!
//Temporary fix for RPDOs being mapped to the same data as TPDOs!!!!
#define RECEIVE_PDO_MAPPING_ENTRY_16XX(RPDO_NUMBER, SUB_INDEX, DATA_SIZE)             \
    {                                                                                 \
        .Key  = CO_KEY(0x1600 + RPDO_NUMBER, SUB_INDEX, CO_OBJ_D___R_),               \
        .Type = CO_TUNSIGNED32,                                                       \
        .Data = (CO_DATA) CO_LINK(0x2200 + RPDO_NUMBER, 0x00 + SUB_INDEX, DATA_SIZE), \
    }
// clang-format on

namespace dev = core::dev;
namespace log = core::log;
namespace io = core::io;

namespace TMS {

/**
 * Main board class for the Temperature Management System (TMS). Holds the object dictionary, handles updating the
 * temperature sensor values, and controlling the pumps.
 */
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
     * @param tca954mux I2C MUX instance to use for getting temp sensor data
     */
    TMS(TCA954MUX& tca954mux, Pump pumps[2]);

    /**
     * Array to store the thermistor values
     * Must be public, so they can be written to by the TMP117Device class
     * TODO: Refactor code to make this private
     */
    static int16_t sensorTemps[NUM_TEMP_SENSORS];

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

    /** TEMPORARY: The node ID for the VCU to identify TPDOs*/
    static constexpr uint8_t VCU_NODE_ID = 0x00;
    /** Current NMT Mode */
    CO_MODE mode = CO_PREOP;

    /** TCA9545A instance */
    TCA954MUX& tca954mux;
    /** Heat pump instance */
    Pump pumps[2];

    /** Current pump speed */
    uint8_t pumpSpeed[2] = {0, 0};
    /** Water flow rate */
    uint16_t flowRate[2] = {0, 0};

    /**
     * Have to know the size of the object dictionary for initialization
     * process
     */
    static constexpr uint16_t OBJECT_DICTIONARY_SIZE = 63;

    CO_OBJ_T objectDictionary[OBJECT_DICTIONARY_SIZE + 1] = {
        MANDATORY_IDENTIFICATION_ENTRIES_1000_1014,
        HEARTBEAT_PRODUCER_1017(2000),
        IDENTITY_OBJECT_1018,
        SDO_CONFIGURATION_1200,// Mandatory Keys

        // Temporary RPDO for VCU to control pump speed, Remove once VCU has SDO (TMS SDO is done)
        RECEIVE_PDO_SETTINGS_OBJECT_140X(0x00, 0x01, VCU_NODE_ID, RECEIVE_PDO_TRIGGER_ASYNC),

        // RPDO0 mapping for pump 1 and 2 speed, Remove once VCU has SDO (TMS SDO is done)
        RECEIVE_PDO_MAPPING_START_KEY_16XX(0, 2),
        RECEIVE_PDO_MAPPING_ENTRY_16XX(0, 1, PDO_MAPPING_UNSIGNED8),
        RECEIVE_PDO_MAPPING_ENTRY_16XX(0, 2, PDO_MAPPING_UNSIGNED8),

        // TPDO for Flow
        TRANSMIT_PDO_SETTINGS_OBJECT_18XX(0, TRANSMIT_PDO_TRIGGER_TIMER, TRANSMIT_PDO_INHIBIT_TIME_DISABLE, 1000),
        // TPDOs for Temps
        TRANSMIT_PDO_SETTINGS_OBJECT_18XX(1, TRANSMIT_PDO_TRIGGER_TIMER, TRANSMIT_PDO_INHIBIT_TIME_DISABLE, 1000),
        TRANSMIT_PDO_SETTINGS_OBJECT_18XX(2, TRANSMIT_PDO_TRIGGER_TIMER, TRANSMIT_PDO_INHIBIT_TIME_DISABLE, 1000),

        // TPDO0 mapping for flow rate
        TRANSMIT_PDO_MAPPING_START_KEY_1AXX(0, 2),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0, 1, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(0, 2, PDO_MAPPING_UNSIGNED16),

        // TPDO0 mapping for 4 temps
        TRANSMIT_PDO_MAPPING_START_KEY_1AXX(1, 4),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 1, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 2, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 3, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(1, 4, PDO_MAPPING_UNSIGNED16),

        // TPDO1 mapping for next 4 temps
        TRANSMIT_PDO_MAPPING_START_KEY_1AXX(2, 4),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(2, 1, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(2, 2, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(2, 3, PDO_MAPPING_UNSIGNED16),
        TRANSMIT_PDO_MAPPING_ENTRY_1AXX(2, 4, PDO_MAPPING_UNSIGNED16),

        // Data link 0 for flow rate
        DATA_LINK_START_KEY_21XX(0, 2),
        DATA_LINK_21XX(0, 1, CO_TUNSIGNED16, &flowRate[0]),
        DATA_LINK_21XX(0, 2, CO_TUNSIGNED16, &flowRate[1]),

        // Data link 1 for temps
        DATA_LINK_START_KEY_21XX(1, 4),
        DATA_LINK_21XX(1, 1, CO_TSIGNED16, &sensorTemps[0]),// On-board sensor
        DATA_LINK_21XX(1, 2, CO_TSIGNED16, &sensorTemps[1]),
        DATA_LINK_21XX(1, 3, CO_TSIGNED16, &sensorTemps[2]),
        DATA_LINK_21XX(1, 4, CO_TSIGNED16, &sensorTemps[3]),

        // Data link 2 for temps
        DATA_LINK_START_KEY_21XX(2, 4),
        DATA_LINK_21XX(2, 1, CO_TSIGNED16, &sensorTemps[4]),
        DATA_LINK_21XX(2, 2, CO_TSIGNED16, &sensorTemps[0]),//UNUSED
        DATA_LINK_21XX(2, 3, CO_TSIGNED16, &sensorTemps[0]),//UNUSED
        DATA_LINK_21XX(2, 4, CO_TSIGNED16, &sensorTemps[0]),//UNUSED

        // Pump Command at 0x2200
        DATA_LINK_START_KEY_21XX(0x100, 2),
        DATA_LINK_21XX(0x100, 1, CO_TUNSIGNED8, &pumpSpeed[0]),
        DATA_LINK_21XX(0x100, 2, CO_TUNSIGNED8, &pumpSpeed[1]),

        // End of dictionary marker
        CO_OBJ_DICT_ENDMARK,
    };
};

}// namespace TMS

#endif//TMS_HPP
