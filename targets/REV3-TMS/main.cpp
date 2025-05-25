/**
 * This is the primary file for running the Thermal Management System.
 */

#include <core/io/CANopen.hpp>
#include <core/io/UART.hpp>
#include <core/io/pin.hpp>
#include <core/io/types/CANMessage.hpp>
#include <core/manager.hpp>
#include <core/utils/log.hpp>
#include <core/utils/types/FixedQueue.hpp>

#include <TMS.hpp>
#include <dev/I2CDevice.hpp>
#include <dev/Pump.hpp>
#include <dev/TCA954MUX.hpp>
#include <dev/TMP117.hpp>

namespace io = core::io;
namespace dev = core::dev;
namespace time = core::time;
namespace log = core::log;

#define NUM_TEMP_SENSORS 5

///////////////////////////////////////////////////////////////////////////////
// EVT-core CAN callback and CAN setup. This will include logic to set
// aside CANopen messages into a specific queue
///////////////////////////////////////////////////////////////////////////////

/**
 * Interrupt handler to get CAN messages. A function pointer to this function
 * will be passed to the EVT-core CAN interface which will in turn call this
 * function each time a new CAN message comes in.
 *
 * @param message[in] The passed in CAN message that was read.
 * @param priv[in] The private data (FixedQueue<CANOPEN_QUEUE_SIZE, CANMessage>)
 */
void canInterrupt(io::CANMessage& message, void* priv) {
    auto* queue = (core::types::FixedQueue<CANOPEN_QUEUE_SIZE, io::CANMessage>*) priv;
    if (queue != nullptr) {
        queue->append(message);
    }
}

TMS::TMS* tmsPtr = nullptr;
// Keep the TMS instance up-to-date with the NMT mode
extern "C" void CONmtModeChange(CO_NMT* nmt, CO_MODE mode) {
    tmsPtr->setMode(mode);
}

int main() {
    // Initialize system
    core::platform::init();

    // Set up Logger
    io::UART& uart = io::getUART<io::Pin::UART_TX, io::Pin::UART_RX>(9600);
    log::LOGGER.setUART(&uart);
    log::LOGGER.setLogLevel(log::Logger::LogLevel::DEBUG);
    log::LOGGER.log(log::Logger::LogLevel::DEBUG, "Logger initialized.");

    io::I2C& i2c = io::getI2C<TMS::TMS::TEMP_SCL, TMS::TMS::TEMP_SDA>();

    //array storing I2CDevices
    //    TMS::TMP117I2CDevice devices[5];

    //BUS POINTERS
    //buses, Specify the number of devices on each bus here
    TMS::I2CDevice* bus0[2];
    TMS::I2CDevice* bus1[2];
    TMS::I2CDevice* bus2[1];
    TMS::I2CDevice* bus3[1];

    //array of buses
    TMS::I2CDevice** buses[4] = {bus0, bus1, bus2, bus3};

    // Setup TMS and necessary device drivers
    TMS::TMP117 devices[NUM_TEMP_SENSORS];

    // Bus 2 on-board sensor
    devices[0] = TMS::TMP117(&i2c, 0x48, &TMS::TMS::sensorTemps[0]);
    bus2[0] = &devices[0];

    // Bus 0 devices
    devices[1] = TMS::TMP117(&i2c, 0x48, &TMS::TMS::sensorTemps[1]);
    bus0[0] = &devices[1];

    devices[2] = TMS::TMP117(&i2c, 0x4A, &TMS::TMS::sensorTemps[2]);
    bus0[1] = &devices[2];

    // Bus 1 devices
    devices[3] = TMS::TMP117(&i2c, 0x48, &TMS::TMS::sensorTemps[3]);
    bus1[0] = &devices[3];

    devices[4] = TMS::TMP117(&i2c, 0x4A, &TMS::TMS::sensorTemps[4]);
    bus1[1] = &devices[4];

    // Setup MUX with all the devices
    uint8_t numDevices[4] = {2, 2, 1, 0};// Repeat Device counts on each bus
    TMS::TCA954MUX tca(i2c, 0x70, buses, numDevices);

    // Setup all the pumps
    TMS::Pump pumps[2] = {TMS::Pump(io::getPWM<TMS::TMS::PUMP1_PWM>()),
                          TMS::Pump(io::getPWM<TMS::TMS::PUMP2_PWM>())};

    // Setup main TMS instance with configured MUX and pumps
    TMS::TMS tms(tca, pumps);
    tmsPtr = &tms;

    ///////////////////////////////////////////////////////////////////////////
    // Setup CAN configuration, this handles making drivers, applying settings.
    // And generally creating the CANopen stack node which is the interface
    // between the application (the code we write) and the physical CAN network
    ///////////////////////////////////////////////////////////////////////////

    // Initialize the timer
    dev::Timer& timer = dev::getTimer<dev::MCUTimer::Timer15>(100);

    // Queue that will store CANopen messages
    core::types::FixedQueue<CANOPEN_QUEUE_SIZE, io::CANMessage> canOpenQueue;

    // Initialize CAN, add an IRQ that will populate the above queue
    io::CAN& can = io::getCAN<TMS::TMS::CAN_TX, TMS::TMS::CAN_RX>();
    can.addIRQHandler(canInterrupt, reinterpret_cast<void*>(&canOpenQueue));

    // Reserved memory for CANopen stack usage
    uint8_t sdoBuffer[CO_SSDO_N * CO_SDO_BUF_BYTE];
    CO_TMR_MEM appTmrMem[16];

    // Make drivers
    CO_IF_DRV canStackDriver;

    CO_IF_CAN_DRV canDriver;
    CO_IF_TIMER_DRV timerDriver;
    CO_IF_NVM_DRV nvmDriver;

    CO_NODE canNode;

    // Test that the board is connected to the can network
    io::CAN::CANStatus result = can.connect();
    if (result != io::CAN::CANStatus::OK) {
        log::LOGGER.log(log::Logger::LogLevel::ERROR, "Failed to connect to CAN network");
        return 1;
    }

    // Initialize all the CANOpen drivers.
    io::initializeCANopenDriver(&canOpenQueue, &can, &timer, &canStackDriver, &nvmDriver, &timerDriver, &canDriver);

    // Initialize the CANOpen node we are using.
    io::initializeCANopenNode(&canNode, &tms, &canStackDriver, sdoBuffer, appTmrMem);

    // Print any CANopen errors
    CO_ERR err = CONodeGetErr(&canNode);
    if (err != CO_ERR_NONE) {
        log::LOGGER.log(log::Logger::LogLevel::ERROR, "CANopen Error: %d", err);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Main loop
    ///////////////////////////////////////////////////////////////////////////

    while (1) {
        tms.process();
        io::processCANopenNode(&canNode);
        time::wait(1);
    }
}
