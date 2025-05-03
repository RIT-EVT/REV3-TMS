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
#include <dev/Pump.hpp>
#include <dev/I2CDevice.hpp>
#include <dev/TMP117.hpp>
#include <dev/TMP117I2CDevice.hpp>

namespace io = core::io;
namespace dev = core::dev;
namespace time = core::time;
namespace log = core::log;

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

// TODO: Eliminate this global variable
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

    io::I2C& i2c = io::getI2C<io::Pin::PB_8, io::Pin::PB_9>();

    //array storing I2CDevices
    TMS::TMP117I2CDevice devices[3];

    //BUS POINTERS
    //array of buses
    TMS::TMP117I2CDevice** buses[4];
    //buses
    TMS::TMP117I2CDevice* bus0[0];
    TMS::TMP117I2CDevice* bus1[3];
    TMS::TMP117I2CDevice* bus2[1];
    TMS::TMP117I2CDevice* bus3[0];

    //set each index in buses array to be a bus
    buses[0] = bus0;
    buses[1] = bus1;
    buses[2] = bus2;
    buses[3] = bus3;

    // TODO: figure out why stuff is "implicitly deleted"
    // Set up TMS and necessary device drivers
    TMS::TMP117 tmpDevices[4];

    tmpDevices[0] = TMS::TMP117(&i2c, 0x48);
    devices[0] = TMS::TMP117I2CDevice(&tmpDevices[0], &TMS::TMS::sensorTemps[0]);
    bus2[0] = &devices[0];

    tmpDevices[1] = TMS::TMP117(&i2c, 0x48);
    devices[1] = TMS::TMP117I2CDevice(&tmpDevices[1], &TMS::TMS::sensorTemps[2]);
    bus1[0] = &devices[1];

    tmpDevices[2] = TMS::TMP117(&i2c, 0x49);
    devices[2] = TMS::TMP117I2CDevice(&tmpDevices[2], &TMS::TMS::sensorTemps[3]);
    bus1[1] = &devices[2];

    uint8_t numDevices[4] = {0, 2, 1, 0};

    TMS::TCA9545A tca(i2c, 0x70, reinterpret_cast<TMS::I2CDevice***>(buses), numDevices);

    TMS::Pump pump(io::getPWM<io::Pin::PA_6>());

    TMS::TMS tms(tca, pump);
    tmsPtr = &tms;

    ///////////////////////////////////////////////////////////////////////////
    // Setup CAN configuration, this handles making drivers, applying settings.
    // And generally creating the CANopen stack node which is the interface
    // between the application (the code we write) and the physical CAN network
    ///////////////////////////////////////////////////////////////////////////

    // Initialize the timer
    dev::Timer& timer = dev::getTimer<dev::MCUTimer::Timer16>(100);

    // Queue that will store CANopen messages
    core::types::FixedQueue<CANOPEN_QUEUE_SIZE, io::CANMessage> canOpenQueue;

    // Initialize CAN, add an IRQ that will populate the above queue
    io::CAN& can = io::getCAN<io::Pin::PA_12, io::Pin::PA_11>();
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
        time::wait(250);
    }
}
