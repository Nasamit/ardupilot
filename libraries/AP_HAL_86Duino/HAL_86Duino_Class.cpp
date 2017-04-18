#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO

#include <stdio.h>
//#include <assert.h>


//#include "AP_HAL_SITL.h"
//#include "AP_HAL_SITL_Namespace.h"
#include "HAL_86Duino_Class.h"
#include "Scheduler.h"
//#include "AnalogIn.h"
#include "UARTDriver.h"
//#include "Storage.h"
//#include "RCInput.h"
//#include "RCOutput.h"
//#include "GPIO.h"
//#include "SITL_State.h"
//#include "Util.h"
#include "USBSerial.h"

//#include <AP_HAL_Empty/AP_HAL_Empty.h>
//#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

using namespace x86Duino;

//static EEPROMStorage sitlEEPROMStorage;
//static SITL_State sitlState;
static Scheduler x86Scheduler;
//static RCInput  sitlRCInput(&sitlState);
//static RCOutput sitlRCOutput(&sitlState);
//static AnalogIn sitlAnalogIn(&sitlState);
//static GPIO sitlGPIO(&sitlState);

//// use the Empty HAL for hardware we don't emulate
//static Empty::I2CDeviceManager i2c_mgr_instance;
//static Empty::SPIDeviceManager emptySPI;
//static Empty::OpticalFlow emptyOpticalFlow;

static UARTDriver Serial1(COM1, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static UARTDriver Serial2(COM2, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static UARTDriver Serial3(COM3, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static UARTDriver Serial485(COM4, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
static USBSerial usbUart;

//static Util utilInstance(&sitlState);

HAL_86Duino::HAL_86Duino() :
    AP_HAL::HAL(
        &usbUart,   /* uartA */
        &Serial1,   /* uartB */
        &Serial2,   /* uartC */
        &Serial3,   /* uartD */
        &Serial485,   /* uartE */
        nullptr,   /* uartF */
        nullptr,
        nullptr,          /* spi */
        nullptr,      /* analogin */
        nullptr, /* storage */
        &usbUart,   /* console */
        nullptr,          /* gpio */
        nullptr,       /* rcinput */
        nullptr,      /* rcoutput */
        &x86Scheduler,     /* scheduler */
        nullptr,      /* util */
        nullptr) /* onboard optical flow */
//        &sitlUart0Driver,   /* uartA */
//        &sitlUart1Driver,   /* uartB */
//        &sitlUart2Driver,   /* uartC */
//        &sitlUart3Driver,   /* uartD */
//        &sitlUart4Driver,   /* uartE */
//        &sitlUart5Driver,   /* uartF */
//        &i2c_mgr_instance,
//        &emptySPI,          /* spi */
//        &sitlAnalogIn,      /* analogin */
//        &sitlEEPROMStorage, /* storage */
//        &sitlUart0Driver,   /* console */
//        &sitlGPIO,          /* gpio */
//        &sitlRCInput,       /* rcinput */
//        &sitlRCOutput,      /* rcoutput */
//        &sitlScheduler,     /* scheduler */
//        &utilInstance,      /* util */
//        &emptyOpticalFlow), /* onboard optical flow */
//    _sitl_state(&sitlState)
{}

void HAL_86Duino::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    printf("Hello ArduPlot!!\n");   // @nasamit
//    assert(callbacks);

//    _sitl_state->init(argc, argv);
//    scheduler->init();
//    uartA->begin(115200);

//    rcin->init();
//    rcout->init();

//    // spi->init();
//    analogin->init();

//    callbacks->setup();
//    scheduler->system_initialized();

    Serial1.begin(115200);

    for (;;) {
        x86Scheduler.delay(100);
        Serial1.printf("ms:%d\n", AP_HAL::millis());    // @nasamit
//        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_86Duino hal;
    return hal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
