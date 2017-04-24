#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO

#include <stdio.h>
#include <assert.h>


//#include "AP_HAL_SITL.h"
//#include "AP_HAL_SITL_Namespace.h"
#include "HAL_86Duino_Class.h"
#include "Scheduler.h"
#include "AnalogIn.h"
#include "UARTDriver.h"
//#include "Storage.h"
//#include "RCInput.h"
//#include "RCOutput.h"
#include "GPIO.h"
//#include "SITL_State.h"
//#include "Util.h"
#include "USBSerial.h"

//#include <AP_HAL_Empty/AP_HAL_Empty.h>
//#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include "io.h"
#include "pins_arduino.h"

using namespace x86Duino;

//static EEPROMStorage sitlEEPROMStorage;
//static SITL_State sitlState;
static Scheduler x86Scheduler;
//static RCInput  sitlRCInput(&sitlState);
//static RCOutput sitlRCOutput(&sitlState);
static AnalogIn x86AnalogIn;
static GPIO x86GPIO;

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
        &x86AnalogIn,      /* analogin */
        nullptr, /* storage */
        &usbUart,   /* console */
        &x86GPIO,          /* gpio */
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

unsigned long CLOCKS_PER_MICROSEC = 300L; // The default value is 300Mhz for 86Duino, you should call init() to set it automatically.
unsigned long VORTEX86EX_CLOCKS_PER_MS = 300000L; // The default value is 300000 for 86Duino, you should call init() to set it automatically.

void HAL_86Duino::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    printf("Hello ArduPlot!!\n");   // @nasamit
    assert(callbacks);

//    int i, crossbarBase, gpioBase;
//    if(io_Init() == false) return;
//    timer_NowTime(); // initialize timer
//    CLOCKS_PER_MICROSEC = vx86_CpuCLK();
//    VORTEX86EX_CLOCKS_PER_MS = CLOCKS_PER_MICROSEC*1000UL;

//    // Set IRQ4 as level-trigger
//    io_outpb(0x4D0, io_inpb(0x4D0) | 0x10);

//    //set corssbar Base Address
//    crossbarBase = sb_Read16(SB_CROSSBASE) & 0xfffe;
//    if(crossbarBase == 0 || crossbarBase == 0xfffe)
//    {
//        sb_Write16(SB_CROSSBASE, CROSSBARBASE | 0x01);
//        crossbarBase = CROSSBARBASE;
//    }

//#if defined CRB_DEBUGMODE
//    for(i=0; i<CRBTABLE_SIZE; i++) io_outpb(crossbarBase+i, CROSSBARTABLE[i]);
//#endif

//    // Force set HIGH speed ISA on SB
//    sb_Write(SB_FCREG, sb_Read(SB_FCREG) | 0x8000C000L);

    // GPIO->init
    x86GPIO.init();

    // AD->init
    x86AnalogIn.init();

//    // set MCM Base Address
//    set_MMIO();
//    mcmInit();
//    for(i=0; i<4; i++)
//        mc_SetMode(i, MCMODE_PWM_SIFB);

//    // init wdt1
//    wdt_init();

//    if(Global_irq_Init == false)
//    {
//        // set MCM IRQ
//        if(irq_Init() == false)
//        {
//            printf("MCM IRQ init fail\n"); return false;
//        }

//        if(irq_Setting(GetMCIRQ(), IRQ_LEVEL_TRIGGER + IRQ_DISABLE_INTR) == false)
//        {
//            printf("MCM IRQ Setting fail\n"); return false;
//        }
//        Set_MCIRQ(GetMCIRQ());
//        Global_irq_Init = true;
//    }

//    // USB-CDC init()

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
    x86GPIO.pinMode(13, HAL_GPIO_OUTPUT);
    Serial1.printf("usb:%s\n", x86GPIO.usb_connected()? "connected" : "not connect");    // @nasamit


    for (;;) {
//        x86Scheduler.delay(100);
//        Serial1.printf("ms:%d, usb:%s\n", AP_HAL::millis(), x86GPIO.usb_connected()? "connected" : "not connect");    // @nasamit
//        Serial1.printf("ms:%d, ch:%d, value:%d\n", AP_HAL::millis(), x86AnalogIn.channel(1)->read_latest(),
//                       x86AnalogIn.channel(0)->read_latest());    // @nasamit
//        x86GPIO.toggle(13);
        x86AnalogIn.update();
//        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_86Duino hal;
    return hal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
