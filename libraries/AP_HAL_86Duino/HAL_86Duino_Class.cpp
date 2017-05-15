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
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "Storage.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
//#include "SITL_State.h"
//#include "Util.h"
#include "USBSerial.h"

#include "io.h"
#include "irq.h"
#include "pins_arduino.h"
#include "i2c.h"

using namespace x86Duino;

//static EEPROMStorage sitlEEPROMStorage;
//static SITL_State sitlState;
static Scheduler x86Scheduler;
static RCInput  x86RCInput;
static RCOutput x86RCOutput;
static AnalogIn x86AnalogIn;
static GPIO x86GPIO;
static I2CDeviceManager x86I2C;
static SPIDeviceManager x86SPI;
static Storage x86Storage;

//// use the Empty HAL for hardware we don't emulate

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
        &x86I2C,   /* i2c */
        &x86SPI,          /* spi */
        &x86AnalogIn,      /* analogin */
        &x86Storage, /* storage */
        &usbUart,   /* console */
        &x86GPIO,          /* gpio */
        &x86RCInput,       /* rcinput */
        &x86RCOutput,      /* rcoutput */
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

    int i, crossbarBase, gpioBase;
    if(io_Init() == false) return;
    timer_NowTime(); // initialize timer
    CLOCKS_PER_MICROSEC = vx86_CpuCLK();
    VORTEX86EX_CLOCKS_PER_MS = CLOCKS_PER_MICROSEC*1000UL;

    // Set IRQ4 as level-trigger
    io_outpb(0x4D0, io_inpb(0x4D0) | 0x10);

    //set corssbar Base Address
    crossbarBase = sb_Read16(SB_CROSSBASE) & 0xfffe;
    if(crossbarBase == 0 || crossbarBase == 0xfffe)
    {
        sb_Write16(SB_CROSSBASE, CROSSBARBASE | 0x01);
        crossbarBase = CROSSBARBASE;
    }

#if defined CRB_DEBUGMODE
    for(i=0; i<CRBTABLE_SIZE; i++) io_outpb(crossbarBase+i, CROSSBARTABLE[i]);
#endif

    // Force set HIGH speed ISA on SB
    sb_Write(SB_FCREG, sb_Read(SB_FCREG) | 0x8000C000L);

    // GPIO->init
    x86GPIO.init();

    // AD->init
    x86AnalogIn.init();

    // set MCM Base Address, init before using PWM in/out
    set_MMIO();
    mcmInit();
    for(int i=0; i<4; i++)
        mc_SetMode(i, MCMODE_PWM_SIFB);

    // set MCM IRQ
    if(irq_Init() == false)
    {
        printf("MCM IRQ init fail\n");
    }
    if(irq_Setting(GetMCIRQ(), IRQ_LEVEL_TRIGGER + IRQ_DISABLE_INTR) == false)
    {
        printf("MCM IRQ Setting fail\n");
    }
    Set_MCIRQ(GetMCIRQ());
//    // init wdt1
//    wdt_init();

    // USB-CDC init()
    usbUart.begin(115200);

//    _sitl_state->init(argc, argv);
    x86Scheduler.init();
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

    x86RCOutput.init();
    x86RCInput.init();

    // I2C init
//    x86I2C.init();
//    AP_HAL::OwnPtr<AP_HAL::I2CDevice> mpu = x86I2C.get_device(0 , 0x68);
//    uint8_t val = 0;
//    mpu->read_registers(0x75, &val, 1 );
//    Serial1.printf("ms:%d, Who am I 0x%x\n", AP_HAL::millis() ,val );

    // SPI test
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> mpu = x86SPI.get_device("mpu9250");
    uint8_t val = 0;
    mpu->set_read_flag(0x80);   // mpu need to set read flag
    mpu->read_registers(0x75, &val, 1 );
    Serial1.printf("SPI ms:%d, Who am I 0x%x\n", AP_HAL::millis() ,val );
//    // RC output test
//    x86RCOutput.write(CH_1, 2000);
//    x86RCOutput.write(CH_2, 1300);
//    x86RCOutput.write(CH_3, 1700);
//    x86RCOutput.write(CH_4, 2000);

//    x86RCOutput.cork();
//    while( AP_HAL::millis()/1000 < 5 ) {};
//    x86RCOutput.write(CH_2, 1000);
//    while( AP_HAL::millis()/1000 < 7 ) {};
//    x86RCOutput.write(CH_3, 1000);
//    while( AP_HAL::millis()/1000 < 9 ) {};
//    x86RCOutput.write(CH_4, 1000);
//    x86RCOutput.push();    

    // Storage test
    x86Storage.init();

    for (;;) {
        x86Scheduler.delay(10);

//        Serial1.printf("PWM %d %d %d %d\n", x86RCOutput.read(CH_1), x86RCOutput.read(CH_2),
//                       x86RCOutput.read(CH_3), x86RCOutput.read(CH_4));
        static uint32_t count = 1000;
        static int8_t sign = 1;
        count+= 20*sign;
        x86RCOutput.write(CH_1, count);
        if(count > 2000) sign = -1;
        if(count < 1000) sign = 1;

        if( x86RCInput.new_input() )
        {
            uint16_t RC_in[6];
            x86RCInput.read( RC_in, 6);
            Serial1.printf("ms:%d CH %d %d %d %d ,%d %d\n",AP_HAL::millis(),RC_in[0] ,RC_in[1] ,RC_in[2] ,RC_in[3] ,RC_in[4] ,RC_in[5]);
//            Serial1.printf("ms:%d \n",AP_HAL::millis());
            x86RCOutput.write(CH_2, RC_in[2]);
        }

        // storage test
        static uint32_t idx_w = 0 ;
        if( idx_w < 16 )
        {
            idx_w ++ ;
            uint8_t buf[1024] ;
            memset( buf, idx_w, sizeof(buf)) ;
            x86Storage.write_block(idx_w*1024, buf, sizeof(buf));
        }

//        if(AP_HAL::millis()/1000 > 15 ) x86Scheduler.reboot(1);

//        Serial1.printf("ms:%d, usb:%s\n", AP_HAL::millis(), x86GPIO.usb_connected()? "connected" : "not connect");    // @nasamit
//        Serial1.printf("ms:%d, ch:%d, value:%d\n", AP_HAL::millis(), x86AnalogIn.channel(1)->read_latest(),
//                       x86AnalogIn.channel(0)->read_latest());    // @nasamit
//        x86GPIO.toggle(13);
//        x86AnalogIn.update();


        x86Scheduler.run_i2c_thread();
        x86Scheduler.run_spi_thread();
//        callbacks->loop();
        x86Scheduler.run_io();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_86Duino hal;
    return hal;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
