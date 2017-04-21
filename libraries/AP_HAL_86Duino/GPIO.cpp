
#include "GPIO.h"
#include "v86clock.h"
#include "io.h"
#include "pins_arduino.h"
#include "USBSerial.h"  // for checking usb connection

using namespace x86Duino;
extern const AP_HAL::HAL& hal ;
#define TRI_STATE     (0x00)
#define PULL_UP       (0x01)
#define PULL_DOWN     (0x02)

GPIO::GPIO()
{}

void GPIO::init()
{
    int gpioBase, i;
    //set SB GPIO Base Address
    gpioBase = sb_Read16(SB_GPIOBASE) & 0xfffe;
    if(gpioBase == 0 || gpioBase == 0xfffe)
    {
        sb_Write16(SB_GPIOBASE, GPIOCTRLBASE | 0x01);
        gpioBase = GPIOCTRLBASE;
    }

    // Enable GPIO 0 ~ 9
    io_outpdw(gpioBase, 0x03ff);

    // set GPIO Port 0~7 dircetion & data Address
    for(i=0;i<10;i++)
        io_outpdw(gpioBase + (i+1)*4,((GPIODIRBASE + i*4)<<16) + GPIODATABASE + i*4);

    setPinStatus();
}

void GPIO::pinMode(uint8_t pin, uint8_t mode)
{
    int crossbar_bit;
    if(pin >= PINS || PIN86[pin].gpN == NOUSED) return;

    crossbar_bit = PIN86[pin].gpN;

    io_DisableINT();
    if (mode == INPUT)  // HAL_GPIO_INPUT
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, TRI_STATE);
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))&~(1<<(crossbar_bit%8)));
    }
    else if(mode == INPUT_PULLDOWN)
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, PULL_DOWN);
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))&~(1<<(crossbar_bit%8)));
    }
    else if (mode == INPUT_PULLUP)  // HAL_GPIO_ALT
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, PULL_UP);
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))&~(1<<(crossbar_bit%8)));
    }
    else    // HAL_GPIO_OUTPUT
        io_outpb(GPIODIRBASE + 4*(crossbar_bit/8), io_inpb(GPIODIRBASE + 4*(crossbar_bit/8))|(1<<(crossbar_bit%8)));

    io_RestoreINT();
}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t GPIO::read(uint8_t pin) {
    int crossbar_bit, val;
    if(pin >= PINS || PIN86[pin].gpN == NOUSED) return LOW;

    crossbar_bit = PIN86[pin].gpN;

//#if defined (DMP_DOS_BC) || defined (DMP_DOS_DJGPP) || defined (DMP_DOS_WATCOM)
//    if(pin == 32) timer1_pin32_isUsed = true;
//#endif

    io_DisableINT();

    if(crossbar_bit > 31)
        io_outpb(CROSSBARBASE + 0x80 + (crossbar_bit/8), 0x01);
    else if(crossbar_bit <= 31 && io_inpb(CROSSBARBASE + 0x90 + crossbar_bit) != 0x01)
    {
        io_outpb(CROSSBARBASE + 0x90 + crossbar_bit, 0x01);
//        Close_Pwm(pin);   // important!!
    }

    val = io_inpb(GPIODATABASE + 4*(crossbar_bit/8))&(1<<(crossbar_bit%8));

    io_RestoreINT();

    if(val != 0) return HIGH;
    return LOW;
}

void GPIO::write(uint8_t pin, uint8_t val)
{
    unsigned int port;
    unsigned int value;
    int crossbar_bit;
    if(pin >= PINS || PIN86[pin].gpN == NOUSED) return;

//#if defined (DMP_DOS_BC) || defined (DMP_DOS_DJGPP) || defined (DMP_DOS_WATCOM)
//    if(pin == 32) timer1_pin32_isUsed = true;
//#endif

    crossbar_bit = PIN86[pin].gpN;
    port = GPIODATABASE + 4*(crossbar_bit/8);
    value = 1<<(crossbar_bit%8);

    io_DisableINT();

    if(crossbar_bit > 31)
        io_outpb(CROSSBARBASE + 0x80 + (crossbar_bit/8), 0x01);
    else if(crossbar_bit <= 31 && io_inpb(CROSSBARBASE + 0x90 + crossbar_bit) != 0x01)
    {
        io_outpb(CROSSBARBASE + 0x90 + crossbar_bit, 0x01);
//        Close_Pwm(pin);    // important!!
    }

    if (val == LOW)
        io_outpb(port, io_inpb(port)&(~value));
    else
        io_outpb(port, io_inpb(port)|value);

    io_RestoreINT();
}

void GPIO::toggle(uint8_t pin)
{
    write( pin, !read(pin));
}

void GPIO::setPinStatus(void)
{
    int i;
    for(i=0; i<PINS; i++)
    {
        if(PIN86[i].gpMode != NOSTATUS) pinMode(i, PIN86[i].gpMode);
        if(PIN86[i].gpOutput != NOSTATUS) write(i, PIN86[i].gpOutput);
    }
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(n);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool GPIO::usb_connected(void)
{    
    return ((USBSerial*)hal.uartA)->isConnected();
}

DigitalSource::DigitalSource(uint8_t v) :
    _v(v)
{}

void DigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_v, output);
}

uint8_t DigitalSource::read() {
    return hal.gpio->read(_v);
}

void DigitalSource::write(uint8_t value) {
    return hal.gpio->write(_v,value);
}

void DigitalSource::toggle() {
    write(!read());
}
