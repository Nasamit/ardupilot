#include <AP_HAL/AP_HAL.h>
#include "Scheduler.h"
#include "AnalogIn.h"
#include "Storage.h"

#include <stdarg.h>
#include <time.h>

#include "io.h"
#include "mcm.h"
#include "irq.h"

using namespace x86Duino;

extern const AP_HAL::HAL& hal;

#define MC_1k 3     // 1k hz timer
#define MD_1k 2

#define WDTIRQ    (7)

static int mcint_offset[3] = {0, 8, 16};
int wdt_count = 0 ;

// timer 1khz ISR'
static char* isrname_one = "timer_1k";
static int timer1k_isr_handler(int irq, void* data)
{
    if((mc_inp(MC_1k, 0x04) & (PULSE_END_INT << mcint_offset[MD_1k])) == 0) return ISR_NONE;
    mc_outp(MC_1k, 0x04, (PULSE_END_INT << mcint_offset[MD_1k]));   // clear flag

//    static uint32_t count = 0 ;
//    count++;
//    if( count%500 == 0 )
//    hal.gpio->toggle(13);
    ((Scheduler*)hal.scheduler)->_run_timer_procs();

    return ISR_HANDLED;
}

// WDT timer
static struct wdt_status {
    unsigned char ctrl_reg;   // 0xA8
    unsigned char sigsel_reg; // 0xA9
    unsigned char count0_reg; // 0xAA
    unsigned char count1_reg; // 0xAB
    unsigned char count2_reg; // 0xAC
    unsigned char stat_reg;   // 0xAD

    unsigned char reload_reg; // 0xAE
} WDT_t;
char* isrname_wdt = "TimerWDT";

void wdt_settimer(unsigned long usec) {
    unsigned long ival;
    double dval;
    dval = ((double)usec) / 30.5;
    ival = (unsigned long)dval;

    if(ival > 0x00ffffffL) ival = 0x00ffffffL;

    io_outpb(0xac, (ival >> 16) & 0x00ff);
    io_outpb(0xab, (ival >> 8) & 0x00ff);
    io_outpb(0xaa, ival & 0x00ff);
}

void _wdt_enable(void) {
    unsigned char val;
    val = io_inpb(0xa8);
    io_outpb(0xa8, val | 0x40);  // reset bit 6 to disable WDT1
}

void _wdt_disable(void) {
    unsigned char val;
    val = io_inpb(0xa8);
    io_outpb(0xa8, val & (~0x40));  // reset bit 6 to disable WDT1
}

static int timerwdt_isr_handler(int irq, void* data) {

    if((io_inpb(0xad) & 0x80) == 0) return ISR_NONE;

    io_outpb(0xad, 0x80); // clear timeout event bit
    _wdt_disable();
    wdt_count++ ;
//    static uint32_t count = 0 ;
//    count++;
//    if( count%500 == 0 )
//        hal.gpio->toggle(13);
    ((Scheduler*)hal.scheduler)->run_spi_thread();
    ((Scheduler*)hal.scheduler)->run_i2c_thread();
    _wdt_enable();

    return ISR_HANDLED;
}

Scheduler::Scheduler()
{
    _timer_1k_enable = false;
    _wdt_1k_enable = false;
    _in_timer_1k = false;
    _timer_suspended = false;
    _initialized = false;
}

void Scheduler::init()
{
    if(_timer_1k_enable) return;

    // setup Time Zone
    setenv("TZ", "", 1);   // set TZ system variable (set to GMT+0)
    tzset();    // setup time zone

    mcpwm_Disable(MC_1k, MD_1k);

    // disable_MCINT
    mc_outp(MC_1k, 0x00, mc_inp(MC_1k, 0x00) & ~(0xffL << mcint_offset[MD_1k]));  // disable mc interrupt
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) | (1L << MC_1k));
    // clear_INTSTATUS
    mc_outp(MC_1k, 0x04, 0xffL << mcint_offset[MD_1k]); //for EX

    if( !irq_InstallISR(GetMCIRQ(), timer1k_isr_handler, isrname_one) )
        printf("timer 1k hz IRQ_install fail\n");

    // enable_MCINT(MC_1k, MD_1k, PULSE_END_INT);
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << MC_1k));
    mc_outp(MC_1k, 0x00, (mc_inp(MC_1k, 0x00) & ~(0xffL<<mcint_offset[MD_1k])) | (PULSE_END_INT << mcint_offset[MD_1k]));

    mcpwm_SetWidth(MC_1k, MD_1k, 1000*SYSCLK, 0L);    // 1k hz timer loop
    mcpwm_Enable(MC_1k, MD_1k);
    _timer_1k_enable = true;

    _wdt_disable();
    // initailize WDT timer for SPI and I2C thread
    // restore current WDT1 register
    WDT_t.ctrl_reg   = io_inpb(0xA8);
    WDT_t.sigsel_reg = io_inpb(0xA9);
    WDT_t.count0_reg = io_inpb(0xAA);
    WDT_t.count1_reg = io_inpb(0xAB);
    WDT_t.count2_reg = io_inpb(0xAC);
    WDT_t.stat_reg   = io_inpb(0xAD);
    WDT_t.reload_reg = io_inpb(0xAE);

    io_outpb(0xad, 0x80); // clear timeout event bit

    // set reset signal mode to INTERRUPT
    io_outpb(0xa9, 0x05 << 4); // use IRQ7

    // set time period
    wdt_settimer(5000); // 200 hz

    // setup WDT interupt
    if(irq_Setting(WDTIRQ, IRQ_LEVEL_TRIGGER) == false)
    {
        printf("WDT IRQ Setting fail\n"); return;
    }
    if(irq_InstallISR(WDTIRQ, timerwdt_isr_handler, isrname_wdt) == false)
    {
        printf("irq_install fail\n"); return;
    }
    _wdt_enable();
    _wdt_1k_enable = true;
}

void Scheduler::delay(uint16_t ms)
{
    if (in_timerprocess()) {
        printf("ERROR: delay() from timer process\n");
        return;
    }

    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms)
    {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) // callback if delay is long enough..
        {
            if (_delay_cb)
            {
                _delay_cb();
            }
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    timer_DelayMicroseconds(us);
}

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
            uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
    printf("registed _delay_cb %p, ms %d\n",proc ,min_time_ms);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _timer_proc[_num_timer_procs] = proc;
        _num_timer_procs++;
    } else {
        hal.console->printf("Out of timer processes\n");
    }
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

AP_HAL::Device::PeriodicHandle Scheduler::register_i2c_process(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    for (uint8_t i = 0; i < _num_i2c_procs; i++) {
        if (_i2c_proc[i].cb == cb) {
            _i2c_proc[i].period_usec = period_usec ;    // update period_usec
            return (&_i2c_proc[i]);
        }
    }

    if (_num_i2c_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _i2c_proc[_num_i2c_procs].cb = cb;
        _i2c_proc[_num_i2c_procs].period_usec = period_usec;
        _i2c_proc[_num_i2c_procs].next_usec = AP_HAL::micros64()+period_usec;
        _num_i2c_procs++;
        return (&_i2c_proc[_num_i2c_procs]);
    } else {
        hal.console->printf("Out of I2C processes\n");
        return nullptr;
    }
}

AP_HAL::Device::PeriodicHandle Scheduler::register_spi_process(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    for (uint8_t i = 0; i < _num_spi_procs; i++) {
        if (_spi_proc[i].cb == cb) {
            _spi_proc[i].period_usec = period_usec ;    // update period_usec
            return (&_spi_proc[i]);
        }
    }

    if (_num_spi_procs < X86_SCHEDULER_MAX_TIMER_PROCS) {
        _spi_proc[_num_spi_procs].cb = cb;
        _spi_proc[_num_spi_procs].period_usec = period_usec;
        _spi_proc[_num_spi_procs].next_usec = AP_HAL::micros64()+period_usec;
        _num_spi_procs++;
        return (&_spi_proc[_num_spi_procs]);
    } else {
        hal.console->printf("Out of SPI processes\n");
        return nullptr;
    }
}

void Scheduler::run_io(void)
{
    _in_io_proc = true;

    // now call the IO based drivers
    for (int i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) _io_proc[i]();

    }
    ((Storage*)hal.storage)->_timer_tick();

    _in_io_proc = false;
}

void Scheduler::run_i2c_thread(void)
{
    _in_i2c_proc = true;    
    // now call the I2C device driver
    for (int i = 0; i < _num_i2c_procs; i++) {
        uint64_t now = AP_HAL::micros64() ;
        if ( now >= _i2c_proc[i].next_usec)
        {
            while( now >= _i2c_proc[i].next_usec ){
                _i2c_proc[i].next_usec += _i2c_proc[i].period_usec ;
            }
            // process!
            _i2c_proc[i].cb();
        }
    }
    _in_i2c_proc = false;
}

void Scheduler::run_spi_thread(void)
{
    _in_spi_proc = true;

    // now call the SPI device driver
    for (int i = 0; i < _num_spi_procs; i++) {
        uint64_t now = AP_HAL::micros64() ;
        if ( now >= _spi_proc[i].next_usec)
        {
            while( now >= _spi_proc[i].next_usec ){
                _spi_proc[i].next_usec += _spi_proc[i].period_usec ;
            }
            // process!
            _spi_proc[i].cb();
        }
    }
    _in_spi_proc = false;
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}

void Scheduler::resume_timer_procs()
{
    _timer_suspended = false;
}

bool Scheduler::in_timerprocess() {
    return _in_timer_1k;
}

void Scheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called"
                   "more than once");
    }
    _initialized = true;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    io_DisableINT();
    // for one, usb_detect_pin = 0 , usb_on_off_pin = 1, (usb_on_off_data = GPIO_BASE_ADDR + 7)
    unsigned short usb_on_off_data = 0xf100 + 7 ;
    char usb_on_off_pin = 1;
    io_outpb(usb_on_off_data, io_inpb(usb_on_off_data) | (1 << usb_on_off_pin));
    if(hold_in_bootloader) io_outpb(0xf21A, 0x5a); // write soft reset key
    io_outpb(0x64, 0xfe); // reboot
}

void Scheduler::_run_timer_procs()
{
    _in_timer_1k = true;
    // 1k timer schedule
    if( !_timer_suspended )
    {
        // now call the timer based drivers
        for (int i = 0; i < _num_timer_procs; i++)
        {
            if (_timer_proc[i]) _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr)
    {
        _failsafe();
    }

    _in_timer_1k = false;

    // AD timer thread
    ((AnalogIn*)hal.analogin)->update();
}
