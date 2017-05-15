#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_86Duino.h"

#define X86_SCHEDULER_MAX_TIMER_PROCS 8

class x86Duino::Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();
    void     init();
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc proc,
                uint16_t min_time_ms);

    void     register_timer_process(AP_HAL::MemberProc proc);
    void     register_io_process(AP_HAL::MemberProc proc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    bool     in_timerprocess();

    void     register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us);

    void     system_initialized();

    void     reboot(bool hold_in_bootloader);
    void    _run_timer_procs(void);
    void     run_io();

    //    void    why();
    AP_HAL::Device::PeriodicHandle    register_i2c_process(
            uint32_t period_usec, AP_HAL::Device::PeriodicCb cb);
    AP_HAL::Device::PeriodicHandle    register_spi_process(
            uint32_t period_usec, AP_HAL::Device::PeriodicCb cb);
    void    run_spi_thread();
    void    run_i2c_thread();
private:
    volatile bool _timer_1k_enable, _timer_400hz_enable;
    volatile bool _timer_suspended;
    bool    _initialized;
    AP_HAL::Proc _delay_cb;
    uint16_t _min_delay_cb_ms;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[X86_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    volatile bool _in_timer_1k;

    AP_HAL::MemberProc _io_proc[X86_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_io_procs;
    volatile bool _in_io_proc;

    volatile bool _in_i2c_proc;
    volatile bool _in_spi_proc;

};
