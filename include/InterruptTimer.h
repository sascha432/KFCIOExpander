/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>
#include "IOExpander.h"

#if defined(ESP8266)
#include <osapi.h>
#include <user_interface.h>
#endif

extern "C" void ICACHE_FLASH_ATTR _etstimer_callback(void *arg);
extern "C" void ets_timer_done(ETSTimer *ptimer);
extern "C" bool can_yield();

namespace IOExpander {

    using Callback = std::function<bool(void)>;

    struct Timer {
        Timer(Callback callback = nullptr) : _timer({}), _callback(callback) {}
        ~Timer() {
            detach();
        }

        operator bool() const {
            return _timer.timer_arg == this;
        }

        void setCallback(Callback callback) {
            _callback = callback;
        }

        void detach() {
            if (_timer.timer_arg == this) {
                ets_timer_disarm(&_timer);
                ets_timer_done(&_timer);
                _timer.timer_arg = nullptr;
            }
        }

        void start(uint32_t interval) {
            _locked = false;
            ets_timer_setfn(&_timer, reinterpret_cast<ETSTimerFunc *>(_etstimer_callback), this);
            #if IOEXPANDER_INTERRUPT_MICROS_TIMER
                ets_timer_arm_new(&_timer, interval, true, false);
            #else
                ets_timer_arm_new(&_timer, interval, true, true);
            #endif
        }

        void run() {
            ets_intr_lock();
            if (_locked) {
                ets_intr_unlock();
                return;
            }
            _locked = true;
            ets_intr_unlock();
            if (!_callback()) {
                detach();
            }
            ets_intr_lock();
            _locked = false;
            ets_intr_unlock();
        }

        ETSTimer _timer;
        Callback _callback;
        volatile bool _locked;
    };

}

inline void ICACHE_FLASH_ATTR _etstimer_callback(void *arg)
{
    reinterpret_cast<IOExpander::Timer *>(arg)->run();
}
