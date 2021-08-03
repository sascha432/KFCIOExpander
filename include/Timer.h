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
            if (can_yield()) {
                optimistic_yield(10000);
            }
            ets_timer_setfn(&_timer, reinterpret_cast<ETSTimerFunc *>(_etstimer_callback), this);
            #if IOEXPANDER_INTERRUPT_MICROS_TIMER
                ets_timer_arm_new(&_timer, interval, true, false);
            #else
                ets_timer_arm_new(&_timer, interval, true, true);
            #endif
        }

        void run() {
            if (!_callback()) {
                detach();
            }
            else {
                if (can_yield()) {
                    yield();
                }
            }
        }

        ETSTimer _timer;
        Callback _callback;
    };

}

inline void ICACHE_FLASH_ATTR _etstimer_callback(void *arg)
{
    reinterpret_cast<IOExpander::Timer *>(arg)->run();
}