    /**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino_compat.h>
#include "IOExpander.h"

#if HAVE_IOEXPANDER

#if DEBUG_IOEXPANDER
#include "debug_helper_enable.h"
#else
#include "debug_helper_disable.h"
#endif

namespace IOExpander {

    #if IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS == 0

        #ifdef IOEXPANDER_DEVICE_CONFIG

            #define LT <
            #define GT >

            ConfigIterator<IOEXPANDER_DEVICE_CONFIG> config;

            #undef LT
            #undef GT

        #else

        ConfigEndIterator config;

        #endif

    #endif

    void scanBus(Print &output, TwoWire &wire, uint8_t fromAddress, uint8_t toAddress, uint32_t delayMillis)
    {
        output.printf_P(PSTR("scanning address range 0x%02x-0x%02x:\n"), fromAddress, toAddress);
        for(uint8_t address = fromAddress; address <= toAddress; address++) {
            wire.beginTransmission(address);
            uint8_t error = wire.endTransmission(true);
            if (error == 0) {
                output.printf_P(PSTR("device @ 0x%02x\n"), address);
            }
            delay(delayMillis);
        }
    }

    #if !HAVE_KFC_FIRMWARE_VERSION

        void ___DBG_printf(nullptr_t, bool, const char *, ...)
        {
        }

        void ___DBG_printf(Stream &output, bool errorsOnly, const char *msg, ...)
        {
            va_list arg;
            va_start(arg, msg);
            char temp[128];
            vsnprintf_P(temp, sizeof(temp) - 1, msg, arg);
            temp[sizeof(temp) - 1] = 0;
            va_end(arg);
            if __CONSTEXPR17 (errorsOnly) {
                output.print(F("ERROR: "));
            }
            output.println(temp);
        }

    #endif

    #pragma GCC optimize ("O2")

    void IRAM_ATTR __interruptHandler(void *arg)
    {
        config._setInterruptFlagRecursive(arg);
    }

}

#if IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS

    #if IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS == 1
        #error IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS requires IOExpander::config
    #endif

    #if defined(ESP8266)
    #include <core_esp8266_version.h>
    #include <core_esp8266_waveform.h>
    #endif

extern "C" {

    #pragma GCC optimize ("O2")

    void __pinMode(uint8_t pin, uint8_t mode);
    void __digitalWrite(uint8_t pin, uint8_t val);
    int __digitalRead(uint8_t pin);
    void __analogWrite(uint8_t pin, int val);
    int __analogRead(uint8_t pin);

    void pinMode(uint8_t pin, uint8_t mode)
    {
        if (pin < IOExpander::kDigitalPinCount) {
            __pinMode(pin, mode);
            return;
        }
        IOExpander::config.pinMode(pin, mode);
    }

    #if defined(ESP8266)

        // digitalRead/digitalWrite are a copy to avoid any extra CPU cycles to check for the pin number

        void IRAM_ATTR digitalWrite(uint8_t pin, uint8_t val)
        {
            if (pin < 16) {
                stopWaveform(pin);
                #if ARDUINO_ESP8266_MAJOR == 3
                    _stopPWM(pin);
                #endif
                if (val) {
                    GPOS = (1 << pin);
                }
                else {
                    GPOC = (1 << pin);
                }
            }
            else if (pin == 16) {
                stopWaveform(pin);
                #if ARDUINO_ESP8266_MAJOR == 3
                    _stopPWM(pin);
                #endif
                if (val) {
                    GP16O |= 1;
                }
                else {
                    GP16O &= ~1;
                }
            }
            else {
                IOExpander::config.digitalWrite(pin, val);
            }
        }

        int IRAM_ATTR digitalRead(uint8_t pin)
        {
            if (pin < 16) {
                return GPIP(pin);
            }
            else if (pin == 16) {
                return GP16I & 0x01;
            }
            return IOExpander::config.digitalRead(pin);
        }

    #else

        void IRAM_ATTR digitalWrite(uint8_t pin, uint8_t val)
        {
            if (pin < IOExpander::kDigitalPinCount) {
                __digitalWrite(pin, val);
                return;
            }
            IOExpander::config.digitalWrite(pin, val);
        }

        int IRAM_ATTR digitalRead(uint8_t pin)
        {
            if (pin < IOExpander::kDigitalPinCount) {
                return __digitalRead(pin);
            }
            return IOExpander::config.digitalRead(pin);
        }

    #endif

    void analogWrite(uint8_t pin, int val)
    {
        if (pin < IOExpander::kDigitalPinCount) {
            __analogWrite(pin, val);
            return;
        }
        IOExpander::config.analogWrite(pin, val);
    }

    int analogRead(uint8_t pin)
    {
        if (pin < IOExpander::kDigitalPinCount) {
            return __analogRead(pin);
        }
        return IOExpander::config.analogRead(pin);
    }

}

#endif

#endif
