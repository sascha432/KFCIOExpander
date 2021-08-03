/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"
#include "Timer.h"

#if DEBUG_IOEXPANDER
#include "debug_helper_enable.h"
#else
#include "debug_helper_disable.h"
#endif

namespace IOExpander {

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::begin(TwoWire &wire)
    {
        _beginRecursive(wire);
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::begin()
    {
        _beginRecursive(Wire);
    }

    template<typename _ConfigType>
    template<bool _HtmlOutput>
    void ConfigIterator<_ConfigType>::printStatus(Print &output)
    {
        if __CONSTEXPR17 (_HtmlOutput) {
            _printStatusHtmlRecursive(output);
        }
        else {
            _printStatusRecursive(output);
        }
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::dumpPins(Print &output)
    {
        _dumpPinsRecursive(output);
    }

    template<typename _ConfigType>
    constexpr size_t ConfigIterator<_ConfigType>::size() const
    {
        return _sizeRecursive();
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::pinMode(uint8_t pin, uint8_t mode)
    {
        _pinModeRecursive(pin, mode);
    }

    template<typename _ConfigType>
    void IRAM_ATTR ConfigIterator<_ConfigType>::digitalWrite(uint8_t pin, uint8_t val)
    {
        _digitalWriteRecursive(pin, val);
    }

    template<typename _ConfigType>
    int IRAM_ATTR ConfigIterator<_ConfigType>::digitalRead(uint8_t pin)
    {
        return _digitalReadRecursive(pin);
    }

    template<typename _ConfigType>
    int ConfigIterator<_ConfigType>::analogRead(uint8_t pin)
    {
        return _analogReadRecursive(pin);
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::analogReference(uint8_t mode)
    {
        _analogReferenceRecursive(mode);
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::analogWrite(uint8_t pin, int val)
    {
        _analogWriteRecursive(pin, val);
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::analogWriteFreq(uint32_t freq)
    {
        _analogWriteFreqRecursive(freq);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void *ConfigIterator<_ConfigType>::getDevicePointer(uint8_t pin)
    {
        return _getDevicePointerRecursive(pin);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    auto ConfigIterator<_ConfigType>::getDeviceByAddress(uint8_t address) -> decltype(&_device)
    {
        return (_device.getAddress() == address) ? &_device : _next.getDeviceByAddress(address);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    auto ConfigIterator<_ConfigType>::getDeviceByType(DeviceTypeEnum type) -> decltype(&_device)
    {
        return (_device.kDeviceType == type) ? &_device : _next.getDeviceByType(type);
    }

    template<typename _ConfigType>
    bool ConfigIterator<_ConfigType>::interruptsEnabled()
    {
        return _interruptsEnabledRecursive();
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::attachInterrupt(uint8_t gpioPin, void *device, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
    {
        // __LDBG_printf("attachInterrupt gpio=%u device=%p mode=%u trigger_mode=%u", gpioPin, device, mode, triggerMode);
        _attachInterruptRecursive(device, gpioPin, pinMask, callback, mode, triggerMode);
    }

    template<typename _ConfigType>
    void ConfigIterator<_ConfigType>::detachInterrupt(uint8_t gpioPin, void *device, uint16_t pinMask)
    {
        __LDBG_printf("detachInterrupt gpio=%u device=%p", gpioPin, device);
        _detachInterruptRecursive(device, gpioPin, pinMask);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_beginRecursive(TwoWire &wire) {
        _device.begin(DeviceConfigType::kI2CAddress, &wire);
        _device.setStartPin(DeviceConfigType::kBeginPin);
        _next._beginRecursive(wire);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_dumpPinsRecursive(Print &output)
    {
        output.printf_P(PSTR("%s@0x%02x: "), _device.getDeviceName(), _device.getAddress());
        for(uint8_t i = DeviceConfigType::kBeginPin; i < DeviceConfigType::kEndPin; i++) {
            Serial.printf_P(PSTR("%u#%u=%u "), i, i - DeviceConfigType::kBeginPin, _device.analogRead(i));
        }
        Serial.print('\n');
        _next._dumpPinsRecursive(output);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_printStatusRecursive(Print &output)
    {
        output.printf_P(PSTR("%s @ I2C address 0x%02x, pin %u-%u, interrupts %s"),
            _device.getDeviceName(),
            _device.getAddress(),
            DeviceConfigType::kBeginPin,
            DeviceConfigType::kEndPin - 1,
            _device.interruptsEnabled() ? PSTR("enabled") : PSTR("disabled")
        );
        if __CONSTEXPR17 (DeviceType::kHasIsConnected) {
            if (!_device.isConnected()) {
                output.print(F(" (ERROR - Device not found!)"));
            }
        }
        output.println();
        _next._printStatusRecursive(output);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_printStatusHtmlRecursive(Print &output)
    {
        output.printf_P(PSTR(HTML_S(div) "%s @ I2C address 0x%02x"), _device.getDeviceName(), _device.getAddress());
        if __CONSTEXPR17 (DeviceType::kHasIsConnected) {
            if (!_device.isConnected()) {
                output.print(F(HTML_S(br) "ERROR - Device not found!"));
            }
        }
        output.print(F(HTML_E(div)));
        _next._printStatusHtmlRecursive(output);
    }

    template<typename _ConfigType>
    constexpr size_t ConfigIterator<_ConfigType>::_sizeRecursive() const
    {
        return _next._sizeRecursive() + 1;
    }

    template<typename _ConfigType>
    constexpr bool ConfigIterator<_ConfigType>::_pinMatch(uint8_t pin) const
    {
        return pin >= DeviceConfigType::kBeginPin && pin < DeviceConfigType::kEndPin;
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_pinModeRecursive(uint8_t pin, uint8_t mode)
    {
        if (DeviceConfigType::pinMatch(pin)) {
            _device.pinMode(pin - DeviceConfigType::kBeginPin, mode);
            return;
        }
        _next._pinModeRecursive(pin, mode);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_digitalWriteRecursive(uint8_t pin, uint8_t val)
    {
        if (DeviceConfigType::pinMatch(pin)) {
            _device.digitalWrite(pin - DeviceConfigType::kBeginPin, val);
            return;
        }
        _next._digitalWriteRecursive(pin, val);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    int ConfigIterator<_ConfigType>::_digitalReadRecursive(uint8_t pin)
    {
        if (DeviceConfigType::pinMatch(pin)) {
            return _device.digitalRead(pin - DeviceConfigType::kBeginPin);
        }
        return _next._digitalReadRecursive(pin);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    int ConfigIterator<_ConfigType>::_analogReadRecursive(uint8_t pin)
    {
        if (DeviceConfigType::pinMatch(pin)) {
            return _device.analogRead(pin - DeviceConfigType::kBeginPin);
        }
        return _next._analogReadRecursive(pin);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_analogReferenceRecursive(uint8_t mode)
    {
        _device.analogReference(mode);
        return _next._analogReferenceRecursive(mode);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_analogWriteRecursive(uint8_t pin, int val)
    {
        if (DeviceConfigType::pinMatch(pin)) {
            _device.analogWrite(pin - DeviceConfigType::kBeginPin, val);
            return;
        }
        _next._analogWriteRecursive(pin, val);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_analogWriteFreqRecursive(uint32_t freq)
    {
        _device.analogWriteFreq(freq);
        _next._analogWriteFreqRecursive(freq);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void *ConfigIterator<_ConfigType>::_getDevicePointerRecursive(uint8_t pin)
    {
        if (DeviceConfigType::pinMatch(pin)) {
            return reinterpret_cast<void *>(&_device);
        }
        _next._getDevicePointerRecursive(pin);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    bool ConfigIterator<_ConfigType>::_interruptsEnabledRecursive()
    {
        if (_device.interruptsEnabled()) {
            return true;
        }
        _next._interruptsEnabledRecursive();
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_attachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
    {
        if (device == reinterpret_cast<void *>(&_device)) {
            bool enabled = _device.interruptsEnabled();
            _device.enableInterrupts(pinMask, callback, mode, triggerMode);
            // attach the interrupt handler if interrupts are not enabled for this GPIO pin
            if (enabled == false && _device.interruptsEnabled()) {
                // set pinMode if device has a preset
                if __CONSTEXPR17 (DeviceType::kIntPinMode) {
                    ::pinMode(gpioPin, DeviceType::kIntPinMode);
                }
                // static_assert(DeviceType::kIntTriggerMode != TriggerMode::NONE, "interrupts not available");
                // __LDBG_printf("attachInterruptArg device=%s gpio=%u mode=%u pin_mode=%u", _device.getDeviceName(), gpioPin, triggerMode, _triggerMode2IntMode(triggerMode), DeviceType::kIntPinMode);
                ::attachInterruptArg(gpioPin, __interruptHandler, device, _triggerMode2IntMode(triggerMode));
            }
            return;
        }
        _next._attachInterruptRecursive(device, gpioPin, pinMask, callback, mode, triggerMode);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_detachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask)
    {
        if (device == reinterpret_cast<void *>(&_device)) {
            _device.disableInterrupts(pinMask);
            if (_device.interruptsEnabled() == false) {
                // remove interrupt handler
                __LDBG_printf("detachInterrupt device=%s gpio=%u", _device.getDeviceName(), gpioPin);
                ::detachInterrupt(gpioPin);
            }
            return;
        }
        _next._detachInterruptRecursive(device, gpioPin, pinMask);
    }

    template<typename _ConfigType>
    inline  __attribute__((__always_inline__))
    void ConfigIterator<_ConfigType>::_setInterruptFlagRecursive(void *device)
    {
        if (device == reinterpret_cast<void *>(&_device)) {
            _device.ISRHandler();
        }
        _next._setInterruptFlagRecursive(device);
    }

    template<typename _ConfigType>
    constexpr int ConfigIterator<_ConfigType>::_triggerMode2IntMode(TriggerMode mode) const
    {
        return (mode == TriggerMode::DEVICE_DEFAULT) ? _triggerMode2IntMode(DeviceType::kIntTriggerMode) : (mode == TriggerMode::ACTIVE_HIGH) ? RISING : FALLING;
    }

}
