/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"

namespace IOExpander {

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_beginRecursive(TwoWire &wire)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_printStatusRecursive(Print &output)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_printStatusHtmlRecursive(Print &output)
    {
    }

    constexpr size_t ConfigEndIterator::_sizeRecursive() const
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_pinModeRecursive(uint8_t pin, uint8_t mode)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_digitalWriteRecursive(uint8_t pin, uint8_t val)
    {
    }

    inline __attribute__((__always_inline__))
    int ConfigEndIterator::_digitalReadRecursive(uint8_t pin)
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_analogReferenceRecursive(uint8_t mode)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_analogWriteRecursive(uint8_t pin, int val)
    {
    }

    inline __attribute__((__always_inline__))
    int ConfigEndIterator::_analogReadRecursive(uint8_t pin)
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_analogWriteFreqRecursive(uint32_t freq)
    {
    }

    inline __attribute__((__always_inline__))
    constexpr void *ConfigEndIterator::_getDevicePointerRecursive(uint8_t pin) const
    {
        return nullptr;
    }

    inline __attribute__((__always_inline__))
    constexpr auto ConfigEndIterator::getDeviceByAddress(uint8_t address) -> nullptr_t const
    {
        return nullptr;
    }

    inline __attribute__((__always_inline__))
    constexpr auto ConfigEndIterator::getDeviceByPin(uint8_t pin) -> nullptr_t const
    {
        return nullptr;
    }

    inline __attribute__((__always_inline__))
    constexpr bool ConfigEndIterator::_pinMatch(uint8_t pin) const
    {
        return true;
    }

    inline __attribute__((__always_inline__))
    constexpr uint8_t ConfigEndIterator::_getDeviceIndexRecursive(uint8_t pin, uint8_t index) const
    {
        return 0xff;
    }

    inline __attribute__((__always_inline__))
    constexpr auto ConfigEndIterator::_getPinMaskRecursive(uint8_t pin) -> uint8_t const
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    auto ConfigEndIterator::_readPortARecursive(uint8_t pin) -> uint8_t
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    auto ConfigEndIterator::_readPortBRecursive(uint8_t pin) -> uint8_t
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    auto ConfigEndIterator::_readPortRecursive(uint8_t pin) -> uint8_t
    {
        return 0;
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_writePortARecursive(uint8_t pin, uint8_t)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_writePortBRecursive(uint8_t pin, uint8_t)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_writePortRecursive(uint8_t pin, uint8_t)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_attachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_detachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_interruptHandlerRecursive(void *device)
    {
    }

    inline __attribute__((__always_inline__))
    void ConfigEndIterator::_dumpPinsRecursive(Print &output)
    {
    }

}
