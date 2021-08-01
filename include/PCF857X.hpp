/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"

#if DEBUG_IOEXPANDER
#include "debug_helper_enable.h"
#else
#include "debug_helper_disable.h"
#endif

namespace IOExpander {

    namespace IOWrapper {

        // ------------------------------------------------------------------
        // PCF857X::PIN
        // ------------------------------------------------------------------

        template<typename _BaseClassType, typename _DataType>
        PCF857X<_BaseClassType, _DataType>::PIN::operator DataType() {
            return _readValue(DataType());
        }

        // ------------------------------------------------------------------
        // PCF8574::PIN
        // ------------------------------------------------------------------


        template<typename _BaseClassType, typename _DataType>
        uint8_t PCF857X<_BaseClassType, _DataType>::PIN::_readValue(uint8_t)
        {
            auto &wire = _parent->getWire();
            if (wire.requestFrom(_parent->getAddress(), 0x01U) == 1) {
                _value = wire.read();
                _parent->captureEvents(_value);
            }
            else {
                __Error_printf("_readValue requestFrom()=1 available=%u", wire.available());
            }
            // __LDBG_printf("_readValue PIN=%s DDR=%s PORT=%s _value=%s", decbin(_parent->PIN._value).c_str(), decbin(_parent->DDR._value).c_str(), decbin(_parent->PORT._value).c_str(), decbin(_value).c_str());
            return _value;
        }

        // ------------------------------------------------------------------
        // PCF8575::PIN
        // ------------------------------------------------------------------

        template<typename _BaseClassType, typename _DataType>
        uint16_t PCF857X<_BaseClassType, _DataType>::PIN::_readValue(uint16_t)
        {
            auto &wire = _parent->getWire();
            if (wire.requestFrom(_parent->getAddress(), 0x02U) == 2) {
                _value = wire.read();
                _value |= (wire.read() << 8);
                _parent->captureEvents(_value);
            }
            else {
                __Error_printf("_readValue requestFrom()=2 available=%u", wire.available());
            }
            __LDBG_printf("_readValue PIN=%04x DDR=%04x PORT=%04x _value=%04x", _parent->PIN._value, _parent->DDR._value, _parent->PORT._value, _value);
            return _value;
        }

        // ------------------------------------------------------------------
        // PCF857X::PORT
        // ------------------------------------------------------------------

        template<typename _BaseClassType, typename _DataType>
        typename PCF857X<_BaseClassType, _DataType>::PORT &PCF857X<_BaseClassType, _DataType>::PORT::operator =(DataType value) {
            _value = value;
            _updatePort(uint8_t());
            return *this;
        }

        // ------------------------------------------------------------------
        // PCF8574::PORT
        // ------------------------------------------------------------------

        template<typename _BaseClassType, typename _DataType>
        void PCF857X<_BaseClassType, _DataType>::PORT::_updatePort(uint8_t)
        {
            // set all ports in INPUT mode to high and add all values for OUTPUT ports
            DataType value = (~_parent->DDR._value) | (_value & _parent->DDR._value);
            // __LDBG_printf("write PIN=%s DDR=%s PORT=%s value=%s", decbin(_parent->PIN._value).c_str(), decbin(_parent->DDR._value).c_str(), decbin(_parent->PORT._value).c_str(), decbin(value).c_str());
            auto &wire = _parent->getWire();
            wire.beginTransmission(_parent->getAddress());
            wire.write(value);
            uint8_t error;
            if ((error = wire.endTransmission()) != 0) {
                __Error_printf("_updatePort write()=1 value=%02x error=%u", value, error);
            }
        }

        // ------------------------------------------------------------------
        // PCF8575::PORT
        // ------------------------------------------------------------------

        template<typename _BaseClassType, typename _DataType>
        void PCF857X<_BaseClassType, _DataType>::PORT::_updatePort(uint16_t)
        {
            // set all ports in INPUT mode to high and add all values for OUTPUT ports
            DataType value = (~_parent->DDR._value) | (_value & _parent->DDR._value);
            // __LDBG_printf("write PIN=%04x DDR=%04x PORT=%04x value=%04x", _parent->PIN._value, _parent->DDR._value, _parent->PORT._value, value);
            auto &wire = _parent->getWire();
            wire.beginTransmission(_parent->getAddress());
            wire.write(value & 0xff);
            wire.write(value >> 8);
            uint8_t error;
            if ((error = wire.endTransmission()) != 0) {
                __Error_printf("_updatePort write()=2 value=%04x error=%u", value, error);
            }
        }

        // ------------------------------------------------------------------
        // PCF857X::DDR
        // ------------------------------------------------------------------

        template<typename _BaseClassType, typename _DataType>
        typename PCF857X<_BaseClassType, _DataType>::DDR &PCF857X<_BaseClassType, _DataType>::DDR::operator =(DataType value) {
            _value = value;
            _updatePort();
            return *this;
        }

        template<typename _BaseClassType, typename _DataType>
        inline void PCF857X<_BaseClassType, _DataType>::DDR::_updatePort()
        {
            _parent->PORT._updatePort(DataType());
        }

    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::begin(uint8_t address, TwoWire *wire)
    {
        _wire = wire;
        begin(address);
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::begin(uint8_t address)
    {
        _address = address;
        // set all ports to input
        DDR._value = ~0;
        // set all ports to high and send settings to device
        PORT = ~0;
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::pinMode(uint8_t pin, uint8_t mode)
    {
        switch(mode) {
            case OUTPUT:
                DDR |= _BV(pin);
                break;
            case INPUT_PULLUP:
                // High-level output current: -1mA
                DDR._value &= ~_BV(pin);
                PORT |= _BV(pin);
                break;
            case INPUT:
            default:
                DDR._value &= ~_BV(pin);
                PORT &= ~_BV(pin);
                break;
        }

    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::digitalWrite(uint8_t pin, uint8_t value)
    {
        if (value) {
            PORT |= _BV(pin);
        }
        else {
            PORT &= ~_BV(pin);
        }
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline uint8_t PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::digitalRead(uint8_t pin)
    {
        return (readPort() & _BV(pin)) != 0;
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline int PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::analogRead(uint8_t pin)
    {
        return digitalRead(pin);
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::analogWrite(uint8_t pin, int val)
    {
        digitalWrite(pin, val);
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::writePort(DataType value)
    {
        PORT = value;
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline typename PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::DataType PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::readPort()
    {
        return static_cast<DataType>(PIN);
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::enableInterrupts(DataType pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
    {
        DataType mask = 0;
        for(uint8_t i = 0; i < DataTypeSizeInBits; i++) {
            if (pinMask & _BV(i)) {
                mask |= _BV(i);
                _intMode.set(i, Interrupts::mode2InterruptMode(mode));
            }
         }
         // mask state and add pin states from port
         _intMode._state = (_intMode._state & ~mask) | (readPort() & mask);
        ets_intr_lock();
        _callback = interruptsEnabled() ? callback : nullptr;
        if (!_callback) {
            _timer.detach();
        }
        ets_intr_unlock();
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline void PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::disableInterrupts(DataType pinMask)
    {
        for(uint8_t i = 0; i < DataTypeSizeInBits; i++) {
            // remove disabled pins
            if (pinMask & _BV(i)) {
                _intMode.set(i, Interrupts::InterruptModeEnum::NONE);
            }
         }
         if (!interruptsEnabled()) {
             ets_intr_lock();
            _callback = nullptr;
            _timer.detach();
            ets_intr_unlock();
         }
    }

    template<typename _BaseClass, typename _IOWrapper, typename _ConfigClass>
    inline bool PCF857X<_BaseClass, _IOWrapper, _ConfigClass>::interruptsEnabled()
    {
        return (_intMode != false);
    }

}
