/**
  Author: sascha_lammers@gmx.de
*/

#include "IOExpander.h"

#if DEBUG_IOEXPANDER
#include "debug_helper_enable.h"
#else
#include "debug_helper_disable.h"
#endif

namespace IOExpander {

    template<typename _DeviceBaseType, typename _DerivedClass, typename _DeviceConfigType>
    inline const __FlashStringHelper *MCP23008Base<_DeviceBaseType, _DerivedClass, _DeviceConfigType>::__regAddrName(uint8_t addr) const
    {
        switch(addr) {
            case IODIR:
                return F("IODIR");
            case IPOL:
                return F("IPOL");
            case GPINTEN:
                return F("GPINTEN");
            case DEFVAL:
                return F("DEFVAL");
            case INTCON:
                return F("INTCON");
            case IOCON:
                return F("IOCON");
            case GPPU:
                return F("GPPU");
            case INTF:
                return F("INTF");
            case INTCAP:
                return F("INTCAP");
            case GPIO:
                return F("GPIO");
        }
        static char buf[16];
        snprintf_P(buf, sizeof(buf), PSTR("#0x%02x"), addr);
        return FPSTR(buf);
    }

    template<typename _DeviceBaseType, typename _DerivedClass, typename _DeviceConfigType>
    inline const __FlashStringHelper *MCP23017Base<_DeviceBaseType, _DerivedClass, _DeviceConfigType>::__regAddrName(uint8_t addr) const
    {
        switch(addr) {
            case IODIR:
                return F("IODIRA");
            case IODIR + PORT_BANK_INCREMENT:
                return F("IODIRB");
            case IPOL:
                return F("IPOLA");
            case IPOL + PORT_BANK_INCREMENT:
                return F("IPOLB");
            case GPINTEN:
                return F("GPINTENA");
            case GPINTEN + PORT_BANK_INCREMENT:
                return F("GPINTENB");
            case DEFVAL:
                return F("DEFVALA");
            case DEFVAL + PORT_BANK_INCREMENT:
                return F("DEFVALB");
            case INTCON:
                return F("INTCONA");
            case INTCON + PORT_BANK_INCREMENT:
                return F("INTCONB");
            case IOCON:
                return F("IOCONA");
            case IOCON + PORT_BANK_INCREMENT:
                return F("IOCONB");
            case GPPU:
                return F("GPPUA");
            case GPPU + PORT_BANK_INCREMENT:
                return F("GPPUB");
            case INTF:
                return F("INTFA");
            case INTF + PORT_BANK_INCREMENT:
                return F("INTFB");
            case INTCAP:
                return F("INTCAPA");
            case INTCAP + PORT_BANK_INCREMENT:
                return F("INTCAPB");
            case GPIO:
                return F("GPIOA");
            case GPIO + PORT_BANK_INCREMENT:
                return F("GPIOB");
        }
        static char buf[16];
        snprintf_P(buf, sizeof(buf), PSTR("#0x%02x"), addr);
        return FPSTR(buf);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline MCP230XX<_DeviceBaseType, _BaseClass>::MCP230XX(uint8_t address, TwoWire *wire) :
        Base(address, wire)
    {
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::begin(uint8_t address, TwoWire *wire)
    {
        Base::_wire = wire;
        begin(address);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::begin(uint8_t address)
    {
        // default values for all register except _IODIR are 0
        Base::begin(address);
        // set all pins to input
        _IODIR = 0xffff;
        _write(IODIR, _IODIR);
        // disable pullups
        _GPPU = 0;
        _write(GPPU, _GPPU);
        // disable interrupts
        _INTCON = 0;
        _DEFVAL = 0;
        _GPINTEN = 0;
        _write(GPINTEN, _GPINTEN);
        // set gpio values to 0
        _GPIO = 0;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::begin()
    {
        begin(Base::_address);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline int MCP230XX<_DeviceBaseType, _BaseClass>::analogRead(uint8_t pinNo)
    {
        return digitalRead(pinNo);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::analogWrite(uint8_t pin, int value)
    {
        return digitalWrite(pin, value);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::digitalWrite(uint8_t pin, uint8_t value)
    {
        auto pam = _pin2PortAndMask(pin);
        if (value) {
            _GPIO[pam.port] |= pam.mask;
        }
        else  {
            _GPIO[pam.port] &= ~pam.mask;
        }
        _write8(GPIO, _GPIO, pam.port);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline uint8_t MCP230XX<_DeviceBaseType, _BaseClass>::digitalRead(uint8_t pin)
    {
        auto pam = _pin2PortAndMask(pin);
        _read8(GPIO, _GPIO, pam.port);
        return (_GPIO[pam.port] & pam.mask) ? 1 : 0;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::pinMode(uint8_t pin, uint8_t mode)
    {
        auto pam = _pin2PortAndMask(pin);
        if (mode == OUTPUT) {
            _IODIR[pam.port] &= ~pam.mask;
        }
        else {
            _IODIR[pam.port] |= pam.mask;
        }
        if (mode == INPUT_PULLUP) {
            _GPPU[pam.port] |= pam.mask;
        }
        else {
            _GPPU[pam.port] &= ~pam.mask;
        }
        _write8(IODIR, _IODIR, pam.port);
        _write8(GPPU, _GPPU, pam.port);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::enableInterrupts(uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
    {
        _IOCON.A = IOCON_MIRROR;
        if (triggerMode == TriggerMode::OPEN_DRAIN) {
            _IOCON.A |= IOCON_ODR;
        }
        else if (triggerMode == TriggerMode::ACTIVE_HIGH) {
            _IOCON.A |= IOCON_INTPOL;
        }
        _IOCON.B = _IOCON.A;
        _write8(IOCON, _IOCON, Port::A);
        _write8(IOCON, _IOCON, Port::B);

        if (mode == CHANGE) {
            _INTCON._value &= ~pinMask; // interrupt on change
        }
        else {
            // setup DEFVAL
            if (mode == FALLING) {
                _DEFVAL._value |= pinMask;
                _write(DEFVAL, _DEFVAL);
            }
            else if (mode == RISING) {
                _DEFVAL._value &= ~pinMask;
                _write(DEFVAL, _DEFVAL);
            }
            _INTCON._value |= pinMask; // compare against DEFVAL
        }
        _write(INTCON, _INTCON);

        _GPINTEN._value |= pinMask;
        _write(GPINTEN, _GPINTEN);

        ets_intr_lock();
        _callback = interruptsEnabled() ? callback : nullptr;
        _timer.detach();
        ets_intr_unlock();
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::disableInterrupts(uint16_t pinMask)
    {
        _GPINTEN._value &= ~pinMask;
        _write(GPINTEN, _GPINTEN);

        if (_GPINTEN._value == 0) {
            ets_intr_lock();
            _timer.detach();
            _callback = nullptr;
            ets_intr_unlock();
        }
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline bool MCP230XX<_DeviceBaseType, _BaseClass>::interruptsEnabled()
    {
        return _GPINTEN._value != 0;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::_write(uint8_t regAddr, Register &regValue)
    {
        beginTransmission();
        writeByte(_portAddress(regAddr, Port::A));
        writeWord(regValue._value);
        endTransmission(true);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline auto MCP230XX<_DeviceBaseType, _BaseClass>::_read(uint8_t regAddr, Register &regValue) -> DataType
    {
        beginTransmission();
        writeByte(_portAddress(regAddr, Port::A));
        if (endTransmissionAndRequestFrom(2, true)) {
            regValue._value = readWord();
        }
        return regValue._value;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::_write8(uint8_t regAddr, Register regValue, Port port)
    {
        beginTransmission();
        writeByte(_portAddress(regAddr, port));
        writeByte(regValue[port]);
        endTransmission(true);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline auto MCP230XX<_DeviceBaseType, _BaseClass>::_read8(uint8_t regAddr, Register &regValue, Port port) -> decltype(regValue[port])
    {
        beginTransmission();
        writeByte(_portAddress(regAddr, port));
        if (endTransmissionAndRequestFrom(1, true)) {
            regValue[port] = readByte();
        }
        return regValue[port];
    }

}
