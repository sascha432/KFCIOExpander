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

    template<DeviceTypeEnum _DeviceType>
    inline const __FlashStringHelper *MCP23008::__regAddrName(uint8_t addr) {
        switch(addr) {
            case MCP23008::IODIR:
                return F("IODIR");
            case MCP23008::IPOL:
                return F("IPOL");
            case MCP23008::GPINTEN:
                return F("GPINTEN");
            case MCP23008::DEFVAL:
                return F("DEFVAL");
            case MCP23008::INTCON:
                return F("INTCON");
            case MCP23008::IOCON:
                return F("IOCON");
            case MCP23008::GPPU:
                return F("GPPU");
            case MCP23008::INTF:
                return F("INTF");
            case MCP23008::INTCAP:
                return F("INTCAP");
            case MCP23008::GPIO:
                return F("GPIO");
        }
        return F("N/A");
    }

    template<DeviceTypeEnum _DeviceType>
    inline const __FlashStringHelper *MCP23017::__regAddrName(uint8_t addr) {
        switch(addr) {
            case MCP23017::IODIR:
                return F("IODIRA");
            case MCP23017::IODIR + MCP23017::PORT_BANK_INCREMENT:
                return F("IODIRB");
            case MCP23017::IPOL:
                return F("IPOLA");
            case MCP23017::IPOL + MCP23017::PORT_BANK_INCREMENT:
                return F("IPOLB");
            case MCP23017::GPINTEN:
                return F("GPINTENA");
            case MCP23017::GPINTEN + MCP23017::PORT_BANK_INCREMENT:
                return F("GPINTENB");
            case MCP23017::DEFVAL:
                return F("DEFVALA");
            case MCP23017::DEFVAL + MCP23017::PORT_BANK_INCREMENT:
                return F("DEFVALB");
            case MCP23017::INTCON:
                return F("INTCONA");
            case MCP23017::INTCON + MCP23017::PORT_BANK_INCREMENT:
                return F("INTCONB");
            case MCP23017::IOCON:
                return F("IOCONA");
            case MCP23017::IOCON + MCP23017::PORT_BANK_INCREMENT:
                return F("IOCONB");
            case MCP23017::GPPU:
                return F("GPPUA");
            case MCP23017::GPPU + MCP23017::PORT_BANK_INCREMENT:
                return F("GPPUB");
            case MCP23017::INTF:
                return F("INTFA");
            case MCP23017::INTF + MCP23017::PORT_BANK_INCREMENT:
                return F("INTFB");
            case MCP23017::INTCAP:
                return F("INTCAPA");
            case MCP23017::INTCAP + MCP23017::PORT_BANK_INCREMENT:
                return F("INTCAPB");
            case MCP23017::GPIO:
                return F("GPIOA");
            case MCP23017::GPIO + MCP23017::PORT_BANK_INCREMENT:
                return F("GPIOB");
        }
        return F("N/A");
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline MCP230XX<_DeviceBaseType, _BaseClass>::MCP230XX(uint8_t address, TwoWire *wire) :
        Base(address, wire),
        _interruptsPending(0)
    {
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::begin(uint8_t address, TwoWire *wire)
    {
        _wire = wire;
        begin(address);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::begin(uint8_t address)
    {
        // default values for all register except _IODIR are 0
        _address = address;
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
    inline int MCP230XX<_DeviceBaseType, _BaseClass>::analogRead(uint8_t pinNo)
    {
        return digitalRead(pinNo);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline bool MCP230XX<_DeviceBaseType, _BaseClass>::analogWrite(uint8_t pin, uint8_t value)
    {
        return false;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::digitalWrite(uint8_t pin, uint8_t value)
    {
        auto pam = _pin2PortAndMask(pin);
        _GPIO.set(pam.port, pam.mask, value);
        _write8(GPIO, _GPIO, pam.port);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline uint8_t MCP230XX<_DeviceBaseType, _BaseClass>::digitalRead(uint8_t pin)
    {
        auto pam = _pin2PortAndMask(pin);
        _read8(GPIO, _GPIO, pam.port);
        return _GPIO.get(pam.port, pam.mask) ? 1 : 0;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline uint8_t MCP230XX<_DeviceBaseType, _BaseClass>::_readPortA()
    {
        _read8(GPIO, _GPIO, Port::A);
        return _GPIO[Port::A];
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline uint8_t MCP230XX<_DeviceBaseType, _BaseClass>::_readPortB()
    {
        _read8(GPIO, _GPIO, Port::B);
        return _GPIO[Port::B];
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline uint16_t MCP230XX<_DeviceBaseType, _BaseClass>::_readPortAB()
    {
        _read(GPIO, _GPIO);
        return _GPIO;
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::writePortA(uint8_t value)
    {
        _GPIO.reset<Port::A>(value);
        _write8(GPIO, _GPIO, Port::A);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::writePortB(uint8_t value)
    {
        _GPIO.reset<Port::B>(value);
        _write8(GPIO, _GPIO, Port::B);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::writePortAB(uint16_t value)
    {
        _GPIO = value;
        _write(GPIO, _GPIO);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::pinMode(uint8_t pin, uint8_t mode)
    {
        __LDBG_printf("pinMode %u=%u", pin, mode);
        auto pam = _pin2PortAndMask(pin);
        _IODIR.set(pam.port, pam.mask, mode != OUTPUT);
        _GPPU.set(pam.port, pam.mask, mode == INPUT_PULLUP);
        _write8(IODIR, _IODIR, pam.port);
        _write8(GPPU, _GPPU, pam.port);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::enableInterrupts(uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
    {
        ets_intr_lock();
        _interruptsPending = 0;
        _callback = callback;
        ets_intr_unlock();
        _IOCON.reset<Port::A>(IOCON_MIRROR);
        if (triggerMode == TriggerMode::OPEN_DRAIN) {
            _IOCON.set<Port::A>(IOCON_ODR);
        }
        else if (triggerMode == TriggerMode::ACTIVE_HIGH) {
            _IOCON.set<Port::A>(IOCON_INTPOL);
        }
        _write8(IOCON, _IOCON, Port::A);

        if (mode == CHANGE) {
            _INTCON &= ~pinMask; // interrupt on change
        }
        else {
            // setup DEFVAL
            if (mode == FALLING) {
                _DEFVAL |= pinMask;
                _write(DEFVAL, _DEFVAL);
            }
            else if (mode == RISING) {
                _DEFVAL &= ~pinMask;
                _write(DEFVAL, _DEFVAL);
            }
            _INTCON._value |= pinMask; // compare against DEFVAL
        }
        _write(INTCON, _INTCON);

        _GPINTEN._value |= pinMask;
        _write(GPINTEN, _GPINTEN);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::disableInterrupts(uint16_t pinMask)
    {
        _GPINTEN._value &= ~pinMask;
        _write(GPINTEN, _GPINTEN);

        if (_GPINTEN._value == 0) {
            _INTCON = 0;
            _DEFVAL = 0;
            _read(INTCAP, _INTCAP); // clear pending interrupts
            _INTCAP = 0;

            ets_intr_lock();
            _interruptsPending = 0;
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
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::invokeCallback()
    {
        // read captured ports that have PINs with interrupts
        if (_GPINTEN[Port::A] && _GPINTEN[Port::B]) {
            _read(INTCAP, _INTCAP);
        }
        else if (_GPINTEN[Port::A]) {
            _read8(INTCAP, _INTCAP, Port::A);
        }
        if (_GPINTEN[Port::B]) {
            _read8(INTCAP, _INTCAP, Port::B);
        }
        auto port = Register(readPortAB());

        ::printf("INTCAP A=%02x B=%02X PORT A=%02x B=%02X\n", _INTCAP[Port::A], _INTCAP[Port::B], port[Port::A], port[Port::B]);

        _callback(static_cast<uint16_t>(_INTCAP)); // TODO read captured pin state
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::interruptHandler()
    {
        uint32_t start = micros();
        ets_intr_lock();
        while (_interruptsPending) {
            _interruptsPending = 0;
            ets_intr_unlock();

            invokeCallback();

            // check again if any new interupts occured while processing
            ets_intr_lock();
            if (_interruptsPending) {
                if (micros() - start > 10000) {
                    // abort after 2ms and reschedule
                    schedule_function([this]() {
                        interruptHandler();
                    });
                    break;
                }
            }
        }
        ets_intr_unlock();
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::_write(uint8_t regAddr, Register &regValue)
    {
        uint8_t error;
        regAddr = _portAddress<Port::A>(regAddr);
        _wire->beginTransmission(_address);
        _wire->write(regAddr);
        _wire->write(reinterpret_cast<uint8_t *>(&regValue._value), 2);
        if ((error = _wire->endTransmission(true)) != 0) {
            __Error_printf("_write reg_addr=%s A=%02x B=%02x error=%u", __regAddrName<kDeviceType>(regAddr), regValue.A, regValue.B, error);
        }
        // __LDBG_printf("_write %s A=%02x B=%02x", __regAddrName(regAddr), regValue.A, regValue.B);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::_read(uint8_t regAddr, Register &regValue)
    {
        uint8_t error;
        regAddr = _portAddress<Port::A>(regAddr);
        _wire->beginTransmission(_address);
        _wire->write(regAddr);
        if ((error = _wire->endTransmission(false)) != 0) {
            __Error_printf("_read write()=1 reg_addr=%s error=%u", __regAddrName<kDeviceType>(regAddr), error);
            return;
        }
        if (_wire->requestFrom(_address, 2U, true) != 2) {
            __Error_printf("_read requestFrom()=2 reg_addr=%s available=%u", __regAddrName<kDeviceType>(regAddr), _wire->available());
            return;
        }
        _wire->readBytes(reinterpret_cast<uint8_t *>(&regValue._value), 2);
        // __LDBG_printf("_read reg_addr=%s A=%02x B=%02x", __regAddrName(regAddr), regValue.A, regValue.B);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::_write8(uint8_t regAddr, Register regValue, Port port)
    {
        uint8_t error;
        auto source = regValue[port];
        regAddr = _portAddress(regAddr, port);
        _wire->beginTransmission(_address);
        _wire->write(regAddr);
        _wire->write(source);
        if ((error = _wire->endTransmission(true)) != 0) {
            __Error_printf("_write8 write()=2 reg_addr=%s value=%02x error=%u", __regAddrName<kDeviceType>(regAddr), source, error);
        }
        // __LDBG_printf("_write8 reg_addr=%s %c=%02x", __regAddrName(regAddr), port == Port::A ? 'A' : 'B', source);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline void MCP230XX<_DeviceBaseType, _BaseClass>::_read8(uint8_t regAddr, Register &regValue, Port port)
    {
        uint8_t error;
        auto &target = regValue[port];
        _wire->beginTransmission(_address);
        regAddr = _portAddress(regAddr, port);
        _wire->write(regAddr);
        if ((error = _wire->endTransmission(false)) != 0) {
            __Error_printf("_read8 write()=1 reg_addr=%s error=%u", __regAddrName<kDeviceType>(regAddr), error);
            return;
        }
        if (_wire->requestFrom(_address, 1U, true) != 1) {
            __Error_printf("_read8 requestFrom()=1 reg_addr=%s available=%u", __regAddrName<kDeviceType>(regAddr), _wire->available());
            return;
        }
        target = _wire->read();
        // __LDBG_printf("_read8 reg_addr=%s %c=%02x", __regAddrName(regAddr), port == Port::A ? 'A' : 'B', target);
    }

    template<typename _DeviceBaseType, typename _BaseClass>
    inline  __attribute__((__always_inline__))
    bool MCP230XX<_DeviceBaseType, _BaseClass>::setInterruptFlag() {
        _interruptsPending++;
        return _interruptsPending - 1;
    }

}
