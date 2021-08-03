/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"

namespace IOExpander {

    namespace MCP230XXHelpers {

        enum class Port : uint8_t {
            A,
            B,
        };

        struct PortAndMask8 {
            static constexpr Port port = Port::A;
            uint8_t mask;
            PortAndMask8(Port _port, uint8_t _mask) : mask(_mask) {}
        };

        struct PortAndMask16 {
            Port port;
            uint8_t mask;
            PortAndMask16(Port _port, uint8_t _mask) : port(_port), mask(_mask) {}
        };

        struct Register16 {
            union {
                uint16_t _value;
                struct __attribute__((__packed__)) {
                    uint8_t A;
                    uint8_t B;
                };
            };

            // 16bit operations

            Register16() : _value(0) {}
            Register16(uint16_t value) : _value(value) {}

            // operator uint8_t *() {
            //     return reinterpret_cast<uint8_t *>(this);
            // }

            // operator const uint8_t *() const {
            //     return reinterpret_cast<const uint8_t *>(this);
            // }

            inline  __attribute__((__always_inline__))
            operator uint16_t() const {
                return _value;
            }

            // inline  __attribute__((__always_inline__))
            // Register16 &operator =(uint16_t value) {
            //     _value = value;
            //     return *this;
            // }

            // inline  __attribute__((__always_inline__))
            // Register16 &operator |=(uint16_t value) {
            //     _value |= value;
            //     return *this;
            // }

            // inline  __attribute__((__always_inline__))
            // Register16 &operator &=(uint16_t value) {
            //     _value &= value;
            //     return *this;
            // }

            // 8bit operations

            inline  __attribute__((__always_inline__))
            uint8_t &operator [](Port port) {
                return port == Port::A ? A : B;
            }

            inline  __attribute__((__always_inline__))
            uint8_t operator [](Port port) const {
                return port == Port::A ? A : B;
            }

            // inline  __attribute__((__always_inline__))
            // uint8_t get(Port port, uint8_t mask) {
            //     return operator[](port) & mask;
            // }

            // inline  __attribute__((__always_inline__))
            // void set(Port port, uint8_t mask) {
            //     operator[](port) |= mask;
            // }

            // inline  __attribute__((__always_inline__))
            // void unset(Port port, uint8_t mask) {
            //     operator[](port) &= ~mask;
            // }

            // inline  __attribute__((__always_inline__))
            // void set(Port port, uint8_t mask, bool value) {
            //     if (value) {
            //         set(port, mask);
            //     }
            //     else {
            //         unset(port, mask);
            //     }
            // }

            // template<Port _Port>
            // inline  __attribute__((__always_inline__))
            // uint8_t get(uint8_t mask) {
            //     return (_Port == Port::A ? A : B) & mask;
            // }

            // template<Port _Port>
            // inline  __attribute__((__always_inline__))
            // void reset(uint8_t value) {
            //     (_Port == Port::A ? A : B) = value;
            // }

            // template<Port _Port>
            // inline  __attribute__((__always_inline__))
            // void set(uint8_t mask) {
            //     (_Port == Port::A ? A : B) |= mask;
            // }

            // template<Port _Port>
            // inline  __attribute__((__always_inline__))
            // void unset(uint8_t mask) {
            //     (_Port == Port::A ? A : B) &= ~mask;
            // }
        };

        struct Register8 {
            uint8_t _value;

            Register8() : _value(0) {}
            Register8(uint8_t value) : _value(value) {}

            inline  __attribute__((__always_inline__))
            operator uint8_t() const {
                return _value;
            }

            inline  __attribute__((__always_inline__))
            Register8 &operator =(uint8_t value) {
                _value = value;
                return *this;
            }

            inline  __attribute__((__always_inline__))
            Register8 &operator |=(uint8_t value) {
                _value |= value;
                return *this;
            }

            inline  __attribute__((__always_inline__))
            Register8 &operator &=(uint8_t value) {
                _value &= value;
                return *this;
            }

            inline  __attribute__((__always_inline__))
            uint8_t &operator [](Port port) {
                return _value;
            }

            inline  __attribute__((__always_inline__))
            uint8_t operator [](Port port) const {
                return _value;
            }

            inline  __attribute__((__always_inline__))
            uint8_t get(Port port, uint8_t mask) {
                return _value & mask;
            }

            inline  __attribute__((__always_inline__))
            void set(Port port, uint8_t mask) {
                _value |= mask;
            }

            inline  __attribute__((__always_inline__))
            void unset(Port port, uint8_t mask) {
                _value &= ~mask;
            }

            inline  __attribute__((__always_inline__))
            void set(Port port, uint8_t mask, bool value) {
                if (value) {
                    set(port, mask);
                }
                else {
                    unset(port, mask);
                }
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            uint8_t get(uint8_t mask) {
                return _value & mask;
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            void reset(uint8_t value = 0) {
                _value = value;
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            void set(uint8_t mask) {
                _value |= mask;
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            void unset(uint8_t mask) {
                _value &= ~mask;
            }
        };

    }

    template<typename _DeviceBaseType, typename _DerivedClass, typename _DeviceConfigType>
    class MCP23008Base : public Base<_DeviceBaseType, _DerivedClass, _DeviceConfigType>  {
    public:
        using BaseClass = Base<_DeviceBaseType, _DerivedClass, _DeviceConfigType>;
        using BaseClass::Base;
        using DeviceType = typename BaseClass::DeviceType;
        using DeviceClassType = typename BaseClass::DeviceClassType;
        using BaseClass::begin;
        using BaseClass::kDefaultAddress;
        using BaseClass::kDeviceType;
        using BaseClass::beginTransmission;
        using BaseClass::endTransmission;
        using BaseClass::requestFrom;
        using BaseClass::writeByte;
        using BaseClass::writeWordLE;
        using BaseClass::writeWordBE;
        using BaseClass::readByte;
        using BaseClass::readWordLE;
        using BaseClass::readWordBE;
        using DataType = uint8_t;
        using Port = MCP230XXHelpers::Port;
        using PortAndMask = MCP230XXHelpers::PortAndMask8;
        using Register = MCP230XXHelpers::Register8;

    public:
        static constexpr uint8_t IODIR = 0x00;
        static constexpr uint8_t IPOL = 0x01;
        static constexpr uint8_t GPINTEN = 0x02;
        static constexpr uint8_t DEFVAL = 0x03;
        static constexpr uint8_t INTCON = 0x04;
        static constexpr uint8_t IOCON = 0x05;
        static constexpr uint8_t GPPU = 0x06;
        static constexpr uint8_t INTF = 0x07;
        static constexpr uint8_t INTCAP = 0x08;
        static constexpr uint8_t GPIO = 0x09;

        // get port for pin #
        constexpr Port _pin2Port(uint8_t pin) const {
            return Port::A;
        }

        // get port for pin # and mask
        constexpr PortAndMask _pin2PortAndMask(uint8_t pin) const {
            return PortAndMask(Port::A, _BV(pin));
        }

        // get register adddress for port A
        constexpr uint8_t _portAddress(uint8_t regAddr, Port port) const {
            return regAddr;
        }

        // get register adddress for port A
        template<Port _Port>
        constexpr uint8_t _portAddress(uint8_t regAddr) const {
            return regAddr;
        }

        const __FlashStringHelper *__regAddrName(uint8_t addr) const;
    };

    template<typename _DeviceBaseType, typename _DerivedClass, typename _DeviceConfigType>
    class MCP23017Base : public Base<_DeviceBaseType, _DerivedClass, _DeviceConfigType>  {
    public:
        using BaseClass = Base<_DeviceBaseType, _DerivedClass, _DeviceConfigType>;
        using BaseClass::Base;
        using DeviceType = typename BaseClass::DeviceType;
        using DeviceClassType = typename BaseClass::DeviceClassType;
        using BaseClass::begin;
        using BaseClass::kDefaultAddress;
        using BaseClass::kDeviceType;
        using BaseClass::beginTransmission;
        using BaseClass::endTransmission;
        using BaseClass::requestFrom;
        using BaseClass::writeByte;
        using BaseClass::writeWordLE;
        using BaseClass::writeWordBE;
        using BaseClass::readByte;
        using BaseClass::readWordLE;
        using BaseClass::readWordBE;
        using DataType = uint16_t;
        using Port = MCP230XXHelpers::Port;
        using PortAndMask = MCP230XXHelpers::PortAndMask16;
        using Register = MCP230XXHelpers::Register16;

    public:
        static constexpr uint8_t PORT_BANK_MULTIPLIER = 2;
        static constexpr uint8_t PORT_BANK_INCREMENT = 1;

        static constexpr uint8_t IODIR = 0x00 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t IPOL = 0x01 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t GPINTEN = 0x02 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t DEFVAL = 0x03 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t INTCON = 0x04 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t IOCON = 0x05 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t GPPU = 0x06 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t INTF = 0x07 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t INTCAP = 0x08 * PORT_BANK_MULTIPLIER;
        static constexpr uint8_t GPIO = 0x09 * PORT_BANK_MULTIPLIER;

        // get port for pin #
        constexpr Port _pin2Port(uint8_t pin) const {
            return (pin < 8) ? Port::A : Port::B;
        }

        // get port for pin # and mask
        constexpr PortAndMask _pin2PortAndMask(uint8_t pin) const {
            if (pin < 8) {
                // __DBG_printf("_pin2PortAndMask(%u)=A%s", pin, decbin((uint8_t)_BV(pin)).c_str());
                return PortAndMask(Port::A, _BV(pin));
            }
            else {
                // __DBG_printf("_pin2PortAndMask(%u)=B%s", pin, decbin((uint8_t)_BV((pin-8))).c_str());
                return PortAndMask(Port::B, _BV((pin - 8)));
            }
        }

        // get register adddress for port A or B
        constexpr uint8_t _portAddress(uint8_t regAddr, Port port) const {
            return (port == Port::A) ? regAddr : regAddr + PORT_BANK_INCREMENT;
        }

        // get register adddress for port A or B
        template<Port _Port>
        constexpr uint8_t _portAddress(uint8_t regAddr) const {
            return (_Port == Port::A) ? regAddr : regAddr + PORT_BANK_INCREMENT;
        }

        const __FlashStringHelper *__regAddrName(uint8_t addr) const;
    };

    template<typename _DeviceBaseType, typename _BaseClass>
    class MCP230XX : public _BaseClass {
    public:
        using Base = _BaseClass;
        using Base::Base;
        using DeviceType = typename Base::DeviceType;
        using DeviceClassType = typename Base::DeviceClassType;
        using DataType = typename Base::DataType;
        using Base::begin;
        using Base::kDefaultAddress;
        using Base::kDeviceType;
        using Base::beginTransmission;
        using Base::endTransmission;
        using Base::requestFrom;
        using Base::writeByte;
        using Base::writeWordLE;
        using Base::writeWordBE;
        using Base::readByte;
        using Base::readWordLE;
        using Base::readWordBE;
        using Base::IODIR;
        using Base::IPOL;
        using Base::GPINTEN;
        using Base::DEFVAL;
        using Base::INTCON;
        using Base::IOCON;
        using Base::GPPU;
        using Base::INTF;
        using Base::INTCAP;
        using Base::GPIO;
        using Base::_address;
        using Base::_wire;
        using Base::_pin2Port;
        using Base::_pin2PortAndMask;
        using Base::_portAddress;
        using Base::__regAddrName;
        using Port = typename Base::Port;
        using PortAndMask = typename Base::PortAndMask;
        // using RegisterWidth = typename std::conditional<DeviceType::kNumPins == 8, uint8_t, uint16_t>::type;
        using Register = typename Base::Register;

    public:
        static constexpr uint8_t PA0 = 0;
        static constexpr uint8_t PA1 = 1;
        static constexpr uint8_t PA2 = 2;
        static constexpr uint8_t PA3 = 3;
        static constexpr uint8_t PA4 = 4;
        static constexpr uint8_t PA5 = 5;
        static constexpr uint8_t PA6 = 6;
        static constexpr uint8_t PA7 = 7;

        static constexpr uint8_t PB0 = 8;
        static constexpr uint8_t PB1 = 9;
        static constexpr uint8_t PB2 = 10;
        static constexpr uint8_t PB3 = 11;
        static constexpr uint8_t PB4 = 12;
        static constexpr uint8_t PB5 = 13;
        static constexpr uint8_t PB6 = 14;
        static constexpr uint8_t PB7 = 15;

    public:
        // INTPOL: This bit sets the polarity of the INT output pin
        // 1 = Active-high
        // 0 = Active-low
        static constexpr uint8_t IOCON_INTPOL = _BV(1);
        // ODR: Configures the INT pin as an open-drain output
        // 1 = Open-drain output (overrides the INTPOL bit.)
        // 0 = Active driver output (INTPOL bit sets the polarity.)
        static constexpr uint8_t IOCON_ODR = _BV(2);
        // DISSLW: Slew Rate control bit for SDA output
        // 1 = Slew rate disabled
        // 0 = Slew rate enabled
        static constexpr uint8_t IOCON_DISSLW = _BV(4);
        // SEQOP: Sequential Operation mode bit
        // 1 = Sequential operation disabled, address pointer does not increment.
        // 0 = Sequential operation enabled, address pointer increments.
        static constexpr uint8_t IOCON_SEQOP = _BV(5);
        // MIRROR: INT Pins Mirror bit
        // 1 = The INT pins are internally connected
        // 0 = The INT pins are not connected. INTA is associated with PORTA and INTB is associated with PORTB
        static constexpr uint8_t IOCON_MIRROR = _BV(6);
        // BANK: Controls how the registers are addressed
        // 1 = The registers associated with each port are separated into different banks.
        // 0 = The registers are in the same bank (addresses are sequential)
        static constexpr uint8_t IOCON_BANK = _BV(7);

    public:
        MCP230XX(uint8_t address = kDefaultAddress, TwoWire *wire = &Wire);

    	void begin(uint8_t address, TwoWire *wire);
        void begin(uint8_t address);

        int analogRead(uint8_t pinNo);
        void analogWrite(uint8_t pin, int value);

        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);

        uint8_t _readPortA();
        uint8_t _readPortB();
        uint16_t _readPortAB();

        uint8_t readPortA() {
            return _readPortA();
        }

        uint8_t readPortB() {
            return _readPortB();
        }

        uint16_t readPort() {
            return _readPortAB();
        }

        uint16_t readPortAB() {
            return _readPortAB();
        }

        uint8_t readIntCapA() {
            return Register(readIntCapAB()).A;
        }

        uint8_t readIntCapB() {
            return Register(readIntCapAB()).B;
        }

        uint16_t readIntCap() {
            return readIntCapAB();
        }

        uint16_t readIntCapAB() {
            _read(INTCAP, _INTCAP);
            return _INTCAP._value;
        }

        void writePortA(uint8_t value);
        void writePortB(uint8_t value);
        void writePortAB(uint16_t value);

        inline  __attribute__((__always_inline__))
        void writePort(uint16_t value) {
            writePortAB(value);
        }

        void pinMode(uint8_t pin, uint8_t mode);

        void analogReference(uint8_t mode) {}
        void analogWriteFreq(uint32_t freq) {}

        // currently only mirrored interrupts for port A&B are supported
        void enableInterrupts(uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void disableInterrupts(uint16_t pinMask);
        bool interruptsEnabled();

        inline  __attribute__((__always_inline__))
        bool interruptHandler() {
            // Serial.println(millis());
            // Serial.printf("GPINTEN=%s\n", decbin(_GPINTEN._value).c_str());
            // _read(INTCAP, _INTCAP);
            // Serial.printf("INTCAP=%s\n", decbin(_INTCAP._value).c_str());
            //
            // Serial.printf("INTF=%s\n", decbin(_INTF._value).c_str());
            // auto value = readPort();
            // Serial.printf("PORT=%s\n", decbin(value).c_str());
            _INTF = 0;
            if (_GPINTEN.A && _GPINTEN.B) {
                _read(INTF, _INTF);
            }
            else if (_GPINTEN.A) {
                _read8(INTF, _INTF, Port::A);
            }
            else if (_GPINTEN.B) {
                _read8(INTF, _INTF, Port::B);
            }
            _callback(_INTF._value);
            // if (_INTF.A) {
            //     _read8(INTCAP, _INTCAP, Port::A);
            // }
            // if (_INTF.B) {
            //     _read8(INTCAP, _INTCAP, Port::B);
            // }
            return false;
        }

        inline  __attribute__((__always_inline__))
        void ISRHandler() {
            interruptHandler();

            // if (!_timer) {
            //     _timer.setCallback([this]() {
            //         return this->interruptHandler();
            //     });
            //     #if IOEXPANDER_INTERRUPT_MICROS_TIMER
            //         _timer.start(333);
            //     #else
            //         _timer.start(1);
            //     #endif
            // }
        }

    // protected:
        // write 16 bit register
        void _write(uint8_t regAddr, Register &regValue);
        // read 16 bit register
        void _read(uint8_t regAddr, Register &regValue);
        // write 8bit part of a 16 bit register
        void _write8(uint8_t regAddr, Register regValue, Port port);
        // read 8bit part of a 16 bit register
        void _read8(uint8_t regAddr, Register &regValue, Port port);

    public:
        InterruptCallback _callback;
        Timer _timer;
        // InterruptMode _intMode;

    // protected:
        Register _IODIR;
        Register _GPPU;
        Register _GPIO;
        Register _IOCON;
        Register _GPINTEN;
        Register _INTCON;
        Register _DEFVAL;
        Register _INTCAP;
        Register _INTF;
        // Register _IPOL;
    };

    class MCP23008 : public MCP230XX<DeviceTypeMCP23008, MCP23008Base<DeviceTypeMCP23008, MCP23008, NullConfig>> {
    public:
    };

    class MCP23017 : public MCP230XX<DeviceTypeMCP23017, MCP23017Base<DeviceTypeMCP23017, MCP23017, NullConfig>> {
    public:
    };

}

