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

            inline  __attribute__((__always_inline__))
            operator uint16_t() const {
                return _value;
            }

            inline  __attribute__((__always_inline__))
            Register16 &operator =(uint16_t value) {
                _value = value;
                return *this;
            }

            inline  __attribute__((__always_inline__))
            Register16 &operator |=(uint16_t value) {
                _value |= value;
                return *this;
            }

            inline  __attribute__((__always_inline__))
            Register16 &operator &=(uint16_t value) {
                _value &= value;
                return *this;
            }

            // 8bit operations

            inline  __attribute__((__always_inline__))
            uint8_t &operator [](Port port) {
                return port == Port::A ? A : B;
            }

            inline  __attribute__((__always_inline__))
            uint8_t operator [](Port port) const {
                return port == Port::A ? A : B;
            }

            inline  __attribute__((__always_inline__))
            uint8_t get(Port port, uint8_t mask) {
                return (*this)[port] & mask;
            }

            inline  __attribute__((__always_inline__))
            void set(Port port, uint8_t mask) {
                (*this)[port] |= mask;
            }

            inline  __attribute__((__always_inline__))
            void unset(Port port, uint8_t mask) {
                (*this)[port] &= ~mask;
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
                return (_Port == Port::A ? A : B) & mask;
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            void reset(uint8_t value) {
                (_Port == Port::A ? A : B) = value;
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            void set(uint8_t mask) {
                (_Port == Port::A ? A : B) |= mask;
            }

            template<Port _Port>
            inline  __attribute__((__always_inline__))
            void unset(uint8_t mask) {
                (_Port == Port::A ? A : B) &= ~mask;
            }
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

    template<typename _DeviceBaseType, typename _DerivedClass>
    class MCP23008Base : public Base<_DeviceBaseType, _DerivedClass, NullConfig>  {
    public:
        using BaseClass = Base<_DeviceBaseType, _DerivedClass, NullConfig>;
        using BaseClass::Base;
        using DeviceType = typename BaseClass::DeviceType;
        using DeviceClassType = typename BaseClass::DeviceClassType;
        using BaseClass::begin;
        using BaseClass::kDefaultAddress;
        using BaseClass::kDeviceType;
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
    };

    template<typename _DeviceBaseType, typename _DerivedClass>
    class MCP23017Base : public Base<_DeviceBaseType, _DerivedClass, NullConfig>  {
    public:
        using BaseClass = Base<_DeviceBaseType, _DerivedClass, NullConfig>;
        using BaseClass::Base;
        using DeviceType = typename BaseClass::DeviceType;
        using DeviceClassType = typename BaseClass::DeviceClassType;
        using BaseClass::begin;
        using BaseClass::kDefaultAddress;
        using BaseClass::kDeviceType;
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
                return PortAndMask(Port::A, _BV(pin));
            }
            else {
                return PortAndMask(Port::B, _BV(pin - 8));
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

    };

    template<typename _DeviceBaseType, typename _BaseClass>
    class MCP230XX : public _BaseClass {
    public:
        using Base = _BaseClass;
        using Base::Base;
        using DeviceType = typename Base::DeviceType;
        using DeviceClassType = typename Base::DeviceClassType;
        using Base::begin;
        using Base::kDefaultAddress;
        using Base::kDeviceType;
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
        using Port = MCP230XXHelpers::Port;
        using PortAndMask = typename Base::PortAndMask;
        using RegisterWidth = typename std::conditional<DeviceType::kNumPins == 8, uint8_t, uint16_t>::type;
        using Register = typename Base::Register;

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
        bool analogWrite(uint8_t pin, uint8_t value);

        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);

        uint8_t _readPortA();
        uint8_t _readPortB();
        uint16_t _readPortAB();

        template<typename _T = uint8_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 8), uint8_t>::type readPortA() {
            return _readPortA();
        }
        template<typename _T = uint8_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 8), uint8_t>::type readPortB() {
            return _readPortA();
        }

        template<typename _T = uint8_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 8), uint8_t>::type readPortAB() {
            return _readPortA();
        }

        template<typename _T = uint8_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 8), uint8_t>::type readPort() {
            return _readPortA();
        }

        template<typename _T = uint8_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 16), uint8_t>::type readPortA() {
            return _readPortA();
        }
        template<typename _T = uint8_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 16), uint8_t>::type readPortB() {
            return _readPortB();
        }

        template<typename _T = uint16_t>
        typename std::enable_if<std::is_same<_T, uint8_t>::value && (DeviceType::kNumPins == 16), uint16_t>::type readPortAB() {
            return _readPortAB();
        }

        template<typename _T = uint16_t>
        typename std::enable_if<std::is_same<_T, uint16_t>::value && (DeviceType::kNumPins == 16), uint16_t>::type readPort() {
            return _readPortAB();
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
        void analogWrite(uint8_t pin, int val) {}
        void analogWriteFreq(uint32_t freq) {}

        // currently only mirrored interrupts for port A&B are supported
        void enableInterrupts(uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void disableInterrupts(uint16_t pinMask);
        bool interruptsEnabled();

        // template<typename _DeviceBaseType, typename _BaseClass>
        // inline void MCP230XX<_DeviceBaseType, _BaseClass>::invokeCallback()
        // {
        //     // read captured ports that have PINs with interrupts
        //     if (_GPINTEN[Port::A] && _GPINTEN[Port::B]) {
        //         _read(INTCAP, _INTCAP);
        //     }
        //     else if (_GPINTEN[Port::A]) {
        //         _read8(INTCAP, _INTCAP, Port::A);
        //     }
        //     if (_GPINTEN[Port::B]) {
        //         _read8(INTCAP, _INTCAP, Port::B);
        //     }
        //     auto port = Register(readPortAB());

        //     // ::printf("INTCAP A=%02x B=%02X PORT A=%02x B=%02X\n", _INTCAP[Port::A], _INTCAP[Port::B], port[Port::A], port[Port::B]);

        //     _callback(static_cast<uint16_t>(_INTCAP)); // TODO read captured pin state
        // }


        inline  __attribute__((__always_inline__))
        bool interruptHandler() {
            // readPort();
            // auto mask = _intMode._captured;
            // if (mask) {
            //     _callback(mask);
            //     _intMode._captured &= ~mask;
            // }
            // if (_intMode._captured == 0) {
            //     return false;
            // }
            return false;
        }

        inline  __attribute__((__always_inline__))
        void ISRHandler() {
            if (!_timer) {
                _timer.setCallback([this]() {
                    return this->interruptHandler();
                });
                _timer.start(100);
            }
        }

        template<DeviceTypeEnum _DeviceType>
        const __FlashStringHelper *__regAddrName(uint8_t addr) {
            return F("N/A");
        }

    protected:
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
        uint32_t _interruptsPending;

    protected:
        Register _IODIR;
        Register _GPPU;
        Register _GPIO;
        Register _IOCON;
        Register _GPINTEN;
        Register _INTCON;
        Register _DEFVAL;
        Register _INTCAP;
        // Register _INTF;
        // Register _IPOL;
    };

    class MCP23008 : public MCP230XX<DeviceTypeMCP23008, MCP23008Base<DeviceTypeMCP23008, MCP23008>> {
    public:
        template<DeviceTypeEnum _DeviceType>
        const __FlashStringHelper *__regAddrName(uint8_t addr);
    };

    class MCP23017 : public MCP230XX<DeviceTypeMCP23017, MCP23017Base<DeviceTypeMCP23017, MCP23017>> {
    public:
        template<DeviceTypeEnum _DeviceType>
        const __FlashStringHelper *__regAddrName(uint8_t addr);
    };

}

