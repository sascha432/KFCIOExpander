/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"
#include "Timer.h"

namespace IOExpander {

    namespace IOWrapper {

        template<typename _BaseClassType, typename _DataType>
        struct PCF857X {

            class PIN;
            class PORT;
            class DDR;

            using DataType = _DataType;
            using BaseClassType = _BaseClassType;

            // read port data
            class PIN : public IOWrapper<BaseClassType, DataType, PIN> {
            public:
                using IOWrapperType = IOWrapper<BaseClassType, DataType, PIN>;
                using IOWrapperType::IOWrapper;
                using IOWrapperType::_parent;
                using IOWrapperType::_value;

                PIN &operator=(DataType) = delete;

                operator DataType();

            protected:
                uint8_t _readValue(uint8_t);
                uint16_t _readValue(uint16_t);
            };

            // write port data
            class PORT : public IOWrapper<BaseClassType, DataType, PORT> {
            public:
                using IOWrapperType = IOWrapper<BaseClassType, DataType, PORT>;
                using IOWrapperType::IOWrapperType;
                using IOWrapperType::_parent;
                using IOWrapperType::_value;
                using IOWrapperType::operator DataType;
                using IOWrapperType::operator|=;
                using IOWrapperType::operator&=;
                using IOWrapperType::operator%=;
                using IOWrapperType::operator*=;
                using IOWrapperType::operator^=;
                using IOWrapperType::operator<<=;
                using IOWrapperType::operator>>=;

                PORT &operator=(DataType value);

            protected:
                friend DDR;

                void _updatePort(uint8_t);
                void _updatePort(uint16_t);
            };

            // set port mode
            class DDR : public IOWrapper<BaseClassType, DataType, DDR> {
            public:
                using IOWrapperType = IOWrapper<BaseClassType, DataType, DDR>;
                using IOWrapperType::IOWrapperType;
                using IOWrapperType::_parent;
                using IOWrapperType::_value;
                using IOWrapperType::operator DataType;
                using IOWrapperType::operator|=;
                using IOWrapperType::operator&=;
                using IOWrapperType::operator%=;
                using IOWrapperType::operator*=;
                using IOWrapperType::operator^=;
                using IOWrapperType::operator<<=;
                using IOWrapperType::operator>>=;

                DDR &operator=(DataType value);

            protected:
                void _updatePort();
            };

        };

    }

    template<typename _DeviceBaseType, typename _DerivedClass, typename _ConfigClass>
    class PCF857X : public Base<_DeviceBaseType, _DerivedClass, _ConfigClass> {
    public:
        using BaseClass = Base<_DeviceBaseType, _DerivedClass, _ConfigClass>;
        using BaseClass::Base;
        using DeviceType = typename BaseClass::DeviceType;
        using DataType = typename DeviceType::DataType;
        using DeviceClassType = typename BaseClass::DeviceClassType;
        using BaseClass::kDefaultAddress;
        using BaseClass::kDeviceType;

    public:
        using SelfType = PCF857X<_DeviceBaseType, _DerivedClass, _ConfigClass>;
        using DDRType = typename IOWrapper::PCF857X<SelfType, DataType>::DDR;
        using PINType = typename IOWrapper::PCF857X<SelfType, DataType>::PIN;
        using PORTType = typename IOWrapper::PCF857X<SelfType, DataType>::PORT;
        using InterruptMode = typename Interrupts::InterruptMode<DataType>;

        static constexpr uint8_t DataTypeSizeInBits = sizeof(DataType) << 3;

    public:
        static constexpr uint8_t P0 = 0;
        static constexpr uint8_t P1 = 1;
        static constexpr uint8_t P2 = 2;
        static constexpr uint8_t P3 = 3;
        static constexpr uint8_t P4 = 4;
        static constexpr uint8_t P5 = 5;
        static constexpr uint8_t P6 = 6;
        static constexpr uint8_t P7 = 7;

        static constexpr uint8_t P8 = 8;
        static constexpr uint8_t P9 = 9;
        static constexpr uint8_t P10 = 10;
        static constexpr uint8_t P11 = 11;
        static constexpr uint8_t P12 = 12;
        static constexpr uint8_t P13 = 13;
        static constexpr uint8_t P14 = 14;
        static constexpr uint8_t P15 = 15;

    public:
        PCF857X(uint8_t address = kDefaultAddress, TwoWire *wire = &Wire) :
            BaseClass(address, wire),
            DDR(*this, 0),
            PIN(*this, 0),
            PORT(*this, ~0)
        {
        }

        void begin(uint8_t address, TwoWire *wire);
        void begin(uint8_t address);
        void begin();

        void pinMode(uint8_t pin, uint8_t mode);
        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);

        int analogRead(uint8_t pin);
        void analogWrite(uint8_t pin, int val);

        DataType readPort();
        uint8_t readPortA();
        uint8_t readPortB();

        void writePort(DataType value);
        void writePortA(uint8_t value);
        void writePortB(uint8_t value);

        void analogReference(uint8_t mode) {}
        void analogWriteFreq(uint32_t freq) {}

        void enableInterrupts(DataType pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void disableInterrupts(DataType pinMask);
        bool interruptsEnabled();

        inline  __attribute__((__always_inline__))
        bool interruptHandler() {
            readPort();
            auto mask = _intMode._captured;
            if (mask) {
                _callback(mask);
                _intMode._captured &= ~mask;
            }
            if (_intMode._captured == 0) {
                return false;
            }
            return true;
        }

        inline  __attribute__((__always_inline__))
        void ISRHandler() {
            if (!_timer) {
                _timer.setCallback([this]() {
                    return this->interruptHandler();
                });
                // less than 300µs misses captured pins sometimes
                _timer.start(300);
            }
        }

        void captureEvents(DataType value) {
            _intMode.captureEvents(value);
        }

    public:
        DDRType DDR;
        PINType PIN;
        PORTType PORT;

    protected:
        Timer _timer;
        InterruptCallback _callback;
        InterruptMode _intMode;
    };

    template<typename _ConfigType>
    using _PCF8574 = PCF857X<DeviceTypePCF8574, PCF8574, _ConfigType>;

    class PCF8574 : public _PCF8574<NullConfig> {
    public:
    };

    template<typename _ConfigType>
    using _PCF8575 = PCF857X<DeviceTypePCF8574, PCF8575, _ConfigType>;

    class PCF8575 : public _PCF8575<NullConfig> {
    public:
    };

}
