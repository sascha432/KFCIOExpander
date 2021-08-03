/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"

namespace IOExpander {

    // ATTiny853 IO expander with 3 pins
    // can be configured as INPUT, INPUT_PULLDOWN, OUTPUT or ANALOG_INPUT

    class TinyPwm : public Base<DeviceTypeTinyPwm, TinyPwm, NullConfig> {
    public:
        using Base = Base<DeviceTypeTinyPwm, TinyPwm, NullConfig>;
        using Base::Base;
        using DeviceType = Base::DeviceType;
        using DeviceClassType = Base::DeviceClassType;
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


        // default pin translation
        static constexpr uint8_t A0 = 0;
        static constexpr uint8_t A1 = 1;
        static constexpr uint8_t A2 = 2;
        static constexpr uint8_t PB1 = 0;
        static constexpr uint8_t PB2 = 1;
        static constexpr uint8_t PB3 = 2;

        // 0x80 is the bit to identify TinyPWM
        static constexpr uint8_t INTERNAL_1V1 = 2 | 0x80;
        static constexpr uint8_t INTERNAL_2V56 = 6 | 0x80;

        static constexpr uint8_t ANALOG_INPUT = 0x80;

    public:
        TinyPwm(uint8_t address = kDefaultAddress, TwoWire *wire = &Wire);

    	void begin(uint8_t address, TwoWire *wire);
        void begin(uint8_t address);

        // // pinMode must be ANALOG_INPUT
        // int analogRead(uint8_t pinNo);

        // // pinMode must be OUTPUT
        // bool analogWrite(uint8_t pin, uint8_t value);

        // // pinMode must be OUTPUT
        // void digitalWrite(uint8_t pin, uint8_t value);

        // uint8_t digitalRead(uint8_t pin);

        // INPUT, INPUT_PULLDOWN, OUTPUT or ANALOG_INPUT
        void pinMode(uint8_t pin, uint8_t mode);

        // INTERNAL_1V1, INTERNAL_2V56
        void analogReference(uint8_t mode);

        // frequency 488 to 65535 Hz
        void analogWriteFreq(uint16_t frequency);

        int analogRead(uint8_t pinNo);
        void analogWrite(uint8_t pin, int value);

        void digitalWrite(uint8_t pin, uint8_t value);
        uint8_t digitalRead(uint8_t pin);

        uint8_t readPortA();
        uint8_t readPortB();
        uint8_t readPortAB();
        void writePortA(uint8_t value);
        void writePortB(uint8_t value);
        void writePortAB(uint8_t value);
        void writePort(uint8_t value);

        // void analogReference(uint8_t mode) {}
        // void analogWriteFreq(uint32_t freq) {}

        inline  __attribute__((__always_inline__))
        void enableInterrupts(uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode)
        {
        }

        inline  __attribute__((__always_inline__))
        void disableInterrupts(uint16_t pinMask)
        {
        }

        inline  __attribute__((__always_inline__))
        bool interruptsEnabled() {
            return false;
        }

        inline  __attribute__((__always_inline__))
        bool interruptHandler() {
            return false;
        }

        inline  __attribute__((__always_inline__))
        void ISRHandler() {
        }

    };

    class TinyPwm_V_0_0_2 : public TinyPwm {
    public:

        void analogWrite(uint8_t pin, int value);
    };

}
