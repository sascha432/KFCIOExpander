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

    // ------------------------------------------------------------------
    // TinyPwm
    // ------------------------------------------------------------------

    namespace TinyPwmNS {

        enum class Commands : uint8_t {
            ANALOG_READ = 0x11,
            ANALOG_WRITE = 0x12,
            DIGITAL_READ = 0x13,
            DIGITAL_WRITE = 0x14,
            PIN_MODE = 0x15,
            WRITE_EEPROM = 0x41,
            RESET_EEPROM = 0x42,
            SET_PWM_FREQUENCY = 0x51,
            ADC_SET_AREF = 0x62,
            ADC_SET_READ_CYCLES = 0x63,
        };

        struct Command {
            union {
                struct {
                    Commands command;
                    union {
                        struct {
                            uint16_t frequency;
                        } SET_PWM_FREQUENCY;
                        struct {
                            uint8_t pin;            // internal pin number
                            uint8_t pwmValue;
                        } ANALOG_WRITE;
                        struct {
                            uint8_t pin;            // internal pin number
                        } ANALOG_READ;
                        struct {
                            uint8_t pin;            // internal pin number
                        } DIGITAL_READ;
                        struct {
                            uint8_t pin;            // internal pin number
                            uint8_t value;
                        } DIGITAL_WRITE;
                        struct {
                            uint8_t pin;            // internal pin number
                            uint8_t mode;
                        } PIN_MODE;
                        struct {
                            uint8_t mode;
                        } ADC_SET_AREF;
                        struct {
                            uint8_t pin;
                            uint8_t cycles;
                        } ADC_SET_READ_CYCLES;
                    };
                };
            };
            operator uint8_t *() {
                return reinterpret_cast<uint8_t *>(this);
            }
            operator const uint8_t *() const {
                return reinterpret_cast<const uint8_t *>(this);
            }
        };

    };

    // ---------------------------------------------------------------------------
    // TinyPwm > v0.0.2
    // ---------------------------------------------------------------------------

    inline TinyPwm::TinyPwm(uint8_t address, TwoWire *wire) : Base(address, wire)
    {
    }

    inline int TinyPwm::analogRead(uint8_t pin)
    {
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::ANALOG_READ};
        command.ANALOG_READ.pin = pin;
        _wire->write(command, sizeof(command.ANALOG_READ) + 1);
        if (endTransmissionAndRequestFrom(2, true)) {
            return static_cast<int16_t>(readWordLE());
        }
        return 0;
    }

    inline uint8_t TinyPwm::digitalRead(uint8_t pin)
    {
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::DIGITAL_READ};
        command.DIGITAL_READ.pin = pin;
        _wire->write(command, sizeof(command.DIGITAL_READ) + 1);
        if (endTransmissionAndRequestFrom(1, true)) {
            return readByte();
        }
        return 0;
    }

    inline void TinyPwm::analogWrite(uint8_t pin, int value)
    {
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::ANALOG_WRITE};
        command.ANALOG_WRITE.pin = pin;
        command.ANALOG_WRITE.pwmValue = std::min(255, std::max(0, value));
        writeBytes(command, sizeof(command.ANALOG_WRITE) + 1);
        endTransmission(true);
    }

    inline void TinyPwm::analogWriteFreq(uint16_t frequency)
    {
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::SET_PWM_FREQUENCY};
        command.SET_PWM_FREQUENCY.frequency = frequency;
        writeBytes(command, sizeof(command.SET_PWM_FREQUENCY) + 1);
        endTransmission(true);
    }

    inline void TinyPwm::analogReference(uint8_t mode)
    {
        if (mode != INTERNAL_1V1 && mode != INTERNAL_2V56) {
            return;
        }
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::ADC_SET_AREF};
        command.ADC_SET_AREF.mode = mode & ~0x80;
        writeBytes(command, sizeof(command.ADC_SET_AREF) + 1);
        endTransmission(true);
    }

    inline void TinyPwm::digitalWrite(uint8_t pin, uint8_t value)
    {
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::DIGITAL_WRITE};
        command.DIGITAL_WRITE.pin = pin;
        command.DIGITAL_WRITE.value = value ? 1 : 0;
        writeBytes(command, sizeof(command.DIGITAL_WRITE) + 1);
        endTransmission(true);
    }

    inline void TinyPwm::pinMode(uint8_t pin, uint8_t mode)
    {
        beginTransmission();
        TinyPwmNS::Command command = {TinyPwmNS::Commands::PIN_MODE};
        command.PIN_MODE.pin = pin;
        command.PIN_MODE.mode = mode;
        writeBytes(command, sizeof(command.PIN_MODE) + 1);
        endTransmission(true);
    }

    // ---------------------------------------------------------------------------
    // TinyPwm_V_0_0_2
    // ---------------------------------------------------------------------------

    inline void TinyPwm_V_0_0_2::analogWrite(uint8_t pin, int value)
    {
        beginTransmission();
        writeByte(0x10); // ANALOG_WRITE
        writeByte(value);
        endTransmission(true);
    }

    inline void TinyPwm_V_0_0_2::digitalWrite(uint8_t pin, uint8_t value)
    {
        analogWrite(pin, value ? 255 : 0);
    }

    inline uint8_t TinyPwm_V_0_0_2::digitalRead(uint8_t pin)
    {
        return analogRead(pin) ? 1 : 0;
    }

}
