/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino_compat.h>
#include <Wire.h>

#include <FunctionalInterrupt.h>
#include <Schedule.h>

#if HAVE_KFC_FIRMWARE_VERSION
#include <PrintHtmlEntities.h>
#endif


#ifndef HAVE_IOEXPANDER
#   if HAVE_PCF8574 || HAVE_TINYPWM || HAVE_MCP23017 || HAVE_MCP23008 || HAVE_PCA9685 || HAVE_NEOPIXEL
#       define HAVE_IOEXPANDER 1
#   else
#       define HAVE_IOEXPANDER 0
#   endif
#endif

#if HAVE_IOEXPANDER

// enable debugging
// to disable error messages set IOEXPANDER_DEFAULT_OUTPUT=nullptr
#ifndef DEBUG_IOEXPANDER
#define DEBUG_IOEXPANDER 0
#endif

// stream for debugging output
#ifndef IOEXPANDER_DEFAULT_OUTPUT
#define IOEXPANDER_DEFAULT_OUTPUT Serial
#endif

// set to 1 to disable the globals variable IOExpander::config
#ifndef IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS
#define IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS 0
#endif

#if IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS == 1 && !defined(IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS)
#define IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS 0
#warning support for global functions like pinMode(), digitalWrite(), etc... has been disabled
#endif

// override arduino pinMode(), digitalRead()/Write(), analogRead()/Write()...
#ifndef IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS
#define IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS 1
#endif

#if DEBUG_IOEXPANDER
#include "debug_helper_enable.h"
#else
#include "debug_helper_disable.h"
#endif

#ifndef __CONSTEXPR17
#   if __GNUC__ >= 10
#       define __CONSTEXPR17 constexpr
#   else
#       define __CONSTEXPR17
#   endif
#endif

#if __GNUC__ <= 5
namespace std {
    template <bool _Ta, class _Tb = void>
    using enable_if_t = typename std::enable_if<_Ta, _Tb>::type;
}
#endif

namespace IOExpander {

    // scan bus for devices
    void scanBus(Print &output, TwoWire &wire = Wire, uint8_t fromAddress = 0x01, uint8_t toAddress = 0x7f, uint32_t delayMillis = 1);

    // use the default digitalWrite/digitalRead/pinMode/etc functions
    // for pins between 0 and (kDigitalPinCount - 1)
    static constexpr uint8_t kDigitalPinCount = NUM_DIGITAL_PINS;
    static constexpr uint8_t kMinimumPinNumber = NUM_DIGITAL_PINS + NUM_ANALOG_INPUTS;

    enum class DeviceTypeEnum {
        PCF8574,
        PCF8575,
        TINYPWM,
        MCP23017,
        MCP23008,
        PCA9685,
        END
    };

    enum class TriggerMode {
        NONE,               // interrupts not available
        DEVICE_DEFAULT,     // use default mode of the device
        ACTIVE_HIGH,        // rising edge
        ACTIVE_LOW,         // falling edge
        OPEN_DRAIN,         // falling edge, requires pullup resistor
    };

    struct NullConfig {
        static constexpr uint8_t kBeginPin = 0;
        static constexpr uint8_t kEndPin = 0;
    };

    using InterruptCallback = std::function<void(uint16_t pinState)>;

    void IRAM_ATTR __interruptHandler(void *arg);

    inline const __FlashStringHelper *getDeviceName(DeviceTypeEnum type) {
        switch(type) {
            case DeviceTypeEnum::PCF8574:
                return F("PCF8574");
            case DeviceTypeEnum::PCF8575:
                return F("PCF8575");
            case DeviceTypeEnum::TINYPWM:
                return F("TinyPWM");
            case DeviceTypeEnum::MCP23017:
                return F("MCP23017");
            case DeviceTypeEnum::MCP23008:
                return F("MCP23008");
            case DeviceTypeEnum::PCA9685:
                return F("PCA9685");
            default:
                break;
        }
        return nullptr;
    }

    template<DeviceTypeEnum _DeviceTypeEnum>
    constexpr const __FlashStringHelper *getDeviceName() {
        return getDeviceName(_DeviceTypeEnum);
    }

    template<typename _DeviceType>
    constexpr const __FlashStringHelper *getDeviceName() {
        return getDeviceName(_DeviceType::kDeviceType);
    }

    template<DeviceTypeEnum _DeviceTypeEnum, uint8_t _DefaultAddress, uint8_t _NumPins, typename _DataType, bool _HasIsConnected, uint8_t _GPIOInterruptPinMode = 0, TriggerMode _IntTriggerMode = TriggerMode::NONE>
    struct DeviceTypeTemplate {
        using DeviceType = DeviceTypeTemplate<_DeviceTypeEnum, _DefaultAddress, _NumPins, _DataType, _HasIsConnected, _GPIOInterruptPinMode, _IntTriggerMode>;
        using DataType = _DataType;

        // device type
        static constexpr DeviceTypeEnum kDeviceType = _DeviceTypeEnum;
        // number of pins available
        static constexpr uint8_t kNumPins = _NumPins;
        // default I2C address
        static constexpr uint8_t kDefaultAddress = _DefaultAddress;
        // has isConnected() method
        static constexpr bool kHasIsConnected = _HasIsConnected;
        // pinMode for GPIO pin used for interrupts
        static constexpr uint8_t kIntPinMode = _GPIOInterruptPinMode;
        // interrupt trigger mode
        static constexpr TriggerMode kIntTriggerMode = _IntTriggerMode;

    };

    // address bits to I2C address helpers

    constexpr uint8_t PCF857XAddress(bool A2 = true, bool A1 = true, bool A0 = true) {
        return (0b00100 << 3) | (A2 << 2) | (A1 << 1) | (A0 << 0);
    }

    constexpr uint8_t MCP230XXAddress(bool A2 = false, bool A1 = false, bool A0 = false) {
        return (0b00100 << 3) | (A2 << 2) | (A1 << 1) | (A0 << 0);
    }

    constexpr uint8_t PCA96X5Address(bool A3 = false, bool A2 = false, bool A1 = false, bool A0 = false) {
        return (0b0100 << 4) | (A3 << 3) | (A2 << 2) | (A1 << 1) | (A0 << 0);
    }

    constexpr uint8_t TinyPwmAddress(uint8_t ofs = 0) {
        return 0b1100000 | ofs;
    }

    using DeviceTypePCF8574 = DeviceTypeTemplate<DeviceTypeEnum::PCF8574, PCF857XAddress(), 8, uint8_t, true, INPUT_PULLUP, TriggerMode::OPEN_DRAIN>;
    using DeviceTypePCF8575 = DeviceTypeTemplate<DeviceTypeEnum::PCF8575, PCF857XAddress(), 16, uint16_t, true, INPUT_PULLUP, TriggerMode::OPEN_DRAIN>;
    using DeviceTypeTinyPwm = DeviceTypeTemplate<DeviceTypeEnum::TINYPWM, TinyPwmAddress(), 2, uint8_t, true>;
    using DeviceTypeMCP23008 = DeviceTypeTemplate<DeviceTypeEnum::MCP23008, MCP230XXAddress(), 8, uint8_t, true, INPUT, TriggerMode::ACTIVE_HIGH>;
    using DeviceTypeMCP23017 = DeviceTypeTemplate<DeviceTypeEnum::MCP23017, MCP230XXAddress(), 16, uint16_t, true, INPUT, TriggerMode::ACTIVE_HIGH>;
    using DeviceTypePCA9685 = DeviceTypeTemplate<DeviceTypeEnum::PCA9685, PCA96X5Address(), 16, uint16_t, false>;

    struct DeviceTypeEnd {
        using ConfigType = DeviceTypeEnd;
        using NextConfigType = nullptr_t;
        using DeviceConfigType = void;
        using DeviceType = void;
        using DeviceClassType = void;
        static constexpr DeviceTypeEnum kDeviceType = DeviceTypeEnum::END;
        static constexpr uint8_t kNumPins = 0;
        static constexpr uint8_t kDefaultAddress = 0;
        static constexpr bool kHasIsConnected = false;
        static constexpr bool kHasNext = false;
    };

    template<typename _ConfigType>
    struct ConfigIterator;

    struct ConfigEndIterator {
        void _beginRecursive(TwoWire &wire);
        template<bool _HtmlOutput>
        void _printStatusRecursive(Print &output);
        constexpr size_t _sizeRecursive() const;
        void _pinModeRecursive(uint8_t pin, uint8_t mode);
        void _digitalWriteRecursive(uint8_t pin, uint8_t val);
        int _digitalReadRecursive(uint8_t pin);
        void _analogReferenceRecursive(uint8_t mode);
        void _analogWriteRecursive(uint8_t pin, int val);
        int _analogReadRecursive(uint8_t pin);
        void _analogWriteFreqRecursive(uint32_t freq);
        void *_getDevicePointerRecursive(uint8_t pin);
        nullptr_t getDeviceByAddress(uint8_t address);
        nullptr_t getDeviceByPin(uint8_t pin);
        constexpr bool _pinMatch(uint8_t pin) const;
        bool interruptsEnabled();
        void _attachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void _detachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask);
        void _setInterruptFlagRecursive(void *device);
        void _dumpPinsRecursive(Print &output);
    };

    template<typename _DeviceConfigType, typename _NextConfigType = DeviceTypeEnd>
    struct Config {
        using ConfigType = Config<_DeviceConfigType, _NextConfigType>;
        using DeviceConfigType = _DeviceConfigType;
        using DeviceType = typename DeviceConfigType::DeviceType;
        using DeviceClassType = typename DeviceConfigType::DeviceClassType;

        static constexpr bool kHasNext = !std::is_same<_NextConfigType, DeviceTypeEnd>::value;

        using NextConfigType = _NextConfigType;
        using NextDeviceConfigType = typename std::conditional<kHasNext, typename NextConfigType::DeviceConfigType, DeviceConfigType>::type;
        using NextDeviceType = typename std::conditional<kHasNext, typename NextDeviceConfigType::DeviceType, DeviceType>::type;
        using NextDeviceClassType = typename std::conditional<kHasNext, typename NextDeviceConfigType::DeviceClassType, nullptr_t>::type;

        using NextConfigIterator = typename std::conditional<kHasNext, ConfigIterator<NextConfigType>, ConfigEndIterator>::type;
    };


    template<typename _ConfigType>
    struct ConfigIterator {
        using ConfigType = _ConfigType;
        using DeviceClassType = typename ConfigType::DeviceClassType;
        using DeviceConfigType = typename ConfigType::DeviceConfigType;
        using DeviceType = typename ConfigType::DeviceType;
        using NextConfigIterator = typename ConfigType::NextConfigIterator;

        DeviceClassType _device;
        NextConfigIterator _next;

        void begin(TwoWire &wire);
        void begin();

        // print status information about all devices
        template<bool _HtmlOutput = false>
        void printStatus(Print &output);

        // print pin state of all devices
        void dumpPins(Print &output);

        constexpr size_t size() const;
        void pinMode(uint8_t pin, uint8_t mode);
        void IRAM_ATTR digitalWrite(uint8_t pin, uint8_t val);
        int IRAM_ATTR digitalRead(uint8_t pin);
        int analogRead(uint8_t pin);
        void analogReference(uint8_t mode);
        void analogWrite(uint8_t pin, int val);
        void analogWriteFreq(uint32_t freq);

        // return device pointer for given pin
        void *getDevicePointer(uint8_t pin);

        auto getDeviceByAddress(uint8_t address) -> decltype(&_device);
        auto getDeviceByPin(uint8_t pin) -> decltype(&_device);

        // return true if any device has interrupts enabled
        bool interruptsEnabled();

        // interrupts can only be enabled per device
        // the hardware interrupt must be triggered from a single GPIO pin
        // the callback will be called as scheduled_function outside the interrupt handler
        // and the handler does not have to be in IRAM
        //
        // gpioPin is the GPIO pin for the interrupt
        // triggerMode is the interrupt mode for the GPIO pin
        void attachInterrupt(uint8_t gpioPin, void *device, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode = TriggerMode::DEVICE_DEFAULT);

        // remove interrupt handler
        void detachInterrupt(uint8_t gpioPin, void *device, uint16_t pinMask);

        // recursive methods
        void _beginRecursive(TwoWire &wire);
        void _dumpPinsRecursive(Print &output);
        template<bool _HtmlOutput>
        void _printStatusRecursive(Print &output);
        constexpr size_t _sizeRecursive() const;
        constexpr bool _pinMatch(uint8_t pin) const;
        void _pinModeRecursive(uint8_t pin, uint8_t mode);
        void _digitalWriteRecursive(uint8_t pin, uint8_t val);
        int _digitalReadRecursive(uint8_t pin);
        int _analogReadRecursive(uint8_t pin);
        void _analogReferenceRecursive(uint8_t mode);
        void _analogWriteRecursive(uint8_t pin, int val);
        void _analogWriteFreqRecursive(uint32_t freq);
        void *_getDevicePointerRecursive(uint8_t pin);
        bool _interruptsEnabledRecursive();
        void _attachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void _detachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask);
        void _setInterruptFlagRecursive(void *device);

    protected:
        constexpr int _triggerMode2IntMode(TriggerMode mode) const;
    };

    template<typename _DeviceClassType, typename _DeviceType, uint8_t _Address, uint8_t _BeginPin, uint8_t _EndPin = 0>
    struct DeviceConfig {
        using DeviceClassType = _DeviceClassType;
        using DeviceType = _DeviceType;

        static constexpr DeviceTypeEnum kDeviceType = DeviceType::kDeviceType;
        static constexpr uint8_t kI2CAddress = _Address;
        static constexpr uint8_t kBeginPin = _BeginPin;
        static constexpr uint8_t kEndPin = _EndPin == 0 ? (kBeginPin + DeviceType::kNumPins) : _EndPin;
        static constexpr uint8_t kNumPins = kEndPin - kBeginPin;

        static_assert(_BeginPin >= kMinimumPinNumber, "_BeginPin must be greater or equal kMinimumPinNumber");

        #if DEBUG_IOEXPANDER && 0
            static bool pinMatch(uint8_t pin) {
                if (pin >= kBeginPin && pin < kEndPin) {
                    __LDBG_printf("pinMatch %s: %u >= %u < %u", getDeviceName<kDeviceType>(), kBeginPin, pin, kEndPin);
                    return true;
                }
                return false;
            }
        #else
            static constexpr bool pinMatch(uint8_t pin) {
                return pin >= kBeginPin && pin < kEndPin;
            }
        #endif
    };

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    class Base {
    public:
        using DeviceType = _DeviceType;
        using DeviceClassType = _DeviceClassType;
        using DeviceConfigType = _DeviceConfigType;

        static constexpr uint8_t kDefaultAddress = DeviceType::kDefaultAddress;
        static constexpr uint8_t kNumPins = DeviceType::kNumPins;
        static constexpr DeviceTypeEnum kDeviceType = DeviceType::kDeviceType;

    public:
        Base(uint8_t address = 0, TwoWire *wire = &Wire) : _address(address), _startPin(0xff), _wire(wire) {}

        void begin(uint8_t address, TwoWire *wire);
        void begin(uint8_t address);
        void begin();

        bool isConnected() const;
        uint8_t getAddress() const;
        TwoWire &getWire();

        void setStartPin(uint8_t pin) {
            _startPin = pin;
        }

        // return virtual pin from zero based pin number
        constexpr uint8_t getPin(uint8_t pinNo) const {
            return (DeviceConfigType::kBeginPin == 0 || DeviceConfigType::kEndPin == 0) ? _getPin(pinNo) :
                pinNo >= (DeviceConfigType::kEndPin - DeviceConfigType::kBeginPin) ? 0xff : (DeviceConfigType::kBeginPin + pinNo);
        }

        constexpr const __FlashStringHelper *getDeviceName() {
            return IOExpander::getDeviceName<DeviceType>();
        }

    protected:
        // fallback runtime configuration
        uint8_t _getPin(uint8_t pinNo) const {
            return pinNo >= (_startPin + kNumPins) ? 0xff : (_startPin + pinNo);
        }

    protected:
        uint8_t _address;
        uint8_t _startPin;
        TwoWire *_wire;
    };

    class PCF8574;
    class PCF8575;

    namespace IOWrapper {

        template<typename _Parent, typename _DataType, typename _ClassType>
        class IOWrapper {
        public:
            using DataType = _DataType;

        public:
            IOWrapper(_Parent &parent, DataType value) :
                _value(value),
                _parent(&parent)
            {
            }

            IOWrapper(_Parent *parent, DataType value) :
                _value(value),
                _parent(parent)
            {
            }

            // the operators force a call of the derived class's operator= to process any changes

            _ClassType &operator|=(DataType value) {
                static_cast<_ClassType &>(*this) = _value | value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator&=(DataType value) {
                static_cast<_ClassType &>(*this) = _value & value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator%=(DataType value) {
                static_cast<_ClassType &>(*this) = _value % value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator*=(DataType value) {
                static_cast<_ClassType &>(*this) = _value * value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator^=(DataType value) {
                static_cast<_ClassType &>(*this) = _value ^ value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator<<=(DataType value) {
                static_cast<_ClassType &>(*this) = _value << value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator>>=(DataType value) {
                static_cast<_ClassType &>(*this) = _value >> value;
                return static_cast<_ClassType &>(*this);
            }

            _ClassType &operator=(DataType value) {
                static_cast<_ClassType &>(*this) = value;
                return static_cast<_ClassType &>(*this);
            }

            operator DataType() {
                return _value;
            }

        protected:
            friend PCF8574;
            friend PCF8575;

            DataType _value;
            _Parent *_parent;
        };

    }

}

#include "PCF857X.h"
#include "TinyPwm.h"
#include "MCP230XX.h"
#include "PCA9685.h"

#include "IOExpander.hpp"
#include "ConfigEndIterator.hpp"
#include "ConfigIterator.hpp"
#include "PCF857X.hpp"
#include "TinyPwm.hpp"
#include "MCP230XX.hpp"
#include "PCA9685.hpp"

#endif