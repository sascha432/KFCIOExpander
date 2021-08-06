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

#if HAVE_IOEXPANDER

// enable debugging
// to disable error messages set IOEXPANDER_DEFAULT_OUTPUT=nullptr
#ifndef DEBUG_IOEXPANDER
#    define DEBUG_IOEXPANDER 0
#endif

// stream for debugging output
#ifndef IOEXPANDER_DEFAULT_OUTPUT
#    define IOEXPANDER_DEFAULT_OUTPUT Serial
#endif

// set to 1 to disable the globals variable IOExpander::config
#ifndef IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS
#    define IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS 0
#endif

#if IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS == 1 && !defined(IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS)
#    define IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS 0
#    warning support for global functions like pinMode(), digitalWrite(), etc... has been disabled
#endif

// override arduino pinMode(), digitalRead()/Write(), analogRead()/Write()...
#ifndef IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS
#    define IOEXPANDER_OVERRIDE_ARDUINO_FUNCTIONS 1
#endif

// use microseconds instead of milliseconds for the timer that invokes the interrupt handler
// if this option is enabled, system_timer_reinit() must be called in setup()
#ifndef IOEXPANDER_INTERRUPT_MICROS_TIMER
#    define IOEXPANDER_INTERRUPT_MICROS_TIMER 0
#endif

#if DEBUG_IOEXPANDER
#    include "debug_helper_enable.h"
#else
#    include "debug_helper_disable.h"
#endif

#ifndef __CONSTEXPR17
#    if __GNUC__ >= 10
#        define __CONSTEXPR17 constexpr
#    else
#        define __CONSTEXPR17
#    endif
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

    // --------------------------------------------------------------------
    // template for device configuration

    template<DeviceTypeEnum _DeviceTypeEnum, uint8_t _DefaultAddress, uint8_t _NumDigitalPins, uint8_t _NumAnalogPins, typename _DataType, bool _HasIsConnected, uint8_t _GPIOInterruptPinMode = 0, TriggerMode _IntTriggerMode = TriggerMode::NONE>
    struct DeviceTypeTemplate {
        using DeviceType = DeviceTypeTemplate<_DeviceTypeEnum, _DefaultAddress, _NumDigitalPins, _NumAnalogPins, _DataType, _HasIsConnected, _GPIOInterruptPinMode, _IntTriggerMode>;
        using DataType = _DataType;

        // device type
        static constexpr DeviceTypeEnum kDeviceType = _DeviceTypeEnum;
        // number of pins available
        static constexpr uint8_t kNumDigitalPins = _NumDigitalPins;
         // number of pins available
        static constexpr uint8_t kNumAnalogPins = _NumAnalogPins;
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

    // --------------------------------------------------------------------
    // preconfigured device templates

    using DeviceTypePCF8574 = DeviceTypeTemplate<DeviceTypeEnum::PCF8574, PCF857XAddress(), 8, 0, uint8_t, true, INPUT_PULLUP, TriggerMode::OPEN_DRAIN>;
    using DeviceTypePCF8575 = DeviceTypeTemplate<DeviceTypeEnum::PCF8575, PCF857XAddress(), 16, 0, uint16_t, true, INPUT_PULLUP, TriggerMode::OPEN_DRAIN>;
    using DeviceTypeTinyPwm = DeviceTypeTemplate<DeviceTypeEnum::TINYPWM, TinyPwmAddress(), 3, 2, uint8_t, true>;
    using DeviceTypeTinyPwm_v0_0_2 = DeviceTypeTemplate<DeviceTypeEnum::TINYPWM, TinyPwmAddress(), 1, 2, uint8_t, true>;
    using DeviceTypeMCP23008 = DeviceTypeTemplate<DeviceTypeEnum::MCP23008, MCP230XXAddress(), 8, 0, uint8_t, true, INPUT, TriggerMode::ACTIVE_HIGH>;
    using DeviceTypeMCP23017 = DeviceTypeTemplate<DeviceTypeEnum::MCP23017, MCP230XXAddress(), 16, 0, uint16_t, true, INPUT, TriggerMode::ACTIVE_HIGH>;
    using DeviceTypePCA9685 = DeviceTypeTemplate<DeviceTypeEnum::PCA9685, PCA96X5Address(), 16, 0, uint16_t, false>;

    struct DeviceTypeEnd {
        using ConfigType = DeviceTypeEnd;
        using NextConfigType = nullptr_t;
        using DeviceConfigType = void;
        using DeviceType = void;
        using DeviceClassType = void;
        using DataType = uint8_t;
        static constexpr DeviceTypeEnum kDeviceType = DeviceTypeEnum::END;
        static constexpr uint8_t kNumDigitalPins = 0;
        static constexpr uint8_t kNumAnalogPins = 0;
        static constexpr uint8_t kDefaultAddress = 0;
        static constexpr bool kHasIsConnected = false;
        static constexpr bool kHasNext = false;
    };

    template<typename _ConfigType>
    struct ConfigIterator;

    // --------------------------------------------------------------------
    // config iterator that is appended to terminate the recursion

    struct ConfigEndIterator {
        void _beginRecursive(TwoWire &wire);
        void _printStatusRecursive(Print &output);
        void _printStatusHtmlRecursive(Print &output);
        void _dumpPinsRecursive(Print &output);
        constexpr size_t _sizeRecursive() const;
        void _pinModeRecursive(uint8_t pin, uint8_t mode);
        void _digitalWriteRecursive(uint8_t pin, uint8_t val);
        int _digitalReadRecursive(uint8_t pin);
        void _analogReferenceRecursive(uint8_t mode);
        void _analogWriteRecursive(uint8_t pin, int val);
        int _analogReadRecursive(uint8_t pin);
        void _analogWriteFreqRecursive(uint32_t freq);
        constexpr void *_getDevicePointerRecursive(uint8_t pin) const;
        constexpr auto getDeviceByAddress(uint8_t address) -> nullptr_t const;
        constexpr auto getDeviceByType(DeviceTypeEnum type) -> nullptr_t const;
        constexpr auto getDeviceByPin(uint8_t pin) -> nullptr_t const;
        constexpr uint8_t _getDeviceIndexRecursive(uint8_t pin, uint8_t index) const;
        constexpr bool pinMatchAny(uint8_t pin) const;
        constexpr bool pinMatchDigital(uint8_t pin) const;
        constexpr bool pinMatchAnalog(uint8_t pin) const;
        constexpr auto _getPinMaskRecursive(uint8_t pin) -> uint8_t const;
        auto _readPortARecursive(uint8_t pin) -> uint8_t;
        auto _readPortBRecursive(uint8_t pin) -> uint8_t;
        auto _readPortRecursive(uint8_t pin) -> uint8_t;
        void _writePortARecursive(uint8_t pin, uint8_t);
        void _writePortBRecursive(uint8_t pin, uint8_t);
        void _writePortRecursive(uint8_t pin, uint8_t);
        void _attachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void _detachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask);
        void _interruptHandlerRecursive(void *device);
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

    // --------------------------------------------------------------------
    // configuration iterator to provide access to all devices via single
    // function call and no loops. recursive functions are all inline
    // to avoid the overhead of function calls

    template<typename _ConfigType>
    struct ConfigIterator {
        using ConfigType = _ConfigType;
        using DeviceClassType = typename ConfigType::DeviceClassType;
        using DeviceConfigType = typename ConfigType::DeviceConfigType;
        using DeviceType = typename ConfigType::DeviceType;
        using NextConfigIterator = typename ConfigType::NextConfigIterator;
        using DataType = typename DeviceType::DataType;

        DeviceClassType _device;
        NextConfigIterator _next;

        void begin(TwoWire &wire);
        void begin();

        // ----------------------------------------------------------------
        // status and debugging

        // print status information about all devices
        template<bool _HtmlOutput = false>
        void printStatus(Print &output);

        // print pin state of all devices
        void dumpPins(Print &output);

        // ----------------------------------------------------------------
        // methods to manage devices

        constexpr size_t size() const;

        // return device pointer for given pin
        void *getDevicePointer(uint8_t pin);
        auto getDeviceByAddress(uint8_t address) -> decltype(&_device);
        auto getDeviceByType(DeviceTypeEnum type) -> decltype(&_device);
        auto getDeviceByPin(uint8_t pin) -> decltype(&_device);

        // for readPort()
        // uint16_t ports[config.size()]; can store all ports
        constexpr uint8_t getDeviceIndex(uint8_t pin) const;
        constexpr auto getPinMask(uint8_t pin) -> DataType const;

        // ----------------------------------------------------------------
        // methods to access ports directly
        // read/writePortA access to port 0/A (access to pin 0-7)
        // read/writePortB access to port 1/B (access to pin 8-15 if the device has more than 8 pins)
        // read/writePort combined access to both ports (access to pin 0-15)

        auto readPort(uint8_t pin) -> DataType;
        auto readPortA(uint8_t pin) -> DataType;
        auto readPortB(uint8_t pin) -> DataType;

        void writePort(uint8_t pin, DataType value);
        void writePortA(uint8_t pin, DataType value);
        void writePortB(uint8_t pin, DataType value);

        // ----------------------------------------------------------------
        // arduino compatible methods

        void pinMode(uint8_t pin, uint8_t mode);
        void IRAM_ATTR digitalWrite(uint8_t pin, uint8_t val);
        int IRAM_ATTR digitalRead(uint8_t pin);
        int analogRead(uint8_t pin);
        void analogReference(uint8_t mode);
        void analogWrite(uint8_t pin, int val);
        void analogWriteFreq(uint32_t freq);

        // ----------------------------------------------------------------
        // methods for interrupts

        // return true if any device has interrupts enabled
        bool interruptsEnabled();

        // interrupts can only be enabled per device
        // the hardware interrupt must be triggered from a single GPIO pin
        // the callback will be called as scheduled_function outside the interrupt handler
        // and the handler does not have to be in IRAM
        //
        // add interrupts for pins set in pinMask
        // if interrupts are already enabled, the pins set in the mask will be added
        // gpioPin is the GPIO pin for the interrupt
        // triggerMode is the interrupt mode for the GPIO pin
        void attachInterrupt(uint8_t gpioPin, void *device, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode = TriggerMode::DEVICE_DEFAULT);

        // remove interrupts for pins set in pinMask
        // if all pins are disabled, the callback handler is removed as well
        void detachInterrupt(uint8_t gpioPin, void *device, uint16_t pinMask = ~0);

        // ----------------------------------------------------------------
        // internal recursive methods

        void _beginRecursive(TwoWire &wire);
        void _dumpPinsRecursive(Print &output);
        void _printStatusRecursive(Print &output);
        void _printStatusHtmlRecursive(Print &output);
        constexpr size_t _sizeRecursive() const;
        constexpr bool pinMatchAny(uint8_t pin) const;
        constexpr bool pinMatchDigital(uint8_t pin) const;
        constexpr bool pinMatchAnalog(uint8_t pin) const;
        void _pinModeRecursive(uint8_t pin, uint8_t mode);
        void _digitalWriteRecursive(uint8_t pin, uint8_t val);
        int _digitalReadRecursive(uint8_t pin);
        int _analogReadRecursive(uint8_t pin);
        void _analogReferenceRecursive(uint8_t mode);
        void _analogWriteRecursive(uint8_t pin, int val);
        void _analogWriteFreqRecursive(uint32_t freq);
        void *_getDevicePointerRecursive(uint8_t pin);

        constexpr uint8_t _getDeviceIndexRecursive(uint8_t pin, uint8_t index) const {
            return DeviceConfigType::pinMatchAny(pin) ? index : _next._getDeviceRecursive(pin, index + 1);
        }

        constexpr auto _getPinMaskRecursive(uint8_t pin) -> DataType const {
            return DeviceConfigType::pinMatchDigital(pin) ? _BV(pin - DeviceConfigType::kBeginPin) : _next._getPinMaskRecursive(pin);
        }

        auto _readPortRecursive(uint8_t pin) -> DataType {
            return (DeviceConfigType::pinMatchDigital(pin)) ? _device.readPort() : _next._readPortRecursive(pin);
        }

        auto _readPortARecursive(uint8_t pin) -> DataType {
            return (DeviceConfigType::pinMatchDigital(pin)) ? _device.readPortA() : _next._readPortARecursive(pin);
        }

        auto _readPortBRecursive(uint8_t pin) -> DataType {
            return (DeviceConfigType::pinMatchDigital(pin)) ? _device.readPortB() : _next._readPortBRecursive(pin);
        }

        void _writePortRecursive(uint8_t pin, DataType value) {
            return (DeviceConfigType::pinMatchDigital(pin)) ? _device.writePort(value) : _next._writePortRecursive(pin, value);
        }

        void _writePortARecursive(uint8_t pin, DataType value) {
            return (DeviceConfigType::pinMatchDigital(pin)) ? _device.writePortA(value) : _next._writePortARecursive(pin, value);
        }

        void _writePortBRecursive(uint8_t pin, DataType value) {
            return (DeviceConfigType::pinMatchDigital(pin)) ? _device.writePortB(value) : _next._writePortBRecursive(pin, value);
        }

        bool _interruptsEnabledRecursive();
        void _attachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask, const InterruptCallback &callback, uint8_t mode, TriggerMode triggerMode);
        void _detachInterruptRecursive(void *device, uint8_t gpioPin, uint16_t pinMask);

        void _interruptHandlerRecursive(void *device);

    protected:
        constexpr int _triggerMode2IntMode(TriggerMode mode) const;
    };

    // --------------------------------------------------------------------
    // configuration class for each device

    template<typename _DeviceClassType, typename _DeviceType, uint8_t _Address, uint8_t _BeginPin>
    struct DeviceConfig {
        using DeviceClassType = _DeviceClassType;
        using DeviceType = _DeviceType;

        static constexpr DeviceTypeEnum kDeviceType = DeviceType::kDeviceType;
        static constexpr uint8_t kI2CAddress = _Address;
        static constexpr uint8_t kBeginPin = _BeginPin;
        static constexpr uint8_t kEndPin = kBeginPin + (DeviceType::kNumDigitalPins > DeviceType::kNumAnalogPins) ? DeviceType::kNumDigitalPins : DeviceType::kNumAnalogPins;
        static constexpr uint8_t kNumDigitalPins = DeviceType::kNumDigitalPins;
        static constexpr uint8_t kNumAnalogPins = DeviceType::kNumAnalogPins;

        static_assert(_BeginPin >= kMinimumPinNumber, "_BeginPin must be greater or equal kMinimumPinNumber");

        static constexpr bool pinMatchAny(uint8_t pin) {
            return pin >= kBeginPin && pin < kEndPin;
        }

        static constexpr bool pinMatchDigital(uint8_t pin) {
            return pin >= kBeginPin && pin < kBeginPin + kNumDigitalPins;
        }

        static constexpr bool pinMatchAnalog(uint8_t pin) {
            return pin >= kBeginPin && pin < kBeginPin + kNumAnalogPins;
        }
    };

    // --------------------------------------------------------------------
    // base class for devices

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
        Base(uint8_t address = 0, TwoWire *wire = &Wire) : _wire(wire), _address(address), _startPin(0xff), _status(StatusType::NONE), _errors(0) {}

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

        // ----------------------------------------------------------------
        // Wire methods

        static const uint8_t kMaxErrors = 10; // stop reporting errors after this number has been reached

        void beginTransmission() {
            _status = StatusType::WRITE;
            _wire->beginTransmission(getAddress());
        }

        bool endTransmission(bool stop = true) {
            uint8_t error;
            if ((error = _wire->endTransmission(stop)) == 0) {
                if (_status != StatusType::WRITE) {
                    _status = StatusType::ERROR;
                    return false;
                }
                _status = StatusType::NONE;
                return true;
            }
            _status = StatusType::ERROR;
            if (_errors++ < kMaxErrors) {
                __DBG_printf("end_transmission error=%u stop=%u", error, stop);
                if (_errors == kMaxErrors) {
                    __DBG_printf("max. errors reached (%u), further errors won't be displayed", _errors);
                }
            }
            return false;
        }

        bool requestFrom(uint8_t len, bool stop = true) {
            auto rLen = static_cast<uint8_t>(_wire->requestFrom(getAddress(), len, static_cast<uint8_t>(stop)));
            if (rLen == len) {
                _status = StatusType::READ;
                return true;
            }
            _status = StatusType::ERROR;
            if (_errors++ < kMaxErrors) {
                __DBG_printf("request_from failed len=%u<>%u stop=%u", len, rLen, stop);
                if (_errors == kMaxErrors) {
                    __DBG_printf("max. errors reached (%u), further errors won't be displayed", _errors);
                }
            }
            return false;
        }

        bool endTransmissionAndRequestFrom(uint8_t len, bool stop = true) {
            return endTransmission(false) && requestFrom(len, stop);
        }

        void writeByte(uint8_t value) {
            if (_status != StatusType::WRITE) {
                return;
            }
            if (_wire->write(value) != 1) {
                _errors++;
                _status = StatusType::ERROR;
            }
        }

        void writeBytes(const uint8_t *buf, size_t len) {
            if (_status != StatusType::WRITE) {
                return;
            }
            if (_wire->write(buf, len) != len) {
                _errors++;
                _status = StatusType::ERROR;
            }
        }

        void writeWord(uint16_t value) {
            writeWordLE(value);
        }

        void writeWordLE(uint16_t value) {
            writeByte(value & 0xff);
            writeByte(value >> 8);
        }

        void writeWordBE(uint16_t value) {
            writeByte(value >> 8);
            writeByte(value & 0xff);
        }

        uint8_t readByte() {
            if (_status != StatusType::READ) {
                return -1;
            }
            auto res = _wire->read();
            if (res == -1) {
                _errors++;
                _status = StatusType::ERROR;
            }
            return res;
        }

        uint16_t readWord() {
            return readWordLE();
        }

        uint16_t readWordLE() {
            uint8_t lo = _wire->read();
            return lo | (_wire->read() << 8);
        }

        uint16_t readWordBE() {
            uint8_t hi = _wire->read();
            return (hi << 8) | _wire->read();
        }

        size_t readBytes(uint8_t *buf, size_t len) {
            if (_status != StatusType::READ) {
                return 0;
            }
            auto res = _wire->readBytes(buf, len);
            if (res != len) {
                _errors++;
                _status = StatusType::ERROR;
            }
            return res;
        }

        void resetErrors() {
            _errors = 0;
        }

        uint32_t getErrorCount() const {
            return _errors;
        }

    protected:
        // fallback runtime configuration
        uint8_t _getPin(uint8_t pinNo) const {
            return pinNo >= (_startPin + kNumPins) ? 0xff : (_startPin + pinNo);
        }

    protected:
        enum class StatusType : uint8_t {
            NONE,
            WRITE,
            READ,
            ERROR
        };

        TwoWire *_wire;
        uint8_t _address;
        uint8_t _startPin;
        StatusType _status;
        uint32_t _errors;
    };

    namespace Interrupts {

        enum class InterruptModeEnum : uint8_t {
            NONE = 0,
            MODE_RISING = 1,
            MODE_FALLING = 2,
            MODE_CHANGE = 3,
            MAX
        };
        static_assert(static_cast<int>(InterruptModeEnum::MAX) <= 0b100, "limited to 2 bit");

        constexpr InterruptModeEnum mode2InterruptMode(uint8_t mode) {
            return mode == RISING ? InterruptModeEnum::MODE_RISING : mode == FALLING ? InterruptModeEnum::MODE_FALLING : mode == CHANGE ? InterruptModeEnum::MODE_CHANGE : InterruptModeEnum::NONE;
        }

        // --------------------------------------------------------------------
        // class to manage interrupt pin states and capture state
        // if the device does not have internal registers for it

        template<typename _DataType>
        struct InterruptMode {

            using DataType = _DataType;
            using ModeDataType = typename std::conditional<sizeof(DataType) == 1,uint16_t,uint32_t>::type;

            static constexpr uint8_t kBits = sizeof(DataType) << 3;

            InterruptModeEnum operator [](uint8_t pin) const {
                // 2 bit value
                // mask = 3 << (pin << 1);
                // shift = (pin << 1);
                // mask bits and shift them to the right
                return static_cast<InterruptModeEnum>((_value & (3 << (pin << 1))) >> (pin << 1));
            }

            void set(uint8_t pin, InterruptModeEnum mode) {
                // 2 bit value
                // mask = 3 << (pin << 1);
                // shift = (pin << 1);
                // mask bits to change and add new settings
                _value = (_value & ~(3 << (pin << 1))) | (static_cast<uint8_t>(mode) << (pin << 1));
                if (mode == InterruptModeEnum::NONE) {
                    _mask &= ~_BV(pin);
                }
                else {
                    _mask |= _BV(pin);
                }
            }

            InterruptMode(ModeDataType value = 0, DataType state = 0) : _value(value), _state(state), _mask(0), _captured(0) {}

            operator bool() const {
                return _value != 0;
            }

            // returns the pins that changed since the last update
            DataType getEvents(DataType portValue) {
                // check if the pin state has changed
                if ((_state & _mask) == (portValue & _mask)) {
                    return 0;
                }
                // find pins with changed states
                DataType events = 0;
                for(uint8_t i = 0; i < kBits; i++) {
                    DataType mask = _BV(i);
                    switch(operator[](i)) {
                        case InterruptModeEnum::MODE_CHANGE:
                            if ((_state & mask) != (portValue & mask)) {
                                events |= mask;
                            }
                            break;
                        case InterruptModeEnum::MODE_RISING:
                            if (((_state & mask) == 0) && ((portValue & mask) != 0)) {
                                events |= mask;
                            }
                            break;
                        case InterruptModeEnum::MODE_FALLING:
                            if (((_state & mask) != 0) && ((portValue & mask) == 0)) {
                                events |= mask;
                            }
                            break;
                        default:
                            break;
                    }
                }
                return events;
            }

            void captureEvents(DataType portValue) {
                auto events = getEvents(portValue);
                if (events) {
                    _captured |= events;
                    _state = portValue;
                }
            }

            static const __FlashStringHelper *toString(InterruptModeEnum mode) {
                switch(mode) {
                    case InterruptModeEnum::NONE:
                        return F("NONE");
                    case InterruptModeEnum::MODE_RISING:
                        return F("RISING");
                    case InterruptModeEnum::MODE_FALLING:
                        return F("FALLING");
                    case InterruptModeEnum::MODE_CHANGE:
                        return F("CHANGE");
                    default:
                        break;
                }
                return F("ERR");
            }

            String toString() const {
                String tmp;
                tmp += F("port=");
                tmp += decbin(_state);
                tmp += ' ';
                tmp += F("mask=");
                tmp += decbin(_mask);
                tmp += ' ';
                for(int8_t i = kBits - 1; i >= 0; i--) {
                    char buf[32];
                    snprintf_P(buf, sizeof(buf), PSTR("%u=%s "), i, toString(operator[](i)));
                    tmp += buf;
                }
                return tmp;
            }

            ModeDataType _value;
            DataType _state;
            DataType _mask;
            DataType _captured;
        };

    }

    // --------------------------------------------------------------------
    // provides access to control bits, pin and port via variable
    // any read or write access will directly access the device
    //
    // for example
    //
    // DDR control bits pin mode OUTPUT, INPUT
    // PIN read pins
    // PORT set pin state in OUTPUT mode or enable pullup resistor in input mode

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
