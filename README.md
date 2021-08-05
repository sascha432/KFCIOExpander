
# KFCIOExpander

ESP8266/ESP32 Arduino core library for various I2C I/O expanders with interrupt support

## Changelog

0.0.1 - initial commit

## Supported Devices

For more details, check the [Setup](#setup) section

- PCF8574
- PCF8575 (untested)
- MCP23017
- MCP23008 (untested)
- TinyPWM
- <s>PCA9685</s> **coming soon**
- <s>NeoPixel WS2812/WS2813</s> **coming soon**

## Design

C++ templates without any virtual classes overhead for optimal performance. Support for digitalRead/Write without adding any extra CPU cycles.

## Compatibility

- ESP8266 Arduino core 2.6.x, 2.7.x, 3.0.x
- ESP32 Arduino core (untested, might need minor adjustments **TODO**)

## <a name="setup"></a>Setup

The documentation is very basic at the moment, please check the examples.

### Configuring the globals variable IOEXpander::config

Adding an I/O expander can be done with the IOEXPANDER_DEVICE_CONFIG macro.

```c++
#define IOEXPANDER_DEVICE_CONFIG Config<DeviceConfig<PCF8574, DeviceTypePCF8574, 0x27, 100>>
//                                                                   I2C Address ^^^^  ^^^  Virtual start PIN
```

`IOExpander::Config<IOExpander::DeviceConfig<DeviceClass, DeviceType, I2CAddress, First Pin [, (Last Pin + 1)]>[, IOExpander::Config<...>]>`

| DeviceClass | DeviceType | pinMode | digitalRead/Write | analogRead/Write | Interrupts |
|---|---|---|---|---|---|
| PCF8574 | DeviceTypePCF8574 | INPUT, INPUT_PULLUP, OUTPUT | No | Yes |Yes |
| PCF8575 | DeviceTypePCF8575 | INPUT, INPUT_PULLUP, OUTPUT | No | Yes |Yes |
| MCP23008 | DeviceTypeMCP23008 | INPUT, INPUT_PULLUP, OUTPUT | Yes | No | Yes |
| MCP23017 | DeviceTypeMCP23017 | INPUT, INPUT_PULLUP, OUTPUT | Yes | No | Yes |
| TinyPwm | DeviceTypeTinyPwm | INPUT, OUTPUT | Yes | Yes | No |
| TinyPwm_V_0_0_2 | DeviceTypeTinyPwm | INPUT, OUTPUT | Yes | Yes | No |
| <s>PCA9685</s> | <s>DeviceTypePCA9685</s> | OUTPUT | Yes | Yes | No |
| <s>NeoPixel</s> | <s>DeviceTypeNeoPixel</s> | OUTPUT | Yes | Yes | No |


The Wire object needs to be initialized and passed to the I/O Expander configuration object. After that, the Arduino functions `pinMode`, `digitalWrite` , `digitalRead`, `analogWrite` and `analogRead` can be used.

```c++
    // initialize wire object
    Wire.begin(4, 5);
    // initialize all I/O Expander devices
    IOExpander::config.begin(Wire);

    // display all devices
    Serial.begin(115200);
    IOExpander::config.printStatus<false>(Serial);

    // display PIN states of all devices
    IOExpander::config.dumpPins(Serial);

    // setup pin 100
    pinMode(100, INPUT_PULLUP);
    Serial.print(F("PCF8574 PIN 100 = "));
    Serial.println(digitalRead(100);

    // direct access to the I/O Expander object
```

### Multiple Devices

```c++
#define IOEXPANDER_DEVICE_CONFIG Config<DeviceConfig<PCF8574, DeviceTypePCF8574, 0x27, 100>, \
    Config<DeviceConfig<MCP23017, DeviceTypeMCP23017, 0x20, 110>, \
        Config<DeviceConfig<MCP23017, DeviceTypeMCP23017, 0x24, 130>,\
        > \
    > \
>
```

To define the macro inside `platformio.ini` replace `<` and `>` with ` LT ` and ` GT `

```ini
build_flags =
    "-DIOEXPANDER_DEVICE_CONFIG=Config LT DeviceConfig LT PCF8574,DeviceTypePCF8574,0x27,100 GT, Config LT DeviceConfig LT MCP23017,DeviceTypeMCP23017,0x20,110 GT, GT GT"

```

### No Globals

To disable global variables, set `IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS` to 1

## Getting The Device Object

The device object can be retrieved by using its I2C address or any pin that belongs to it.
It allows to configure und use device specific features.

```c++
auto &deviceByAddress = *getDeviceByAddress(0x27);
auto &deviceByPin = *getDeviceByPin(0x80);
```

By default all devices support to read/write the entire port(s) in a single operation. 8bit devices have a single port, 16bit devices might be split up into 2 8 bit ports or have 3 methods.

```c++
device.readPort();          // read 8 or 16bit port
device.readPortA();         // read first 8bit port
device.readPortB();         // read second 8bit port

device.writePort(uint8_t / uint16_t);
device.writePortA(uint8_t);
device.writePortB(uint8_t);
```
pinMode, digitalWrite etc.. is also available per device or through the config object

```c++
IOExpander::config.pinMode(uint8_t pin, uint8_t mode);
device.pinMode(uint8_t pin, uint8_t mode);
```

## Hardware Interrupts

Interrupts are device based and require a dedicated GPIO pin to receive the interrupts.

***currently work in progress***


```c++
// high priority interrupt handler, not an ISR though
// reading from I2C is possible
// delay is not allowed and the function should return within 5-10 milliseconds or
// use schedule_function() to handle the interrupt in the main loop
void handler(uint16_t pinState)
{
    Serial.printf_P(PSTR("interrupt port=%s\n"), decbin(static_cast<uint8_t>(pinState)).c_str());
}

auto &mcp = *IOExpander::config.getDeviceByAddress(0x27);

// enable interrupts and monitor all pins
IOExpander::config.attachInterrupt(14, &mcp, 0xffff, handler, RISING, IOExpander::TriggerMode::DEVICE_DEFAULT);

// disable interrupts for the entire device
IOExpander::config.detachInterrupt(14, &mcp);
```

....documentation **TODO**
examples can be found in the `examples/` directory


## Datasheets

[PCA9635 16-bit Fm+ I2C-bus LED driver](https://www.nxp.com/docs/en/data-sheet/PCA9635.pdf)
[PCA9685 16-channel, 12-bit PWM Fm+ I2C-bus LED controller](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)
[PCF8574 Remote 8-Bit I/O Expander for I2C Bus datasheet (Rev. J)](https://www.ti.com/lit/ds/symlink/pcf8574.pdf?ts=1627704682334&ref_url=https%253A%252F%252Fwww.google.com%252F)
[PCF8575 Remote16-BIT I2C AND SMBus I/O Expander with Interrupt Output datasheet (Rev. H)](https://www.ti.com/lit/ds/symlink/pcf8575.pdf?ts=1627806036130&ref_url=https%253A%252F%252Fwww.google.com%252F)
[MCP23008 8-Bit I/O Expander with Serial Interface Data Sheet](https://ww1.microchip.com/downloads/en/DeviceDoc/21919e.pdf)
[MCP23017 Data Sheet](https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf)