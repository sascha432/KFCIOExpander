/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino_compat.h>
#include "IOExpander.h"
#include "InterruptTimer.h"


void handler(uint16_t pinState)
{
    Serial.printf_P(PSTR("PCF8574 interrupt port=%s\n"), decbin((uint8_t)pinState).c_str());
}

// first device
auto &pcf8574_1st = IOExpander::config._device;
// second device if available
// auto &pcf8574_2nd = IOExpander::config._next._device;

// getting device by its I2C Address
auto &pcf8574 = *IOExpander::config.getDeviceByAddress(0x27);

// getting the device by any pin
auto &pcf8574ByPin = *IOExpander::config.getDeviceByPin(0x80);

static constexpr auto kBeginPin = decltype(IOExpander::config)::DeviceConfigType::kBeginPin;
static constexpr auto kEndPin = decltype(IOExpander::config)::DeviceConfigType::kEndPin;

void setup()
{
    #if IOEXPANDER_INTERRUPT_MICROS_TIMER
        system_timer_reinit();
    #endif

    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    Wire.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);

    IOExpander::config.begin(Wire);
    IOExpander::config.printStatus<false>(Serial);

    for(uint8_t i = pcf8574.getPin(0); i <= pcf8574.getPin(7); i++) {
        pinMode(i, INPUT_PULLUP);
    }
    pinMode(0x87, OUTPUT);

    // monitor pin 5, 6 and 7
    IOExpander::config.attachInterrupt(14, &pcf8574, 0b01110000, handler, CHANGE, IOExpander::TriggerMode::DEVICE_DEFAULT/*=OPEN_DRAIN*/);

    // // stop monitoring pins
    // IOExpander::config.detachInterrupt(14, &pcf8574);

    IOExpander::config.printStatus<false>(Serial);
    // delay(1000);

    IOExpander::config.readPort(0x80);
    IOExpander::config.readPortA(0x80);
    IOExpander::config.readPortB(0x80);

    IOExpander::config.writePort(0x80, 0x12);
    IOExpander::config.writePortA(0x80, 1);
    IOExpander::config.writePortB(0x80, 2);
}

bool state = false;

void loop()
{
    uint8_t portValue;

    // // read all pins with digitalRead()
    // portValue = 0;
    // for(uint8_t i = 0; i < pcf8574.kNumPins; i++) {
    //     portValue |= digitalRead(pcf8574.getPin(i)) ? _BV(i) : 0;
    // }
    // Serial.printf_P(PSTR("digitalRead(0x%02x-0x%02x): "), pcf8574.getPin(0), pcf8574.getPin(pcf8574.kNumPins - 1));
    // Serial.println(decbin(portValue));

    // read the entire port at once
    portValue = pcf8574.readPort();
    Serial.print(F("readPort(): "));
    Serial.println(decbin(portValue));

    // toggle pin P7
    state = !state;
    Serial.printf_P(PSTR("digitalWrite(0x%02x, %u)\n"), pcf8574.getPin(7), state);
    // arduino digitalWrite function
    digitalWrite(pcf8574.getPin(7), state);

    // using the device object
    // pcf8574.digitalWrite(pcf8574.getPin(7), state);

    delay(5000);
}
