/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino_compat.h>
#include "IOExpander.h"

void handler(uint16_t pinState)
{
    // Serial.printf("interrupt %04x\n", pinState);
    Serial.printf("interrupt %u\n", pinState & 0x0100 ? 1 : 0);
}

// first device
auto &pcf8574_1st = IOExpander::config._device;
// second device if available
// auto &pcf8574_2nd = IOExpander::config._next._device;

// getting device by its I2C Address
auto &pcf8574 = *IOExpander::config.getDeviceByAddress(0x27);

// getting the device by any pin
auto &pcf8574ByPin = *IOExpander::config.getDeviceByPin(0x80);

void setup()
{
    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    Wire.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);

    IOExpander::config.begin(Wire);
    IOExpander::config.printStatus<false>(Serial);

    for(uint8_t i = 0x80; i < 0x80 + 8; i++) {
        pinMode(i, INPUT_PULLUP);
    }
    pinMode(0x87, OUTPUT);

    // IOExpander::config.attachInterrupt(14, &device1, 0b01110000, handler, CHANGE);

    IOExpander::config.printStatus<false>(Serial);
}

bool state = false;

void loop()
{
    uint8_t portValue;

    // read all pins with digitalRead()
    portValue = 0;
    for(uint8_t i = 0; i < pcf8574.kNumPins; i++) {
        portValue |= digitalRead(pcf8574.getPin(i)) ? _BV(i) : 0;
    }
    Serial.printf_P(PSTR("digitalRead(0x%02x-0x%02x): "), pcf8574.getPin(0), pcf8574.getPin(pcf8574.kNumPins - 1));
    Serial.println(decbin(portValue));

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

    delay(1000);
}
