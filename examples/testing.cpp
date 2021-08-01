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

auto &device1 = IOExpander::config._device;

void setup()
{
    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    TWOWIRE_OBJECT.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);

    delay(5000);
    Serial.println("start");

    Wire.begin(4, 5);
    IOExpander::config.begin(Wire);

    for(uint8_t i = 0; i < device1.kNumPins; i++) {
        // device1.pinMode(i, INPUT);
        device1.pinMode(i, INPUT_PULLUP);
    }
    device1.pinMode(7, OUTPUT);

    // device1.pinMode(15, OUTPUT);

    // IOExpander::config.attachInterrupt(14, &device1, 0b01110000, handler, CHANGE);

    IOExpander::config.printStatus<false>(Serial);

}

bool state = false;

void loop()
{
    // auto tmp1 = device1.readPortAB();
    // Serial.println(decbin(tmp1));


    auto tmp1 = device1.readPort();
    Serial.println(decbin(tmp1));


    state = !state;
    // Serial.println("---");
    device1.digitalWrite(7, state);
    // device1.digitalWrite(15, state);

    // Serial.println(F("delay-start"));
    // // uint32_t start = millis();
    // // while(millis() - start < 5000) {
    // //     delay(1);
    // // }
    // Serial.println(F("delay-end"));

    delay(1000);
}