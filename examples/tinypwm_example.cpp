/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino_compat.h>
#include "IOExpander.h"

static constexpr auto kBeginPin = decltype(IOExpander::config)::DeviceConfigType::kBeginPin;
static constexpr auto kEndPin = decltype(IOExpander::config)::DeviceConfigType::kEndPin;

void setup()
{
    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    Wire.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);

    IOExpander::config.begin(Wire);
    IOExpander::config.printStatus<false>(Serial);

    for(uint8_t i = 0; i < 16; i++) {
        Serial.print(decbin(IOExpander::config.getPinMask(140 + i)));
    }

    IOExpander::config.readPort(140);
    IOExpander::config.readPortA(140);
    IOExpander::config.readPortB(140);

    IOExpander::config.writePort(140, 0b11);
    IOExpander::config.writePortA(140, 0b1);
    IOExpander::config.writePortB(140, 0b1);

}

void loop()
{
}
