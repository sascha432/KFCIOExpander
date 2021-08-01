/**
  Author: sascha_lammers@gmx.de
*/

// scans the I2C bus continuously

#include <Arduino_compat.h>
#include "IOExpander.h"

void setup()
{
    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    Wire.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);
}

bool state = false;

void loop()
{
    IOExpander::scanBus(Serial, TWOWIRE_OBJECT);
    delay(5000);
}
