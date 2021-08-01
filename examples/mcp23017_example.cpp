/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino_compat.h>
#include "IOExpander.h"
#include "Timer.h"

void handler(uint16_t pinState)
{
    Serial.printf_P(PSTR("MCP23017 interrupt port=%s\n"), decbin((uint8_t)pinState).c_str());
}

// first device
auto &mcp = IOExpander::config._device;

void setup()
{
    #if IOEXPANDER_INTERRUPT_MICROS_TIMER
        system_timer_reinit();
    #endif

    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    Wire.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);

    IOExpander::config.begin(Wire);
    IOExpander::config.printStatus<false>(Serial);

    for(uint8_t i = mcp.getPin(0); i <= mcp.getPin(15); i++) {
        pinMode(i, INPUT_PULLUP);
    }

    IOExpander::config.attachInterrupt(14, &mcp, 0xffff, handler, RISING);

    IOExpander::config.printStatus<false>(Serial);
}

bool state = false;

void loop()
{
    auto portValue = mcp.readPort();
    Serial.print(F("readPort(): "));
    Serial.println(decbin(portValue));
    delay(5000);
}
