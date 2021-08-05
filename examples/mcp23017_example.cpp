/**
  Author: sascha_lammers@gmx.de
*/

#include <Arduino_compat.h>
#include "IOExpander.h"
#include "InterruptTimer.h"

// reads a 2x4 digits seven segment display with a MCP23017

// upper 4 digits cathode => pin
// D1 => A0
// D2 => A1
// D3 => A2
// D4 => A3
//
// lower 4 digits cathode => pin
// D1 => A4
// D2 => A5
// D3 => A6
// D4 => A7
//
// segment => pin (+pulldown)
// A => B0
// B => B1
// C => B2
// D => B3
// E => B4
// F => B5
// G => B6
// DP => B7

struct PortAB {
    union {
        uint16_t _value;
        struct {
            union {
                uint8_t digits;
                struct {
                    uint8_t u_d1: 1;
                    uint8_t u_d2: 1;
                    uint8_t u_d3: 1;
                    uint8_t u_d4: 1;
                    uint8_t l_d1: 1;
                    uint8_t l_d2: 1;
                    uint8_t l_d3: 1;
                    uint8_t l_d4: 1;
                };
            };
            union {
                struct {
                    uint8_t segments: 7;
                    uint8_t dp: 1;
                };
                struct {
                    uint8_t a: 1;
                    uint8_t b: 1;
                    uint8_t c: 1;
                    uint8_t d: 1;
                    uint8_t e: 1;
                    uint8_t f: 1;
                    uint8_t g: 1;
                    uint8_t dp: 1;
                } segment;
            };
        };
    };
    PortAB(uint16_t value) : _value(value) {}
};

using PairType = std::pair<uint8_t, char>;
// using ArrayType = std::array<PairType, 10>;

PairType __digits[] = {
    // { 0x7f, '-' },
    // { 0xff, ' ' },
    // { 0x7f, ' ' },
    // { 0x00, ' ' },
    // { 0x12, '' },
    // { 0x40, '-' },

    { 0xc0, '0' },
    { 0xf9, '1' },
    { 0xa4, '2' },
    { 0xb0, '3' },
    { 0x99, '4' },
    { 0x92, '5' },
    { 0x82, '6' },
    { 0xf8, '7' },
    { 0x80, '8' },
    { 0x90, '9' },

    { 0x3f, '0' },
    { 0x06, '1' },
    { 0x5b, '2' },
    { 0x4f, '3' },
    { 0x66, '4' },
    { 0x6d, '5' },
    { 0x7d, '6' },
    { 0x07, '7' },
    { 0x7f, '8' },
    { 0x6f, '9' },

    { 0x33, 'F' },
    { 0x73, '8' }

};

// first device
auto &mcp = IOExpander::config._device;
char digits[8];
uint8_t segments[8];
uint32_t start;

void handler(uint8_t digit, char segment, bool dp)
{
    for(uint8_t i = 0; i < 8; i++) {
        if (digit & _BV(i)) {
            // Serial.print(micros());
            // Serial.print(' ');
            // Serial.print(decbin(digit));
            // Serial.print(' ');
            // Serial.println(decbin(segment));
            segments[i] = segment;
            auto iterator = std::find_if(std::begin(__digits), std::end(__digits), [segment](const PairType &p) {
                return p.first == segment;
            });
            if (iterator != std::end(__digits)) {
                digits[i] = iterator->second;
            }
            else {
                Serial.printf("%u(%02x): %02x %u %s\n", i, digit, segment, dp, decbin(segment).c_str());
                if (!dp) {
                    // digits[i] = '?';
                    // Serial.printf("%u: %02x %u %s\n", i, segment, dp, decbin(segment).c_str());
                    // Serial.printf("%u(%02x): %02x %u %s\n", i, digit, segment, dp, decbin(segment).c_str());
                }
            }
        }
    }


    // Serial.printf_P(PSTR("MCP23017 interrupt captured=%s port=%s state=%s\n"), decbin(capValue).c_str(), decbin(value).c_str(), decbin(pinState).c_str());
    // Serial.println(micros());
}

uint8_t digit2Position(uint8_t digit) {
    for(uint8_t i = 0; i < 8; i++) {
        if (digit & _BV(i)) {
            return i;
        }
    }
    return 0xff;
}

char translate(uint8_t segment) {
    switch(segment) {
        case 0x3f:
            return '0';
        case 0x06:
            return '1';
        case 0x5b:
            return '2';
        case 0x4f:
            return '3';
        case 0x66:
            return '4';
        case 0x6d:
            return '5';
        case 0x7d:
            return '6';
        case 0x07:
            return '7';
        case 0x7f:
            return '8';
        case 0x6f:
            return '9';
        // case 0x38:
        //     return 'F';
        case 0xff:
        case 0:
            break;
            // return 0xff;
    }
    return '?';
    // return 0xff;
}

void _handler(uint16_t pinState)
{
    auto value = PortAB(pinState);
    auto pos = digit2Position(value.digits);
    if (pos != 0xff) {
        uint8_t n = pinState >> 8;
        n ^= 0xff;
        auto tn = translate(n);
        ::printf(PSTR("\x1b[%u;%uH%u: %02x \x1b[1;37;41m %c \x1b[0;37m %u"), 1, pos * 10, pos, n, tn, value.dp);
    }
    //     ::printf(PSTR("\x1b[?25h"));
    //     int row = (pos / 4) + 1;
    //     uint8_t n = pinState >> 8;
    //     n ^= 0xff;
        // auto tn = translate(n);
    //     if (tn == 0xff) {
    //         int row = (pos / 4) + 1;
    //         ::printf(PSTR("\x1b[%u;%uH\x1b[1;37;44m.\x1b[2;37m"), row + 3, (pos % 4) * 4 + 2);
    //     }
    //     else {
    //         ::printf(PSTR("\x1b[%u;%uH%u: %02x \x1b[1;37;41m %c \x1b[2;37m %u"), row, (pos % 4) * 10, pos, n, tn, value.dp);
    //         ::printf(PSTR("\x1b[%u;%uH\x1b[1;37;44m %c \x1b[2;37m"), row + 3, (pos % 4) * 4, tn);
    //         // if (value.dp) {
    //         //     Serial.printf_P(PSTR("\x1b[%u;%uH\x1b[1;37;44m.\x1b[0;37m"), row + 3, (pos % 4) * 4 + 2);
    //         // }
    //     }
    //     ::printf(PSTR("\x1b[10;1H\x1b[0m\x1b[?25h"));
    // }
    handler(value._value, value.segments, value.dp);
}

void clear() {
    Serial.print(F("\x1B[2J\x1B[H"));
    for(int j = 0; j < 24; j++) {
        for(int i = 0; i < 79; i++) {
            Serial.print(' ');
        }
        Serial.print('\n');
    }
    Serial.print(F("\x1B[2J\x1B[H"));
}

void handleIntPortA(uint16_t pinState);
void handleIntPortB(uint16_t pinState);

void handleIntPortA(uint16_t pinState) {

    // Serial.print("handleIntPortA ");

    IOExpander::config.attachInterrupt(14, &mcp, 0xff00, handleIntPortB, FALLING);
    // Serial.println(decbin(mcp.readPortA()));
    // Serial.println(decbin(mcp.readIntCapB()));

    // auto value = mcp.readPortAB();
    // Serial.println(decbin(value));

    auto portA = mcp.readPortA();
    auto value = (mcp.readIntCapB() << 8) | portA;
    _handler(value);
    mcp._DEFVAL.B = 0xff;
    mcp._write8(mcp.DEFVAL, mcp._DEFVAL, IOExpander::MCP230XXHelpers::Port::B);

}

void handleIntPortB(uint16_t pinState)
{
    IOExpander::config.attachInterrupt(14, &mcp, 0xff00, handleIntPortA, CHANGE);
    mcp.readIntCapB();
    // Serial.println(decbin(mcp.readIntCapA()));
}

void setup()
{
    #if IOEXPANDER_INTERRUPT_MICROS_TIMER
        system_timer_reinit();
    #endif

    Serial.begin(IOEXPANDER_DEFAULT_BAUDRATE);
    Wire.begin(IOEXPANDER_TWOWIRE_SDA, IOEXPANDER_TWOWIRE_SCL);

    IOExpander::config.begin(Wire);
    IOExpander::config.printStatus<false>(Serial);

    for(uint8_t i = 0; i < 8; i++) {
        mcp.pinMode(i, INPUT);
    }
    for(uint8_t i = 8; i < 16; i++) {
        mcp.pinMode(i, INPUT);
    }

    for(uint8_t i = 0; i < 16; i++) {
        Serial.println(decbin(IOExpander::config.getPinMask(0x90 + i)));
    }

    // IOExpander::config.readPort(0x90);
    // IOExpander::config.readPortA(0x90);
    // IOExpander::config.readPortB(0x90);

    // IOExpander::config.writePort(0x90, 0b11);
    // IOExpander::config.writePortA(0x90, 0b1);
    // IOExpander::config.writePortB(0x90, 0b1);

    IOExpander::config.attachInterrupt(14, &mcp, 0xff, handleIntPortA, CHANGE);

    //     [](uint16_t pinState) {
    //     // Serial.println(pinState);
    //     // auto port = mcp.readPortAB();
    //     // auto capPort = mcp.readIntCapA();
    //     // Serial.print(decbin(capPort));
    //     // Serial.print(' ');
    //     // Serial.println(decbin(port));
    //     // _handler(capPort);
    //     // mcp.ReadPortA();

    //     mcp._DEFVAL.B = 0;
    //     mcp._write(mcp.DEFVAL, mcp._DEFVAL);

    //     IOExpander::config.attachInterrupt(14, &mcp, 0xff00, FALLING);

    //     mcp.readIntCapA();

    // }, RISING);

    // mcp._IOCON.A = mcp.IOCON_INTPOL;
    // mcp._IOCON.B = mcp._IOCON.A;
    // mcp._write8(mcp.IOCON, mcp._IOCON, Port::A);
    // mcp._write8(mcp.IOCON, mcp._IOCON, Port::B);

    //     _GPINTEN._value |= pinMask;
    //     _write(GPINTEN, _GPINTEN);


    // INTCON.B = 0xff;
    // mcp.writePortB(INTCON, _INTCON);

    clear();
}

// PortAB oldValue = 0;
// size_t x = sizeof(PortAB);

void loop()
{
    // create_digits_table();
    // delay(10000);
    // state = !state;
    // mcp.digitalWrite(0, state);
    // mcp.digitalWrite(8, state);

    ets_intr_lock();
    PortAB(mcp.readPort());
    ets_intr_unlock();
    // if (value.digits != oldValue.digits) {
    //     delayMicroseconds(500);
    //     value = PortAB(mcp.readPort());
    //     oldValue = value;
    //     handler(value.digits, value.segments, value.dp);
    // }
    // Serial.print('.');
    // if (portValue >> 8 == 0xff) {
    //     delayMicroseconds(250);
    //     return;
    // }
    // delay(1);
    // uint8_t findValue = (captureValue >> 8) & 0x7f;
    // auto iterator = std::find(std::begin(digits), std::end(digits), findValue);
    // if (iterator != std::end(digits)) {

    // }
    // handler(portValue);

    if (millis() - start > 1000) {
        // doUpdate = false;
        start = millis();
        Serial.printf_P(PSTR("%c%c%c%c "), digits[0], digits[1], digits[2], digits[3]);
        Serial.printf_P(PSTR("%c%c%c%c\r"), digits[4], digits[5], digits[6], digits[7]);
    }

    // portValue &= 0x7fff;

    // Serial.print(F("readPort(): "));
    // Serial.printf("%02x %u\n", (portValue >> 8) & 0x7f, portValue & 0xff);
    // Serial.print(' ');
    // Serial.print(state);

    // Serial.println();

    // mcp.writePortAB(state & 1 ? (_BV(0)|_BV(8)) : 0);
    // mcp.writePortA(state ? 0xff : 0);
    // mcp.writePortB(state ? ~0 : 0);
    // mcp.writePortAB(state ? ~0 : 0);


    // Serial.print(n);
    // Serial.print(' ');
    // mcp.writePortA(n);
    // n++;
    // delay(50);

    // Serial.print('\r');
    // delay(500);

    // digitalWrite(mcp.getPin(mcp.PB5), bState);
}
