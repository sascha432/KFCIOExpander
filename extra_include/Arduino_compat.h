/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>

#ifndef __CONSTEXPR17
#   if __GNUC__ >= 10
#       define __CONSTEXPR17 constexpr
#   else
#       define __CONSTEXPR17
#   endif
#endif

#ifndef _STRINGIFY
#   define _STRINGIFY(...) ___STRINGIFY(__VA_ARGS__)
#endif
#ifndef ___STRINGIFY
#   define ___STRINGIFY(...) #__VA_ARGS__
#endif
#ifndef HTML_S
#   define HTML_S(tag) "<" ___STRINGIFY(tag) ">"
#   define HTML_E(tag) "</" ___STRINGIFY(tag) ">"
#endif

namespace IOExpander {
    void ___DBG_printf(nullptr_t, bool errorsOnly, const char *, ...);
    void ___DBG_printf(Stream &, bool errorsOnly, const char *, ...);
}

// convert integer to binary string
// add a space every _Space bits
// use 0xff to disable extra spaces
// set _Reverse to true to display the lowest bit first
template<typename _Ta, uint8_t _Space = 8, bool _Reverse = false>
String decbin(_Ta value) {
    constexpr uint8_t addSpaces = (sizeof(_Ta) << 3) < _Space ? 0xff : _Space;
    constexpr uint8_t extraSpace = ((sizeof(_Ta) << 3) / addSpaces) + 1;
    constexpr _Ta mask = _Reverse ? 1 : 1 << ((sizeof(_Ta) << 3) - 1);
    char buf[(sizeof(_Ta) << 3) + extraSpace];
    auto endPtr = &buf[(sizeof(_Ta) << 3)];
    auto ptr = buf;
    uint8_t space;
    if __CONSTEXPR17 (addSpaces != 0xff) {
        space = addSpaces;
    }
    for(;;) {
        *ptr++ = (value & mask) ? '1' : '0';
        if (ptr >= endPtr) {
            break;
        }
        if __CONSTEXPR17 (_Reverse) {
            value >>= 1;
        }
        else {
            value <<= 1;
        }
        if __CONSTEXPR17 (addSpaces != 0xff) {
            if (--space == 0) {
                space = addSpaces;
                *ptr++ = ' ';
                endPtr++;
            }
        }
    }
    *ptr = 0;
    return buf;
}

// convert integer to binary string
// decbin with _Reverse = true
template<typename _Ta, uint8_t _Space = 8>
inline __attribute__((__always_inline__))
String decrbin(_Ta value) {
    return decbin<_Ta, _Space, true>(value);
}
