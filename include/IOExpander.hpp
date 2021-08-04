/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "IOExpander.h"

#if HAVE_IOEXPANDER

#if DEBUG_IOEXPANDER
#include "debug_helper_enable.h"
#else
#include "debug_helper_disable.h"
#endif

namespace IOExpander {

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    inline void Base<_DeviceType, _DeviceClassType, _DeviceConfigType>::begin(uint8_t address, TwoWire *wire)
    {
        _wire = wire;
        begin(address);
    }

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    inline void Base<_DeviceType, _DeviceClassType, _DeviceConfigType>::begin(uint8_t address)
    {
        _address = address;
        begin();
    }

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    inline void Base<_DeviceType, _DeviceClassType, _DeviceConfigType>::begin()
    {
        _errors = 0;
        _status = StatusType::NONE;
    }

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    inline bool Base<_DeviceType, _DeviceClassType, _DeviceConfigType>::isConnected() const
    {
        if (!_address || !_wire) {
            return false;
        }
        _wire->beginTransmission(_address);
        return _wire->endTransmission(true) == 0;
    }

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    inline uint8_t Base<_DeviceType, _DeviceClassType, _DeviceConfigType>::getAddress() const
    {
        return _address;
    }

    template<typename _DeviceType, typename _DeviceClassType, typename _DeviceConfigType>
    inline TwoWire &Base<_DeviceType, _DeviceClassType, _DeviceConfigType>::getWire()
    {
        return *_wire;
    }

    #if IOEXPANDER_DEVICE_CONFIG_NO_GLOBALS == 0
        #ifdef IOEXPANDER_DEVICE_CONFIG

            #define LT <
            #define GT >

            extern ConfigIterator<IOEXPANDER_DEVICE_CONFIG> config;

            #undef LT
            #undef GT

        #else

            extern ConfigEndIterator config;

        #endif
    #endif

}

#endif
