/**
  Author: sascha_lammers@gmx.de
*/


#ifndef __DBG_printf

#define __DBG_printf(fmt, ...) IOExpander::___DBG_printf(IOEXPANDER_DEFAULT_OUTPUT, false, PSTR(fmt), ##__VA_ARGS__)
#define __LDBG_printf(fmt, ...) IOExpander::___DBG_printf(IOEXPANDER_DEFAULT_OUTPUT, false, PSTR(fmt), ##__VA_ARGS__)

#endif