/**
  Author: sascha_lammers@gmx.de
*/

// error messages
#define __Error_printf(fmt, ...) IOExpander::___DBG_printf(IOEXPANDER_DEFAULT_OUTPUT, true, PSTR(fmt), ##__VA_ARGS__)


// all other debug info
#define __DBG_printf(fmt, ...) IOExpander::___DBG_printf(IOEXPANDER_DEFAULT_OUTPUT, false, PSTR(fmt), ##__VA_ARGS__)
#define __LDBG_printf(...)
