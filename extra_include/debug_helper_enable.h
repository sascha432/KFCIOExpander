/**
  Author: sascha_lammers@gmx.de
*/


#define __Error_printf(fmt, ...) IOExpander::___DBG_printf(IOEXPANDER_DEFAULT_OUTPUT, true, PSTR(fmt), ##__VA_ARGS__)
#define __DBG_printf(fmt, ...) IOExpander::___DBG_printf(IOEXPANDER_DEFAULT_OUTPUT, false, PSTR(fmt), ##__VA_ARGS__)
#define __LDBG_printf(fmt, ...) __DBG_printf(fmt, ##__VA_ARGS__)
