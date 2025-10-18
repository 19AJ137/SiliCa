// fuse settings for ATtiny1616
#include <avr/io.h>
#ifdef __AVR_ATtiny1616__
FUSES = {
    .WDTCFG = FUSE_WDTCFG_DEFAULT,
    .BODCFG = BOD_LVL_BODLEVEL0_gc | BOD_ACTIVE_ENABLED_gc, // Brown-out detection enabled at 1.8V
    .OSCCFG = FUSE_OSCCFG_DEFAULT,
    .TCD0CFG = FUSE_TCD0CFG_DEFAULT,
    .SYSCFG0 = FUSE_SYSCFG0_DEFAULT | FUSE_EESAVE_bm, // do not erase EEPROM on chip erase
    .SYSCFG1 = SUT_1MS_gc, // 1ms startup time
    .APPEND = FUSE_APPEND_DEFAULT,
    .BOOTEND = FUSE_BOOTEND_DEFAULT,
};
#endif
