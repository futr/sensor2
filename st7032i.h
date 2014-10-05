#ifndef ST7032I_H_INCLUDED
#define ST7032I_H_INCLUDED

/*
 * 3.3Vではcontrastを0x24から0x26くらいにすると良いようです
 *
 */

#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    ST7032IAddress = 0x3E,
};

typedef enum {
    ST7032IIconAddrAntena  = 0x00,
    ST7032IIconAddrTel     = 0x20,
    ST7032IIconAddrConnect = 0x40,
    ST7032IIconAddrDataIn  = 0x60,
    ST7032IIconAddrUpDown  = 0x70,
    ST7032IIconAddrLock    = 0x90,
    ST7032IIconAddrMute    = 0xB0,
    ST7032IIconAddrBattery = 0xD0,
    ST7032IIconAddrMoney   = 0xF0,
} ST7032IIconAddr;

typedef enum {
    ST7032IIconAntena    = 0x08,
    ST7032IIconTel       = 0x28,
    ST7032IIconConnect   = 0x48,
    ST7032IIconDataIn    = 0x68,
    ST7032IIconUp        = 0x78,
    ST7032IIconDown      = 0x74,
    ST7032IIconLock      = 0x98,
    ST7032IIconMute      = 0xB8,
    ST7032IIconBattL1    = 0xD8,
    ST7032IIconBattL2    = 0xD4,
    ST7032IIconBattL3    = 0xD2,
    ST7032IIconBattFrame = 0xD1,
    ST7032IIconMoney     = 0xF8,
} ST7032IIcon;

void st7032i_init( uint8_t contrast );
void st7032i_set_icon( ST7032IIconAddr address, ST7032IIcon on );
void st7032i_set_char( char line, char pos, char character );
void st7032i_puts( char line, char pos, char *str );
void st7032i_clear( void );

#ifdef __cplusplus
}
#endif

#endif
