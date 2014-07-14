/* デバッグ用putsとか */

#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

#include "usart.h"
#include "ide.h"
#include <stdlib.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

void debug_puts( char *str );
void debug_int( int data );
void debug_hex( uint16_t data );
void debug_uint( uint16_t data );

#ifdef __cplusplus
}
#endif

#endif
