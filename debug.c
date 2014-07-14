#include "debug.h"

void debug_puts( char *str )
{
    /* 一行書く */
    char *pos;

    pos = str;

    while ( *pos != '\0' ) {
        while ( !usart_can_write() );
        usart_write( *pos );

        pos++;
    }

    /* 改行 */
    while ( !usart_can_write() );
    usart_write( 0x0D );
    while ( !usart_can_write() );
    usart_write( 0x0A );
}

void debug_int( int data )
{
    /* なんかしら数字を出す */
    char str[16];

    sprintf( str, "i : %d", data );

    debug_puts( str );
}

void debug_uint( uint16_t data )
{
    char str[16];

    sprintf( str, "ui : %u", data );

    debug_puts( str );
}

void debug_hex( uint16_t data )
{
    /* なんかしら数字を出す */
    char str[16];

    sprintf( str, "x : %X", data );

    debug_puts( str );
}
