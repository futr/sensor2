#include "usart.h"

void usart_init( unsigned long bps, UsartMode mode_enable, UsartInt interrupt_enable )
{
    /* USART初期化 */
    unsigned long ubrr;

    /* 一旦停止させる */
    usart_release();

    /* ボーレートレジスター計算 */
    if ( mode_enable & Usart2X ) {
        ubrr = F_CPU / ( 8 * bps ) - 1;
    } else {
        ubrr = F_CPU / ( 16 * bps ) - 1;
    }

    /* 2倍速 */
    if ( mode_enable & Usart2X ) {
        UCSR0A |= _BV( U2X0 );
    } else {
        UCSR0A &= ~_BV( U2X0 );
    }

    /* UBRRに転送 */
    UBRR0L = ubrr;
    UBRR0H = ubrr >> 8;

    /* MPCM解除 */
    UCSR0A &= ~_BV( MPCM0 );

    /* USARTのモードを設定 ( 8N1しか作ってない ) */
    UCSR0B &= ~_BV( UCSZ02 );
    UCSR0C = 0x06;

    /* 起動 */
    if ( mode_enable & UsartRX ) {
        usart_enable_rx( 1 );
    }

    if ( mode_enable & UsartTX ) {
        usart_enable_tx( 1 );
    }

    /* 割り込み起動 */
    if ( interrupt_enable & UsartIntTX ) {
        usart_enable_int_tx( 1 );
    }

    if ( interrupt_enable & UsartIntRX ) {
        usart_enable_int_rx( 1 );
    }

    if ( interrupt_enable & UsartIntDRE ) {
        usart_enable_int_dre( 1 );
    }
}

void usart_enable_tx( char enable )
{
    /* トランスミッター起動・停止 */
    if ( enable ) {
        UCSR0B |= _BV( TXEN0 );
    } else {
        UCSR0B &= ~_BV( TXEN0 );
    }
}

void usart_enable_rx( char enable )
{
    /* レシーバー起動・停止 */
    if ( enable ) {
        UCSR0B |= _BV( RXEN0 );
    } else {
        UCSR0B &= ~_BV( RXEN0 );
    }
}

void usart_enable_int_tx( char enable )
{
    /* TX割り込み起動・停止 */
    if ( enable ) {
        UCSR0B |= _BV( TXCIE0 );
    } else {
        UCSR0B &= ~_BV( TXCIE0 );
    }
}

void usart_enable_int_rx( char enable )
{
    /* RX割り込み起動・停止 */
    if ( enable ) {
        UCSR0B |= _BV( RXCIE0 );
    } else {
        UCSR0B &= ~_BV( RXCIE0 );
    }
}

void usart_enable_int_dre( char enable )
{
    /* DRE割り込み起動・停止 */
    if ( enable ) {
        UCSR0B |= _BV( UDRIE0 );
    } else {
        UCSR0B &= ~_BV( UDRIE0 );
    }
}

void usart_release( void )
{
    /* USART停止 */
    usart_enable_int_tx( 0 );
    usart_enable_int_rx( 0 );
    usart_enable_int_dre( 0 );
    usart_enable_tx( 0 );
    usart_enable_rx( 0 );
}

char usart_can_read( void )
{
    /* 読み込み可能か調べる */
    if ( bit_is_set( UCSR0A, RXC0 ) ) {
        return 1;
    } else {
        return 0;
    }
}

char usart_can_write( void )
{
    /* 書き込み可能か調べる */
    if ( bit_is_set( UCSR0A, UDRE0 ) ) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t usart_read( void )
{
    /* 読み込み */
    return UDR0;
}


void usart_write( uint8_t data )
{
    /* 書き込み */
    UDR0 = data;
}

uint8_t usart_get_error( void )
{
    /* エラー情報取得 */
    return UCSR0A & ( _BV( FE0 ) | _BV( DOR0 ) | _BV( UPE0 ) );
}

char usart_error_frame( uint8_t error )
{
    /* フレームエラー発生 */
    if ( error & _BV( FE0 ) ) {
        return 1;
    } else {
        return 0;
    }
}

char usart_error_parity( uint8_t error )
{
    /* パリティーエラー発生 */
    if ( error & _BV( UPE0 ) ) {
        return 1;
    } else {
        return 0;
    }
}

char usart_error_overrun( uint8_t error )
{
    /* データーオーバーラン発生 */
    if ( error & _BV( DOR0 ) ) {
        return 1;
    } else {
        return 0;
    }
}
