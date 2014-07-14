/*
 * ATmega88用UART0モジュール
 * 8N1専用
 *
 * 可能・許可・問題なしであれば1，不可能・不許可・問題ありは0を返す
 *
 * 初期化中は割り込みを禁止する必要があります．
 * 初期化処理はUSART使用中に実行できません．
 *
 * DREとRXの割り込みフラグは，書き込み・読み込みを実行しなければ解除されません．
 * DREは，送信バッファーが送信データを受付可能になったことを意味しており，送信は完了していません．
 * read/write関数は，読み書き可能かのチェックは行いません．ユーザー側で確認してください
 *
 * 使える割り込み
 * TX  : トランスミッターから出力完了．割り込みベクターで自動解除．
 * RX  : 未読データーがバッファー内にある．バッファーをカラにしなければ割り込み続ける．受信割り込み．
 * DRE : トランスミッターバッファーに受け入れ可能．受け入れ不可能でない限り常に発生する．連続送信に使う．
 */

#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#define USART_BUFFER_LENGTH 16

#include "ide.h"
#include "fifo.h"
#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

typedef enum UsartMode_tag {
    UsartRX = 0x01,
    UsartTX = 0x02,
    Usart2X = 0x04,
} UsartMode;

typedef enum UsartInt_tag {
    UsartIntRX  = 0x01,
    UsartIntTX  = 0x02,
    UsartIntDRE = 0x04,
} UsartInt;

#ifdef __cplusplus
extern "C" {
#endif

/* 初期化・終了 */
void usart_init( unsigned long bps, UsartMode mode_enable, UsartInt interrupt_enable );
void usart_release( void );

/* 起動・停止 */
void usart_enable_tx( char enable );
void usart_enable_rx( char enable );
void usart_enable_int_tx( char enable );
void usart_enable_int_rx( char enable );
void usart_enable_int_dre( char enable );

/* 通信 */
char usart_can_read( void );
char usart_can_write( void );
uint8_t usart_read( void );
void usart_write( uint8_t data );

/* エラー処理 */
uint8_t usart_get_error( void );
char usart_error_frame( uint8_t error );
char usart_error_parity( uint8_t error );
char usart_error_overrun( uint8_t error );

#ifdef __cplusplus
}
#endif

#endif
