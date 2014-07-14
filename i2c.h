/*
 * I2Cマスター専用モジュール
 * マルチマスターに対応していない非常に簡素なものです．
 * 何かしら失敗すればすぐに諦めるシステムです．
 * i2cは高度にしようと思えばとことん高度になってしまいそうなので，自分の利用する範囲で実装するのが良いかと思います．
 * マルチマスターで使用して調停に失敗した場合，I2Cを停止させます．
 *
 * プルアップはPORTレジスターによって制御されます．
 *
 * i2cの割り込みフラグは，解錠することによって次の処理が実行されるようになっています．
 * 必ずユーザーは解除しなければなりません( 1を書き込むと解除されます )
 * このライブラリでは，関数を順序よく実行すれば勝手に解除しています．
 * よって，割り込みを使用しても，使用しなくても大丈夫です．
 *
 * コードサイズが大きすぎる場合は，コールバック処理，プルアップ処理，自動処理をソースコードから消しても問題ありません．
 *
 * 8MHzの場合，TWBR = 2, Prescale = 1で400KHz TWBR = 32, Prescale = 1で100KHz
 *
 **********
 * 使い方
 *
 * 各i2c_status関数はI2CWorkingが解除されるまで繰り返す．
 **********
 *** MT
 * i2c_master_start();
 *  i2c_status_master_started();
 * i2c_master_address( slave, I2CW );
 *  i2c_status_master_address_ack();
 *   I2CNACK : i2c_master_stop();
 *   I2CACK  : 次へ
 *   else    : 何もしないで停止
 * i2c_master_write( data );
 *  i2c_status_master_writ_ack();
 *   I2CNACK : i2c_master_stop();
 *   I2CACK  : 必要ならi2c_maste_writeを繰り返し，次へ
 *   else    : 何もしないで停止
 * 終了する場合
 *  i2c_master_stop();
 * 続ける場合
 *  i2c_mater_start();
 *
 *** MR
 * i2c_master_start();
 *  i2c_status_master_started();
 * i2c_master_address( slave, I2CR );
 *  i2c_status_master_address_ack();
 *   I2CNACK : i2c_master_stop();
 *   I2CACK  : 次へ
 *   else    : 何もしないで停止
 * i2c_master_read_ack( ack ); このバイトで受信終了ならNACK，続行ならACK
 *  i2c_status_master_read();
 *   I2CSuccess : data = i2c_read()でデータを読み込みの後，必要ならi2c_master_read_ackを繰り返し次へ，
 *   I2CError   : 何もしないで停止
 * 終了する場合
 *  i2c_master_stop();
 * 続ける場合
 *  i2c_master_start();
 *
 *********
 * 自動関数
 *********
 * 基本的には，i2c_auto_master_start()を実行し，
 * i2c_auto_process()がI2CSuccessになるのをまち，
 * i2c_auto_master_stop()かi2c_auto_master_start()を実行する．
 *
 * i2c_auto_process()でI2CSuccess以外が帰ってきた場合は，そのまま終了して放置して良い．
 *
 * i2c_auto_process()を割り込みで実行し，i2c_auto_complete()を調べることで，ゆったりとしたポーリングすることも可能
 * また，コールバックを登録している場合は，i2c_auto_complete()が変化したタイミングでコールバックされる
 *  I2CError   : どこかのタイミングで何かしらのエラーが発生した
 *  I2CSuccess : 成功して処理を完了している
 *  I2CNACK    : どこかのタイミングでNACKされた
 *  I2CWorking : 処理中
 *
 */

#ifndef I2C_H_INCLUDED
#define I2C_H_INCLUDED

#include "ide.h"
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>

typedef enum I2CPin_tag {
    I2CSCL = 0x01,
    I2CSDA = 0x02,
} I2CPin;

typedef enum I2CPrescale_tag {
    I2CPrescale1  = 0x00,
    I2CPrescale4  = 0x01,
    I2CPrescale16 = 0x02,
    I2CPrescale64 = 0x03,
} I2CPrescale;

typedef enum I2CSide_tag {
    I2CMaster,
    I2CSlave,
} I2CSide;

typedef enum I2CRW_tag {
    I2CR,
    I2CW,
} I2CRW;

typedef enum I2CStatus_tag {
    I2CError = 0,
    I2CWorking,
    I2CSuccess,
    I2CACK,
    I2CNACK,
} I2CStatus;

typedef enum I2CStep_tag {
    I2CStepNone,
    I2CStepMasterStart,
    I2CStepMasterAddressR,
    I2CStepMasterAddressW,
    I2CStepMasterReadACK,
    I2CStepMasterReadNACK,
    I2CStepMasterWrite,
    I2CStepMasterStop,
} I2CStep;

typedef enum I2CProcessMode_tag {
    I2CPolling,
    I2CInterrupt,
} I2CProcessMode;

/* 自動処理の内部用 */
typedef struct I2CUnit_tag {
    I2CStep   step;
    I2CRW     rw;
    I2CSide   side;
    uint8_t   *data;
    size_t    size;
    size_t    pos;
    uint8_t   slave;
    volatile I2CStatus complete;
} I2CUnit;

#ifdef __cplusplus
extern "C" {
#endif

/* 初期化・解放 */
void i2c_init_master( uint8_t baud, I2CPrescale prescale, I2CPin pullup, char int_enable );
void i2c_release( void );
void i2c_enable( char enable );
void i2c_enable_int( char enable );
void i2c_enable_pullup_SDA( char enable );
void i2c_enable_pullup_SCL( char enable );

/* マスター通信関数 */
void i2c_master_start( void );
void i2c_master_address( uint8_t slave, I2CRW rw );
void i2c_master_read_ack( I2CStatus ack );
void i2c_master_stop( void );
void i2c_master_write( uint8_t data );
uint8_t i2c_read( void );

/* 状態確認 */
uint8_t i2c_get_status( void );
I2CStatus i2c_status_master_started( void );
I2CStatus i2c_status_master_address_ack( void );
I2CStatus i2c_status_master_writ_ack( void );
I2CStatus i2c_status_master_read( void );

/* マスター専用自動処理 */
void i2c_auto_set_callback( void (*func)( void ) );		/* autoが何らかのcomplete状態に入ったら＊割り込み中に＊呼ばれる */
I2CStatus i2c_auto_process( void );						/* 読み書き完了後のstop/startは実行せず，ユーザーに任せる */
I2CStatus i2c_auto_complete( void );					/* どのような状態で処理が完了したかを返す．処理中ならI2CWorking */
void i2c_auto_master_start( uint8_t slave, I2CRW rw, uint8_t *data, size_t size );
void i2c_auto_master_stop( void );

char i2c_write_register( uint8_t slave, uint8_t address, uint8_t *data, size_t size, I2CProcessMode mode );
char i2c_read_register( uint8_t slave, uint8_t address, uint8_t *data, size_t size, I2CProcessMode mode );

#ifdef __cplusplus
}
#endif

#endif
