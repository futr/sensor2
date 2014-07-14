#include "i2c.h"

/* モジュール内変数 */
static I2CUnit unit;
static void (*auto_callback)( void ) = NULL;

void i2c_init_master( uint8_t baud, I2CPrescale prescale, I2CPin pullup, char int_enable )
{
    /* マスターモードで初期化 */

    /* 速度設定 */
    TWBR = baud;

    switch ( prescale ) {
    case I2CPrescale1:
        TWSR = 0;
        break;

    case I2CPrescale4:
        TWSR = 2;
        break;

    case I2CPrescale16:
        TWSR = 3;
        break;

    case I2CPrescale64:
        TWSR = 3;
        break;

    default:
        TWSR = 3;
        break;
    }

    /* 有効化 */
    i2c_enable( 1 );

    /* 必要なら割り込み有効化 */
    if ( int_enable ) {
        i2c_enable_int( 1 );
    }

    /* プルアップ */
    if ( pullup & I2CSDA ) {
        i2c_enable_pullup_SDA( 1 );
    } else {
        i2c_enable_pullup_SDA( 0 );
    }

    if ( pullup & I2CSCL ) {
        i2c_enable_pullup_SCL( 1 );
    } else {
        i2c_enable_pullup_SCL( 0 );
    }

    /* 状態を初期化 */
    unit.step = I2CStepNone;
    unit.side = I2CMaster;
}

void i2c_release( void )
{
    /* I2Cバスを解放する */
    i2c_enable_int( 0 );
    i2c_enable( 0 );
}

void i2c_enable( char enable )
{
    /* I2Cバスを取得・解放 */
    if ( enable ) {
        TWCR = ( TWCR & ~_BV( TWINT ) ) | _BV( TWEN );
    } else {
        TWCR = ( TWCR & ~_BV( TWINT ) ) & ~_BV( TWEN );
    }
}

void i2c_enable_int( char enable )
{
    /* 割り込み有効化 */
    if ( enable ) {
        TWCR = ( TWCR & ~_BV( TWINT ) ) | _BV( TWIE );
    } else {
        TWCR = ( TWCR & ~_BV( TWINT ) ) & ~_BV( TWIE );
    }
}

void i2c_enable_pullup_SDA( char enable )
{
    /* SDAプルアップ指定 */
    if ( enable ) {
        PORTC |= _BV( PC4 );
    } else {
        PORTC &= ~_BV( PC4 );
    }
}

void i2c_enable_pullup_SCL( char enable )
{
    /* SCLプルアップ指定 */
    if ( enable ) {
        PORTC |= _BV( PC5 );
    } else {
        PORTC &= ~_BV( PC5 );
    }
}

void i2c_master_start( void )
{
    /* スタートコンディション発行 */
    TWCR = ( TWCR & _BV( TWIE ) ) | ( _BV( TWINT ) | _BV( TWSTA ) | _BV( TWEN ) );

    unit.step = I2CStepMasterStart;
}

void i2c_master_address( uint8_t slave, I2CRW rw )
{
    /* スレーブをアドレスする */
    if ( rw == I2CW ) {
        /* 書き込み */
        TWDR = ( slave << 1 );

        unit.step = I2CStepMasterAddressW;
    } else {
        /* 読み込み */
        TWDR = ( slave << 1 ) | 0x01;

        unit.step = I2CStepMasterAddressR;
    }

    TWCR = ( TWCR & _BV( TWIE ) ) | ( _BV( TWINT ) | _BV( TWEN ) );
}

void i2c_master_read_ack( I2CStatus ack )
{
    /* 読み込み返答を返し次へ */
    if ( ack == I2CACK ) {
        /* ACK送信 */
        TWCR = ( TWCR & _BV( TWIE ) ) | ( _BV( TWINT ) | _BV( TWEN ) | _BV( TWEA ) );

        unit.step = I2CStepMasterReadACK;
    } else {
        /* NACK送信 */
        TWCR = ( TWCR & _BV( TWIE ) ) | ( _BV( TWINT ) | _BV( TWEN ) );

        unit.step = I2CStepMasterReadNACK;
    }
}

void i2c_master_write( uint8_t data )
{
    /* 書き込み */
    TWDR = data;
    TWCR = ( TWCR & _BV( TWIE ) ) | ( _BV( TWINT ) | _BV( TWEN ) );

    unit.step = I2CStepMasterWrite;
}

void i2c_master_stop( void )
{
    /* ストップコンディション作成 */
    TWCR = ( TWCR & _BV( TWIE ) ) | ( _BV( TWINT ) | _BV( TWSTO ) | _BV( TWEN ) );

    unit.step = I2CStepMasterStop;
}

uint8_t i2c_get_status( void )
{
    /* 状態を読み取りプリスケーラーをマスクする */
    return TWSR & 0xF8;
}

I2CStatus i2c_status_master_started( void )
{
    /* マスターによるスタートコンディションの判定 */
    uint8_t status;

    status = i2c_get_status();

    if ( status == TW_START || status == TW_REP_START ) {
        /* 成功 */
        return I2CSuccess;
    } else {
        /* 失敗 */
        return I2CError;
    }
}

I2CStatus i2c_status_master_address_ack( void )
{
    /* マスターによるアドレス送信への返答 */
    uint8_t status;

    status = i2c_get_status();

    switch ( status ) {
    case TW_MR_SLA_ACK:
    case TW_MT_SLA_ACK:
        return I2CACK;

    case TW_MR_SLA_NACK:
    case TW_MT_SLA_NACK:
        return I2CNACK;

    default:
        /* 主に調停に負けた場合 */
        return I2CError;
    }
}

I2CStatus i2c_status_master_writ_ack( void )
{
    /* マスターからの書き込みに対するスレーブ側からの(N)ACK */
    uint8_t status;

    status = i2c_get_status();

    switch ( status ) {
    case TW_MT_DATA_ACK:
        return I2CACK;

    case TW_MT_DATA_NACK:
        return I2CNACK;

    default:
        /* 主に調停に負けた場合 */
        return I2CError;
    }
}

I2CStatus i2c_status_master_read( void )
{
    /* マスターがスレーブからデータを受診し終え，(N)ACKを送信完了した */
    uint8_t status;

    status = i2c_get_status();

    switch ( status ) {
    case TW_MR_DATA_ACK:
        return I2CSuccess;

    case TW_MR_DATA_NACK:
        return I2CSuccess;

    default:
        /* 主に調停に負けた場合 */
        return I2CError;
    }
}

void i2c_auto_set_callback( void (*func)( void ) )
{
    /* 自動実行完了時に呼ばれるコールバック登録 */
    auto_callback = func;
}

I2CStatus i2c_auto_process( void )
{
    /* I2Cの処理を自動実行する */
    /* DEBUG : 最後にStepNoneを書いてるのは省いてもいいはず */
    I2CStatus status;

    /* 無効な状態か停止後なら何もしない */
    if ( unit.step == I2CStepNone || unit.step == I2CStepMasterStop ) {
        return I2CError;
    }

    /* I2Cが起動していなければ何もしない */
    if ( !( TWCR & _BV( TWEN ) ) ) {
        return I2CError;
    }

    /* フラグをチェックし，稼働中なら即返す */
    if ( !( TWCR & _BV( TWINT ) ) ) {
        return I2CWorking;
    }

    /* ひとつのステップが完了したので結果確認と次のステップの実行 */
    switch ( unit.step ) {
    case I2CStepMasterStart:
        /* マスターでスタートコンディションが発行された */
        if ( i2c_status_master_started() == I2CSuccess ) {
            /* 成功したのでアドレス送信開始 */
            i2c_master_address( unit.slave, unit.rw );

            return I2CWorking;
        } else {
            /* 失敗したので後処理 */
            goto L_I2C_PROCESS_FAILED;
        }

    case I2CStepMasterAddressR:
        /* マスターがSLA+Rを送信完了した */
        status = i2c_status_master_address_ack();

        if ( status == I2CACK ) {
            /* ACKだったので受信と(N)ACK送信開始 */
            /* 読み込みポインター初期化 */
            unit.pos = 0;

            if ( unit.size < 2 ) {
                /* サイズが1だったので一回目でNACK */
                i2c_master_read_ack( I2CNACK );
            } else {
                i2c_master_read_ack( I2CACK );
            }

            return I2CWorking;
        } else if ( status == I2CNACK ) {
            /* NACKが帰ってきたので終了 */
            i2c_master_stop();				/* このストップもユーザー任せでもいいかも */

            unit.complete = I2CNACK;
            unit.step     = I2CStepNone;
            if ( auto_callback != NULL ) {
                auto_callback();
            }
            return I2CNACK;					/* DEBUG : ErrorでなくてNACKとかを返すべきかも？->そうしてみた */
        } else {
            /* 失敗したので後処理 */
            goto L_I2C_PROCESS_FAILED;
        }

    case I2CStepMasterReadACK:
        /* 前回の読み込みをACKで実行した ( 読み込み続行 ) */
        status = i2c_status_master_read();

        if ( status == I2CSuccess ) {
            /* 問題なくACKを送信できた */
            /* データー読み込み */
            unit.data[unit.pos] = i2c_read();
            unit.pos++;

            /* 読み込み終了判定 */
            if ( unit.pos == unit.size - 1 ) {
                /* 次回で読み込み完了 */
                i2c_master_read_ack( I2CNACK );
            } else {
                /* まだ完了していない */
                i2c_master_read_ack( I2CACK );
            }

            return I2CWorking;
        } else {
            /* 失敗したので後処理 */
            goto L_I2C_PROCESS_FAILED;
        }

    case I2CStepMasterReadNACK:
        /* 前回の読み込みをNACKで実行した ( これで読み込み完了 ) */
        status = i2c_status_master_read();

        if ( status == I2CSuccess ) {
            /* 問題なくNACKを送信できた */
            /* データー読み込み */
            unit.data[unit.pos] = i2c_read();

            /* 読み込み終了したのでユーザーに完了を知らせる */
            unit.step     = I2CStepNone;
            unit.complete = I2CSuccess;
            if ( auto_callback != NULL ) {
                auto_callback();
            }
            return I2CSuccess;
        } else {
            /* 失敗したので後処理 */
            goto L_I2C_PROCESS_FAILED;
        }

    case I2CStepMasterAddressW:
        /* マスターがSLA+Wを送信完了した */
        status = i2c_status_master_address_ack();

        if ( status == I2CACK ) {
            /* ACKだったので送信と(N)ACK受信開始 */
            /* 書き込みポインター初期化 */
            unit.pos = 0;

            /* 送信 */
            i2c_master_write( unit.data[unit.pos] );

            return I2CWorking;
        } else if ( status == I2CNACK ) {
            /* NACKが帰ってきたので終了 */
            i2c_master_stop();

            unit.complete = I2CNACK;
            unit.step     = I2CStepNone;
            if ( auto_callback != NULL ) {
                auto_callback();
            }
            return I2CNACK;
        } else {
            /* 失敗したので後処理 */
            goto L_I2C_PROCESS_FAILED;
        }

    case I2CStepMasterWrite:
        /* マスターが送信完了した */
        status = i2c_status_master_writ_ack();

        if ( status == I2CACK ) {
            /* スレーブからACKが帰ってきたので必要なら続きを書き込む */
            /* ポインタ加算 */
            unit.pos++;

            /* 終了確認 */
            if ( unit.pos == unit.size ) {
                /* 全部書きこみ終わったので終了 */
                unit.complete = I2CSuccess;
                unit.step     = I2CStepNone;
                if ( auto_callback != NULL ) {
                    auto_callback();
                }
                return I2CSuccess;
            } else {
                /* まだ書き込む必要があるので次を書き込む */
                i2c_master_write( unit.data[unit.pos] );

                return I2CWorking;
            }
        } else if ( status == I2CNACK ) {
            /* スレーブからNACKが帰ってきたので即終了 */
            i2c_master_stop();

            unit.complete = I2CNACK;
            unit.step     = I2CStepNone;
            if ( auto_callback != NULL ) {
                auto_callback();
            }
            return I2CNACK;
        } else {
            /* 失敗したので後処理 */
            goto L_I2C_PROCESS_FAILED;
        }

    default:
        /* ありえないので即停止 */
        goto L_I2C_PROCESS_FAILED;
    }

L_I2C_PROCESS_FAILED:
    /* i2cプロセスに問題が発生した */
    unit.complete = I2CError;
    unit.step     = I2CStepNone;
    if ( auto_callback != NULL ) {
        auto_callback();
    }

    return I2CError;
}

void i2c_auto_master_start( uint8_t slave, I2CRW rw, uint8_t *data, size_t size )
{
    /* 自動実行開始 */

    /* sizeが1未満なら何もしない */
    if ( size < 1 ) {
        return;
    }

    /* 必要な情報を保存 */
    unit.rw       = rw;
    unit.data     = data;
    unit.size     = size;
    unit.side     = I2CMaster;
    unit.slave    = slave;
    unit.complete = I2CWorking;

    /* スタートコンディション発行 */
    i2c_master_start();
}

uint8_t i2c_read( void )
{
    /* i2cデーターレジスターから読み出し */
    return TWDR;
}

void i2c_auto_master_stop( void )
{
    /* 単純にstopコンディションを発行するだけ */
    i2c_master_stop();
}


I2CStatus i2c_auto_complete( void )
{
    /* 自動モードで完了フラグを読む */
    return unit.complete;
}

char i2c_write_register( uint8_t slave, uint8_t address, uint8_t *data, size_t size, I2CProcessMode mode )
{
    /* レジスタ書き込み */
    uint8_t wdata[17];
    int i;
    I2CStatus status;

    /* i2cアクセスの制限で16バイト以上かけない */
    if ( size > 16 ) {
        return 0;
    }

    wdata[0] = address;

    for ( i = 0; i < size; i++ ) {
        wdata[i + 1] = data[i];
    }

    i2c_auto_master_start( slave, I2CW, wdata, size + 1 );

    if ( mode == I2CPolling ) {
        while ( ( status = i2c_auto_process() ) == I2CWorking );
    } else {
        while ( ( status = i2c_auto_complete() ) == I2CWorking );
    }

    if ( status == I2CError ) {
        return 0;
    }
    i2c_auto_master_stop();

    return 1;
}

char i2c_read_register( uint8_t slave, uint8_t address, uint8_t *data, size_t size, I2CProcessMode mode )
{
    /* レジスタ読み込み */
    I2CStatus status;

    i2c_auto_master_start( slave, I2CW, &address, 1 );

    if ( mode == I2CPolling ) {
        while ( ( status = i2c_auto_process() ) == I2CWorking );
    } else {
        while ( ( status = i2c_auto_complete() ) == I2CWorking );
    }

    if ( status == I2CError ) {
        return 0;
    }

    i2c_auto_master_start( slave, I2CR, data, size );

    if ( mode == I2CPolling ) {
        while ( ( status = i2c_auto_process() ) == I2CWorking );
    } else {
        while ( ( status = i2c_auto_complete() ) == I2CWorking );
    }

    if ( status == I2CError ) {
        return 0;
    }
    i2c_auto_master_stop();

    return 1;
}
