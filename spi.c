#include "spi.h"

static SPIUnit unit;

void spi_init( SPISide side, SPIMode mode, SPISpeed speed, SPIOrder order, SPIPin pullup, char int_enable )
{
    /* SPI初期化 */
    uint8_t spcr_buf = 0;

    /* SPI停止 */
    spi_release();
    spi_release_slave();

    /* 情報保存・初期化 */
    unit.side  = side;
    unit.status = SPINone;

    /* ピンの入出力とモード指定 */
    if ( side == SPIMaster ) {
        /* マスター */
        spcr_buf |= _BV( MSTR );
        DDRB |= _BV( PB3 ) | _BV( PB5 ) | _BV( PB2 );
    } else {
        /* スレーブ */
        spcr_buf &= ~_BV( MSTR );
        DDRB |= _BV( PB4 );
    }

    /* モード指定 */
    spcr_buf |= mode;

    /* データーオーダー指定 */
    spcr_buf |= order;

    /* SPCR設定 */
    SPCR = spcr_buf;

    /* スピード指定 */
    spi_set_speed( speed );

    /* SPI有効化 */
    spi_enable( 1 );

    /* 必要なら割り込み有効化 */
    if ( int_enable ) {
        spi_enable_int( 1 );
    } else {
        spi_enable_int( 0 );
    }

    /* 必要に応じてプルアップ設定 */
    if ( side == SPIMaster ) {
        PORTB = ( PORTB & ~SPIMISO ) | ( pullup & SPIMISO );
    } else {
        PORTB = ( PORTB & ~( SPIMOSI | SPISCK | SPISS ) ) | ( pullup & ( SPIMOSI | SPISCK | SPISS ) );
    }
}

void spi_enable( char enable )
{
    /* SPI有効化 */
    if ( enable ) {
        SPCR |= _BV( SPE );
    } else {
        SPCR &= ~_BV( SPE );
    }
}

void spi_release( void )
{
    /* SPI解放 */
    spi_enable_int( 0 );
    spi_enable( 0 );
}

void spi_enable_int( char enable )
{
    /* SPI割り込み有効化 */
    if ( enable ) {
        SPCR |= _BV( SPIE );
    } else {
        SPCR &= ~_BV( SPIE );
    }
}

char spi_write_collision( void )
{
    /* 書き込み衝突発生 */
    if ( SPSR & _BV( WCOL ) ) {
        return 1;
    } else {
        return 0;
    }
}

char spi_complete( void )
{
    /* 通信完了フラグ確認 */
    if ( SPSR & _BV( SPIF ) ) {
        return 1;
    } else {
        return 0;
    }
}

void spi_write( uint8_t data )
{
    /* トランスミッターにデータ書き込み */
    SPDR = data;
}

uint8_t spi_read( void )
{
    /* トランスミッターからデータ読み出し */
    return SPDR;
}

void spi_select_slave( void )
{
    /* スレーブを選択 */
    PORTB &= ~_BV( PB2 );
}

void spi_release_slave( void )
{
    /* スレーブを解放 */
    PORTB |= _BV( PB2 );
}

#if 0
void spi_auto_master_start( uint8_t *write_data, uint8_t *read_data, size_t size, uint8_t write_const )
{
    /* 自動処理実行開始 */
    uint8_t data;

    unit.read_data   = read_data;
    unit.write_data  = write_data;
    unit.size        = size;
    unit.write_const = write_const;
    unit.pos         = 0;

    /* 設定が変なら何もしない */
    if ( size < 1 ) {
        unit.status = SPIError;
        return;
    }

    /* ポインター確認 */
    if ( read_data == NULL ) {
        unit.rd_NULL = 1;
    } else {
        unit.rd_NULL = 0;
    }

    if ( write_data == NULL ) {
        unit.wd_NULL = 1;
    } else {
        unit.wd_NULL = 0;
    }


    /* 実行 */
    unit.status = SPIWorking;

    if ( unit.wd_NULL ) {
        data = unit.write_const;
    } else {
        data = unit.write_data[unit.pos];
    }

    spi_write( data );
}

SPIStatus spi_auto_proccess( void )
{
    /* 自動処理 */
    if ( unit.side == SPIMaster ) {
        /* マスターの自動処理 */

        /* 通信完了したか確認 */
        if ( spi_complete() ) {
            /* 完了したので次の処理 */

            /* 読み込み */
            if ( !unit.rd_NULL ) {
                unit.read_data[unit.pos] = spi_read();
            } else {
                spi_read();
            }

            /* ポインタを進めて終了確認 */
            unit.pos++;

            if ( unit.pos == unit.size ) {
                /* 終了した */
                unit.status = SPISuccess;
                return SPISuccess;
            } else {
                /* 終わってないので次へ */
                if ( unit.wd_NULL ) {
                    spi_write( unit.write_const );
                } else {
                    spi_write( unit.write_data[unit.pos] );
                }

                return SPIWorking;
            }
        } else {
            /* 完了してないのでそのまま */
            return SPIWorking;
        }
    } else {
        /* スレーブモードの自動処理 ( とりあえずエラー ) */
        return SPIError;
    }
}

SPIStatus spi_auto_complete( void )
{
    /* 自動処理が完了したか */
    return unit.status;
}

SPISpeed spi_bps_to_speed( uint32_t max_bps )
{
    /* 引数のbpsを最大として最も近いものを探す */
    uint32_t div;
    uint32_t bps;

    /* 速い方から試していく */
    for ( div = 2; div < 256; div *= 2 ) {
        /* bps計算 */
        bps = F_CPU / div;

        if ( bps < max_bps ) {
            /* 最大速度より遅かったので設定 */
            switch ( div ) {
            case 2:
                return SPIOscDiv2;
            case 4:
                return SPIOscDiv4;
            case 8:
                return SPIOscDiv8;
            case 16:
                return SPIOscDiv16;
            case 32:
                return SPIOscDiv32;
            case 64:
                return SPIOscDiv64;
            case 128:
                return SPIOscDiv128;
            default:
                return SPIOscDiv128;
            }
        }
    }

    /* 何かしら失敗したら低速 */
    return SPIOscDiv128;
}
#endif

void spi_set_speed( SPISpeed speed )
{
    /* SPIの速度を設定 */
    SPCR |= ( 0x03 & speed );
    SPSR |= ( 0x40 & speed ) >> 2;
}
