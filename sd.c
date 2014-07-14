#include "sd.h"

/* 内部状態保持用 */
static SDUnit unit;

char sd_init( SPISpeed max_speed, uint16_t block_size, SPIPin pullup )
{
    /* カード初期化 */
    int i;
    SDResp resp;
    uint32_t ret;
    uint8_t csd[16];
    uint8_t data_token;
    uint32_t c_size;
    uint8_t read_block_len;
    uint8_t c_size_mult;

    /* 設定保存 */
    unit.block_size = block_size;

    /* 初期化のためにSPIを停止させる */
    spi_release();

    /* 自分の都合がいいように設定する */
    spi_init( SPIMaster, SPIMode0, SPIOscDiv128, SPIMSB, pullup, 0 );

    /* CS=H,DI=Hで74クロック以上送る */
    spi_release_slave();

    for ( i = 0; i < 10; i++ ) {
        spi_write( 0xFF );
        while ( !spi_complete() );
    }

    /* アサート */
    spi_select_slave();

    /* CMD0送信 */
    sd_command( 0, 0 );

    /* レスポンスまち */
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* アイドルステートでなければ失敗 */
    if ( resp != SDRespIdleState ) {
        return 0;
    }

    /* CMD8送信 */
    sd_command( 8, 0x1AA );

    /* レスポンスまち */
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* 何かしらリジェクトされれば失敗なのでV1として処理 */
    if ( resp & SDRespIlligalCommand ) {
        /* V1 */
        unit.version = SDV1;
        unit.address = SDByte;

        /* 初期化開始 */
        while ( 1 ) {
            sd_command( 1, 0 );
            while ( ( resp = sd_response() ) == SDRespWorking );

            /* 終了してたら脱出 */
            if ( ( resp & SDRespIdleState ) == 0 ) {
                break;
            }
        }
    } else if ( resp == SDRespIdleState ) {
        /* V2*/
        unit.version = SDV2;

        /* 戻り値取得 */
        ret = sd_get_returned();

        if ( ( ret & 0xFFF ) != 0x1AA ) {
            /* 対応できないカードだった */
            return 0;
        }

        /* 初期化開始 */
        while ( 1 ) {
            /* ACMD41発行 */
            sd_command( 55, 0 );
            while ( ( resp = sd_response() ) == SDRespWorking );

            sd_command( 41, 0x40000000UL );
            while ( ( resp = sd_response() ) == SDRespWorking );

            /* 終了してたら脱出 */
            if ( ( resp & SDRespIdleState ) == 0 ) {
                break;
            }
        }
    } else {
        /* 何かしら失敗 */
        return 0;
    }

    /* OCR確認 */
    sd_command( 58, 0 );
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* 戻り値確認 */
    ret = sd_get_returned();

    if ( ret & 0x40000000UL ) {
        /* ブロックアドレスだった */
        unit.address = SDBlock;
    } else {
        unit.address = SDByte;
    }

    /* 速度を最高速度に再設定 */
    spi_set_speed( max_speed );

    /* ブロック長さ再設定 */
    sd_command( 16, block_size );
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* エラーなら失敗 */
    if ( resp != SDRespSuccess ) {
        return 0;
    }

    /* カードサイズ取得 ( CSD取得 ) */
    sd_command( 9, 0 );
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* データトークンを待つ */
    do {
        spi_write( 0xFF );
        while ( !spi_complete() );
        data_token = spi_read();
    } while ( data_token == 0xFF );

    /* エラートークンなら失敗 */
    if ( ( data_token & 0x80 ) == 0 ) {
        spi_release_slave();
        return 0;
    }

    /* 16バイト受信 */
    for ( i = 0; i < 16; i++ ) {
        spi_write( 0xFF );
        while ( !spi_complete() );
        csd[i] = spi_read();
    }

    /* CRC */
    spi_write( 0xFF );
    while ( !spi_complete() );
    spi_write( 0xFF );
    while ( !spi_complete() );

    /* CSDバージョン判定 */
    if ( csd[0] & 0x40 ) {
        /* バージョン2でカードサイズ作成 */
        unit.card_size = ( 1 + csd[9] + (uint64_t)csd[8] * 256 + ( (uint64_t)csd[7] & 0x3F ) * 256 * 256 ) * 512 * 1024;
    } else {
        /* バージョン1でカードサイズ作成 */
        c_size_mult    = ( csd[9] >> 3 ) & 0x07;
        read_block_len = ( csd[5] & 0x0F );
        c_size = ( ( (uint32_t)csd[6] & 0x03 ) << 10 ) | ( (uint32_t)csd[7] << 2 ) | ( csd[8] >> 6 );

        unit.card_size = ( 1 << read_block_len ) * ( 1 << ( c_size_mult + 2 ) ) * ( c_size + 1 );
    }
    // unit.card_size = ( 1 + csd[9] + (uint32_t)csd[8] * 256 + ( (uint32_t)csd[7] & 0x3F ) * 256 * 256 );

    /* チップセレクト解除 */
    spi_release_slave();

    /* 初期化完了 */
    return 1;
}

void sd_command( uint8_t cmd, uint32_t arg )
{
    /* SPIにコマンド送信 */
    int i;
    uint8_t cmd_buf[6];
    uint8_t *parg;

    /* 準備 */
    unit.cmd = cmd;
    parg = (uint8_t *)&arg;

    /* コマンド作成 */
    cmd_buf[0] = 0x40 | cmd;
    cmd_buf[1] = parg[3];
    cmd_buf[2] = parg[2];
    cmd_buf[3] = parg[1];
    cmd_buf[4] = parg[0];

    /* CRCが必要なら固定値を */
    if ( cmd == 8 ) {
        cmd_buf[5] = 0x87;
    } else if ( cmd == 0 ) {
        cmd_buf[5] = 0x95;
    } else {
        cmd_buf[5] = 0x01;
    }

    /* 一個無駄に送る ( 秋月の8MBSDはこれがないと動かない ) */
    spi_write( 0xFF );
    while ( !spi_complete() );
    spi_write( 0xFF );
    while ( !spi_complete() );

    /* SPIコマンド送信 */
    for ( i = 0; i < 6; i++ ) {
        spi_write( cmd_buf[i] );
        while ( !spi_complete() );
    }
}

SDResp sd_response( void )
{
    /* レスポンスを待って結果を返す */
    uint8_t recv;
    uint8_t *pret;
    SDResp resp;
    int i;

    pret = (uint8_t *)&unit.ret;

    /* NCR解除を100回まつ */
    for ( i = 0; i < 100; i++ ) {
        spi_write( 0xFF );
        while ( !spi_complete() );
        recv = spi_read();

        /* 解除されたか */
        if ( recv != 0xFF ) {
            break;
        }
    }

    /* タイムアウトしていれば失敗 */
    if ( i == 100 ) {
        return SDRespFailed;
    }

    /* 必要な場合は戻り値受信 */
    if ( unit.cmd == 58 || unit.cmd == 8 ) {
        for ( i = 0; i < 4; i++ ) {
            spi_write( 0xFF );
            while ( !spi_complete() );

            pret[3-i] = spi_read();
        }
    }

    /* レスポンスを作る */
    if ( recv != 0 ) {
        resp = recv;
    } else {
        resp = SDRespSuccess;
    }

    return resp;
}

uint32_t sd_get_returned( void )
{
    /* コマンドのリターンデーターを取得 */
    return unit.ret;
}

char sd_block_write( uint32_t address, uint8_t *data, size_t offset, size_t size )
{
    /* シングルブロックライト */
    uint16_t i;

    /* 初期化 */
    if ( !sd_start_step_block_write( address ) ) {
        return 0;
    }

    /* 読み込み */
    for ( i = 0; i < unit.block_size; i++ ) {
        /* 指定範囲外のデーターは0が書き込まれる */
        if ( i >= offset && ( i - offset ) < size ) {
            sd_step_block_write( data[i - offset] );
        } else {
            sd_step_block_write( 0x00 );
        }
    }

    /* 終了 */
    if ( !sd_stop_step_block_write() ) {
        return 0;
    }

    return 1;
}

SDAddress sd_get_address_mode( void )
{
    /* アドレスモードを返す */
    return unit.address;
}

char sd_block_read( uint32_t address, uint8_t *data, size_t offset, size_t size )
{
    /* シングルブロックリード */
    uint16_t i;

    /* 初期化 */
    if ( !sd_start_step_block_read( address ) ) {
        return 0;
    }

    /* 読み込み */
    for ( i = 0; i < unit.block_size; i++ ) {
        if ( i >= offset && ( i - offset ) < size ) {
            data[i - offset] = sd_step_block_read();
        } else {
            sd_step_block_read();
        }
    }

    /* 終了 */
    if ( !sd_stop_step_block_read() ) {
        return 0;
    }

    return 1;
}

SDVersion sd_get_version()
{
    /* バージョンを返す */
    return unit.version;
}

char sd_start_step_block_write( uint32_t address )
{
    /* ステップ動作でブロックライト開始 */
    SDResp resp;

    /* アサート */
    spi_select_slave();

    /* CMD24 */
    sd_command( 24, address );

    /* レスポンスまち */
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* 変なレスポンスなら失敗 */
    if ( resp != SDRespSuccess ) {
        spi_release_slave();

        return 0;
    }

    /* 1バイト開ける */
    spi_write( 0xFF );
    while ( !spi_complete() );

    /* データトークン */
    spi_write( 0xFE );
    while ( !spi_complete() );

    return 1;
}

void sd_step_block_write( uint8_t data )
{
    /* 1バイトだけ書き込み */
    spi_write( data );
    while ( !spi_complete() );
}

char sd_stop_step_block_write()
{
    /* ステップ動作ブロックライト完了 */
    uint8_t data_resp;
    uint8_t busy;

    /* CRC */
    spi_write( 0x00 );
    while ( !spi_complete() );
    spi_write( 0x00 );
    while ( !spi_complete() );

    /* データレスポンス */
    spi_write( 0xFF );
    while ( !spi_complete() );
    data_resp = spi_read();

    /* ビジー解除まち */
    do {
        spi_write( 0xFF );
        while ( !spi_complete() );
        busy = spi_read();
    } while ( busy == 0x00 );

    /* 成功か */
    if ( ( data_resp & 0x05 ) == 0x05 ) {
        spi_release_slave();
        return 1;
    } else {
        spi_release_slave();
        return 0;
    }
}

char sd_start_step_block_read( uint32_t address )
{
    /* ステップ動作のシングルブロックリード */
    SDResp resp;
    uint8_t data_token;

    /* アサート */
    spi_select_slave();

    /* CMD17 */
    sd_command( 17, address );

    /* レスポンスまち */
    while ( ( resp = sd_response() ) == SDRespWorking );

    /* 変なレスポンスなら失敗 */
    if ( resp != SDRespSuccess ) {
        spi_release_slave();
        return 0;
    }

    /* データトークンを待つ */
    do {
        spi_write( 0xFF );
        while ( !spi_complete() );
        data_token = spi_read();
    } while ( data_token == 0xFF );

    /* エラートークンなら失敗 */
    if ( ( data_token & 0x80 ) == 0 ) {
        spi_release_slave();
        return 0;
    }

    return 1;
}

uint8_t sd_step_block_read( void )
{
    /* ステップ動作のシングルブロックリードで１バイト受信 */
    spi_write( 0xFF );
    while ( !spi_complete() );

    return spi_read();
}

char sd_stop_step_block_read()
{
    /* ステップ動作のシングルブロックリード完了 */

    /* CRC */
    spi_write( 0xFF );
    while ( !spi_complete() );
    spi_write( 0xFF );
    while ( !spi_complete() );

    /* ( 一応 )ビジー解除を待つ */
    do {
        spi_write( 0xFF );
        while ( !spi_complete() );
    } while ( spi_read() != 0xFF );

    spi_release_slave();
    return 1;
}

uint64_t sd_get_size( void )
{
    /* カードサイズをバイトで返す */
    return unit.card_size;
}

uint16_t sd_get_block_size( void )
{
    /* ブロックサイズをバイトで返す */
    return unit.block_size;
}
