#include "ide.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "usart.h"
#include "debug.h"
#include "sd.h"
#include "adxl345.h"
#include "l3gd20.h"
#include "lps331ap.h"
#include "hmc5883l.h"
#include "device_id.h"

int main( void )
{
    // ATmega88いろいろテスト
    int i;
    int j;
    uint8_t data[512];
    char msg[32];
    ADXLUnit adxl;
    L3GD20Unit gyro;
    LPS331APUnit pres;
    HMC5883LUnit mag;

    // 初期化
    cli();

    usart_init( 19200UL, UsartRX | UsartTX, 0 );

    // sei();

    // なんか数字をだす
    /*
    for ( i = 0; i < 65535; i++ ) {
        debug_int( i );
        debug_puts( "端末日本語\r\nUTF8" );

        _delay_ms( 500 );
    }
    */

    // SD初期化
    if ( sd_init( SPIOscDiv2, 512, SPIMISO ) ) {
        debug_puts( "成功" );
    } else {
        debug_puts( "失敗" );
    }

    // 読みだす
    if ( sd_get_address_mode() != SDByte ) {
        debug_puts( "ブロックアドレス" );
    } else {
        debug_puts( "バイトアドレス" );
    }

    if ( sd_get_version() != SDV2 ) {
        debug_puts( "SDV1" );
    } else {
        debug_puts( "SDV2" );
    }

    debug_puts( "先頭セクター読み出し" );
    for ( i = 0; i < 2; i++ ) {
        if ( !sd_block_read( 0, data ) ) {
            debug_puts( "エラー" );
        } else {
            debug_puts( "OK" );
        }
    }

    for ( i = 0; i < 512; i++ ) {
        sprintf( msg, "%d\r\n", data[i] );

        for ( j = 0; j < strlen( msg ); j++ ) {
            while ( !usart_can_write() ) {
                ;
            }
            usart_write( msg[j] );
        }
    }

    /* i2c初期化 */
    i2c_init_master( 32, I2CPrescale1, 0, 0 );

    /* ADXL初期化 */
    if ( adxl_init( &adxl, 0x53, ADXLRange16, ADXLResolutionFull, ADXL400Hz ) ) {
        debug_puts( "ADXLOk" );
    } else {
        debug_puts( "ADXLFailed" );
    }

    /* 測定開始 */
    adxl_start( &adxl );

    /* ジャイロ初期化 */
    if ( l3gd20_init( &gyro, 0x6A, L3GD20_760Hz, L3G2D20_500dps ) ) {
        debug_puts( "Gyro OK" );
    }

    /* 測定開始 */
    l3gd20_start( &gyro );

    /* 気圧初期化 */
    if ( lps331ap_init( &pres, 0x5C, LPS331AP25_25Hz, LPS331APPresAvg1, LPS331APTempAvg1 ) ) {
        debug_puts( "Pressure OK" );
    }

    /* 測定開始 */
    lps331ap_start( &pres );

    /* 磁気センサー初期化 */
    if ( hmc5883l_init( &mag, 0x1E, HMC5883L75Hz, HMC5883LAvg1, HMC5883L2_5Ga ) ) {
        debug_puts( "Mag OK" );
    }

    /* 測定開始 */
    hmc5883l_start( &mag );


    debug_puts( "start" );

    /*
    while ( 1 ) {
        if ( hmc5883l_data_ready( &mag ) && hmc5883l_read( &mag ) ) {
            sprintf( msg, "xyz : %8d %8d %8d", mag.x, mag.y, mag.z );
            debug_puts( msg );
        }
    }
    */

    ///*
    while ( 1 ) {
        if ( lps331ap_data_ready( &pres ) && lps331ap_temp_data_ready( &pres ) && lps331ap_read( &pres ) && lps331ap_read_temp( &pres ) ) {
            sprintf( msg, "pressure temp : %2x %2x %2x %8d", *( (uint8_t *)&pres.pressure + 0 ), *( (uint8_t *)&pres.pressure + 1 ), *( (uint8_t *)&pres.pressure + 2 ), pres.temp );
            debug_puts( msg );
        }
    }
    //*/

    /* 準備完了次第読んで出力 */
    /*
    while ( 1 ) {
        if ( l3gd20_data_ready( &gyro ) && l3gd20_read( &gyro ) && l3gd20_read_temp( &gyro ) ) {
            sprintf( msg, "xyzt : %8d %8d %8d %8d", gyro.x, gyro.y, gyro.z, gyro.temp );
            debug_puts( msg );
        }
    }
    */

    /* 準備完了次第読んで出力 */
    /*
    while ( 1 ) {
        if ( adxl_data_ready( &adxl ) && adxl_read( &adxl ) ) {
            sprintf( msg, "xyz : %8d %8d %8d", adxl.x, adxl.y, adxl.z );
            debug_puts( msg );
        }
    }

    // エコー
    while ( 1 ) {
        if ( usart_can_read() ) {
            while ( !usart_can_write() ) {
                ;
            }

            usart_write( usart_read() );
        }
    }
    */

    return 0;
}
