#include "ide.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "usart.h"
#include "sd.h"
#include "st7032i.h"
#include "lps331ap.h"
#include "ak8975.h"
#include "mpu9150.h"
#include "micomfs.h"
#include "fifo.h"

#include "debug.h"

#define SW_STOP   _BV( PB4 )
#define SW_LIGHT  _BV( PB5 )
#define SW_SELECT _BV( PB6 )
#define SW_NEXT   _BV( PB7 )

/* 100usごとにカウントされるタイマー */
static volatile uint32_t system_clock;

static FIFO gps_fifo;
static char gps_buf[100];
static char line_str[17];
/*
static char gga_buf[100];
static char rmc_buf[100];
*/
static char dbg[128];

ISR( USART_RX_vect )
{
    /* USART受信割り込み */
    uint8_t data;

    /* 読める限り */
    while ( usart_can_read() ) {
        /* 読む */
        data = usart_read();

        /* FIFOにPSUH */
        fifo_write( &gps_fifo, &data );
    }
}

ISR( TIMER0_COMPA_vect )
{
    /* タイマー0コンペアマッチA割り込みベクター */
    /* システムクロックを100usごとに1更新 */
    system_clock++;
}

char get_battery_level( void )
{
    /* バッテリー残量取得 */
    uint16_t data;

    /* 最新の結果がない */
    if ( !( ADCSRA & _BV( ADIF ) ) ) {
        /* まだ計測開始していない場合計測 */
        if ( !( ADCSRA & _BV( ADSC ) ) ) {
            ADCSRA |= _BV( ADSC );
        }

        /* Failed */
        return -1;
    }

    /* get data */
    data = ADCH;

    /* Clear IF */
    ADCSRA |= _BV( ADIF );

    /* Start conversion */
    ADCSRA |= _BV( ADSC );

    if ( data < 65 ) {
        /* V < 2.5 */
        return 0;
    } else if ( data < 79 ) {
        /* V < 3.0 */
        return 1;
    } else if ( data < 86 ) {
        /* V < 3.3 */
        return 2;
    } else {
        return 3;
    }
}

int main( void )
{
    char use_sd;
    char use_3d;
    char use_press;
    uint8_t data;
    LPS331APUnit pres;
    AK8975Unit mag;
    MPU9150Unit mpu9150;
    MicomFS fs;
    MicomFSFile fp;
    int max_level = 0;

    /* 割り込み停止 */
    cli();

    /* ポート向き初期化 */
    DDRD = 0x0F;    /* PD4-7 : 入力 */
    DDRC = 0xFE;    /* PC0   : ADC */
    DDRB = 0xFF;    /* PB0   : 液晶ライト */

    /* 入力プルアップ指定 */
    PORTD = 0xF0;

    /* ADC setting */
    ADMUX  = 0x20;  /* ADC0, Left, AVcc */
    ADCSRA = 0x86;  /* Enable, 1/64, Disable int, Disable Auto Trigger */

    /* I2Cバス初期化 */
    i2c_init_master( 32, I2CPrescale1, 0, 0 );

    /* デバイス初期化 */
    use_sd     = sd_init( SPIOscDiv2, 512, SPIMISO );
    use_press  = lps331ap_init( &pres, 0x5D, LPS331AP25_25Hz, LPS331APPresAvg512, LPS331APTempAvg1 );
    use_3d     = mpu9150_init( &mpu9150, 0x68, 0, MPU9150LPFCFG0, MPU9150AccFSR4g, MPU9150AccHPFReset, MPU9150GyroFSR500DPS );
    use_3d    *= ak8975_init( &mag, 0x0C );

    st7032i_init();

    /* GPS用FIFO初期化 */
    fifo_init( &gps_fifo, gps_buf, sizeof( char ), 0, sizeof( gps_buf ) / sizeof( char ) );

    /* 初期化完了 */

    /* ちょっと待つ */
    _delay_ms( 500 );

    /* 光る */
    PORTB |= _BV( PB0 );

    /* DEBUG */
    st7032i_set_icon( ST7032IIconAddrBattery, ST7032IIconBattFrame | ST7032IIconBattL1 );
    st7032i_set_icon( ST7032IIconAddrAntena,  ST7032IIconAntena );

    /* Welcome message */
    st7032i_puts( 0, 0, "Sensor recorder" );
    use_sd    ? st7032i_puts( 1, 0,  "SD:O" ) : st7032i_puts( 1, 0,  "SD:X" );
    use_press ? st7032i_puts( 1, 5,  "PR:O" ) : st7032i_puts( 1, 5,  "PR:X" );
    use_3d    ? st7032i_puts( 1, 10, "3D:O" ) : st7032i_puts( 1, 10, "3D:X" );

    _delay_ms( 1000 );

    /* USART起動 */
    usart_init( 4800UL, UsartRX | UsartTX, UsartIntRX );

    /* 割り込み開始 */
    sei();

    /* 全センサー測定開始（ 連続なもの ） */
    lps331ap_start( &pres );

    /* DEBUG */
    /* fs初期化 */
    if ( use_sd ) {
        int i;
        int j;

        /* フォーマット */
        micomfs_format( &fs, 512, sd_get_size() / sd_get_block_size(), 10, 0 );

        /* ファイル作る */
        micomfs_fcreate( &fs, &fp, "hellllll.txt", 10 );

        /* 書きまくる */
        for ( i = 0; i < 5; i++ ) {
            micomfs_start_fwrite( &fp, i );

            for ( j = 0; j < fs.sector_size; j++ ) {
                data = 0xFA;

                micomfs_fwrite( &fp, &data, 1 );
            }

            micomfs_stop_fwrite( &fp, 0 );
        }

        /* ファイル閉じる */
        micomfs_fclose( &fp );

        /* ファイル作る */
        micomfs_fcreate( &fs, &fp, "foooo.txt", MICOMFS_MAX_FILE_SECOTR_COUNT );

        /* 書きまくる */
        for( i = 0; /*micomfs_start_fwrite( &fp, i )*/i < 10; i++ ) {

            for ( j = 0; j < fs.sector_size; j++ ) {
                data = 0x3C;

                micomfs_fwrite( &fp, &data, 1 );
            }

            micomfs_stop_fwrite( &fp, 0 );
        }

        /* ファイル閉じる */
        micomfs_fclose( &fp );

        st7032i_puts( 1, 0, "Fyuu" );

        /* 第一セクター15バイトを全部USARTにぶちまける */
        for ( i = 0; i < 10; i++ ) {
            sd_start_step_block_read( i );

            for ( j = 0; j < fs.sector_size; j++ ) {
                data = sd_step_block_read();

                while ( !usart_can_write() );
                usart_write( data );
            }

            sd_stop_step_block_read();
        }

        /*
        sd_block_read( 0, buf, 0, 15 );
        for ( i = 0; i < sizeof( buf ); i++ ) {
            while ( !usart_can_write() );
            usart_write( buf[i] );
        }
        */

        while ( 1 );
    }

    /* メインルーチン */
    while ( 1 ) {
        /* ボタン */
        if ( !( PIND & _BV( PD4 ) ) ) {
            PORTB |= _BV( PB0 );
        } else {
            PORTB &= ~_BV( PB0 );
        }

        /* Display battery level */
        switch ( get_battery_level() ) {
        case 0:
            st7032i_set_icon( ST7032IIconAddrBattery, ST7032IIconBattFrame );
            break;
        case 1:
            st7032i_set_icon( ST7032IIconAddrBattery, ST7032IIconBattFrame | ST7032IIconBattL1 );
            break;
        case 2:
            st7032i_set_icon( ST7032IIconAddrBattery, ST7032IIconBattFrame | ST7032IIconBattL1 | ST7032IIconBattL2 );
            break;
        case 3:
            st7032i_set_icon( ST7032IIconAddrBattery, ST7032IIconBattFrame | ST7032IIconBattL1 | ST7032IIconBattL2 | ST7032IIconBattL3 );
            break;
        default:
            break;
        }

        if ( fifo_level( &gps_fifo ) > max_level ) {
            max_level = fifo_level( &gps_fifo );
        }

        sprintf( line_str, "%d", max_level );
        st7032i_puts( 1, 0, line_str );

        cli();
        while ( fifo_read( &gps_fifo, &data ) );
        sei();

        /*
        while ( !usart_can_write() );
        if ( fifo_read( &gps_fifo, &data ) ) {
            usart_write( data );
        }
        */

        /* 全データ取得デバッグ */

        /* 単独測定開始 */
        ak8975_start( &mag );

        /* 全センサーDRDY確認 */
        while ( !( ak8975_data_ready( &mag ) && mpu9150_data_ready( &mpu9150 ) && lps331ap_data_ready( &pres ) ) );

        /* 全センサー受信 */
        ak8975_read( &mag );
        mpu9150_read( &mpu9150 );
        lps331ap_read( &pres );

        /* 補正・計算 */
        ak8975_calc_adjusted_h( &mag );

        /* 全てusartへ送信 Acc Gyro Mag Temp */
        sprintf( dbg, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
                 mpu9150.acc_x, mpu9150.acc_y, mpu9150.acc_z,
                 mpu9150.gyro_x, mpu9150.gyro_y, mpu9150.gyro_z,
                 mag.adj_x, mag.adj_y, mag.adj_z,
                 mag.x, mag.y, mag.z,
                 (int)( pres.pressure / 4096 ),
                 (int)mpu9150_get_temp_in_c( &mpu9150 ) );
        debug_puts( dbg );

        /* 気圧表示 */
        sprintf( line_str, "%d          ", (int)( pres.pressure / 4096 ) );
        st7032i_puts( 0, 0, line_str );
    }


    /* 終了 */
    return 0;
}
