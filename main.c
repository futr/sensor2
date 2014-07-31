#include "ide.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include "usart.h"
#include "sd.h"
#include "st7032i.h"
#include "lps331ap.h"
#include "ak8975.h"
#include "mpu9150.h"
#include "micomfs.h"
#include "fifo.h"
#include "device_id.h"

#include "debug.h"

#define LOG_SIGNATURE 0x3E      /* ログファイルのデーターごとの先頭バイトシグネチャ */
#define GPS_WRITE_UNIT 32       /* GPSデーターの書き込み単位 */

#define SW_STOP   _BV( PB7 )
#define SW_START  _BV( PB6 )
#define SW_TOGGLE _BV( PB4 )
#define SW_NEXT   _BV( PB5 )

/* 100usごとにカウントされるタイマー */
static volatile uint32_t system_clock;
static uint8_t input_counter;
static volatile uint8_t input;
static volatile uint8_t before_input;
static volatile uint8_t pushed_input;
static FIFO gps_fifo;
static char gps_buf[70];
static char line_str[2][17];
static char dbg[20];

typedef enum {
    OtherSentence,
    GPGGA,
    GPRMC,
} NMEASentence;

typedef enum {
    DispNone,
    DispGPS1,
    DispGPS2,
    DispPressTemp,
    DispAcc,
    DispMag,
    DispGyro,
    DispStatus,
    DispFormat,
    DispStartWrite,
    DispStopWrite,
} Display;

typedef enum {
    DEV_PRESS = 0x01,
    DEV_GYRO  = 0x02,
    DEV_MAG   = 0x04,
    DEV_ACC   = 0x08,
    DEV_TEMP  = 0x10,
    DEV_GPS   = 0x20,
    DEV_SD    = 0x40,
} Devices;

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

    /* 10msごとに入力読み込み */
    if ( 100 <= input_counter ) {
        input = ~PIND;

        input_counter = 0;
    } else {
        input_counter++;
    }
}

char get_battery_level( void )
{
    /* バッテリー残量取得 */
    uint8_t data;

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

void display_battery_level( void )
{
    /* バッテリレベル表示 */
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
}

void clear_line_buf( void )
{
    /* 画面バッファクリア */
    int i;
    int j;

    for ( i = 0; i < 2; i++ ) {
        for ( j = 0; j < 16; j++ ) {
            line_str[i][j] = ' ';
        }

        line_str[i][16] = '\0';
    }
}

int main( void )
{
    Devices enabled_dev;
    Devices write_dev;
    Devices write_dev_buf;
    Devices updated_dev;
    Devices sensor_pos;
    Display display;
    char display_changed;

    char elem_buf[16];
    uint8_t elem_pos;
    NMEASentence sentence;
    uint8_t sentence_pos;
    uint8_t data;
    uint8_t gps_ready;
    uint8_t sec_timer;
    uint32_t before_system_clock;
    uint32_t now_system_clock;

    LPS331APUnit pres;
    AK8975Unit mag;
    MPU9150Unit mpu9150;
    MicomFS fs;
    MicomFSFile fp;
    int max_level = 0;
    int i;
    int j;
    int ret;

    /* 割り込み停止 */
    cli();

    /* ポート向き初期化 */
    DDRD = 0x0F;    /* PD4-7 : 入力 */
    DDRC = 0xFE;    /* PC0   : ADC ( = 入力 ADCに出力するとその出力をADCしてしまう ) */
    DDRB = 0xFF;    /* PB0   : 液晶ライト */

    /* 入力プルアップ指定 */
    PORTD = 0xF0;

    /* ADC setting */
    ADMUX  = 0x20;  /* ADC0, Left, AVcc */
    ADCSRA = 0x86;  /* Enable, 1/64, Disable int, Disable Auto Trigger */

    /* タイマー0を初期化 */
    TCCR0B = 0x02;          /* プリスケール8，CTCモード */
    TCCR0A = 0x02;          /* CTCモード */
    TIMSK0 = 0x02;          /* コンペアマッチA割り込み有効 */
    OCR0A  = 100;           /* 100usごとに割りこみ発生 */

    /* I2Cバス初期化 */
    i2c_init_master( 32, I2CPrescale1, 0, 0 );

    /* デバイス初期化 */
    enabled_dev = DEV_GPS;

    if ( sd_init( SPIOscDiv2, 512, 0 ) ) {
        enabled_dev |= DEV_SD;
    }

    if ( lps331ap_init( &pres, 0x5D, LPS331AP25_25Hz, LPS331APPresAvg512, LPS331APTempAvg1 ) ) {
        enabled_dev |= DEV_PRESS;
    }

    if ( mpu9150_init( &mpu9150, 0x68, 0, MPU9150LPFCFG0, MPU9150AccFSR4g, MPU9150AccHPFReset, MPU9150GyroFSR500DPS ) ) {
        enabled_dev |= DEV_GYRO | DEV_ACC;
    }

    if ( ak8975_init( &mag, 0x0C ) ) {
        enabled_dev |= DEV_MAG;
    }

    st7032i_init();

    /* GPS用FIFO初期化 */
    fifo_init( &gps_fifo, gps_buf, sizeof( char ), 0, sizeof( gps_buf ) / sizeof( char ) );

    /* 初期化完了 */

    /* ちょっと待つ */
    _delay_ms( 500 );

    /* 光る */
    PORTB |= _BV( PB0 );

    /* バッテリレベル表示 */
    display_battery_level();

    /* Welcome message */
    st7032i_puts( 0, 0, "Sensor recorder" );
    ( enabled_dev & DEV_SD    ) ? st7032i_puts( 1, 0,  "SD:O" ) : st7032i_puts( 1, 0,  "SD:X" );
    ( enabled_dev & DEV_PRESS ) ? st7032i_puts( 1, 5,  "PR:O" ) : st7032i_puts( 1, 5,  "PR:X" );
    ( enabled_dev & DEV_ACC   ) ? st7032i_puts( 1, 10, "3D:O" ) : st7032i_puts( 1, 10, "3D:X" );

    _delay_ms( 1000 );

    /* USART起動 */
    usart_init( 4800, UsartRX | UsartTX, UsartIntRX );

    /* 各種変数初期化 */
    display_changed = 1;
    write_dev     = 0;
    write_dev_buf = 0;
    sentence      = OtherSentence;
    sentence_pos  = 0;
    elem_pos      = 0;
    display       = DispGPS1;
    gps_ready     = 0;
    sensor_pos    = 0;
    before_system_clock = 0;
    now_system_clock    = 0;
    system_clock        = 0;

    /* ピン入力初期化 */
    input = ~PIND;
    before_input = input;

    /* 割り込み開始 */
    sei();

    /* 測定開始 */
    lps331ap_start( &pres );
    ak8975_start( &mag );

    /* メインループ */
    while ( 1 ) {
        updated_dev = 0;

        /* Get system clock */
        cli();
        now_system_clock = system_clock;
        sei();

        /* センサー情報取得 */
        if ( ( enabled_dev & DEV_PRESS ) && lps331ap_data_ready( &pres ) ) {
            /* 気圧 */
            lps331ap_read( &pres );

            updated_dev |= DEV_PRESS;
        }

        if ( ( enabled_dev & DEV_MAG ) && ak8975_data_ready( &mag ) ) {
            /* 地磁気 */
            ak8975_read( &mag );

            /* 地磁気補正 */
            ak8975_calc_adjusted_h( &mag );

            /* 測定開始 */
            ak8975_start( &mag );

            updated_dev |= DEV_MAG;
        }

        if ( ( enabled_dev & ( DEV_ACC | DEV_GYRO ) ) && mpu9150_data_ready( &mpu9150 ) ) {
            /* 加速度・温度・ジャイロ */
            mpu9150_read( &mpu9150 );

            updated_dev |= ( DEV_ACC | DEV_GYRO | DEV_TEMP );
        }

        /* 必要なら各センサーデータ処理と書き込み */
        if ( ( write_dev & DEV_PRESS ) && ( updated_dev & DEV_PRESS ) ) {
            /* 気圧書き込み */
            data = LOG_SIGNATURE;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
            data = ID_LPS331AP;
            micomfs_seq_fwrite( &fp, &data, 1 );
            data = sizeof( pres.pressure );
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &pres.pressure, sizeof( pres.pressure ) );
        }

        if ( ( write_dev & DEV_ACC ) && ( updated_dev & DEV_ACC ) ) {
            /* 加速度書き込み */
            data = LOG_SIGNATURE;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
            data = ID_MPU9150_ACC;
            micomfs_seq_fwrite( &fp, &data, 1 );
            data = sizeof( mpu9150.acc_x ) * 3;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &mpu9150.acc_x, sizeof( mpu9150.acc_x ) );
            micomfs_seq_fwrite( &fp, &mpu9150.acc_y, sizeof( mpu9150.acc_y ) );
            micomfs_seq_fwrite( &fp, &mpu9150.acc_z, sizeof( mpu9150.acc_z ) );
        }

        if ( ( write_dev & DEV_GYRO ) && ( updated_dev & DEV_GYRO ) ) {
            /* ジャイロ書き込み */
            data = LOG_SIGNATURE;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
            data = ID_MPU9150_GYRO;
            micomfs_seq_fwrite( &fp, &data, 1 );
            data = sizeof( mpu9150.gyro_x ) * 3;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &mpu9150.gyro_x, sizeof( mpu9150.gyro_x ) );
            micomfs_seq_fwrite( &fp, &mpu9150.gyro_y, sizeof( mpu9150.gyro_y ) );
            micomfs_seq_fwrite( &fp, &mpu9150.gyro_z, sizeof( mpu9150.gyro_z ) );
        }

        if ( ( write_dev & DEV_MAG ) && ( updated_dev & DEV_MAG ) ) {
            /* 地磁気書き込み */
            data = LOG_SIGNATURE;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
            data = ID_AK8975;
            micomfs_seq_fwrite( &fp, &data, 1 );
            data = sizeof( mag.adj_x ) * 3;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &mag.adj_x, sizeof( mag.adj_x ) );
            micomfs_seq_fwrite( &fp, &mag.adj_y, sizeof( mag.adj_y ) );
            micomfs_seq_fwrite( &fp, &mag.adj_z, sizeof( mag.adj_z ) );
        }

        if ( ( write_dev & DEV_TEMP ) && ( updated_dev & DEV_TEMP ) ) {
            /* 温度書き込み */
            data = LOG_SIGNATURE;
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
            data = ID_MPU9150_TEMP;
            micomfs_seq_fwrite( &fp, &data, 1 );
            data = sizeof( mpu9150.temp );
            micomfs_seq_fwrite( &fp, &data, 1 );
            micomfs_seq_fwrite( &fp, &mpu9150.temp, sizeof( mpu9150.temp ) );
        }

        /* DEBUG */
        /*
        if ( fifo_level( &gps_fifo ) > max_level ) {
            max_level = fifo_level( &gps_fifo );
        }

        sprintf( dbg, "%d\n", max_level );
        for ( i = 0; i < strlen( dbg ); i++ ) {
            while ( !usart_can_write() );
            usart_write( dbg[i] );
        }
        */

        /* GPSデーターが書きこみ単位以上たまってれば処理 ( なぜか >= だとミスる　FIFOにバグあり？ ) */
        if ( ( enabled_dev & DEV_GPS ) && ( fifo_level( &gps_fifo ) >= GPS_WRITE_UNIT ) ) {
            /* 書きこみ単位分処理 */

            /* 必要ならヘッダ書き込み */
            if ( write_dev & DEV_GPS ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_GPS;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = GPS_WRITE_UNIT;
                micomfs_seq_fwrite( &fp, &data, 1 );
            }

            for ( i = 0; i < GPS_WRITE_UNIT; i++ ) {
                /* 1バイトよみ */
                cli();
                ret = fifo_read( &gps_fifo, &data );
                sei();

                /* DEBUG */
                if ( !ret ) {
                    PINB |= _BV( PB0 );

                    /* DEBUG */
                    sprintf( dbg, "%d %p %p\n", (int)fifo_level( &gps_fifo ), gps_fifo.r, gps_fifo.w );
                    for ( j = 0; j < strlen( dbg ); j++ ) {
                        while ( !usart_can_write() );
                        usart_write( dbg[j] );
                    }
                }
                /*
                while ( !usart_can_write() );
                usart_write( data );
                */

                /* 必要なら書き */
                if ( write_dev & DEV_GPS ) {
                    micomfs_seq_fwrite( &fp, &data, 1 );
                }

                /* 必要ならエレメント分解 */
                if ( display == DispGPS1 || display == DispGPS2 ) {
                    elem_buf[elem_pos] = data;
                    elem_pos++;

                    /* なぜかうまってしまった場合処理状態クリア */
                    if ( elem_pos >= sizeof( elem_buf ) - 1 ) {
                        elem_pos     = 0;
                        sentence     = OtherSentence;
                        sentence_pos = 0;

                        continue;
                    }

                    /* カンマか改行を見つけたらエレメント処理 */
                    if ( data == ',' || data == 0x0A ) {
                        /* DEBUG */
                        /*
                        for ( j = 0; j < elem_pos; j++ ) {
                            while ( !usart_can_write() );
                            usart_write( elem_buf[j] );
                        }
                        while ( !usart_can_write() );
                        usart_write( 0x0A );
                        */
                        /*
                        sprintf( dbg, "%d\n", (int)fifo_level( &gps_fifo ) );
                        for ( j = 0; j < strlen( dbg ); j++ ) {
                            while ( !usart_can_write() );
                            usart_write( dbg[j] );
                        }
                        */


                        /* 現在処理中のセンテンスで分岐 */
                        switch ( sentence ) {
                        case GPGGA:
                            switch ( sentence_pos ) {
                            case 0:
                                if ( display == DispGPS1 ) {
                                    /* 時刻をバッファに印字 */
                                    line_str[0][0] = elem_buf[0];
                                    line_str[0][1] = elem_buf[1];
                                    line_str[0][2] = 'H';
                                    line_str[0][3] = elem_buf[2];
                                    line_str[0][4] = elem_buf[3];
                                    line_str[0][5] = 'M';
                                    line_str[0][6] = elem_buf[4];
                                    line_str[0][7] = elem_buf[5];
                                    line_str[0][8] = 'S';
                                    line_str[0][9] = ' ';
                                }

                                break;

                            case 1:
                                if ( display == DispGPS2 ) {
                                    /* 緯度 */
                                    for ( j = 0; j < 16; j++ ) {
                                        line_str[1][j] = ' ';
                                    }

                                    line_str[1][0] = 'L';
                                    line_str[1][1] = 'A';
                                    line_str[1][2] = 'T';

                                    /* カラならなにもしない */
                                    if ( elem_buf[0] == ',' ) {
                                        break;
                                    }

                                    /* 度 */
                                    for ( j = 0; j < 2; j++ ) {
                                        line_str[1][3 + j] = elem_buf[j];
                                    }

                                    line_str[1][5] = 0xDF;

                                    /* 分 */
                                    for ( j = 0; elem_buf[j + 2] != ','; j++ ) {
                                        line_str[1][6 + j] = elem_buf[j + 2];
                                    }

                                    line_str[1][6 + j] = '\'';
                                }

                                break;

                            case 2:
                                if ( display == DispGPS2 ) {
                                    /* 緯度の向き */
                                    if ( elem_buf[0] != ',' ) {
                                        line_str[1][15] = elem_buf[0];
                                    }
                                }

                                break;

                            case 3:
                                if ( display == DispGPS2 ) {
                                    /* 経度 */
                                    for ( j = 0; j < 16; j++ ) {
                                        line_str[0][j] = ' ';
                                    }

                                    line_str[0][0] = 'L';
                                    line_str[0][1] = 'O';
                                    line_str[0][2] = 'N';

                                    /* カラならなにもしない */
                                    if ( elem_buf[0] == ',' ) {
                                        break;
                                    }

                                    /* 度 */
                                    for ( j = 0; j < 3; j++ ) {
                                        line_str[0][3 + j] = elem_buf[j];
                                    }

                                    line_str[0][6] = 0xDF;

                                    /* 分 */
                                    for ( j = 0; elem_buf[j + 3] != ','; j++ ) {
                                        line_str[0][7 + j] = elem_buf[j + 3];
                                    }

                                    line_str[0][7 + j] = '\'';
                                }

                                break;

                            case 4:
                                if ( display == DispGPS2 ) {
                                    /* 経度の向き */
                                    if ( elem_buf[0] != ',' ) {
                                        line_str[0][15] = elem_buf[0];
                                    }
                                }

                                break;

                            case 5:
                                if ( display == DispGPS1 ) {
                                    /* 受信クオリティ */
                                    line_str[0][14] = 'Q';
                                    line_str[0][15] = elem_buf[0];
                                }

                                /* Set ready */
                                if ( elem_buf[0] != '0' ) {
                                    gps_ready = 1;
                                } else {
                                    gps_ready = 0;
                                }

                                break;

                            case 6:
                                if ( display == DispGPS1 ) {
                                    /* 衛星個数 */
                                    line_str[0][10] = 'S';

                                    for ( j = 0; elem_buf[j] != ','; j++ ) {
                                        line_str[0][11 + j] = elem_buf[j];
                                    }

                                    line_str[0][11 + j] = ' ';
                                }

                                break;

                            case 7:
                                break;

                            case 8:
                                if ( display == DispGPS1 ) {
                                    /* 高さ */
                                    for ( j = 0; j < 8; j++ ) {
                                        line_str[1][j] = ' ';
                                    }

                                    for ( j = 0; elem_buf[j] != ','; j++ ) {
                                        line_str[1][j] = elem_buf[j];
                                    }

                                    line_str[1][j] = 'M';
                                }

                                /* あとは使わないので次のセンテンスへ */
                                sentence = OtherSentence;

                                break;

                            default:
                                break;
                            }

                            /* 次の要素へ */
                            sentence_pos++;

                            break;

                        case GPRMC:
                            switch ( sentence_pos ) {
                            case 6:
                                if ( display == DispGPS1 ) {
                                    /* 速度しか使わない */
                                    for ( j = 0; j < 8; j++ ) {
                                        line_str[1][8 + j] = ' ';
                                    }

                                    for ( j = 0; elem_buf[j] != ','; j++ ) {
                                        line_str[1][8 + j] = elem_buf[j];
                                    }

                                    line_str[1][8 + j + 0] = 'K';
                                    line_str[1][8 + j + 1] = 'T';
                                }

                                /* あとは使わないので次のセンテンスへ */
                                sentence = OtherSentence;

                                /* GPSデーター受信完了 */
                                updated_dev |= DEV_GPS;

                                break;

                            default:
                                break;
                            }

                            /* 次の要素へ */
                            sentence_pos++;

                            break;

                        case OtherSentence:
                            /* GGAかRMCを探し続ける */
                            if ( strncmp( "$GPGGA", elem_buf, 6 ) == 0 ) {
                                /* GPGGA見つけたのでGPGGAモードへ */
                                sentence = GPGGA;
                                sentence_pos = 0;
                            } else if ( strncmp( "$GPRMC", elem_buf, 6 ) == 0 ) {
                                /* GPRMCを見つけたのでGPRMCモードへ */
                                sentence = GPRMC;
                                sentence_pos = 0;
                            }

                            break;

                        default:
                            break;
                        }

                        /* 次のエレメントへ */
                        elem_pos = 0;
                    }
                }
            }
        }

        /* スイッチ処理 */
        /* 押されたスイッチを調べる */
        pushed_input = ~before_input & input;
        before_input = input;

        if ( ( SW_START & pushed_input ) && ( display < DispFormat ) && !write_dev ) {
            /* スタートが押されたらスタート画面へ */
            display = DispStartWrite;

            /* 画面更新指示 */
            display_changed = 1;
        }

        if ( ( SW_STOP & pushed_input ) && ( display < DispFormat ) && write_dev ) {
            /* If stop is pushed, go to stop */
            display = DispStopWrite;

            /* 画面更新指示 */
            display_changed = 1;
        }

        if ( ( SW_TOGGLE & pushed_input ) && ( display <= DispFormat ) ) {
            /* LEDのONOFF */
            PINB |= _BV( PB0 );
        }

        if ( ( SW_NEXT & pushed_input ) && ( display <= DispFormat ) ) {
            /* 次の画面へ */
            display++;

            if ( display > DispFormat ) {
                display = DispNone;
            }

            /* 画面更新指示 */
            display_changed = 1;
        }

        /* １秒ごとに更新 */
        if ( now_system_clock > before_system_clock + 10000 ) {
            /* バッテリレベル更新 */
            display_battery_level();

            before_system_clock = now_system_clock;

            /* 1sec */
            sec_timer = 1;
        } else {
            sec_timer = 0;
        }

        /* アイコン表示更新 */
        if ( display_changed || sec_timer ) {
            if ( write_dev ) {
                st7032i_set_icon( ST7032IIconAddrDataIn, ST7032IIconDataIn );
            } else {
                st7032i_set_icon( ST7032IIconAddrDataIn, 0 );
            }

            if ( gps_ready ) {
                st7032i_set_icon( ST7032IIconAddrAntena, ST7032IIconAntena );
            } else {
                st7032i_set_icon( ST7032IIconAddrAntena, 0 );
            }
        }

        /* 画面ごとに分岐 */
        switch ( display ) {
        case DispNone:
            /* 何もしない画面 */
            if ( display_changed ) {
                st7032i_clear();
                st7032i_puts( 0, 0, "None" );

                display_changed = 0;
            }

            break;

        case DispGPS1:
        case DispGPS2:
            /* GPS用1,2 */
            if ( display_changed ) {
                /* 画面更新開始 */
                clear_line_buf();
                st7032i_clear();
                st7032i_puts( 0, 0, "Wait" );

                /* GPS解析状態マシン初期化 */
                sentence     = OtherSentence;
                sentence_pos = 0;
                elem_pos     = 0;

                display_changed = 0;
            }

            if ( updated_dev & DEV_GPS ) {
                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );
            }

            break;

        case DispPressTemp:
            /* 気圧温度用 */
            if ( display_changed || ( updated_dev & DEV_TEMP & DEV_PRESS ) ) {
                st7032i_clear();

                sprintf( line_str[0], "PRES %dhPa", (int)( pres.pressure / 4096 ) );
                i = sprintf( line_str[1], "TEMP %d%cC", (int)mpu9150_get_temp_in_c( &mpu9150 ), 0xDF );

                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispAcc:
            /* 加速度 */
            if ( display_changed || ( updated_dev & DEV_ACC ) ) {
                clear_line_buf();

                sprintf( line_str[0], "Acc" );

                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispMag:
            /* 地磁気 */
            if ( display_changed || ( updated_dev & DEV_MAG ) ) {
                st7032i_clear();

                st7032i_puts( 0, 0, "Magnetic[uT]" );
                sprintf( line_str[1], "X%d Y%d Z%d",
                        (int)( mag.adj_x * 0.3 ),
                        (int)( mag.adj_y * 0.3 ),
                        (int)( mag.adj_z * 0.3 ) );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispGyro:
            /* ジャイロ */
            if ( display_changed || ( updated_dev & DEV_GYRO ) ) {
                clear_line_buf();

                sprintf( line_str[0], "Gyro" );

                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispStatus:
            /* Status */
            if ( display_changed || sec_timer ) {
                st7032i_clear();

                sprintf( line_str[0], "CLK:%lu", now_system_clock );
                sprintf( line_str[1], "FILE:%d%%", (int)( (float)fp.current_sector / fp.max_sector_count * 100 ) );

                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispFormat:
            /* フォーマット */
            if ( display_changed ) {
                st7032i_clear();

                /* If writing now, Format is failed */
                if ( write_dev ) {
                    st7032i_puts( 0, 0, "Format SDCard" );
                    st7032i_puts( 1, 0, "FAIL:writing" );
                } else {
                    st7032i_puts( 0, 0, "Format SDCard" );
                    st7032i_puts( 1, 0, "Push START" );
                }

                display_changed = 0;
            } else {
                /* Check keys */
                if ( !write_dev ) {
                    if ( pushed_input & SW_START ) {
                        /* Start format */
                        st7032i_clear();
                        st7032i_puts( 0, 0, "Formatting now" );
                        st7032i_puts( 1, 0, "Please wait" );

                        /* format */
                        ret = micomfs_format( &fs, 512, sd_get_size() / sd_get_block_size(), 64, 0 );

                        /* Put message */
                        st7032i_clear();

                        if ( ret ) {
                            st7032i_puts( 1, 0, "Succeeded" );
                        } else {
                            st7032i_puts( 1, 0, "Failed" );
                        }

                        /* Wait */
                        _delay_ms( 1000 );

                        /* Update display */
                        display_changed = 1;

                        /* GPSバッファをクリア */
                        cli();
                        while ( fifo_can_read( &gps_fifo ) ) {
                            fifo_read( &gps_fifo, &data );
                        }
                        sei();
                    }
                }
            }

            break;

        case DispStartWrite:
            /* 書きこみ開始 */
            if ( display_changed ) {
                /* Put message */

                /* カーソル初期化 */
                sensor_pos = DEV_PRESS;

                st7032i_clear();
                st7032i_puts( 0, 0, "Start writing" );
                st7032i_puts( 1, 0, "Y:START N:STOP" );

                /* Read EEPROM */
                eeprom_busy_wait();
                write_dev_buf = eeprom_read_byte( 0x00 );

                display_changed = 0;
            } else {
                /* Check keys */
                if ( pushed_input & SW_START ) {
                    if ( write_dev_buf ) {
                        /* Create file */
                        micomfs_fcreate( &fs, &fp, "log", MICOMFS_MAX_FILE_SECOTR_COUNT );

                        /* Start file writing */
                        micomfs_start_fwrite( &fp, 0 );

                        /* Update write flag */
                        write_dev = write_dev_buf;

                        /* Put message */
                        st7032i_clear();
                        st7032i_puts( 0, 0, "Start Writing" );

                        /* Write EEPROM */
                        eeprom_busy_wait();
                        eeprom_write_byte( 0x00, write_dev );
                    } else {
                        /* Since nothing is selected, Writing fails */
                        st7032i_clear();
                        st7032i_puts( 0, 0, "Writing failed!" );
                        st7032i_puts( 1, 0, "Please select" );
                    }

                    /* Wait */
                    _delay_ms( 1000 );

                    /* return */
                    display = DispGPS1;
                    display_changed = 1;

                    /* GPSバッファをクリア */
                    cli();
                    while ( fifo_can_read( &gps_fifo ) ) {
                        fifo_read( &gps_fifo, &data );
                    }
                    sei();
                } else if ( pushed_input & SW_STOP ) {
                    /* Stop writing */
                    display = DispGPS1;
                    display_changed = 1;

                    /* GPSバッファをクリア */
                    cli();
                    while ( fifo_can_read( &gps_fifo ) ) {
                        fifo_read( &gps_fifo, &data );
                    }
                    sei();
                } else {
                    /* Select sensor */
                    if ( pushed_input & SW_NEXT ) {
                        sensor_pos = sensor_pos << 1;

                        /* If over, reset cursor */
                        if ( sensor_pos & DEV_SD ) {
                            sensor_pos = DEV_PRESS;
                        }
                    } else if ( pushed_input & SW_TOGGLE ) {
                        /* Toggle current sensor will be written */
                        if ( write_dev_buf & sensor_pos ) {
                            write_dev_buf &= ~sensor_pos;
                        } else {
                            write_dev_buf |= sensor_pos;
                        }
                    }

                    /* Display press */
                    if ( sensor_pos & DEV_PRESS ) {
                        st7032i_puts( 0, 0, ">" );
                    } else {
                        st7032i_puts( 0, 0, " " );
                    }

                    st7032i_puts( 0, 1, "P:" );

                    if ( write_dev_buf & DEV_PRESS ) {
                        st7032i_puts( 0, 3, "W " );
                    } else {
                        st7032i_puts( 0, 3, "_ " );
                    }

                    /* Display gyro */
                    if ( sensor_pos & DEV_GYRO ) {
                        st7032i_puts( 0, 5, ">" );
                    } else {
                        st7032i_puts( 0, 5, " " );
                    }

                    st7032i_puts( 0, 6, "G:" );

                    if ( write_dev_buf & DEV_GYRO ) {
                        st7032i_puts( 0, 8, "W " );
                    } else {
                        st7032i_puts( 0, 8, "_ " );
                    }

                    /* Display mag */
                    if ( sensor_pos & DEV_MAG ) {
                        st7032i_puts( 0, 10, ">" );
                    } else {
                        st7032i_puts( 0, 10, " " );
                    }

                    st7032i_puts( 0, 11, "M:" );

                    if ( write_dev_buf & DEV_MAG ) {
                        st7032i_puts( 0, 13, "W " );
                    } else {
                        st7032i_puts( 0, 13, "_ " );
                    }

                    st7032i_puts( 0, 15, " " );

                    /* Display Acc */
                    if ( sensor_pos & DEV_ACC ) {
                        st7032i_puts( 1, 0, ">" );
                    } else {
                        st7032i_puts( 1, 0, " " );
                    }

                    st7032i_puts( 1, 1, "A:" );

                    if ( write_dev_buf & DEV_ACC ) {
                        st7032i_puts( 1, 3, "W " );
                    } else {
                        st7032i_puts( 1, 3, "_ " );
                    }

                    /* Display Temp */
                    if ( sensor_pos & DEV_TEMP ) {
                        st7032i_puts( 1, 5, ">" );
                    } else {
                        st7032i_puts( 1, 5, " " );
                    }

                    st7032i_puts( 1, 6, "T:" );

                    if ( write_dev_buf & DEV_TEMP) {
                        st7032i_puts( 1, 8, "W " );
                    } else {
                        st7032i_puts( 1, 8, "_ " );
                    }

                    /* Display GPS */
                    if ( sensor_pos & DEV_GPS ) {
                        st7032i_puts( 1, 10, ">" );
                    } else {
                        st7032i_puts( 1, 10, " " );
                    }

                    st7032i_puts( 1, 11, "S:" );

                    if ( write_dev_buf & DEV_GPS ) {
                        st7032i_puts( 1, 13, "W " );
                    } else {
                        st7032i_puts( 1, 13, "_ " );
                    }

                    st7032i_puts( 1, 15, " " );
                }
            }

            break;

        case DispStopWrite:
            /* Stop writing */
            if ( display_changed ) {
                st7032i_clear();
                st7032i_puts( 0, 0, "Stop writing" );
                st7032i_puts( 1, 0, "Y:START N:STOP" );

                display_changed = 0;
            } else {
                /* Check buttons */
                if ( pushed_input & SW_START ) {
                    /* Stop writing */
                    st7032i_clear();
                    st7032i_puts( 0, 0, "Stopping now" );
                    st7032i_puts( 1, 0, "Please wait" );

                    /* Stop file writing */
                    ret  = micomfs_stop_fwrite( &fp, 0 );
                    ret += micomfs_fclose( &fp );

                    /* 書き込み指示クリア */
                    write_dev = 0;

                    /* Put message */
                    st7032i_clear();

                    if ( ret ) {
                        st7032i_puts( 1, 0, "Succeeded" );
                    } else {
                        st7032i_puts( 1, 0, "Failed" );
                    }

                    /* Wait */
                    _delay_ms( 1000 );

                    /* Update display */
                    display = DispGPS1;
                    display_changed = 1;

                    /* GPSバッファをクリア */
                    cli();
                    while ( fifo_can_read( &gps_fifo ) ) {
                        fifo_read( &gps_fifo, &data );
                    }
                    sei();
                } else if ( pushed_input & SW_STOP ) {
                    /* Update display */
                    display = DispGPS1;
                    display_changed = 1;

                    /* GPSバッファをクリア */
                    cli();
                    while ( fifo_can_read( &gps_fifo ) ) {
                        fifo_read( &gps_fifo, &data );
                    }
                    sei();
                }
            }

            break;

        default:
            break;
        }
    }




#if 0


/*
    DispNone,
    DispGPS1,
    DispGPS2,
    DispPressTemp,
    DispAcc,
    DispMag,
    DispGyro,
    DispStatus,
    DispFormat,
    DispStartWrite,
    DispStopWrite,

    DEV_PRESS = 0x01,
    DEV_GYRO  = 0x02,
    DEV_MAG   = 0x04,
    DEV_ACC   = 0x08,
    DEV_TEMP  = 0x10,
    DEV_GPS   = 0x20,
    DEV_SD    = 0x40,

*/






    /* 全センサー測定開始（ 連続なもの ） */
    lps331ap_start( &pres );

    /* DEBUG */
    /* fs初期化 */
    if ( use_sd ) {
        int i;
        int j;

        /* フォーマット */
        micomfs_format( &fs, 512, sd_get_size() / sd_get_block_size(), 4, 0 );

        /* ファイル作る */
        micomfs_fcreate( &fs, &fp, "ATMEGA.txt", 1 );

        /* 書きまくる */
        for ( i = 0; i < 3; i++ ) {
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
        for( i = 0; micomfs_start_fwrite( &fp, i ); i++ ) {
            // micomfs_start_fwrite( &fp, i );

            for ( j = 0; j < fs.sector_size; j++ ) {
                data = 0x3C;

                micomfs_fwrite( &fp, &data, 1 );
            }

            micomfs_stop_fwrite( &fp, 0 );

            // put
            sprintf( line_str, "%u", i );
            st7032i_puts( 0, 0, line_str );
        }

        /* ファイル閉じる */
        micomfs_fclose( &fp );

        st7032i_puts( 1, 0, "Fyuu" );

        /* 第一セクター15バイトを全部USARTにぶちまける */
        /*
        for ( i = 0; i < 10; i++ ) {
            sd_start_step_block_read( i );

            for ( j = 0; j < fs.sector_size; j++ ) {
                data = sd_step_block_read();

                while ( !usart_can_write() );
                usart_write( data );
            }

            sd_stop_step_block_read();
        }
        */

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

#endif


    /* 終了 */
    return 0;
}
