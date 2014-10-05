/*
 *
 * 書き込みエラーが出た場合正常に終了する必要があるけど今は対策がないので作る
 *
 */

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

#define GPS_WRITE_UNIT 32       /* GPSデーターの最小書き込み単位 */
#define PRESSURE_AVR   100      /* 気圧の平均に使うデーター数 */

#define SW_STOP   _BV( PB7 )
#define SW_START  _BV( PB6 )
#define SW_TOGGLE _BV( PB4 )
#define SW_NEXT   _BV( PB5 )

static volatile uint32_t system_clock;  /* 100usごとにカウントされるタイマー */
static uint16_t input_counter;
static volatile uint8_t input;
static FIFO gps_fifo;
static char gps_buf[128];
static char line_str[2][17];
static volatile uint8_t battery_level;

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
    TimerUpdate = 0x01,
    TimerSec    = 0x02,
} TimerFlags;

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

    /* 50msごとに入力読み込み */
    if ( 500 <= input_counter ) {
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

    /* ADCH = ( 51 / ( 51 + 100 ) * VBAT ) / 3.3 * 256 */

    if ( data < 69 ) {
        /* V < 2.6 */
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
    battery_level = get_battery_level();

    switch ( battery_level ) {
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

    char file_name[16];

    char elem_buf[16];
    uint8_t elem_pos;
    NMEASentence sentence;
    uint8_t sentence_pos;
    uint8_t gps_ready;

    uint8_t sec_timer01;
    uint8_t update_timer;
    uint8_t sec_timer;
    TimerFlags timer_flags;
    uint32_t before_system_clock;
    uint32_t now_system_clock;

    uint8_t before_input;
    uint8_t pushed_input;

    int32_t base_pressure;
    int64_t avr_pressure_buf;
    int32_t avr_pressure;
    int avr_pressure_counter;
    int32_t height;

    LPS331APUnit pres;
    AK8975Unit mag;
    MPU9150Unit mpu9150;

    MicomFS fs;
    MicomFSFile fp;

    int max_fifo_level = 0;
    int fifolevel;
    uint8_t readSize;
    uint8_t data;
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
    ADMUX  = 0x20;          /* ADC0, Left, AVcc */
    ADCSRA = 0x86;          /* Enable, 1/64, Disable int, Disable Auto Trigger */

    /* タイマー0を初期化 */
    TCCR0B = 0x02;          /* プリスケール8，CTCモード */
    TCCR0A = 0x02;          /* CTCモード */
    TIMSK0 = 0x02;          /* コンペアマッチA割り込み有効 */
    OCR0A  = 100;           /* 100usごとに割りこみ発生 */

    /* I2Cバス初期化 */
    i2c_init_master( 32, I2CPrescale1, 0, 0 );

    /* デバイス初期化 */
    enabled_dev = DEV_GPS;

    /* SDとSPI初期化 外部プルアップの場合は上を有効 */
    // if ( sd_init( SPIOscDiv2, 512, 0 ) ) {
    if ( sd_init( SPIOscDiv2, 512, SPIMISO | SPIMOSI ) ) {
        enabled_dev |= DEV_SD;
    }

    /* AN 384/128 DS 512/64 */
    if ( lps331ap_init( &pres, 0x5D, LPS331AP25_25Hz, LPS331APPresAvg512, LPS331APTempAvg64 ) ) {
        enabled_dev |= DEV_PRESS;
    }

    if ( mpu9150_init( &mpu9150, 0x68, 0, MPU9150LPFCFG0, MPU9150AccFSR4g, MPU9150AccHPFReset, MPU9150GyroFSR500DPS ) ) {
        enabled_dev |= DEV_GYRO | DEV_ACC;
    }

    if ( ak8975_init( &mag, 0x0C ) ) {
        enabled_dev |= DEV_MAG;
    }

    st7032i_init( 0x25 );

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
    st7032i_puts( 0, 0, "System status" );
    ( enabled_dev & DEV_SD    ) ? st7032i_puts( 1, 0,  "SD:O" ) : st7032i_puts( 1, 0,  "SD:X" );
    ( enabled_dev & DEV_PRESS ) ? st7032i_puts( 1, 5,  "PR:O" ) : st7032i_puts( 1, 5,  "PR:X" );
    ( enabled_dev & DEV_ACC   ) ? st7032i_puts( 1, 10, "3D:O" ) : st7032i_puts( 1, 10, "3D:X" );

    /* Wait */
    _delay_ms( 1000 );

    /* USART起動 */
    usart_init( 4800, UsartRX | UsartTX, UsartIntRX );

    /* 各種変数初期化 */
    display_changed = 1;
    write_dev       = 0;
    write_dev_buf   = 0;
    sentence        = OtherSentence;
    sentence_pos    = 0;
    elem_pos        = 0;
    display         = DispGPS1;
    gps_ready       = 0;
    sensor_pos      = 0;
    before_system_clock = 0;
    now_system_clock    = 0;
    system_clock        = 0;
    sec_timer       = 0;
    sec_timer01     = 0;
    update_timer    = 0;
    max_fifo_level  = 0;
    battery_level   = 0;
    base_pressure        = (int32_t)1013 * 4096;
    avr_pressure         = base_pressure;
    avr_pressure_buf     = 0;
    avr_pressure_counter = 0;

    /* Init file name */
    strcpy( file_name, "log.log" );

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

        /* 最大FIFO使用量記録 ほぼDEBUG用 */
        if ( ( fifo_level( &gps_fifo ) > max_fifo_level ) && display <= DispFormat ) {
            max_fifo_level = fifo_level( &gps_fifo );
        }

        /* GPSデーターが書きこみ単位以上たまってれば処理 */
        if ( ( enabled_dev & DEV_GPS ) && ( ( fifolevel = fifo_level( &gps_fifo ) ) >= GPS_WRITE_UNIT ) ) {
            /* 読み込み量決定 */
            if ( fifolevel < 256 ) {
                readSize = fifolevel;
            } else {
                readSize = 255;
            }

            /* 必要ならヘッダ書き込み */
            if ( write_dev & DEV_GPS ) {
                data = LOG_SIGNATURE;
                micomfs_seq_fwrite( &fp, &data, 1 );
                micomfs_seq_fwrite( &fp, &now_system_clock, sizeof( now_system_clock ) );
                data = ID_GPS;
                micomfs_seq_fwrite( &fp, &data, 1 );
                data = readSize;
                micomfs_seq_fwrite( &fp, &data, 1 );
            }

            /* 全バイト処理 */
            for ( i = 0; i < readSize; i++ ) {
                /* 1バイトよみ */
                cli();
                ret = fifo_read( &gps_fifo, &data );
                sei();

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
                        /* 現在処理中のセンテンスで分岐 */
                        switch ( sentence ) {
                        case GPGGA:
                            switch ( sentence_pos ) {
                            case 0:
                                if ( display == DispGPS2 ) {
                                    /* 時刻をバッファに印字 */
                                    line_str[0][2] = 'H';
                                    line_str[0][5] = 'M';
                                    line_str[0][8] = 'S';
                                    line_str[0][9] = ' ';

                                    if ( elem_pos >= 6 ) {
                                        /* 時刻が６文字以上なら保存 */
                                        line_str[0][0] = elem_buf[0];
                                        line_str[0][1] = elem_buf[1];
                                        line_str[0][3] = elem_buf[2];
                                        line_str[0][4] = elem_buf[3];
                                        line_str[0][6] = elem_buf[4];
                                        line_str[0][7] = elem_buf[5];
                                    } else {
                                        line_str[0][0] = 'x';
                                        line_str[0][1] = 'x';
                                        line_str[0][3] = 'x';
                                        line_str[0][4] = 'x';
                                        line_str[0][6] = 'x';
                                        line_str[0][7] = 'x';
                                    }
                                }

                                /* ついでにファイルアクセス中でなければファイル名にもコピー */
                                if ( !write_dev && elem_pos >= 6 ) {
                                    memcpy( file_name, elem_buf, 6 );
                                    file_name[6]  = '.';
                                    file_name[7]  = 'l';
                                    file_name[8]  = 'o';
                                    file_name[9]  = 'g';
                                    file_name[10] = '\0';
                                }

                                break;

                            case 1:
                                if ( display == DispGPS1 ) {
                                    /* 緯度 */
                                    for ( j = 0; j < 16; j++ ) {
                                        line_str[1][j] = ' ';
                                    }

                                    line_str[1][0] = 'L';
                                    line_str[1][1] = 'A';
                                    line_str[1][2] = ' ';

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
                                if ( display == DispGPS1 ) {
                                    /* 緯度の向き */
                                    if ( elem_buf[0] != ',' ) {
                                        line_str[1][15] = elem_buf[0];
                                    }
                                }

                                break;

                            case 3:
                                if ( display == DispGPS1 ) {
                                    /* 経度 */
                                    for ( j = 0; j < 16; j++ ) {
                                        line_str[0][j] = ' ';
                                    }

                                    line_str[0][0] = 'L';
                                    line_str[0][1] = 'O';
                                    line_str[0][2] = ' ';

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
                                if ( display == DispGPS1 ) {
                                    /* 経度の向き */
                                    if ( elem_buf[0] != ',' ) {
                                        line_str[0][15] = elem_buf[0];
                                    }
                                }

                                break;

                            case 5:
                                if ( display == DispGPS2 ) {
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
                                if ( display == DispGPS2 ) {
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
                                if ( display == DispGPS2 ) {
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
                                if ( display == DispGPS2 ) {
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

        if ( ( SW_START & pushed_input ) && ( display < DispFormat ) && ( display != DispPressTemp ) && !write_dev ) {
            /* スタートが押されたらスタート画面へ */
            display = DispStartWrite;

            /* 画面更新指示 */
            display_changed = 1;
        }

        if ( ( SW_STOP & pushed_input ) && ( display <= DispFormat ) && write_dev ) {
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

            /* Clear display */
            st7032i_clear();
        }

        /* タイマーフラッグクリア */
        timer_flags = 0;

        /* 0.1秒ごとに更新 */
        if ( now_system_clock > before_system_clock + 1000 ) {
            /* 前回時刻更新 */
            before_system_clock = now_system_clock;

            /* 0.1sec */
            sec_timer01 = 1;
        } else {
            sec_timer01 = 0;
        }

        /* 1sec timer */
        if ( sec_timer01 ) {
            if ( sec_timer < 10 ) {
                sec_timer++;
            } else {
                sec_timer = 0;

                timer_flags |= TimerSec;
            }
        }

        /* 0.2sec timer */
        if ( sec_timer01 ) {
            if ( update_timer < 2 ) {
                update_timer++;
            } else {
                update_timer = 0;

                /* センサーデーター表示更新 */
                timer_flags |= TimerUpdate;
            }
        }

        /* アイコン表示更新 */
        if ( display_changed || ( timer_flags & TimerSec ) ) {
            /* バッテリレベル更新 */
            display_battery_level();

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

        /* 書き込み中にバッテリレベルが低下したので書き込み中止 */
        if ( write_dev && ( battery_level < 1 ) ) {
            /* Stop file writing */
            micomfs_stop_fwrite( &fp, 0 );
            micomfs_fclose( &fp );

            /* 書き込み指示クリア */
            write_dev = 0;

            /* GPSバッファをクリア */
            cli();
            fifo_clear( &gps_fifo );
            sei();

            /* 画面表示をNoneに */
            display = DispNone;
            display_changed = 1;
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
            /* 平均気圧計算 */
            if ( ( updated_dev & DEV_PRESS ) ) {
                avr_pressure_buf += pres.pressure;
                avr_pressure_counter++;

                if ( avr_pressure_counter == PRESSURE_AVR ) {
                    avr_pressure = avr_pressure_buf / PRESSURE_AVR;
                    avr_pressure_counter = 0;
                    avr_pressure_buf     = 0;
                }
            }

             /* 気圧モートでスタートがおされた場合高度用基準気圧を更新 */
            if ( pushed_input & SW_START ) {
                base_pressure = avr_pressure;
            }

            if ( display_changed || ( timer_flags & TimerUpdate ) ) {
                /* 表示 */
                /* 高さ計算 */
                // DBEUG
                height = ( base_pressure - avr_pressure ) / 4096.0 * 835;
                // height = ( base_pressure - pres.pressure ) / 4096.0 * 835;

                /* 表示気圧も平均気圧 */
                snprintf( line_str[0], 17, "Pres %ld[Pa] ", (int32_t)( pres.pressure / 4096.0 * 100 ) );
                snprintf( line_str[1], 17, "%02d[%cC] %ld[cm]    ", (int)mpu9150_get_temp_in_c( &mpu9150 ), 0xDF, height );

                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispAcc:
            /* 加速度 */
            if ( display_changed || ( timer_flags & TimerUpdate ) ) {
                snprintf( line_str[0], 17, "Acc[mG] X%+06d", (int)( mpu9150.acc_x * 0.122 ) );
                snprintf( line_str[1], 17, "Y%+06d Z%+06d",
                        (int)( mpu9150.acc_y * 0.122 ),
                        (int)( mpu9150.acc_z * 0.122 ) );
                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispMag:
            /* 地磁気 */
            if ( display_changed || ( timer_flags & TimerUpdate ) ) {
                snprintf( line_str[0], 17, "Mag[uT] X%+06d", (int)( mag.adj_x * 0.3 ) );
                snprintf( line_str[1], 17, "Y%+06d Z%+06d",
                        (int)( mag.adj_y * 0.3 ),
                        (int)( mag.adj_z * 0.3 ) );
                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispGyro:
            /* ジャイロ */
            if ( display_changed || ( timer_flags & TimerUpdate ) ) {
                snprintf( line_str[0], 17, "Gy[dps] X%+06d", (int)( mpu9150.gyro_x * 0.01526 ) );
                snprintf( line_str[1], 17, "Y%+06d Z%+06d",
                        (int)( mpu9150.gyro_y * 0.01526 ),
                        (int)( mpu9150.gyro_z * 0.01526 ) );
                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispStatus:
            /* Status */
            if ( display_changed || ( timer_flags & TimerSec ) ) {
                st7032i_clear();
                clear_line_buf();

                snprintf( line_str[0], 17, "CLK:%lu", now_system_clock );

                if ( write_dev ) {
                    snprintf( line_str[1], 17, "%s %d", file_name, max_fifo_level );
                } else {
                    snprintf( line_str[1], 17, "%d", max_fifo_level );
                }

                st7032i_puts( 0, 0, line_str[0] );
                st7032i_puts( 1, 0, line_str[1] );

                display_changed = 0;
            }

            break;

        case DispFormat:
            /* フォーマット */
            if ( display_changed ) {
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
                        st7032i_puts( 0, 0, "Format SDCard" );
                        st7032i_puts( 1, 0, "Please wait" );

                        /* format */
                        ret = micomfs_format( &fs, 512, sd_get_size() / sd_get_block_size(), 1024, 0 );

                        /* Put message */
                        st7032i_clear();
                        st7032i_puts( 0, 0, "Format SDCard" );

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
                        fifo_clear( &gps_fifo );
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
                st7032i_puts( 1, 0, "Y:START/N:STOP" );

                /* ユーザーが読むのを少しまつ */
                _delay_ms( 1000 );

                /* Read EEPROM */
                eeprom_busy_wait();
                write_dev_buf = ( DEV_ACC | DEV_GPS | DEV_GYRO | DEV_MAG | DEV_PRESS | DEV_TEMP ) & eeprom_read_byte( 0x00 );

                display_changed = 0;
            } else {
                /* Check keys */
                if ( pushed_input & SW_START ) {

                    if ( battery_level < 1 ) {
                        /* バッテリーが電圧不足だと安全のため実行できない */
                        st7032i_clear();
                        st7032i_puts( 0, 0, "Writing failed!" );
                        st7032i_puts( 1, 0, "Low battery" );
                    } else if ( write_dev_buf ) {
                        /* 開始するたびにFS初期化とファイル作成 */
                        ret  = micomfs_init_fs( &fs );
                        ret *= micomfs_fcreate( &fs, &fp, file_name, MICOMFS_MAX_FILE_SECOTR_COUNT );

                        /* Create file */
                        if ( !ret ) {
                            /* ファイルの確保失敗 */
                            st7032i_clear();
                            st7032i_puts( 0, 0, "Writing failed!" );
                            st7032i_puts( 1, 0, "Cant create file" );
                        } else {
                            /* Start file writing */
                            micomfs_start_fwrite( &fp, 0 );

                            /* Write signature */
                            data = DEVICE_LOG_SIGNATURE;
                            micomfs_seq_fwrite( &fp, &data, 1 );

                            /* Write device list */
                            data = write_dev_buf;
                            micomfs_seq_fwrite( &fp, &data, 1 );

                            /* Update write flag */
                            write_dev = write_dev_buf;

                            /* Write EEPROM */
                            eeprom_busy_wait();
                            eeprom_write_byte( 0x00, write_dev );

                            /* Put message */
                            st7032i_clear();
                            st7032i_puts( 0, 0, "Start Writing" );
                            st7032i_puts( 1, 0, "Succeeded" );
                        }
                    } else {
                        /* Since nothing is selected, The Writing fails */
                        st7032i_clear();
                        st7032i_puts( 0, 0, "Writing failed!" );
                        st7032i_puts( 1, 0, "Please select" );
                    }

                    /* Wait */
                    _delay_ms( 1000 );

                    /* return */
                    display = DispNone;
                    display_changed = 1;

                    /* GPSバッファをクリア */
                    cli();
                    fifo_clear( &gps_fifo );
                    sei();
                } else if ( pushed_input & SW_STOP ) {
                    /* Stop writing */
                    display = DispNone;
                    display_changed = 1;

                    /* GPSバッファをクリア */
                    cli();
                    fifo_clear( &gps_fifo );
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
                        /* 指定センサーを書くかどうか切り替え */
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
                st7032i_puts( 1, 0, "Y:START/N:STOP" );

                display_changed = 0;
            } else {
                /* Check buttons */
                if ( pushed_input & SW_START ) {
                    /* Stop writing */
                    st7032i_clear();
                    st7032i_puts( 0, 0, "Stop writing" );
                    st7032i_puts( 1, 0, "Please wait" );

                    /* 終了シグネチャ書き込み */
                    data = LOG_END_SIGNATURE;
                    micomfs_seq_fwrite( &fp, &data, 1 );

                    /* Stop file writing */
                    ret  = micomfs_stop_fwrite( &fp, 0 );
                    ret += micomfs_fclose( &fp );

                    /* 書き込み指示クリア */
                    write_dev = 0;

                    /* Put message */
                    st7032i_clear();
                    st7032i_puts( 0, 0, "Stop writing" );

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
                    fifo_clear( &gps_fifo );
                    sei();
                } else if ( pushed_input & SW_STOP ) {
                    /* Update display */
                    display = DispNone;
                    display_changed = 1;
                }
            }

            break;

        default:
            break;
        }
    }

    /* 終了 */
    return 0;
}
