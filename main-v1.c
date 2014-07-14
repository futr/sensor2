/*
 * main.c センサーロガー
 *
 * プロトコル
 * | SIZE[1] | DEVICE_ID[1] | TIME[4] | DATA[1-256] |
 * SIZE      : ヘッダー部6バイトを含まないDATA部のみのサイズ
 * DEVICE_ID : device_id.hで定義されるセンサーの固有ID
 * TIME      : 100usごとの時間
 *
 *
 */

#include "ide.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "usart.h"
#include "sd.h"
#include "adxl345.h"
#include "l3gd20.h"
#include "lps331ap.h"
#include "hmc5883l.h"
#include "device_id.h"

#define CMD_HEADER    0xA5
#define CMD_READ_RAW  0x52
#define CMD_READ_CARD 0x43
#define CMD_STOP      0x53

#define LED_PORT PORTD
#define LED_PIN  PD2
#define SW_PORT  PINB
#define SW_0     PB0
#define SW_1     PB1

typedef enum SystemMode_tag {
    WriteCardMode,
    ReadCardMode,
    ReadRawMode,
} SystemMode;

/* プロトタイプ */
void system_stop( void );

/* 100usごとにカウントされるタイマー */
static uint32_t system_clock;

ISR( BADISR_vect )
{
    /* 異常な割り込み発生 */
    system_stop();
}

ISR( TIMER0_COMPA_vect )
{
    /* タイマー0コンペアマッチA割り込みベクター */

    /* システムクロックを100usごとに1更新 */
    system_clock++;
}

int main( void )
{
    /* いろいろセンサーロガー */
    char use_sd;
    char use_acc;
    char use_gyro;
    char use_pres;
    char use_mag;
    char mesure;
    uint8_t cmd;
    uint8_t data;
    uint8_t packet[1 + 4 + 1 + 8];

    uint32_t now;
    uint32_t sector;
    uint32_t write_size;
    uint32_t pos = 0;
    uint32_t address;

    SensorDeviceId device;
    SystemMode mode;
    uint16_t i;

    ADXLUnit acc;
    L3GD20Unit gyro;
    LPS331APUnit pres;
    HMC5883LUnit mag;

    /* 初期化開始 */

    /* 割り込み停止 */
    cli();

    /* LEDのポートを出力に */
    DDRD  |= _BV( PD2 );

    /* スイッチを入力ポートに設定しプルアップ有効 */
    DDRB  &= ~( _BV( PB0 ) | _BV( PB1 ) );
    PORTB |= ( _BV( PB0 ) | _BV( PB1 ) );

    /* タイマー0を初期化 */
    TCCR0B = 0x02;          /* プリスケール8，CTCモード */
    TCCR0A = 0x02;          /* CTCモード */
    TIMSK0 = 0x02;          /* コンペアマッチA割り込み有効 */
    OCR0A  = 100;           /* 100usごとに割りこみ発生 */

    /* USART初期化 */
    usart_init( 9600UL, UsartRX | UsartTX, 0 );

    /* SD初期化 */
    use_sd = sd_init( SPIOscDiv2, 512, SPIMISO );

    /* i2c初期化 */
    i2c_init_master( 32, I2CPrescale1, 0, 0 );

    /* ADXL初期化 */
    use_acc  = adxl_init( &acc, 0x53, ADXLRange16, ADXLResolutionFull, ADXL400Hz );

    /* ジャイロ初期化 */
    use_gyro = l3gd20_init( &gyro, 0x6A, L3GD20_760Hz, L3G2D20_500dps );

    /* 気圧初期化 */
    use_pres = lps331ap_init( &pres, 0x5C, LPS331AP25_25Hz, LPS331APPresAvg1, LPS331APTempAvg1 );

    /* 磁気センサー初期化 */
    use_mag  =  hmc5883l_init( &mag, 0x1E, HMC5883L75Hz, HMC5883LAvg1, HMC5883L2_5Ga );

    /* 全センサー測定開始 */
    hmc5883l_start( &mag );
    lps331ap_start( &pres );
    l3gd20_start( &gyro );
    adxl_start( &acc );

    /* 割り込み開始 */
    sei();

    /* 初期化完了 */

L_PROCESS_RESET:
    /* ちょっと待つ */
    _delay_ms( 500 );

    /* LED点灯 */
    PORTD |= _BV( PD2 );

    /* ユーザーからの指示を待つ */
    while ( 1 ) {
        if ( !( PINB & _BV( PB0 ) ) ) {
            /* SW0がONになったのでSD書きこみモードへ */
            mode = WriteCardMode;

            break;
        } else if ( usart_can_read() ) {
            /* USART経由で何か飛んできた */
            cmd = usart_read();

            /* コマンド確認 */
            if ( cmd == CMD_READ_CARD ) {
                /* カード読み出しへ */
                mode = ReadCardMode;

                break;
            } else if ( cmd == CMD_READ_RAW ) {
                /* 生データー読み出しへ */
                mode = ReadRawMode;

                break;
            }
        }
    }

    /* エラーチェック */
    if ( ( mode == ReadCardMode || mode == WriteCardMode ) && ( use_sd == 0 ) ) {
        /* カード処理モードでカードがなければエラー */

        /* Lチカする */
        for ( i = 0; i < 6; i++ ) {
            PIND |= _BV( PD2 );
            _delay_ms( 500 );
        }

        /* 指示待ちへ */
        goto L_PROCESS_RESET;
    } else if ( ( mode != ReadCardMode ) && ( ( use_acc + use_gyro + use_mag + use_pres ) == 0 ) ) {
        /* センサーを扱うのにセンサーがひとつもない */

        /* Lチカする */
        for ( i = 0; i < 6; i++ ) {
            PIND |= _BV( PD2 );
            _delay_ms( 500 );
        }

        /* 指示待ちへ */
        goto L_PROCESS_RESET;
    }

    /* 準備が完了したのでLEDを消す */
    PORTD &= ~_BV( PD2 );

    /* モードに従って処理開始 */
    if ( mode == ReadCardMode ) {
        /* カード内のデーターをUSART経由で転送 */

        /* 総転送セクター数を第一セクターから取得 */
        sd_block_read( 0, (uint8_t *)&write_size, 0, 4 );

        /* 転送開始 */
        for ( sector = 0; sector < write_size; sector++ ) {
            /* 止まれと指示が来れば停止 */
            if ( usart_can_read() ) {
                if ( usart_read() == CMD_STOP ) {
                    goto L_PROCESS_RESET;
                }
            }

            /* アドレス確定 */
            address = sector;

            if ( sd_get_address_mode() == SDByte ) {
                address = sector * 512;
            }

            /* 転送開始 */
            sd_start_step_block_read( address );

            for ( i = 0; i < 512; i++ ) {
                while ( !usart_can_write() );
                usart_write( sd_step_block_read() );
            }

            sd_stop_step_block_read();
        }

        /* ユーザーの指示を待つ */
        goto L_PROCESS_RESET;
    } else {
        /* 測定-転送モード */
        write_size = 0;
        device = 0;

        /* SDモードならカード書きこみ開始 */
        if ( mode == WriteCardMode ) {
            /* 先頭セクターにカードの最大容量を書きこんでおく ( 容量オーバーした場合の対処 ) */
            address = sd_get_size() / 512;

            sd_block_write( 0, (uint8_t *)&address, 0, 4 );

            /* アドレスを第一セクターへ */
            if ( sd_get_address_mode() == SDByte ) {
                address = 512;
            } else {
                address = 1;
            }

            /* セクター内位置初期化 */
            pos = 0;

            if ( !sd_start_step_block_write( address ) ) {
                /* 書きこみ開始失敗したらシステム停止 */
                system_stop();
            }
        }

        /* タイマークリア */
        cli();
        system_clock = 0;
        TCNT0  = 0x00;
        GTCCR |= _BV( PSRSYNC );
        sei();

        /* 測定ループ開始 */
        while ( 1 ) {
            /* USART経由モードで停止指示がきてたらユーザーの指示をまつ */
            if ( mode == ReadRawMode &&  usart_can_read() ) {
                data = usart_read();

                if ( data == CMD_STOP ) {
                    /* 停止 */
                    goto L_PROCESS_RESET;
                }
            }

            /* SDモードでスタートボタンが切れてたら止まる */
            if ( ( mode == WriteCardMode ) && ( PINB & _BV( PB0 ) ) ) {
                /* 残りセクターを転送する */
                for ( i = pos; i < 512; i++ ) {
                    sd_step_block_write( 0x00 );
                }

                /* ストップ */
                sd_stop_step_block_write();

                /* アドレスをブロックサイズに調整 */
                if ( sd_get_address_mode() == SDBlock ) {
                    address = address + 1;
                } else {
                    address = address / 512 + 1;
                }

                /* 総転送ブロック数を記録 */
                sd_block_write( 0, (uint8_t *)&address, 0, 4 );

                /* 停止 */
                goto L_PROCESS_RESET;
            }

            /* パケット作成 */

            /* 測定完了フラグ解除 */
            mesure = 0;

            switch ( device ) {
            case ID_ADXL345:
                /* 加速度センサー */
                if ( use_acc && adxl_data_ready( &acc ) && adxl_read( &acc ) ) {
                    /* 測定できた */
                    cli();
                    now = system_clock;
                    sei();

                    *(uint32_t *)( packet + 2 ) = now;

                    packet[0] = 6;
                    packet[1] = device;

                    *( (int16_t *)( packet + 6 ) + 0 ) = acc.x;
                    *( (int16_t *)( packet + 6 ) + 1 ) = acc.y;
                    *( (int16_t *)( packet + 6 ) + 2 ) = acc.z;

                    /* 測定完了フラグ */
                    mesure = 1;
                }

                break;

            case ID_L3GD20:
                /* ジャイロセンサー */
                if ( use_gyro && l3gd20_data_ready( &gyro ) && l3gd20_read( &gyro ) ) {
                    /* 測定できた */
                    cli();
                    now = system_clock;
                    sei();

                    *(uint32_t *)( packet + 2 ) = now;

                    packet[0] = 6;
                    packet[1] = device;

                    *( (int16_t *)( packet + 6 ) + 0 ) = gyro.x;
                    *( (int16_t *)( packet + 6 ) + 1 ) = gyro.y;
                    *( (int16_t *)( packet + 6 ) + 2 ) = gyro.z;

                    /* 測定完了フラグ */
                    mesure = 1;
                }

                break;

            case ID_LPS331AP:
                /* 気圧センサー */
                if ( use_pres && lps331ap_data_ready( &pres ) && lps331ap_read( &pres ) ) {
                    /* 測定できた */
                    cli();
                    now = system_clock;
                    sei();

                    *(uint32_t *)( packet + 2 ) = now;

                    packet[0] = 4;
                    packet[1] = device;

                    *( (int32_t *)( packet + 6 ) + 0 ) = pres.pressure;

                    /* 測定完了フラグ */
                    mesure = 1;
                }

                break;

            case ID_LPS331AP_TEMP:
                /* 温度センサー */
                if ( use_pres && lps331ap_temp_data_ready( &pres ) && lps331ap_read_temp( &pres ) ) {
                    /* 測定できた */
                    cli();
                    now = system_clock;
                    sei();

                    *(uint32_t *)( packet + 2 ) = now;

                    packet[0] = 2;
                    packet[1] = device;

                    *( (int16_t *)( packet + 6 ) + 0 ) = pres.temp;

                    /* 測定完了フラグ */
                    mesure = 1;
                }

                break;

            case ID_HMC5883L:
                /* 地磁気センサー */
                if ( use_mag && hmc5883l_data_ready( &mag ) && hmc5883l_read( &mag ) ) {
                    /* 測定できた */
                    cli();
                    now = system_clock;
                    sei();

                    *(uint32_t *)( packet + 2 ) = now;

                    packet[0] = 6;
                    packet[1] = device;

                    *( (int16_t *)( packet + 6 ) + 0 ) = mag.x;
                    *( (int16_t *)( packet + 6 ) + 1 ) = mag.y;
                    *( (int16_t *)( packet + 6 ) + 2 ) = mag.z;

                    /* 測定完了フラグ */
                    mesure = 1;
                }

                break;

            default:
                break;
            }

            /* センサーIDをすすめる */
            device++;

            if ( device == DEVICE_COUNT ) {
                device = 0;
            }

            /* 測定できてなければ次のセンサーへ */
            if ( !mesure ) {
                continue;
            }

            /* 測定できていたのでモードに従い転送 */
            if ( mode == WriteCardMode ) {
                /* カードへ書き込み */
                for ( i = 0; i < ( packet[0] + 6 ); i++ ) {
                    /* 1Byte転送 */
                    sd_step_block_write( packet[i] );

                    /* アドレスを進め必要ならブロックをすすめる */
                    pos++;

                    /* 1セクタ書き終えたら */
                    if ( pos >= 512 ) {
                        /* ブロックライト完了 */
                        sd_stop_step_block_write();

                        /* ブロック内アドレスをクリア */
                        pos = 0;

                        /* ブロックアドレス更新 */
                        if ( sd_get_address_mode() == SDBlock ) {
                            address++;
                        } else {
                            address += 512;
                        }

                        /* 次のブロックライト開始 */
                        if ( !sd_start_step_block_write( address ) ) {
                            /* ブロックライトが失敗したので総出力ブロック数を記録して終了したいがうまくいかないので放置する */
                            /*
                            if ( sd_get_address_mode() == SDBlock ) {
                                address = address;
                            } else {
                                address = address / 512;
                            }
                            */

                            /* 一応書きこむがうまくいかない */
                            // sd_block_write( 0, (uint8_t *)&address, 0, 4 );

                            /* システム停止 */
                            system_stop();
                        }

                    }
                }
            } else {
                /* USART経由で生データー読み込みモード */
                for ( i = 0; i < 6 + packet[1]; i++ ) {
                    while ( !usart_can_write() );
                    usart_write( packet[i] );
                }
            }
        }
    }

    /* 終了 */
    return 0;
}

void system_stop( void )
{
    /* 何らかの理由でシステムが停止したのでLEDをチカチカさせ続ける */
    while ( 1 ) {
        PIND |= _BV( PD2 );
        _delay_ms( 100 );
    }
}
