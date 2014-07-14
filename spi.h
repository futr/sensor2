/*
 * SPIMasterモード専用モジュール
 *
 * SPIに使用されるピンには方向を手動で設定する必要があるものが存在します．( mega88 Data sheet Table 19-1 )
 * spi_initをコールすると，設定に応じで自動でオーバーライドしますが，
 * spi_releaseをコールしても復旧されません．注意が必要です．
 *
 * 入力ピンのプルアップはSPI起動中でもPORTピンで制御されます．ただし，spi_initは指定された設定をPORTに適用します．
 * また，プルアップ制御してもMCUCRのPUDが0でなければプルアップは有効になりません．( デフォルトで0 )
 *
 * complete(SPIF)フラグは通信完了時に立てられ，割り込み完了時にクリアされます．
 * もしくは，SPIステータスレジスターを読み込み，その後SPIデータレジスターにアクセスするとクリアされます．
 *
 * 使える割り込み
 * SPIF : SPI通信の完了を通知．割り込みベクターにより解除される．もしくは，SPSRを読んだ後に，SPDRにアクセスすることで解除される．
 *
 */

#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

#include "ide.h"
#include <avr/io.h>
#include <stdlib.h>

typedef enum SPIOrder_tag {
    SPILSB = 0x20,
    SPIMSB = 0x00,
} SPIOrder;

typedef enum SPIPin_tag {
    SPISS   = _BV( PB2 ),
    SPIMISO = _BV( PB4 ),
    SPIMOSI = _BV( PB3 ),
    SPISCK  = _BV( PB5 ),
} SPIPin;

typedef enum SPISpeed_tag {
    SPIOscDiv4     = 0x00,
    SPIOscDiv16    = 0x01,
    SPIOscDiv64    = 0x02,
    SPIOscDiv128   = 0x03,
    SPIOscDiv2     = 0x04,
    SPIOscDiv8     = 0x05,
    SPIOscDiv32    = 0x06,
    SPIOscDiv64_X2 = 0x07,
} SPISpeed;

typedef enum SPISide_tag {
    SPIMaster = 0x01,
    SPISlave  = 0x02,
} SPISide;

typedef enum SPIMode_tag {
    SPIMode0 = 0x00,
    SPIMode1 = 0x40,
    SPIMode2 = 0x80,
    SPIMode3 = 0xC0,
} SPIMode;

typedef enum SPIStatus_tag {
    SPIError,
    SPISuccess,
    SPIWorking,
    SPINone,
} SPIStatus;

/* 自動処理用 */
typedef struct SPIUnit_tag {
    SPISide   side;
    uint8_t   *write_data;
    uint8_t   *read_data;
    size_t    size;
    uint8_t   write_const;
    size_t    pos;
    SPIStatus status;
    char      wd_NULL;
    char      rd_NULL;
} SPIUnit;

#ifdef __cplusplus
extern "C" {
#endif

/* 初期化・停止 */
void spi_init( SPISide side, SPIMode mode, SPISpeed speed, SPIOrder order, SPIPin pullup, char int_enable );
void spi_release( void );
//SPISpeed spi_bps_to_speed( uint32_t max_bps );
void spi_set_speed( SPISpeed speed );

void spi_enable( char enable );
void spi_enable_int( char enable );

/* 通信 */
char spi_write_collision( void );
char spi_complete( void );
void spi_write( uint8_t data );
uint8_t spi_read( void );
void spi_select_slave( void );
void spi_release_slave( void );

/* ほぼマスター専用自動処理 */
//SPIStatus spi_auto_proccess( void );
//SPIStatus spi_auto_complete( void );
//void spi_auto_master_start( uint8_t *write_data, uint8_t *read_data, size_t size, uint8_t write_const );

#ifdef __cplusplus
}
#endif

#endif
