/*
 * MMC制御ライブラリ
 *
 * 初期化関数はブロッキングします
 * 初期化中はSPIを占有し，現在のSPIの設定をすべて破棄してしまうので注意が必要です
 *
 * spi.hがあるのが前提です
 *
 * TODO CSD
 *
 */

#ifndef SD_H_INCLUDED
#define SD_H_INCLUDED

#include <avr/io.h>
#include <stdint.h>
#include <util/crc16.h>
#include "spi.h"
#include "debug.h"

typedef enum SDResp_tag {
    SDRespIdleState          = 1 << 0,
    SDRespEraseReset         = 1 << 1,
    SDRespIlligalCommand     = 1 << 2,
    SDRespCommandCRCError    = 1 << 3,
    SDRespEraseSequenceError = 1 << 4,
    SDRespAddressError       = 1 << 5,
    SDRespParameterError     = 1 << 6,
    SDRespWorking            = 1 << 7,
    SDRespSuccess            = 1 << 8,
    SDRespFailed             = 1 << 9,
} SDResp;

typedef enum SDVersion_tag {
    SDV1,
    SDV2,
} SDVersion;

typedef enum SDAddress_tag {
    SDByte,
    SDBlock,
} SDAddress;

typedef struct SDUnit_tag {
    uint16_t block_size;
    SDVersion version;
    SDAddress address;
    uint8_t cmd;
    uint32_t ret;
    uint64_t card_size;		/* card_size[Bytes] */
} SDUnit;

#ifdef __cplusplus
extern "C" {
#endif

/* 初期化等 */
char sd_init( SPISpeed max_speed, uint16_t block_size , SPIPin pullup );
SDAddress sd_get_address_mode( void );
SDVersion sd_get_version( void );
uint64_t sd_get_size( void );
uint16_t sd_get_block_size( void );

/* 送受信 */
char sd_block_write( uint32_t address, uint8_t *data, size_t offset, size_t size );
char sd_block_read( uint32_t address, uint8_t *data, size_t offset, size_t size );

char sd_start_step_block_write( uint32_t address );
void sd_step_block_write( uint8_t data );
char sd_stop_step_block_write( void );

char sd_start_step_block_read( uint32_t address );
uint8_t sd_step_block_read( void );
char sd_stop_step_block_read( void );

// void sd_write( uint32_t address, uint8_t *data, uint32_t size );
// void sd_read( uint32_t address, uint8_t *data, uint32_t size );

/* コマンド発行 */
void sd_command( uint8_t cmd, uint32_t arg );
SDResp sd_response( void );
uint32_t sd_get_returned( void );


#ifdef __cplusplus
}
#endif

#endif
