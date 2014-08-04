#include "micomfs_dev.h"
#include "sd.h"

char micomfs_dev_get_info( MicomFS *fs, uint16_t *sector_size, uint32_t *sector_count )
{
    /* ファイルシステムに必要な情報を返す */
    *sector_size  = sd_get_block_size();
    *sector_count = sd_get_size() / *sector_size;

    return 1;
}

char micomfs_dev_start_write( MicomFS *fs, uint32_t sector )
{
    /* セクターライト開始 */
    uint32_t address;

    /* アドレス作成 */
    if ( sd_get_address_mode() == SDByte ) {
        address = sector * fs->dev_sector_size;
    } else {
        address = sector;
    }

    return sd_start_step_block_write( address );
}

char micomfs_dev_write( MicomFS *fs, const void *src, uint16_t count )
{
    /* 1バイト書き込み */
    uint32_t i;

    for ( i = 0; i < count; i++ ) {
        sd_step_block_write( ( (uint8_t *)src )[i] );
    }

    return 1;
}

char micomfs_dev_stop_write( MicomFS *fs )
{
    /* セクターライト終了 */
    return sd_stop_step_block_write();
}

char micomfs_dev_start_read( MicomFS *fs, uint32_t sector )
{
    /* セクターリード開始 */
    uint32_t address;

    /* アドレス作成 */
    if ( sd_get_address_mode() == SDByte ) {
        address = sector * fs->dev_sector_size;
    } else {
        address = sector;
    }

    return sd_start_step_block_read( address );
}

char micomfs_dev_read( MicomFS *fs, void *dest, uint16_t count )
{
    /* 1バイト読み込み */
    uint32_t i;

    for ( i = 0; i < count; i++ ) {
        ( (uint8_t *)dest )[i] = sd_step_block_read();
    }

    return 1;
}

char micomfs_dev_stop_read( MicomFS *fs )
{
    /* セクターリード終了 */
    return sd_stop_step_block_read();
}
