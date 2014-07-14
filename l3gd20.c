#include "l3gd20.h"

char l3gd20_init( L3GD20Unit *unit, uint8_t address , L3GD20DataRate rate, L3GD20FullScale scale )
{
    /* 初期化 */
    uint8_t data;
    unit->address = address;

    /* デバイスID確認 */
    if ( !i2c_read_register( unit->address, 0x0F, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* デバイスIDが間違っていれば失敗 */
    if ( data != 0xD4 ) {
        return 0;
    }

    /* バンド幅とデーターレート設定 */
    data = rate | 0x30 | 0x07;
    unit->ctrl_1 = data;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* フルスケールを設定，データ読み込み中にデーターが更新されないようにしている */
    data = 0x80 | scale;

    if ( !i2c_write_register( unit->address, 0x23, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* FIFOを有効にする */
    data = 0x04;

    if ( !i2c_write_register( unit->address, 0x24, &data, 1, I2CPolling ) ) {
        return 0;
    }


    return 1;
}

char l3gd20_start( L3GD20Unit *unit )
{
    /* パワーダウン解除 */
    uint8_t data;

    data = unit->ctrl_1 | 0x08;

    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char l3gd20_stop( L3GD20Unit *unit )
{
    /* パワーダウンへ */
    uint8_t data;

    data = unit->ctrl_1 & ~0x08;

    /* 修正 */
    if ( !i2c_write_register( unit->address, 0x20, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char l3gd20_read_temp( L3GD20Unit *unit )
{
    /* 温度を読む */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x26, &data, 1, I2CPolling ) ) {
        return 0;
    }

    unit->temp = data;

    return 1;
}

char l3gd20_data_ready( L3GD20Unit *unit )
{
    /* 新しいデーターがあるか確認 */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x27, &data, 1, I2CPolling ) ) {
        return 0;
    }

    if ( data & 0x08 ) {
        return 1;
    }

    return 0;
}

char l3gd20_read( L3GD20Unit *unit )
{
    /* 全軸の測定データー読み込み */
    uint8_t data[6];

    /* マルチバイトリードを行うにはMSBを1にする必要がある */
    if ( !i2c_read_register( unit->address, 0x28 | 0x80, data, 6, I2CPolling ) ) {
        return 0;
    }

    /* なぜか別々にしか読めない? */
    /*
    for ( i = 0; i < 3; i++ ) {
        if ( !i2c_read_register( unit->address, 0x28 + i * 2, data + i * 2, 2, I2CPolling ) ) {
            return 0;
        }
    }
    */

    unit->x = *(uint16_t *)( data + 0 );
    unit->y = *(uint16_t *)( data + 2 );
    unit->z = *(uint16_t *)( data + 4 );

    return 1;
}

