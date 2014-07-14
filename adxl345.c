#include "adxl345.h"

char adxl_init( ADXLUnit *unit, uint8_t address, ADXLRange range, ADXLResolution resolution , ADXLDataRate rate )
{
    /* 初期化 */
    uint8_t data;
    unit->address = address;

    /* デバイスID確認 */
    if ( !i2c_read_register( unit->address, 0x00, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* デバイスIDが間違っていれば失敗 */
    if ( data != 0xE5 ) {
        return 0;
    }

    /* 測定レンジ等設定 */
    data = 0x00;

    if ( resolution == ADXLResolutionFull ) {
        data |= 0x08;
    }

    data |= range;
    if ( !i2c_write_register( unit->address, 0x31, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* データレート設定 */
    data = 0x0F & rate;
    if ( !i2c_write_register( unit->address, 0x2C, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char adxl_start( ADXLUnit *unit )
{
    /* 測定開始 */
    uint8_t data = 0x08;

    if ( !i2c_write_register( unit->address, 0x2D, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char adxl_stop( ADXLUnit *unit )
{
    /* 測定停止 */
    uint8_t data = 0x00;

    if ( !i2c_write_register( unit->address, 0x2D, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char adxl_data_ready( ADXLUnit *unit )
{
    /* データーが準備できているか確認 */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x30, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return data & 0x80;
}

char adxl_read( ADXLUnit *unit )
{
    /* 全軸の測定データー読み込み */
    uint8_t data[6];

    if ( !i2c_read_register( unit->address, 0x32, data, 6, I2CPolling ) ) {
        return 0;
    }

    unit->x = *(uint16_t *)( data + 0 );
    unit->y = *(uint16_t *)( data + 2 );
    unit->z = *(uint16_t *)( data + 4 );

    return 1;
}
