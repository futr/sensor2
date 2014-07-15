#include "ak8975.h"

char ak8975_init( AK8975Unit *unit, uint8_t address )
{
    /* init and test */
    uint8_t data;

    unit->address = address;

    /* Read device ID */
    if ( !i2c_read_register( unit->address, 0x00, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* check ID */
    if ( data != 0x48 ) {
        return 0;
    }

    /* ヒューズROMアクセス */
    data = 0x0F;

    if ( !i2c_write_register( unit->address, 0x0A, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* 工場出荷時の調整値を読み込み */
    if ( !i2c_read_register( unit->address, 0x10, &unit->coeff_x, 1, I2CPolling ) ) {
        return 0;
    }

    if ( !i2c_read_register( unit->address, 0x11, &unit->coeff_y, 1, I2CPolling ) ) {
        return 0;
    }

    if ( !i2c_read_register( unit->address, 0x12, &unit->coeff_z, 1, I2CPolling ) ) {
        return 0;
    }

    /* モード0へ */
    data = 0x00;

    if ( !i2c_write_register( unit->address, 0x0A, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* success */
    return 1;
}

char ak8975_start( AK8975Unit *unit )
{
    /* 測定開始 */
    uint8_t data;

    /* 測定開始 */
    data = 0x01;

    if ( !i2c_write_register( unit->address, 0x0A, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char ak8975_data_ready( AK8975Unit *unit )
{
    /* データレディー */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x02, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return data;
}

char ak8975_over_flow( AK8975Unit *unit )
{
    /* 磁気センサが|X|+|Y|+|Z| >= 2400μTになった */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x09, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return data & 0x08;
}

char ak8975_read( AK8975Unit *unit )
{
    /* 測定結果読む */
    int16_t data[3];

    if ( !i2c_read_register( unit->address, 0x03, data, 6, I2CPolling ) ) {
        return 0;
    }

    /* 分配 */
    unit->x = data[0];
    unit->y = data[1];
    unit->z = data[2];

    return 1;
}

void ak8975_calc_adjusted_h( AK8975Unit *unit )
{
    /* 補正値を使って補正済みHを計算 */
    unit->adj_x = ( ( ( unit->coeff_x - 128 ) * 0.5 ) / 128 + 1 ) * unit->x;
    unit->adj_y = ( ( ( unit->coeff_y - 128 ) * 0.5 ) / 128 + 1 ) * unit->y;
    unit->adj_z = ( ( ( unit->coeff_z - 128 ) * 0.5 ) / 128 + 1 ) * unit->z;
}
