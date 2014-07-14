#include "hmc5883l.h"

char hmc5883l_init( HMC5883LUnit *unit, uint8_t address, HMC5883LDataRate rate , HMC5883LDataAvg avg , HMC5883LGain gain )
{
    /* 初期化 */
    uint8_t data;
    unit->address = address;

    /* デバイスID確認 */
    if ( !i2c_read_register( unit->address, 0x0A, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* デバイスIDが間違っていれば失敗 */
    if ( data != 0x48 ) {
        return 0;
    }

    /* 出力レートと平均回数を設定，測定モードはノーマル */
    data = rate | avg;

    if ( !i2c_write_register( unit->address, 0x00, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* ゲインを設定 */
    data = gain;

    if ( !i2c_write_register( unit->address, 0x01, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char hmc5883l_start( HMC5883LUnit *unit )
{
    /* 連続モードで測定開始 */
    uint8_t data;

    data = 0x00;

    if ( !i2c_write_register( unit->address, 0x02, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char hmc5883l_stop( HMC5883LUnit *unit )
{
    /* デバイスをアイドルモードへ */
    uint8_t data;

    data = 0x03;

    if ( !i2c_write_register( unit->address, 0x02, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char hmc5883l_data_ready( HMC5883LUnit *unit )
{
    /* データが準備できているか確認 */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x09, &data, 1, I2CPolling ) ) {
        return 0;
    }

    if ( data & 0x01 ) {
        return 1;
    }

    return 0;
}

char hmc5883l_read( HMC5883LUnit *unit )
{
    /* 読み出し */
    uint8_t data[6];

    if ( !i2c_read_register( unit->address, 0x03, data, 6, I2CPolling ) ) {
        return 0;
    }

    /* 環境依存動作に依存してるかも？ */
    unit->x = ( ( (uint16_t)data[0] ) << 8 ) + data[1];
    unit->z = ( ( (uint16_t)data[2] ) << 8 ) + data[3];
    unit->y = ( ( (uint16_t)data[4] ) << 8 ) + data[5];

    return 1;
}
