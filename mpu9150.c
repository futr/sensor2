#include "mpu9150.h"


char mpu9150_init( MPU9150Unit *unit, uint8_t address,
                   uint8_t sample_rate_divider, MPU9150LPFCFG lpf_cfg,
                   MPU9150AccFSR acc_fsr, MPU9150AccHPF acc_hpf, MPU9150GyroFSR gyro_fsr )
{
    /* MPU9150初期化 */
    uint8_t data;
    unit->address = address;

    /* デバイスID確認 */
    if ( !i2c_read_register( unit->address, 0x75, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* デバイスIDが間違っていれば失敗 */
    if ( data != 0x68 ) {
        return 0;
    }

    /* リセット解除 クロックソースをジャイロ1に */
    data = 0x01;

    if ( !i2c_write_register( unit->address, 0x6B, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* サンプルレートディバイダー設定 Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
    data = sample_rate_divider;

    if ( !i2c_write_register( unit->address, 0x19, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* LPF設定 */
    data = lpf_cfg;

    if ( !i2c_write_register( unit->address, 0x1A, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* ジャイロFSR設定 */
    data = gyro_fsr << 3;

    if ( !i2c_write_register( unit->address, 0x1B, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* 加速度FSRとハイパス設定 */
    data = ( acc_fsr << 3 ) | ( acc_hpf );

    if ( !i2c_write_register( unit->address, 0x1C, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* 地磁気センサーへの通信有効 */
    data = 0x02;

    if ( !i2c_write_register( unit->address, 0x37, &data, 1, I2CPolling ) ) {
        return 0;
    }

    /* データレディー割り込み（ビット）有効 */
    data = 0x01;

    if ( !i2c_write_register( unit->address, 0x38, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char mpu9150_sleep( MPU9150Unit *unit )
{
    /* スリープモードへ */
    uint8_t data;

    data = 0x41;

    if ( !i2c_write_register( unit->address, 0x6B, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return 1;
}

char mpu9150_read( MPU9150Unit *unit )
{
    /* データを読む */
    uint8_t data[14];

    if ( !i2c_read_register( unit->address, 0x3B, data, 14, I2CPolling ) ) {
        return 0;
    }

    /* 構造体に振り分け */
    unit->acc_x  = ( data[0]  << 8 ) | data[1];
    unit->acc_y  = ( data[2]  << 8 ) | data[3];
    unit->acc_z  = ( data[4]  << 8 ) | data[5];
    unit->temp   = ( data[6]  << 8 ) | data[7];
    unit->gyro_x = ( data[8]  << 8 ) | data[9];
    unit->gyro_y = ( data[10] << 8 ) | data[11];
    unit->gyro_z = ( data[12] << 8 ) | data[13];

    return 1;
}

char mpu9150_data_ready( MPU9150Unit *unit )
{
    /* データレデイー */
    uint8_t data;

    if ( !i2c_read_register( unit->address, 0x3A, &data, 1, I2CPolling ) ) {
        return 0;
    }

    return data & 0x01;
}


float mpu9150_get_temp_in_c( MPU9150Unit *unit )
{
    /* 構造体内のtempを度にして返す */
    /* Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 35 */
    return unit->temp / 340.0 + 35;
}
