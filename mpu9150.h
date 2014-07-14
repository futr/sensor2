#ifndef MPU9150_H_INCLUDED
#define MPU9150_H_INCLUDED

#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MPU9150GyroFSR250DPS  = 0x00,
    MPU9150GyroFSR500DPS  = 0x01,
    MPU9150GyroFSR1000DPS = 0x02,
    MPU9150GyroFSR2000DPS = 0x03,
} MPU9150GyroFSR;

typedef enum {
    MPU9150AccFSR2g  = 0x00,
    MPU9150AccFSR4g  = 0x01,
    MPU9150AccFSR8g  = 0x02,
    MPU9150AccFSR16g = 0x03,
} MPU9150AccFSR;

typedef enum {
    MPU9150AccHPFReset   = 0x00,
    MPU9150AccHPF5Hz     = 0x01,
    MPU9150AccHPF2_5Hz   = 0x02,
    MPU9150AccHPF1_25Hz  = 0x03,
    MPU9150AccHPF0_63Hz  = 0x04,
    MPU9150AccHPFHold    = 0x07,
} MPU9150AccHPF;

typedef enum {
    MPU9150LPFCFG0 = 0x00,
    MPU9150LPFCFG1 = 0x01,
    MPU9150LPFCFG2 = 0x02,
    MPU9150LPFCFG3 = 0x03,
    MPU9150LPFCFG4 = 0x04,
    MPU9150LPFCFG5 = 0x05,
    MPU9150LPFCFG6 = 0x06,
    MPU9150LPFCFG7 = 0x07,
} MPU9150LPFCFG;

typedef struct {
    uint8_t address;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} MPU9150Unit;

char mpu9150_init( MPU9150Unit *unit, uint8_t address, uint8_t sample_rate_divider, MPU9150LPFCFG lpf_cfg,
                   MPU9150AccFSR acc_fsr, MPU9150AccHPF acc_hpf, MPU9150GyroFSR gyro_fsr );
char mpu9150_sleep( MPU9150Unit *unit );
char mpu9150_data_ready( MPU9150Unit *unit );
char mpu9150_read( MPU9150Unit *unit );

float mpu9150_get_temp_in_c( MPU9150Unit *unit );

#ifdef __cplusplus
}
#endif

#endif
