/*
 * ADXL345 I2Cコントローラー
 * ブロッキング動作
 *
 * 100k-400k
 * 最大テーター転送レートは200-800Hz
 *
 * POWER_CTL 0x2D Measureビットセット - クリアでスタンバイモード
 * DATA_READY 0x32 - 0x37　読むとクリア
 *
 */

#ifndef ADXL345_H_INCLUDED
#define ADXL345_H_INCLUDED

#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include "debug.h"

typedef enum ADXLRange_tag {
    ADXLRange2  = 0x00,
    ADXLRange4  = 0x01,
    ADXLRange8  = 0x02,
    ADXLRange16 = 0x03,
} ADXLRange;

typedef enum ADXLDataRate_tag {
    ADXL3200Hz = 0x0F,
    ADXL1600Hz = 0x0E,
    ADXL800Hz  = 0x0D,
    ADXL400Hz  = 0x0C,
    ADXL200Hz  = 0x0B,
    ADXL100Hz  = 0x0A,
    ADXL50Hz   = 0x09,
    ADXL25Hz   = 0x08,
} ADXLDataRate;

typedef enum ADXLResolution_tag {
    ADXLResolutionFull,
    ADXLResolutionFix,
} ADXLResolution;

typedef struct ADXLUnit_tag {
    uint8_t address;
    int16_t x;
    int16_t y;
    int16_t z;
} ADXLUnit;

#ifdef __cplusplus
extern "C" {
#endif

char adxl_init( ADXLUnit *unit, uint8_t address, ADXLRange range, ADXLResolution resolution, ADXLDataRate rate );
char adxl_start( ADXLUnit *unit );
char adxl_stop( ADXLUnit *unit );
char adxl_data_ready( ADXLUnit *unit );
char adxl_read( ADXLUnit *unit );

/*
uint16_t adxl_get_x( ADXLUnit *unit );
uint16_t adxl_get_y( ADXLUnit *unit );
uint16_t adxl_get_z( ADXLUnit *unit );
*/

#ifdef __cplusplus
}
#endif

#endif
