/*
 * L3GD20コントロールライブラリ
 *
 * ブロッキング動作
 * SLA : SDO=0で6A
 *
 * ハイパスは使用せず，バンド幅(BW)は最高に設定
 *
 */

#ifndef L3GD20_H_INCLUDED
#define L3GD20_H_INCLUDED

#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include "debug.h"

typedef enum L3GD20FullScale_tag {
    L3G2D20_250dps  = 0x00,
    L3G2D20_500dps  = 0x10,
    L3G2D20_2000dps = 0x20,
} L3GD20FullScale;

typedef enum L3GD20DataRate_tag {
    L3GD20_95Hz  = 0x00,
    L3GD20_190Hz = 0x40,
    L3GD20_380Hz = 0x80,
    L3GD20_760Hz = 0xC0,
} L3GD20DataRate;

typedef struct L3GD20Unit_tag {
    uint8_t address;
    int16_t x;
    int16_t y;
    int16_t z;
    int8_t temp;
    uint8_t ctrl_1;
} L3GD20Unit;

#ifdef __cplusplus
extern "C" {
#endif

char l3gd20_init( L3GD20Unit *unit, uint8_t address, L3GD20DataRate rate, L3GD20FullScale scale );
char l3gd20_start( L3GD20Unit *unit );
char l3gd20_stop( L3GD20Unit *unit );
char l3gd20_data_ready( L3GD20Unit *unit );
char l3gd20_read( L3GD20Unit *unit );
char l3gd20_read_temp( L3GD20Unit *unit );

#ifdef __cplusplus
}
#endif

#endif
