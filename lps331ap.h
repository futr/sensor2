/*
 * LPS331APコントロールライブラリ
 *
 * ブロッキング動作
 * SLA : SDO=0で5C
 * オフセット指定なし，内部での平均化なし
 *
 * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480
 *
 */

#ifndef LPS331AP_H_INCLUDED
#define LPS331AP_H_INCLUDED

#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include "debug.h"

typedef enum LPS331APTempAvg_tag {
    LPS331APTempAvg1   = 0x00,
    LPS331APTempAvg2   = 0x10,
    LPS331APTempAvg4   = 0x20,
    LPS331APTempAvg8   = 0x30,
    LPS331APTempAvg16  = 0x40,
    LPS331APTempAvg32  = 0x50,
    LPS331APTempAvg64  = 0x60,
    LPS331APTempAvg128 = 0x70,
} LPS331APTempAvg;

typedef enum LPS331APPresAvg_tag {
    LPS331APPresAvg1   = 0x00,
    LPS331APPresAvg2   = 0x01,
    LPS331APPresAvg4   = 0x02,
    LPS331APPresAvg8   = 0x03,
    LPS331APPresAvg16  = 0x04,
    LPS331APPresAvg32  = 0x05,
    LPS331APPresAvg64  = 0x06,
    LPS331APPresAvg128 = 0x07,
    LPS331APPresAvg256 = 0x08,
    LPS331APPresAvg384 = 0x09,
    LPS331APPresAvg512 = 0x0A,
} LPS331APPresAvg;

typedef enum LPS331APDataRate_tag {
    LPS331APOneShot         = 0x00,
    LPS331AP1_1Hz           = 0x10,
    LPS331AP7_1Hz           = 0x20,
    LPS331AP12dot5_1Hz      = 0x30,
    LPS331AP25_1Hz          = 0x40,
    LPS331AP7_7Hz           = 0x50,
    LPS331AP12dot5_12dot5Hz = 0x60,
    LPS331AP25_25Hz         = 0x70,
} LPS331APDataRate;

typedef struct LPS331APUnit_tag {
    uint8_t address;
    int32_t pressure;
    int16_t temp;
    uint8_t ctrl_1;
} LPS331APUnit;

#ifdef __cplusplus
extern "C" {
#endif

char lps331ap_init( LPS331APUnit *unit, uint8_t address, LPS331APDataRate rate, LPS331APPresAvg pres_avg, LPS331APTempAvg temp_avg );
char lps331ap_start( LPS331APUnit *unit );
char lps331ap_stop( LPS331APUnit *unit );
char lps331ap_one_shot( LPS331APUnit *unit );
char lps331ap_data_ready( LPS331APUnit *unit );
char lps331ap_temp_data_ready( LPS331APUnit *unit );
char lps331ap_read( LPS331APUnit *unit );
char lps331ap_read_temp( LPS331APUnit *unit );

#ifdef __cplusplus
}
#endif

#endif
