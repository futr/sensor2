/*
 * HMC5883Lコントロールライブラリー
 *
 *
 *
 */

#ifndef HMC5883L_H_INCLUDED
#define HMC5883L_H_INCLUDED

#include "i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include "debug.h"

typedef enum HMC5883LGain_tag {
    HMC5883L0_88Ga = 0x00 << 5,
    HMC5883L1_3Ga = 0x01 << 5,
    HMC5883L1_9Ga = 0x02 << 5,
    HMC5883L2_5Ga = 0x03 << 5,
    HMC5883L4_0Ga = 0x04 << 5,
    HMC5883L4_7Ga = 0x05 << 5,
    HMC5883L5_6Ga = 0x06 << 5,
    HMC5883L8_1Ga = 0x07 << 5,
} HMC5883LGain;

typedef enum HMC5883LDataAvg_tag {
    HMC5883LAvg1 = 0x00 << 5,
    HMC5883LAvg2 = 0x01 << 5,
    HMC5883LAvg4 = 0x02 << 5,
    HMC5883LAvg8 = 0x03 << 5,
} HMC5883LDataAvg;

typedef enum HMC5883LDataRate_tag {
    HMC5883L0_75Hz = 0x00 << 2,
    HMC5883L1_5Hz  = 0x01 << 2,
    HMC5883L0_3Hz  = 0x02 << 2,
    HMC5883L7_5Hz  = 0x03 << 2,
    HMC5883L15Hz     = 0x04 << 2,
    HMC5883L30Hz     = 0x05 << 2,
    HMC5883L75Hz     = 0x06 << 2,
} HMC5883LDataRate;

typedef struct HMC5883LUnit_tag {
    uint8_t address;
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883LUnit;

#ifdef __cplusplus
extern "C" {
#endif

char hmc5883l_init(HMC5883LUnit *unit, uint8_t address , HMC5883LDataRate rate, HMC5883LDataAvg avg, HMC5883LGain gain );
char hmc5883l_start( HMC5883LUnit *unit );
char hmc5883l_stop( HMC5883LUnit *unit );
char hmc5883l_data_ready( HMC5883LUnit *unit );
char hmc5883l_read( HMC5883LUnit *unit );
#ifdef __cplusplus
}
#endif

#endif
