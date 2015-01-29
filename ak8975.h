#ifndef AK8975_H_INCLUDED
#define AK8975_H_INCLUDED

#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t address;
    int16_t x, y, z;
    uint8_t coeff_x;
    uint8_t coeff_y;
    uint8_t coeff_z;
    int16_t adj_x;
    int16_t adj_y;
    int16_t adj_z;
} AK8975Unit;

char ak8975_init( AK8975Unit *unit, uint8_t address );
char ak8975_start( AK8975Unit *unit );
char ak8975_data_ready( AK8975Unit *unit );
char ak8975_over_flow( AK8975Unit *unit );
char ak8975_read( AK8975Unit *unit );
void ak8975_calc_adjusted_h( AK8975Unit *unit );
char ak8975_self_test( AK8975Unit *unit );

#ifdef __cplusplus
}
#endif

#endif
