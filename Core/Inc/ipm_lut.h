#ifndef __IPM_LUT_H
#define __IPM_LUT_H

#include <stdint.h>

#define IPM_BEV_ROWS    120
#define IPM_BEV_COLS    188
#define IPM_SRC_W       188
#define IPM_SRC_H       120

extern const uint16_t ipm_lut[IPM_BEV_ROWS][IPM_BEV_COLS];

#endif /* __IPM_LUT_H */
