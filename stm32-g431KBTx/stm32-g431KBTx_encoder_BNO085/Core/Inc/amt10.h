#ifndef __AMT10_H__
#define __AMT10_H__

#include "main.h"
#include <stdint.h>

#define ENCODER_PPR 2048

void get_encoder_count(void);
float calc_RPM(uint16_t now, uint16_t *prev_val);

#endif