#ifndef SEQ_ANALYSIS_H
#define SEQ_ANALYSIS_H

#include "main.h"

#define sq_buff_size 50
#define time_window 99  // ms
#define sample_time 33    // ms

#define n_sq_delays 2

static uint8_t sq_buff[sq_buff_size];
static uint8_t sq_mask[sq_buff_size];

void listen();
void init_mask();
int matched_filter();

#endif
