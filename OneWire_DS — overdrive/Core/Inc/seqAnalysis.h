#ifndef SEQ_ANALYSIS_H
#define SEQ_ANALYSIS_H

#include "main.h"

#define buff_size 500
#define time_window 100  // ms
#define sample_time 1    // ms

#define n_delays 5

const uint8_t delays[] = {1, 2, 3, 4};
uint8_t buff[buff_size];
uint8_t mask[buff_size];

void listen();
void init_mask();
int matched_filter();

#endif
