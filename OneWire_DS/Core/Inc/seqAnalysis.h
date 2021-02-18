#ifndef SEQ_ANALYSIS_H
#define SEQ_ANALYSIS_H

#define buff_size 500
#define time_window 100  // ms
#define sample_time 1    // ms

#define n_delays 5

const unsigned char delays[] = {1, 2, 3, 4};
unsigned char buff[buff_size];
unsigned char mask[buff_size];

void listen();
void init_mask();
int matched_filter();

#endif
