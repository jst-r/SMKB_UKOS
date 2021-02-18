#include "main.h"
#include "seqAnalysis.h"

void init_mask() {
    for (int i = 0; i < buff_size; i++) {
        mask[i] = 0;
        buff[i] = 0;
    }

    mask[0] = 1;
    mask[1] = 1;
    mask[2] = 1;
    int t = 1;
    for (int i = 0; i < n_delays; i++) {
        t += delays[i] * 1000 / time_window;
        mask[t - 1] = 1;
        mask[t] = 1;
        mask[t + 1] = 1;
    }
}

void listen(){
	for (int i = 0; i < buff_size; i++) {
	        max_val = 0;
	        for (int j = 0; j < time_window / sample_time; j++) {
	            // FIXME add actual signal receiving
	            max_val = max(max_val, v);
	        }
	        buff[i] = max_val;
	    }
}

int matched_filter() {
    int max_score = 0;
    int score;
    for (int offset = 0; offset < buff_size; offset++) {
        score = 0;
        for (int i = 0; i < buff_size; i++) {
            score += mask[i] * buff[(i + offset) % buff_size];
        }
        max_score = max(max_score, score);
    }
    return max_score;
}
