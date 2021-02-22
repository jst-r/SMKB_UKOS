#include "seqAnalysis.h"


void init_mask() {
		for (int32_t i = 0; i < buff_size; i++) {
				mask[i] = 0;
				buff[i] = 0;
		}

		mask[0] = 1;
		mask[1] = 1;
		mask[2] = 1;
		int32_t t = 1;
		for (int32_t i = 0; i < n_delays; i++) {
				t += delays[i] * 1000 / time_window;
				mask[t - 1] = 1;
				mask[t] = 1;
				mask[t + 1] = 1;
		}
}

void listen(){
	for (int i = 0; i < buff_size; i++) {
		uint8_t max_val = 0;
		uint8_t v;

		for (int j = 0; j < time_window / sample_time; j++) {
			for (int32_t i = 0; i < buff_size; i++) {
					max_val = 0;
					for (int32_t j = 0; j < time_window / sample_time; j++) {
					// FIXME add actual signal receiving
						if (max_val < v){
							max_val = v;
						}
				}
				buff[i] = max_val;
			}
		}
	}
}

int32_t matched_filter(void) {
		int32_t max_score = 0;
		int32_t score;
		for (int32_t offset = 0; offset < buff_size; offset++) {
				score = 0;
				for (int32_t i = 0; i < buff_size; i++) {
						score += mask[i] * buff[(i + offset) % buff_size];
				}
				if (max_score < score)
				{
					max_score = score;
				}
		}
		return max_score;
}
