#include "seqAnalysis.h"
#include "OneWire_Hi4Tech.h"
#include "main.h"
#include "stdio.h"
extern UART_HandleTypeDef huart3;
char huart3buf[16] = {0}; 

static const uint8_t sq_delays[n_sq_delays] = {5,5,2,2,2,10,10};


void init_mask() {
		for (int32_t i = 0; i < sq_buff_size; i++) {
				sq_mask[i] = 0;
				sq_buff[i] = 0;
		}

		sq_mask[0] = 1;
		sq_mask[1] = 1;
		sq_mask[2] = 1;
		int32_t t = 1;
		for (int32_t i = 0; i < n_sq_delays; i++) {
				t += sq_delays[i] * 1000 / time_window;
				sq_mask[t - 1] = 1;
				sq_mask[t] = 1;
				sq_mask[t + 1] = 1;
		}
}

void listen(){
		uint8_t max_val = 0;
		int8_t v;

			for (int32_t i = 0; i < sq_buff_size; i++) {
					max_val = 0;
					for (int32_t j = 0; j < time_window / sample_time; j++) {
						v = run_OW() < 2000;
						HAL_UART_Transmit(&huart3, (uint8_t*)huart3buf, sprintf(huart3buf," %d\n", v), 25);							
						if (max_val < v){
							max_val = v;
						}
				}
				sq_buff[i] = max_val;
			}
	}

int32_t matched_filter(void) {
		int32_t max_score = 0;
		int32_t score;
		for (int32_t offset = 0; offset < sq_buff_size; offset++) {
				score = 0;
				for (int32_t i = 0; i < sq_buff_size; i++) {
						score += sq_mask[i] * sq_buff[(i + offset) % sq_buff_size];
				}
				if (max_score < score)
				{
					max_score = score;
				}
		}
		return max_score;
}
