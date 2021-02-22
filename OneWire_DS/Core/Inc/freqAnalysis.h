#ifndef FREQANALYSIS
#define FREQANALYSIS
#include "main.h"
#include "arm_math.h"

typedef struct{
	float32_t freq;
	float32_t period;
	float32_t convConst;

	float32_t time;

	float32_t scoreReal;
	float32_t scoreImag;
} freqAnalyser;


freqAnalyser initAnaliser(float32_t freq);
void setFreq(freqAnalyser * a, float32_t freq);
void processSet(freqAnalyser * a, uint16_t resetLenght);
void processReset(freqAnalyser * a, uint16_t setLenght);
int getScoreSquare(freqAnalyser * a);


typedef struct{
	float32_t root_freq;
	float32_t delta_freq;
	const uint16_t n_steps;
	
	freqAnalyser* analisers;
	const size_t nAnalisers;
} multibandAnalyser;

multibandAnalyser initmultibandAnalyser(size_t nAnalisers, float32_t root_freq, float32_t delta_freq);
void setRootFreq(multibandAnalyser * a, float32_t freq);

void processSet(multibandAnalyser * a, uint16_t resetLenght);
void processReset(multibandAnalyser * a, uint16_t setLenght);
uint32_t getScoreSquare(multibandAnalyser * a);



#endif
