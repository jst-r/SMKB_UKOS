#ifndef FREQANALYSIS
#define FREQANALYSIS
#include "main_F302.h"
#include "stdint.h"

typedef struct{
	float freq;
	float period;
	float convConst;

	float time;

	float scoreReal;
	float scoreImag;
} freqAnaliser;

freqAnaliser initAnaliser(float freq);
void processSet(freqAnaliser * a, float resetLenght);
float getScoreSquare(freqAnaliser * a);
void resetScore(freqAnaliser * a);
#endif
