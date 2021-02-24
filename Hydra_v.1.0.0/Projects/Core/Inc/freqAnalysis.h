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
void processSet(freqAnaliser * a, float dt);
void processReset(freqAnaliser * a, uint16_t setLenght);
float getScoreSquare(freqAnaliser * a);
void resetScore(freqAnaliser * a);
#endif
