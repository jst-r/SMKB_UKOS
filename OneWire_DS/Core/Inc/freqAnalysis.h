#ifndef FREQANALYSIS
#define FREQANALYSIS
#include "main.h"

typedef struct{
	float freq;
	float period;
	float convConst;

	float time;

	float scoreReal;
	float scoreImag;
} freqAnalyser;


freqAnalyser initAnaliser(float freq);
void setFreq(freqAnalyser * a, float freq);
void processSet(freqAnalyser * a, float dt);
float getScoreSquare(freqAnalyser * a);



#endif
