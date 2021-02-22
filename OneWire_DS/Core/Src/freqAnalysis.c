#include "freqAnalysis.h"
#include "main.h"
#include <math.h>


static const float PI = 3.14159265358979;

freqAnalyser initAnaliser(float freq){
	freqAnalyser res;
	res.time = 0;
	res.freq = freq;
	res.period = 1/freq;
	res.convConst = 2 * PI * freq;

	res.scoreReal = 0;
	res.scoreImag = 0;

	return res;
}

void processSet(freqAnalyser * a, float dt){
	a->time += (float)dt / 1000.;
	a->time = fmod(a->time, a->period);
	a->scoreReal += cosf(a->time * a->convConst);
	a->scoreImag += sinf(a->time * a->convConst);
}

float getScoreSquare(freqAnalyser * a){
	return powf(a->scoreReal, 2) + powf(a->scoreImag, 2);
}


