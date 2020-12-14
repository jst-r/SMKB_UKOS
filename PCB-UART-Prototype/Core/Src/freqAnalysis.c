#include "freqAnalysis.h"
#include "main.h"
#include <math.h>

static const float PI = 3.14159265358979;

freqAnaliser initAnaliser(float freq){
	freqAnaliser res;
	res.time = 0;
	res.freq = freq;
	res.period = 1/freq;
	res.convConst = 2 * PI * freq;

	res.scoreReal = 0;
	res.scoreImag = 0;

	return res;
}

void processSet(freqAnaliser * a, uint16_t resetLenght){
	a->time += (float)resetLenght / 10.;
	a->time = fmod(a->time, a->period);
	a->scoreReal += cosf(a->time * a->convConst);
	a->scoreImag += sinf(a->time * a->convConst);
}

void processReset(freqAnaliser * a, uint16_t setLenght){
	a->time += (float)setLenght / 10.;
	a->time = fmod(a->time, a->period);
}

int getScoreSquare(freqAnaliser * a){
	return powf(a->scoreReal, 2) + powf(a->scoreImag, 2);
}
