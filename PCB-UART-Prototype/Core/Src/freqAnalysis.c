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

void processSet(freqAnalyser * a, uint16_t resetLenght){
	a->time += (float)resetLenght / 10.;
	a->time = fmod(a->time, a->period);
	a->scoreReal += arm_cos_f32(a->time * a->convConst);
	a->scoreImag += arm_sin_f32(a->time * a->convConst);
}

void processReset(freqAnalyser * a, uint16_t setLenght){
	a->time += (float)setLenght / 10.;
	a->time = fmod(a->time, a->period);
}

uint32_t getScoreSquare(freqAnalyser * a){
	return powf(a->scoreReal, 2) + powf(a->scoreImag, 2);
}

/*!
Initializes multiple freq analysers

There will be (1 + nSteps * 2) analysers initialised.
Frequencies of analysers are rootFreq + (i - nSteps) * deltaFreq where i is in [0...(1 + nSteps * 2))
*/
multibandAnalyser initmultibandAnalyser
(int16_t nSteps, float32_t rootFreq, float32_t deltaFreq){
	
	multibandAnalyser res;
	res.nAnalisers = (1 + nSteps * 2);
	res.rootFreq = rootFreq;
	res.deltaFreq = deltaFreq;
	res.nSteps = nSteps;
	
	/// WARNING may be fatal if number analysers is too large
	/// TODO find something better than malloc
	res.analisers = (freqAnalyser*) malloc(sizeof(multibandAnalyser) * res.nAnalisers); /// allocates memory for analysers
	
	for (int16_t i = 0; i < res.nAnalisers; i++){
		res.analisers[i] = initAnaliser(rootFreq + (i - nSteps) * deltaFreq);
	}
}

void setRootFreq(multibandAnalyser * a, float32_t rootFreq){
	res->rootFreq = rootFreq;
	
	for (int16_t i = 0; i < res->nAnalisers; i++){
		res->analisers[i] = initAnaliser(rootFreq + (i - res->nSteps) * res->deltaFreq);
	}
}

void processSet(multibandAnalyser * a, uint16_t resetLenght){
	for (int16_t i = 0; i < res->nAnalisers; i++){
		processSet(*(res->analisers[i]), resetLenght);
	}
}

void processReset(multibandAnalyser * a, uint16_t setLenght);
	for (int16_t i = 0; i < res->nAnalisers; i++){
		processReset(*(res->analisers[i]), setLenght);
	}
}

uint32_t getScoreSquare(multibandAnalyser * a){
	uint32_t res = 0;
	for (int16_t i = 0; i < res->nAnalisers; i++){
		getScoreSquare(*(res->analisers[i]));
	}
	return res;
}



