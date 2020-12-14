#ifndef FREQANALYSIS
#define FREQANALYSIS


typedef struct{
	float freq;
	float period;
	float convConst;

	float time;

	float scoreReal;
	float scoreImag;
} freqAnaliser;

freqAnaliser initAnaliser(float freq);
void processSet(freqAnaliser * a, uint16_t resetLenght);
void processReset(freqAnaliser * a, uint16_t setLenght);
int getScoreSquare(freqAnaliser * a);
#endif
