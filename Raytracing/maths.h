#ifndef __MATHS__
#define __MATHS__

#include <stdlib.h>

// prototypes

double min(double x0, double x1);

double max(double x0, double x1);

double clamp(const double x, const double min, const double max);

int	rand_int(void);

float rand_float(void);

void set_rand_seed(const int seed);


// inlined functions

//--------------------------------------------------------float to min
inline unsigned int
float_to_int(double x) {
	return ((x) >= 0 ? (unsigned int)((x)+0.5) : (unsigned int)((x)-0.5));
}

// ----------------------------------------------------------------- min

inline double
min(double x0, double x1) {
	return ((x0 < x1) ? x0 : x1);
}


// ----------------------------------------------------------------- max

inline double
max(double x0, double x1) {
	return ((x0 > x1) ? x0 : x1);
}

// ---------------------------------------------------- clamp

inline double
clamp(const double x, const double min, const double max) {
	return (x < min ? min : (x > max ? max : x));
}

inline int
clampint(const float x, const int min, const int max) {
	return (x < min ? min : (x > max ? max : (int) round(x)));
}

// ---------------------------------------------------- rand_int
// a wrapper for rand()

inline int
rand_int(void) {
	return(rand());
}


// ---------------------------------------------------- rand_float

inline float
rand_float(void) {
	return((float)rand() / (float)RAND_MAX);
}


// ---------------------------------------------------- set_rand_seed

inline void
set_rand_seed(const int seed) {
	srand(seed);
}

// ---------------------------------------------------- float to byte (unsigned char)
inline uint8_t u8fromfloat(float x)
{
	//return (uint8_t)(x * 255.99f);
	return ((x * 255.99f) >= 255.0f ? 255 : (uint8_t)(x * 255.99f));

}

// ---------------------------------------------------- byte (unsigned char) to float
inline  float u8tofloat(uint8_t x)
{
	return (float)(x / 255.99f);
}

#endif
