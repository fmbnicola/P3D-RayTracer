#include "vector.h"
#include "maths.h"

// Sampling with rejection method
Vector sample_unit_disk(void) {
	Vector p;
	do {
		p = Vector(rand_float(), rand_float(), 0.0) * 2 - Vector(1.0, 1.0, 0.0);
	} while (p*p >= 1.0);
	return p;
}