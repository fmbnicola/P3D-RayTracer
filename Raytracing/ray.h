#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir, int ix = 0, int jx = 0) : origin(o), direction(dir), i(ix), j(jx) {};

	Vector origin;
	Vector direction;
	int i, j;

	Vector getDirection() {
		return direction.normalize();
	}
};
#endif