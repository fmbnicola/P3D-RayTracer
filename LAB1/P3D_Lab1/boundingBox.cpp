#ifndef AABB_H
#define AABB_H

#include "vector.h"
#include "boundingBox.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void) 
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector& v0, const Vector& v1)
{
	min = v0; max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB& bbox) 
{
	min = bbox.min; max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator= (const AABB& rhs) {
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// --------------------------------------------------------------------- inside
// used to test if a ray starts inside a grid

bool AABB::isInside(const Vector& p) 
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}

bool AABB::intercepts(const Ray& ray, float& t)
{
	
	return true;
}
#endif