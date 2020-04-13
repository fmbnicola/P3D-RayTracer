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
	float o_x = ray.origin.x;
	float o_y = ray.origin.y;
	float o_z = ray.origin.z;

	float d_x = ray.direction.x;
	float d_y = ray.direction.y;
	float d_z = ray.direction.z;

	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	float a = 1.0f / d_x;
	if (a >= 0) {
		tx_min = (min.x - o_x) * a;
		tx_max = (max.x - o_x) * a;
	}
	else {
		tx_min = (max.x - o_x) * a;
		tx_max = (min.x - o_x) * a;
	}

	float b = 1.0f / d_y;
	if (b >= 0) {
		ty_min = (min.y - o_y) * b;
		ty_max = (max.y - o_y) * b;
	}
	else {
		ty_min = (max.y - o_y) * b;
		ty_max = (min.y - o_y) * b;
	}

	float c = 1.0f / d_z;
	if (c >= 0) {
		tz_min = (min.z - o_z) * c;
		tz_max = (max.z - o_z) * c;
	}
	else {
		tz_min = (max.z - o_z) * c;
		tz_max = (min.z - o_z) * c;
	}

	float t0, t1;

	//largest entering t value
	if (tx_min > ty_min) {
		t0 = tx_min;
	}
	else {
		t0 = ty_min;
	}

	if (tz_min > t0) 
		t0 = tz_min;

	//smallest exiting t value
	if (tx_max < ty_max) {
		t1 = tx_max;
	}
	else {
		t1 = ty_max;
	}

	if (tz_max < t1)
		t1 = tz_max;
	
	if (t0 < 0) {
		t = t1;
	}
	else {
		t = t0;
	}

	return (t0 < t1 && t1 > 0.0001);
}
#endif