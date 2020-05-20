#include "vector.h"
#include "boundingBox.h"
#include "scene.h"

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
	t0 = MAX3(tx_min, ty_min, tz_min);

	//smallest exiting t value
	t1 = MIN3(tx_max, ty_max, tz_max);
	
	t = (t0 < 0) ? t1 : t0;

	return (t0 < t1 && t1 > 0.0001);
}

Vector AABB::centroid(void) {
	return (min + max) / 2;
}

void AABB::extend(AABB box) {
	if (min.x > box.min.x) min.x = box.min.x;
	if (min.y > box.min.y) min.y = box.min.y;
	if (min.z > box.min.z) min.z = box.min.z;

	if (max.x < box.max.x) max.x = box.max.x;
	if (max.y < box.max.y) max.y = box.max.y;
	if (max.z < box.max.z) max.z = box.max.z;
}