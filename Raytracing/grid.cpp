#include "grid.h"

void Grid::Build()
{

	// Find the grid bounds
	Vector p0 = find_min_bounds();
	Vector p1 = find_max_bounds();

	bbox = AABB(p0, p1);

	Vector w = p1 - p0; //grid dim

	int num_obj = getNumObjects();
	float s = powf(num_obj / (w.x * w.y * w.z), 1 / 3);

	// Number of cells in each coordinate
	nx = trunc(m * w.x * s) + 1;
	ny = trunc(m * w.y * s) + 1;
	nz = trunc(m * w.z * s) + 1;

	int cell_num = nx * ny * nz; // = m**3 * num_obj

	// Init cell vector
	// Cells stored as 1D array of length Nx * Ny * Nz
	// Array index of cell(ix, iy, iz) is index = ix + Nx * iy + Nx * Ny * iz
	for (int c = 0; c < cell_num; c++) {
		cells.push_back(vector<Object*>());
	}

	AABB obj_bbox; Object* obj;
	for (int j = 0; j < num_obj; j++) {
		obj = getObject(j);
		obj_bbox = obj->GetBoundingBox();

		// Compute indices of both cells that contain min and max coord of obj bbox
		int ixmin = clamp(
			(obj_bbox.min.x - p0.x) * nx / (p1.x - p0.x),
			0, nx - 1);
		int iymin = clamp(
			(obj_bbox.min.y - p0.y) * ny / (p1.y - p0.y),
			0, ny - 1);
		int izmin = clamp(
			(obj_bbox.min.z - p0.z) * nz / (p1.z - p0.z),
			0, nz - 1);

		int ixmax = clamp(
			(obj_bbox.max.x - p0.x) * nx / (p1.x - p0.x),
			0, nx - 1);
		int iymax = clamp(
			(obj_bbox.max.y - p0.y) * ny / (p1.y - p0.y),
			0, ny - 1);
		int izmax = clamp(
			(obj_bbox.max.z - p0.z) * nz / (p1.z - p0.z),
			0, nz - 1);

		int index;

	// insert obj to the overlaped cells
		for (int iz = izmin; iz <= izmax; iz++)
			for (int iy = iymin; iy <= iymax; iy++)
				for (int ix = ixmin; ix <= ixmax; ix++) {
					index = ix + nx * iy + nx * ny * iz;
					cells.at(index).push_back(obj);
				}
	}

}

//Traverse throught the grid find object the was hit 
bool Grid::Traverse(Ray& ray, Object** hitobject, Vector& hitpoint)
{
	//starting cell indices
	int ix, iy, iz;
	double tx_next, ty_next, tz_next;
	double dtx, dty, dtz;
	int ix_step, iy_step, iz_step;
	int ix_stop, iy_stop, iz_stop;
	float t, min_t;

	if (!Init_Traverse(ray, ix, iy, iz, dtx, dty, dtz, tx_next, ty_next, tz_next, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop)) {
		return false;
	}
	
	vector<Object*> cell;
	Object* obj, *min_obj;

	min_obj = NULL;
	min_t = FLT_MAX;
	t = FLT_MAX;

	while (true) {

		cell = cells.at(ix + nx * iy + nx * ny * iz);

		for (int i = 0; i < cell.size(); i++) {
			obj = cell.at(i);

			if (obj->intercepts(ray, t)) {
				if (t < min_t) {
					min_t = t;
					min_obj = obj;
				}
			}
		}
		
		if (tx_next < ty_next && tx_next < tz_next) {
			if (min_obj != NULL && min_t < tx_next) {
				*hitobject = min_obj;
				hitpoint = ray.origin + ray.direction * min_t;
				return true;
			}

			tx_next += dtx;
			ix += ix_step;

			if (ix == ix_stop) {
				return false;
			}

		}
		else if (ty_next < tz_next) {
			if (min_obj != NULL && min_t < ty_next) {
				*hitobject = min_obj;
				hitpoint = ray.origin + ray.direction * min_t;
				return true;
			}

			ty_next += dty;
			iy += iy_step;

			if (iy == iy_stop) {
				return false;
			}
		}
		else {
			if (min_obj != NULL && min_t < tz_next) {
				*hitobject = min_obj;
				hitpoint = ray.origin + ray.direction * min_t;
				return true;
			}

			tz_next += dtz;
			iz += iz_step;

			if (iz == iz_stop) {
				return false;
			}
		}
	}
}

//Traverse used for feeler rays (where we only care if there is intersection or not)
bool Grid::Traverse(Ray& ray)
{
	int ix, iy, iz; //starting cell indices
	double tx_next, ty_next, tz_next;
	double dtx, dty, dtz;
	int ix_step, iy_step, iz_step;
	int ix_stop, iy_stop, iz_stop;
	float t, min_t;

	// Find the start cell of the ray, in which direction it will travel and how much and for how long
	if (!Init_Traverse(ray, ix, iy, iz, dtx, dty, dtz, tx_next, ty_next, tz_next, ix_step, iy_step, iz_step, ix_stop, iy_stop, iz_stop)) {
		return false; // If not inside box
	}

	vector<Object*> cell;
	Object* obj, * min_obj;

	while (true) {
		cell = cells.at(ix + nx * iy + nx * ny * iz);

		for (int i = 0; i < cell.size(); i++) {
			obj = cell.at(i);

			if (obj->intercepts(ray, t)) {
				return true;
			}
		}

		if (tx_next < ty_next && tx_next < tz_next) {
			tx_next += dtx;
			ix += ix_step;

			if (ix == ix_stop) {
				return false;
			}

		}
		else if (ty_next < tz_next) {
			ty_next += dty;
			iy += iy_step;

			if (iy == iy_stop) {
				return false;
			}
		}
		else {
			tz_next += dtz;
			iz += iz_step;

			if (iz == iz_stop) {
				return false;
			}
		}
	}
}


Vector Grid::find_min_bounds() {
	float kEpsilon = 0.0001;
	Vector bbox_min;
	Vector p0 = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
	int num_objects = getNumObjects();

	// find min
	for (int j = 0; j < num_objects; j++) {
		bbox_min = getObject(j)->GetBoundingBox().min;

		if (bbox_min.x < p0.x)
			p0.x = bbox_min.x;
		if (bbox_min.y < p0.y)
			p0.y = bbox_min.y;
		if (bbox_min.z < p0.z)
			p0.z = bbox_min.z;
	}

	p0.x -= kEpsilon;
	p0.y -= kEpsilon;
	p0.z -= kEpsilon;

	return p0;
}

Vector Grid::find_max_bounds(void)
{
	float kEpsilon = 0.0001;
	Vector bbox_max;
	Vector p1 = Vector(FLT_MIN, FLT_MIN, FLT_MIN);
	int num_objects = getNumObjects();

	// find min
	for (int j = 0; j < num_objects; j++) {
		bbox_max = getObject(j)->GetBoundingBox().max;

		if (bbox_max.x > p1.x)
			p1.x = bbox_max.x;
		if (bbox_max.y > p1.y)
			p1.y = bbox_max.y;
		if (bbox_max.z > p1.z)
			p1.z = bbox_max.z;
	}
	p1.x += kEpsilon;
	p1.y += kEpsilon;
	p1.z += kEpsilon;

	return p1;
}

bool Grid::Init_Traverse(Ray& ray, int& ix, int& iy, int& iz, double& dtx, double& dty, double& dtz, double& tx_next, double& ty_next, double& tz_next, int& ix_step, int& iy_step, int& iz_step, int& ix_stop, int& iy_stop, int& iz_stop)
{
	Vector o = ray.origin;
	Vector dir = ray.direction;
	Vector bbox_min = bbox.min;
	Vector bbox_max = bbox.max;
	
	// Where the ray intersets the bbox
	float tx_min = (bbox_min.x - o.x) / ray.direction.x;
	float ty_min = (bbox_min.y - o.y) / ray.direction.y;
	float tz_min = (bbox_min.z - o.z) / ray.direction.z;

	float tx_max = (bbox_max.x - o.x) / ray.direction.x;
	float ty_max = (bbox_max.y - o.y) / ray.direction.y;
	float tz_max = (bbox_max.z - o.z) / ray.direction.z;

	// May be inverted: max must me bigger in value
	if (tx_min > tx_max) swap(tx_max, tx_min);
	if (ty_min > ty_max) swap(ty_max, ty_min);
	if (tz_min > tz_max) swap(tz_max, tz_min);

	// Find the entering and exiting bounding box t's
	float t0 = max(max(tx_min, ty_min), tz_min);
	float t1 = min(min(tx_max, ty_max), tz_max);
	

	if (t0 > t1 || t1 < 0) { //crossover: ray disjoint the Grid’s BB OR leaving point is behind the ray origin
		return false;
	}

	// ray parameter increments per cell in the x, y, and z directions
	dtx = (tx_max - tx_min) / nx;
	dty = (ty_max - ty_min) / ny;
	dtz = (tz_max - tz_min) / nz;

	//Calculate Starting Cell
	//int ix, iy, iz; 
	if (bbox.isInside(o)) { // does the ray start inside the grid?
		ix = clamp((o.x - bbox_min.x) * nx / (bbox_max.x - bbox_min.x), 0, nx - 1);
		iy = clamp((o.y - bbox_min.y) * ny / (bbox_max.y - bbox_min.y), 0, ny - 1);
		iz = clamp((o.z - bbox_min.z) * nz / (bbox_max.z - bbox_min.z), 0, nz - 1);
	}
	else {
		Vector p = o + dir * t0; // initial hit point with grid’s BB
		ix = clamp((p.x - bbox_min.x) * nx / (bbox_max.x - bbox_min.x), 0, nx - 1);
		iy = clamp((p.y - bbox_min.y) * ny / (bbox_max.y - bbox_min.y), 0, ny - 1);
		iz = clamp((p.z - bbox_min.z) * nz / (bbox_max.z - bbox_min.z), 0, nz - 1);
	}

	float dx = dir.x;
	float dy = dir.y;
	float dz = dir.z;

	//A ray has direction (dx, dy, dz).

	//Possible cases for direction xx’:
	if (dx > 0) {
		tx_next = tx_min + (ix + 1) * dtx;
		ix_step = +1;
		ix_stop = nx;
	} else {
		tx_next = tx_min + (nx - ix) * dtx;
		ix_step = -1;
		ix_stop = -1;
	}

	if (dx == 0.0) {
		// Next cell will never be found with this coordinate
		// because ray is paralel to the axis
		tx_next = FLT_MAX;
	}

	//Possible cases for direction yy’:
	if (dy > 0) {
		ty_next = ty_min + (iy + 1) * dty;
		iy_step = +1;
		iy_stop = ny;
	}
	else {
		ty_next = ty_min + (ny - iy) * dty;
		iy_step = -1;
		iy_stop = -1;
	}

	if (dy == 0.0) {
		// Next cell will never be found with this coordinate
		// because ray is paralel to the axis
		ty_next = FLT_MAX;
	}

	//Possible cases for direction zz’:
	if (dz > 0) {
		tz_next = tz_min + (iz + 1) * dtz;
		iz_step = +1;
		iz_stop = nz;
	}
	else {
		tz_next = tz_min + (nz - iz) * dtz;
		iz_step = -1;
		iz_stop = -1;
	}

	if (dz == 0.0) {
		// Next cell will never be found with this coordinate
		// because ray is paralel to the axis
		tz_next = FLT_MAX;
	}

	return true;
}

Grid::Grid(void)
{
}

int Grid::getNumObjects()
{
	return objects.size();
}

void Grid::addObject(Object* o)
{
	objects.push_back(o);
}

Object* Grid::getObject(unsigned int index)
{
	return objects.at(index);
}
