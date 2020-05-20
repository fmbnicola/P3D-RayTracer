#include <vector>
#include <cmath>
#include <IL/il.h>
using namespace std;

#include "camera.h"
#include "color.h"
#include "vector.h"
#include "ray.h"
#include "boundingBox.h"

#include <vector> 
#include <iostream> 
#include <fstream> 
#include <limits> 
#include <cmath> 
#include <chrono> 
#include <queue> 
#include "vector.h"
#include "boundingBox.h"
#include "scene.h"

#ifndef M_PI 
#define M_PI (3.14159265358979323846) 
#endif 

class BVH
{

	int Threshold = 2;
	vector<Object*> objs;

	class Comparator {
	public:
		int dimension;

		bool operator() (Object* a, Object* b) {
			AABB box;
			box = a->GetBoundingBox();
			float ca = (box.max.getIndex(dimension) + box.min.getIndex(dimension)) * 0.5f;
			box = b->GetBoundingBox();
			float cb = (box.max.getIndex(dimension) + box.min.getIndex(dimension)) * 0.5f;
			return ca < cb;
		}
	};

	class BVHNode {
	private:
		AABB bbox;
		bool leaf;
		unsigned int n_objs;
		unsigned int index;	// if leaf == false: index to left child node,
							// else if leaf == true: index to first Intersectable in Objsvector

	public: 
		void setAABB(AABB& bbox_) {
			bbox = bbox;
		}

		void makeLeaf(unsigned int index_, unsigned int n_objs_) { this->index = index; this->n_objs = n_objs; }
		void makeNode(unsigned int left_index_, unsigned int n_objs) { this->index = left_index_; this->n_objs = n_objs; }
			// n_objsin makeNode is for debug purposes only, and may be omitted later on
		bool isLeaf() { return leaf; }
		unsigned int getIndex() { return index; }
		unsigned int getNObjs() { return n_objs; }
		AABB &getAABB() { return bbox; };

	};

	public:
		void build(vector<Object *> &objects) {
			vector<BVHNode> nodes;

			BVHNode root;

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(FLT_MIN, FLT_MIN, FLT_MIN);

			for (Object* obj : objects) {
				AABB bbox = obj->GetBoundingBox();
				if (bbox.min.x < min.x) min.x = bbox.min.x;
				if (bbox.min.y < min.y) min.y = bbox.min.y;
				if (bbox.min.z < min.z) min.z = bbox.min.z;

				if (bbox.max.x > max.x) max.x = bbox.max.x;
				if (bbox.max.y > max.y) max.y = bbox.max.y;
				if (bbox.max.z > max.z) max.z = bbox.max.z;

				objs.push_back(obj);
			}

			AABB finalBBox = AABB(min, max);

			root.setAABB(finalBBox);

			build_recursive(0, objs.size(), root, 0);
		}

		void build_recursive(int left_index, int right_index, BVHNode& node, int depth) {

			if ((right_index - left_index) <= Threshold ) {
				//node.makeLeaf(left_index, )
			}
			else {
				// Get largest axis
				AABB node_bb = node.getAABB();

				int op; // 0->x, 1->y, 2->z

				Vector len = node_bb.max - node_bb.min;

				if (len.x >= len.y && len.x >= len.z) op = 0;
				else if (len.y >= len.x && len.y >= len.z) op = 1;
				else op = 2;

				// sorting of the objects
				Comparator cmp;
				cmp.dimension = op;

				sort(objs.begin() + left_index, objs.end() + right_index, cmp);

				float mid_coord = len.getIndex(op);
				bool found = false;
				int i;

				// if no objects are gonna be on the left or right divisions, use mean
				if (objs[left_index]->getCentroid().getIndex(op) > mid_coord ||
					objs[right_index - 1]->getCentroid().getIndex(op) < mid_coord) {
					mid_coord = 0;
					for (i = left_index; i < right_index; i++) {
						mid_coord += objs[i]->getCentroid().getIndex(op);
					}
					mid_coord /= (right_index - left_index);
				}

				// the i value obtained is the split_index
				for (i = left_index; i < right_index; i++) {
					if (objs[i]->getCentroid().getIndex(op) > mid_coord) {
						break;
					}
				}

				Vector min_left, min_right, max_left, max_right;

			}
		}
		
};