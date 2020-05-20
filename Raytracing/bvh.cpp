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
			this->bbox = bbox_;
		}

		void makeLeaf(unsigned int index_, unsigned int n_objs_) {
			this->leaf = true;
			this->index = index_; 
			this->n_objs = n_objs_; 
		}

		void makeNode(unsigned int left_index_, unsigned int n_objs_ = 0) { 
			this->leaf = false;
			this->index = left_index_; 
			this->n_objs = n_objs_; 
		}

		bool isLeaf() { return leaf; }
		unsigned int getIndex() { return index; }
		unsigned int getNObjs() { return n_objs; }
		AABB &getAABB() { return bbox; };

	};

	int Threshold = 2;
	vector<Object*> objs;
	vector<BVHNode*> nodes;

	public:
		void build(vector<Object *> &objects) {

			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB final_bbox = AABB(min, max);

			for (Object* obj : objects) {
				AABB bbox = obj->GetBoundingBox();
				final_bbox.extend(bbox);
				objs.push_back(obj);
			}

			root->setAABB(final_bbox);
			root->makeNode(0);
			nodes.push_back(root);

			build_recursive(0, objs.size(), root);

			int i = 0;
		}

		void build_recursive(int left_index, int right_index, BVHNode *node) {

			if ((right_index - left_index) <= Threshold ) {
				node->makeLeaf(left_index, (right_index - left_index));
			}
			else {
				// Get largest axis
				AABB node_bb = node->getAABB();

				int op; // 0->x, 1->y, 2->z

				Vector len = node_bb.max - node_bb.min;

				if (len.x >= len.y && len.x >= len.z) op = 0;
				else if (len.y >= len.x && len.y >= len.z) op = 1;
				else op = 2;

				// sorting of the objects
				Comparator cmp;
				cmp.dimension = op;

				sort(objs.begin() + left_index, objs.begin() + right_index, cmp);

				float mid_coord = (node_bb.max.getIndex(op) + node_bb.min.getIndex(op)) * 0.5;
				bool found = false;
				int i;

				// if no objects are gonna be on the left or right divisions, use mean
				if (objs[left_index]->getCentroid().getIndex(op) > mid_coord ||
					objs[right_index - 1]->getCentroid().getIndex(op) <= mid_coord) {
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

				Vector min_right, min_left = min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
				Vector max_right, max_left = max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

				AABB left_bbox(min_left, max_left), right_bbox(min_right, max_right);

				for (int j = left_index; j < i; j++) {
					left_bbox.extend(objs[j]->GetBoundingBox());
				}

				for (int j = i; j < right_index; j++) {
					right_bbox.extend(objs[j]->GetBoundingBox());
				}

				BVHNode* left_node = new BVHNode();
				BVHNode* right_node = new BVHNode();
				left_node->setAABB(left_bbox);

				right_node->setAABB(right_bbox);

				nodes.push_back(left_node);
				nodes.push_back(right_node);

				left_node->makeNode(node->getIndex() * 2 + 1);
				right_node->makeNode(node->getIndex() * 2 + 2);

				build_recursive(left_index, i, left_node);
				build_recursive(i, right_index, right_node);
			}
		}
		
};