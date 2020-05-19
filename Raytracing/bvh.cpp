#include <vector>
#include <cmath>
#include <IL/il.h>
using namespace std;

#include "camera.h"
#include "color.h"
#include "vector.h"
#include "ray.h"
#include "boundingBox.h"
#include <atomic> 
#include <memory> 
#include <cassert> 
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

const float kEpsilon = 1e-8;
const float kInfinity = std::numeric_limits<float>::max();

std::atomic<uint32_t> numPrimaryRays(0);
std::atomic<uint32_t> numRayTriangleTests(0);
std::atomic<uint32_t> numRayTriangleIntersections(0);
std::atomic<uint32_t> numRayAABBTests(0);
std::atomic<uint32_t> numRayBoundingVolumeTests(0);

class BVH
{
    static const uint8_t kNumPlaneSetNormals = 7;
    static const Vector planeSetNormals[kNumPlaneSetNormals];
    struct Extents
    {
        AABB aabb;
        Extents()
        {
            for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i)
                d[i][0] = kInfinity,
                d[i][1] = -kInfinity;
        }
        void extendBy(const Extents& e)
        {

            for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i) {
                if (e.d[i][0] < d[i][0]) d[i][0] = e.d[i][0];
                if (e.d[i][1] > d[i][1]) d[i][1] = e.d[i][1];
            }
        }
        /* inline */
        Vector centroid() const
        {
            return Vector(
                d[0][0] + d[0][1] * 0.5,
                d[1][0] + d[1][1] * 0.5,
                d[2][0] + d[2][1] * 0.5);
        }
        bool intersect(const float*, const float*, float&, float&, uint8_t&) const;
        float d[kNumPlaneSetNormals][2];
        const Object* mesh;
    };

    struct Octree
    {
        Octree(const Extents& sceneExtents)
        {
            float xDiff = sceneExtents.d[0][1] - sceneExtents.d[0][0];
            float yDiff = sceneExtents.d[1][1] - sceneExtents.d[1][0];
            float zDiff = sceneExtents.d[2][1] - sceneExtents.d[2][0];
            float maxDiff = std::max(xDiff, std::max(yDiff, zDiff));
            Vector minPlusMax(
                sceneExtents.d[0][0] + sceneExtents.d[0][1],
                sceneExtents.d[1][0] + sceneExtents.d[1][1],
                sceneExtents.d[2][0] + sceneExtents.d[2][1]);
            bbox[0] = (minPlusMax - maxDiff) * 0.5;
            bbox[1] = (minPlusMax + maxDiff) * 0.5;
            root = new OctreeNode;
        }

        ~Octree() { deleteOctreeNode(root); }

        void insert(const Extents* extents) { insert(root, extents, bbox, 0); }
        void build() { build(root, bbox); };

        struct OctreeNode
        {
            OctreeNode* child[8] = { nullptr };
            std::vector<const Extents*> nodeExtentsList; // pointer to the objects extents 
            Extents nodeExtents; // extents of the octree node itself 
            bool isLeaf = true;
        };

        struct QueueElement
        {
            const OctreeNode* node; // octree node held by this element in the queue 
            float t; // distance from the ray origin to the extents of the node 
            QueueElement(const OctreeNode* n, float tn) : node(n), t(tn) {}
            // priority_queue behaves like a min-heap
            friend bool operator < (const QueueElement& a, const QueueElement& b) { return a.t > b.t; }
        };

        OctreeNode* root = nullptr; // make unique son don't have to manage deallocation 
        AABB<> bbox;

    private:

        void deleteOctreeNode(OctreeNode*& node)
        {
            for (uint8_t i = 0; i < 8; i++) {
                if (node->child[i] != nullptr) {
                    deleteOctreeNode(node->child[i]);
                }
            }
            delete node;
        }

        void insert(OctreeNode*& node, const Extents* extents, const AABB<>& bbox, uint32_t depth)
        {
            if (node->isLeaf) {
                if (node->nodeExtentsList.size() == 0 || depth == 16) {
                    node->nodeExtentsList.push_back(extents);
                }
                else {
                    node->isLeaf = false;
                    // Re-insert extents held by this node
                    while (node->nodeExtentsList.size()) {
                        insert(node, node->nodeExtentsList.back(), bbox, depth);
                        node->nodeExtentsList.pop_back();
                    }
                    // Insert new extent
                    insert(node, extents, bbox, depth);
                }
            }
            else {
                // Need to compute in which child of the current node this extents should
                // be inserted into
                Vector extentsCentroid = extents->centroid();
                Vector nodeCentroid = (bbox[0] + bbox[1]) * 0.5;
                AABB<> childAABB;
                uint8_t childIndex = 0;
                // x-axis
                if (extentsCentroid.x > nodeCentroid.x) {
                    childIndex = 4;
                    childAABB[0].x = nodeCentroid.x;
                    childAABB[1].x = bbox[1].x;
                }
                else {
                    childAABB[0].x = bbox[0].x;
                    childAABB[1].x = nodeCentroid.x;
                }
                // y-axis
                if (extentsCentroid.y > nodeCentroid.y) {
                    childIndex += 2;
                    childAABB[0].y = nodeCentroid.y;
                    childAABB[1].y = bbox[1].y;
                }
                else {
                    childAABB[0].y = bbox[0].y;
                    childAABB[1].y = nodeCentroid.y;
                }
                // z-axis
                if (extentsCentroid.z > nodeCentroid.z) {
                    childIndex += 1;
                    childAABB[0].z = nodeCentroid.z;
                    childAABB[1].z = bbox[1].z;
                }
                else {
                    childAABB[0].z = bbox[0].z;
                    childAABB[1].z = nodeCentroid.z;
                }

                // Create the child node if it doesn't exsit yet and then insert the extents in it
                if (node->child[childIndex] == nullptr)
                    node->child[childIndex] = new OctreeNode;
                insert(node->child[childIndex], extents, childAABB, depth + 1);
            }
        }

        void build(OctreeNode*& node, const AABB<>& bbox)
        {
            if (node->isLeaf) {
                for (const auto& e : node->nodeExtentsList) {
                    node->nodeExtents.extendBy(*e);
                }
            }
            else {
                for (uint8_t i = 0; i < 8; ++i) {
                    if (node->child[i]) {
                        AABB<> childAABB;
                        Vector centroid = bbox.centroid();
                        // x-axis
                        childAABB[0].x = (i & 4) ? centroid.x : bbox[0].x;
                        childAABB[1].x = (i & 4) ? bbox[1].x : centroid.x;
                        // y-axis
                        childAABB[0].y = (i & 2) ? centroid.y : bbox[0].y;
                        childAABB[1].y = (i & 2) ? bbox[1].y : centroid.y;
                        // z-axis
                        childAABB[0].z = (i & 1) ? centroid.z : bbox[0].z;
                        childAABB[1].z = (i & 1) ? bbox[1].z : centroid.z;

                        // Inspect child
                        build(node->child[i], childAABB);

                        // Expand extents with extents of child
                        node->nodeExtents.extendBy(node->child[i]->nodeExtents);
                    }
                }
            }
        }
    };

    std::vector<Extents> extentsList;
    Octree* octree = nullptr;
public:
    BVH(std::vector<std::unique_ptr<const Object>>& m);
    bool intersect(const Vector&, const Vector&, const uint32_t&, float&) const;
    ~BVH() { delete octree; }
};

const Vector BVH::planeSetNormals[BVH::kNumPlaneSetNormals] = {
    Vector(1, 0, 0),
    Vector(0, 1, 0),
    Vector(0, 0, 1),
    Vector(sqrtf(3) / 3.f,  sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    Vector(-sqrtf(3) / 3.f,  sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    Vector(-sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    Vector(sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f)
};

BVH::BVH(std::vector<std::unique_ptr<const Object>>& m)
{
    Extents sceneExtents; // that's the extent of the entire scene which we need to compute for the octree 
    extentsList.reserve(meshes.size());
    for (uint32_t i = 0; i < meshes.size(); ++i) {
        for (uint8_t j = 0; j < kNumPlaneSetNormals; ++j) {
            for (const auto vtx : meshes[i]->vertexPool) {
                float d = dot(planeSetNormals[j], vtx);
                // set dNEar and dFar
                if (d < extentsList[i].d[j][0]) extentsList[i].d[j][0] = d;
                if (d > extentsList[i].d[j][1]) extentsList[i].d[j][1] = d;
            }
        }
        sceneExtents.extendBy(extentsList[i]); // expand the scene extent of this object's extent 
        extentsList[i].mesh = meshes[i].get(); // the extent itself needs to keep a pointer to the object its holds 
    }

    // Now that we have the extent of the scene we can start building our octree
    // Using C++ make_unique function here but you don't need to, just to learn something... 
    octree = new Octree(sceneExtents);

    for (uint32_t i = 0; i < meshes.size(); ++i) {
        octree->insert(&extentsList[i]);
    }

    // Build from bottom up
    octree->build();
}

bool BVH::Extents::intersect(
    const float* precomputedNumerator,
    const float* precomputedDenominator,
    float& tNear,   // tn and tf in this method need to be contained 
    float& tFar,    // within the range [tNear:tFar] 
    uint8_t& planeIndex) const
{
    numRayBoundingVolumeTests++;
    for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i) {
        float tNearExtents = (d[i][0] - precomputedNumerator[i]) / precomputedDenominator[i];
        float tFarExtents = (d[i][1] - precomputedNumerator[i]) / precomputedDenominator[i];
        if (precomputedDenominator[i] < 0) std::swap(tNearExtents, tFarExtents);
        if (tNearExtents > tNear) tNear = tNearExtents, planeIndex = i;
        if (tFarExtents < tFar) tFar = tFarExtents;
        if (tNear > tFar) return false;
    }

    return true;
}

bool BVH::intersect(const Vector& orig, const Vector& dir, const uint32_t& rayId, float& tHit) const
{
    tHit = kInfinity;
    const Object* intersectedObject = nullptr;
    float precomputedNumerator[BVH::kNumPlaneSetNormals];
    float precomputedDenominator[BVH::kNumPlaneSetNormals];
    for (uint8_t i = 0; i < kNumPlaneSetNormals; ++i) {
        precomputedNumerator[i] = dot(planeSetNormals[i], orig);
        precomputedDenominator[i] = dot(planeSetNormals[i], dir);
    }

    /*
    tNear = kInfinity; // set
    for (uint32_t i = 0; i < meshes.size(); ++i) {
        numRayVolumeTests++;
        float tn = -kInfinity, tf = kInfinity;
        uint8_t planeIndex;
        if (extents[i].intersect(precomputedNumerator, precomputedDenominator, tn, tf, planeIndex)) {
            if (tn < tNear) {
                intersectedObject = meshes[i].get();
                tNear = tn;
                // normal = planeSetNormals[planeIndex];
            }
        }
    }
    */

    uint8_t planeIndex;
    float tNear = 0, tFar = kInfinity; // tNear, tFar for the intersected extents 
    if (!octree->root->nodeExtents.intersect(precomputedNumerator, precomputedDenominator, tNear, tFar, planeIndex) || tFar < 0)
        return false;
    tHit = tFar;
    std::priority_queue<BVH::Octree::QueueElement> queue;
    queue.push(BVH::Octree::QueueElement(octree->root, 0));
    while (!queue.empty() && queue.top().t < tHit) {
        const Octree::OctreeNode* node = queue.top().node;
        queue.pop();
        if (node->isLeaf) {
            for (const auto& e : node->nodeExtentsList) {
                float t = kInfinity;
                if (e->mesh->intersect(orig, dir, t) && t < tHit) {
                    tHit = t;
                    intersectedObject = e->mesh;
                }
            }
        }
        else {
            for (uint8_t i = 0; i < 8; ++i) {
                if (node->child[i] != nullptr) {
                    float tNearChild = 0, tFarChild = tFar;
                    if (node->child[i]->nodeExtents.intersect(precomputedNumerator, precomputedDenominator, tNearChild, tFarChild, planeIndex)) {
                        float t = (tNearChild < 0 && tFarChild >= 0) ? tFarChild : tNearChild;
                        queue.push(BVH::Octree::QueueElement(node->child[i], t));
                    }
                }
            }
        }
    }

    return (intersectedObject != nullptr);
}