#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}



BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        // auto middling = objects.begin() + (objects.size() / 2);
        auto middling = getSAHPartition(objects);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

std::vector<Object*>::iterator BVHAccel::getSAHPartition(std::vector<Object*>& objects)
{
    std::vector<Object*>::iterator best_pivot;
    double best_sah = std::numeric_limits<double>::max();
    
    for (int i = 1; i < objects.size(); ++i)
    {
        Bounds3 bound_a, bound_b;
        for (int j = 0; j < i; ++j)
            bound_a = Union(bound_a, objects[j]->getBounds());
        for (int j = i; j < objects.size(); ++j)
            bound_b = Union(bound_b, objects[j]->getBounds());
        
        double sah = bound_a.SurfaceArea() * i + bound_b.SurfaceArea() * (objects.size() - i);
        if (sah < best_sah)
        {
            best_sah = sah;
            best_pivot = objects.begin() + i;
        }
    }

    return best_pivot;
}


Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Vector3f invDir = Vector3f(1.f / ray.direction.x,
                               1.f / ray.direction.y,
                               1.f / ray.direction.z);
    // Unused.
    std::array<int, 3> dirIsNeg = {int(ray.direction.x>0),
                                   int(ray.direction.y>0),
                                   int(ray.direction.z>0)};
    Intersection isect;
    if (!node->bounds.IntersectP(ray, invDir, dirIsNeg))
    {
        return isect;
    }

    if (node->object) // Leaf node.
    {
        isect = node->object->getIntersection(ray);
        return isect;
    }
    else
    {
        Intersection isect_left, isect_right;
        if (node->left)
            isect_left = getIntersection(node->left, ray);
        if (node->right)
            isect_right = getIntersection(node->right, ray);

        if (!isect_left.happened)
        {
            return isect_right;
        }
        else if (!isect_right.happened)
        {
            return isect_left;
        }
        else
        {
            if (isect_left.distance < isect_right.distance)
            {
                return isect_left;
            }
            else
            {
                return isect_right;
            }
        }
    }
}