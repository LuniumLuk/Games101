//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth >= maxDepth) return Vector3f(0, 0, 0);
    // TO DO Implement Path Tracing Algorithm here
    auto isect = intersect(ray);
    if (isect.happened) {
        if (isect.m->hasEmission()) {
            return isect.m->getEmission();
        }

        auto p = isect.coords;
        auto N = isect.normal;
        auto wo = -ray.direction;

        auto L_dir = Vector3f(0, 0, 0);
        Intersection isect_light;
        float pdf_light;
        sampleLight(isect_light, pdf_light);

        auto x = isect_light.coords;
        auto px = x - p;
        auto ws = px.normalized();
        auto px2 = dotProduct(px, px);
        auto dist = sqrtf(px2);
        auto NN = isect_light.normal;
        auto emit = isect_light.emit;

        Ray r_light(p, ws);
        auto isect_block = intersect(r_light);
        if (isect_block.distance * 1.01f > dist) {
            L_dir = emit
                  * isect.m->eval(wo, ws, N)
                  * std::max(dotProduct(ws, N), 0.f)
                  * std::max(dotProduct(-ws, NN), 0.f)
                  / px2
                  / std::max(pdf_light, 1e-6f);
        }

        auto L_indir = Vector3f(0, 0, 0);
        auto rnd = get_random_float();
        if (rnd > RussianRoulette) {
            return L_dir + L_indir;
        }

        auto wi = isect.m->sample(wo, N);
        auto pdf = isect.m->pdf(wi, wo, N);

        Ray r(p, wi);
        auto isect_indir = intersect(ray);
        if (!isect_indir.m->hasEmission()) {
            L_indir = castRay(r, depth)
                    * isect.m->eval(wi, wo, N)
                    * std::max(dotProduct(wi, N), 0.f)
                    / std::max(pdf, 1e-6f)
                    / RussianRoulette;
        }
        
        return L_dir + L_indir;
    }

    return Vector3f(0, 0, 0);
}