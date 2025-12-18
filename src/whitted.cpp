#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/mesh.h>
#include <nori/ray.h>
#include <nori/common.h>
#include <limits>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) { }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f &ray) const override {
        return LiRecursive(scene, sampler, ray, 0);
    }

private:
    Color3f LiRecursive(const Scene* scene, Sampler* sampler, const Ray3f &ray, int depth) const {
        if (depth > 100)
            return Color3f(0.0f);

        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f); // background

        Color3f Lo(0.0f);

        // Check if surface is specular or diffuse
        if (!its.bsdf->isDiffuse()) {
            // SPECULAR CASE: reflect/refract
            BSDFQueryRecord bRec(its.toLocal(-ray.d));
            Color3f c = its.bsdf->sample(bRec, sampler->next2D());
            
            if (!c.isZero()) {
                // Russian roulette to terminate the recursion
                float xi = sampler->next1D();
                if (xi < 0.95f) {
                    Ray3f reflRay(its.p, its.toWorld(bRec.wo), Epsilon, 
                                  std::numeric_limits<float>::infinity());
                    Lo += (1.0f / 0.95f) * c * LiRecursive(scene, sampler, reflRay, depth + 1);
                }
            }
            return Lo;
        } else {
            // DIFFUSE CASE: direct lighting from emitters
            // Iterate over all meshes and find those that are emitters
            for (uint32_t meshIdx = 0; meshIdx < scene->getAccel()->getMeshCount(); ++meshIdx) {
                const Mesh* mesh = scene->getAccel()->getMesh(meshIdx);
                if (!mesh->isEmitter())
                    continue;

                const Emitter* emitter = mesh->getEmitter();
                
                EmitterQueryRecord lRec;
                lRec.ref = its.p;

                float pdf;
                Color3f Le = emitter->sample(lRec, sampler->next2D(), pdf);
                
                if (pdf == 0.0f || Le.isZero())
                    continue;

                // Visibility check
                Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                if (scene->rayIntersect(shadowRay))
                    continue; // Light is occluded

                // Evaluate BSDF for this light direction
                BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(-ray.d), ESolidAngle);
                Color3f fr = its.bsdf->eval(bRec);
                
                // Geometry factor
                float G = std::abs(its.shFrame.n.dot(lRec.wi)) *
                          std::abs(lRec.n.dot(-lRec.wi)) /
                          (lRec.dist * lRec.dist);
                
                Lo += fr * Le * G / pdf;
            }
            return Lo;
        }
    }

    std::string toString() const override {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");

NORI_NAMESPACE_END
