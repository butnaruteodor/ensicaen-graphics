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

class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList &props) { }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Color3f Lo(0.0f);
        Color3f throughput(1.0f);
        Ray3f currentRay = ray;
        int depth = 0;

        while (depth < 20) {
            Intersection its;
            // If ray misses everything, we stop
            if (!scene->rayIntersect(currentRay, its))
                break;

            // Check if we hit an emitter
            if (its.isEmitter()) {
                EmitterQueryRecord lRec;
                lRec.ref = currentRay.o; 
                lRec.p = its.p;          
                lRec.n = its.shFrame.n;  
                lRec.wi = -currentRay.d; 
                
                // Add the light contribution weighted by path throughput
                Lo += throughput * its.emitter->eval(lRec);
            }

            // Russian Roulette
            if (depth >= 3) {
                float survivalProb = std::min(0.90f, throughput.maxCoeff());
                if (sampler->next1D() > survivalProb)
                    break;
                throughput /= survivalProb;
            }

            // Indirect Illumination (BSDF Sampling)
            if (!its.bsdf)
                break;
            
            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f bsdfSample = its.bsdf->sample(bRec, sampler->next2D());
            
            if (bsdfSample.isZero())
                break;

            throughput *= bsdfSample;
            
            // Construct the next ray
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, std::numeric_limits<float>::infinity());
            depth++;
        }

        return Lo;
    }

    std::string toString() const override {
        return "PathMatsIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");

NORI_NAMESPACE_END