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

class PathEMSIntegrator : public Integrator {
public:
    PathEMSIntegrator(const PropertyList &props) { }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Color3f Lo(0.0f);
        Color3f throughput(1.0f);
        Ray3f currentRay = ray;
        
        int depth = 0;
        bool includeEmitted = true; // "lastBounceSpecular" logic for EMS

        // Pre-collect emitters
        std::vector<const Emitter *> emitters;
        for (uint32_t i = 0; i < scene->getAccel()->getMeshCount(); ++i) {
            const Mesh *mesh = scene->getAccel()->getMesh(i);
            if (mesh->isEmitter()) {
                emitters.push_back(mesh->getEmitter());
            }
        }
        
        while (depth < 20) {
            Intersection its;
            if (!scene->rayIntersect(currentRay, its))
                break;

            // Check if we hit an emitter
            if (its.isEmitter() && includeEmitted) {
                EmitterQueryRecord lRec;
                lRec.ref = currentRay.o;
                lRec.p = its.p;
                lRec.n = its.shFrame.n;
                lRec.wi = -currentRay.d;
                
                Lo += throughput * its.emitter->eval(lRec);
            }

            // Next event estimation
            if (!emitters.empty() && its.bsdf) {
                size_t lightIdx = std::min(
                    (size_t)(sampler->next1D() * emitters.size()), 
                    emitters.size() - 1
                );
                const Emitter *emitter = emitters[lightIdx];
                
                EmitterQueryRecord lRec;
                lRec.ref = its.p;
                
                float lightPdf; 
                Color3f Le = emitter->sample(lRec, sampler->next2D(), lightPdf);
                
                if (!Le.isZero() && lightPdf > 0.0f) {
                    BSDFQueryRecord bRec(
                        its.toLocal(-currentRay.d), 
                        its.toLocal(lRec.wi), 
                        ESolidAngle
                    );
                    
                    Color3f fr = its.bsdf->eval(bRec); 

                    if (!fr.isZero()) {
                        Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                        if (!scene->rayIntersect(shadowRay)) {
                            float cosAtShading = Frame::cosTheta(bRec.wo); 
                            float cosAtLight   = std::abs(lRec.n.dot(-lRec.wi));
                            float distSq       = lRec.dist * lRec.dist;
                            float G            = cosAtLight / distSq; 
                            float weightFactor = (float)emitters.size();

                            // EMS Contribution
                            Lo += throughput * fr * Le * cosAtShading * G * weightFactor / lightPdf;
                        }
                    }
                }
            }

            // Russian Roulette
            if (depth >= 3) {
                float prob = std::min(0.99f, throughput.maxCoeff());
                if (sampler->next1D() > prob) break;
                throughput /= prob;
            }

            // Indirect Illumination (BSDF Sampling)
            if (!its.bsdf) break;

            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f bsdfSample = its.bsdf->sample(bRec, sampler->next2D());
            
            if (bsdfSample.isZero()) break;

            throughput *= bsdfSample;
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, INFINITY);
            includeEmitted = (bRec.measure == EDiscrete);
            
            depth++;
        }

        return Lo;
    }

    std::string toString() const override { return "PathEMSIntegrator[]"; }
};

NORI_REGISTER_CLASS(PathEMSIntegrator, "path_ems");

NORI_NAMESPACE_END