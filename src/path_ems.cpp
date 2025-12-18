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

/**
 * \brief Path Tracer with Explicit Light Sampling (EMS / Next Event Estimation)
 * 
 * Strategy:
 * 1. Shoot a ray.
 * 2. If it hits a light, add emitted radiance (only from BSDF samples).
 * 3. Explicitly sample lights for direct illumination (shadow rays).
 * 4. Sample the BSDF to get the next direction (indirect).
 * 5. Repeat.
 */
class PathEMSIntegrator : public Integrator {
public:
    PathEMSIntegrator(const PropertyList &props) { }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Color3f Lo(0.0f);
        Color3f throughput(1.0f);
        Ray3f currentRay = ray;
        int depth = 0;
        const int maxDepth = 100;

        while (depth < maxDepth) {
            Intersection its;
            if (!scene->rayIntersect(currentRay, its))
                break;

            // ===== 1. Direct Hit on Emitter (BSDF path only) =====
            if (its.isEmitter()) {
                EmitterQueryRecord lRec;
                lRec.ref = currentRay.o;
                lRec.p = its.p;
                lRec.n = its.shFrame.n;
                lRec.wi = -currentRay.d;
                
                Color3f Le = its.emitter->eval(lRec);
                Lo += throughput * Le;
            }

            // ===== 2. Russian Roulette =====
            if (depth >= 3) {
                float survivalProb = std::min(0.99f, throughput.maxCoeff());
                if (sampler->next1D() > survivalProb)
                    break;
                throughput /= survivalProb;
            }

            // ===== 3. Explicit Light Sampling (Emitter Sampling / Next Event Estimation) =====
            std::vector<const Emitter *> emitters;
            for (uint32_t i = 0; i < scene->getAccel()->getMeshCount(); ++i) {
                const Mesh *mesh = scene->getAccel()->getMesh(i);
                if (mesh->isEmitter()) {
                    emitters.push_back(mesh->getEmitter());
                }
            }
            
            if (!emitters.empty() && its.bsdf) {
                // Uniformly sample one of the emitters
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
                    Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                    if (!scene->rayIntersect(shadowRay)) {
                        BSDFQueryRecord bRec(
                            its.toLocal(lRec.wi),
                            its.toLocal(-currentRay.d),
                            ESolidAngle
                        );
                        Color3f fr = its.bsdf->eval(bRec);
                        
                        // Geometry term: cosines at both surfaces divided by distance squared
                        float cosAtShading = std::abs(its.shFrame.n.dot(lRec.wi));
                        float cosAtLight = std::abs(lRec.n.dot(-lRec.wi));
                        float G = (cosAtShading * cosAtLight) / (lRec.dist * lRec.dist);
                        
                        // We sampled 1 light uniformly out of N
                        float weightFactor = (float)emitters.size();
                        
                        Color3f directContrib = fr * Le * G * weightFactor / lightPdf;
                        Lo += throughput * directContrib;
                    }
                }
            }

            // ===== 4. Indirect Illumination (BSDF Sampling) =====
            if (!its.bsdf)
                break;
            
            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f bsdfSample = its.bsdf->sample(bRec, sampler->next2D());
            
            if (bsdfSample.isZero())
                break;

            throughput *= bsdfSample;
            
            Vector3f nextDir = its.toWorld(bRec.wo);
            currentRay = Ray3f(its.p, nextDir, Epsilon, std::numeric_limits<float>::infinity());
            
            depth++;
        }

        return Lo;
    }

    std::string toString() const override {
        return "PathEMSIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathEMSIntegrator, "path_ems");

NORI_NAMESPACE_END