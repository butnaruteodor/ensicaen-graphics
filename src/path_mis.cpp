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

class PathMISIntegrator : public Integrator {
public:
    PathMISIntegrator(const PropertyList &props) { }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Color3f Lo(0.0f);
        Color3f throughput(1.0f);
        Ray3f currentRay = ray;
        
        int depth = 0;
        float lastBsdfPdf = 0.0f;       // PDF of the direction we arrived from (for MIS)
        bool lastBounceSpecular = true; // Start true to accept camera rays full weight

        // Pre-collect emitters
        std::vector<const Emitter*> emitters;
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
            if (its.isEmitter()) {
                EmitterQueryRecord lRec;
                lRec.ref = currentRay.o;
                lRec.p = its.p;
                lRec.n = its.shFrame.n;
                lRec.wi = -currentRay.d;
                lRec.dist = its.t;

                Color3f Le = its.emitter->eval(lRec);

                if (!Le.isZero()) {
                    float misWeight = 1.0f;

                    // If we arrived via a Diffuse bounce, we must balance against NEE
                    if (!lastBounceSpecular && !emitters.empty()) {
                        float pdfLightArea = its.emitter->pdf(lRec); 
                        
                        // Geometry terms (recalculated here for the 'virtual' NEE ray)
                        float distSq   = lRec.dist * lRec.dist;
                        float cosTheta = std::abs(lRec.n.dot(lRec.wi));
                        float G        = cosTheta / distSq;
                        
                        // Convert Area PDF to Solid Angle PDF (pdfSa = pdfArea / G)
                        float pdfLightSa = pdfLightArea / G;
                        
                        // Weight by selection probability
                        float effectivePdfLight = pdfLightSa / (float)emitters.size();

                        // Balance Heuristic
                        misWeight = lastBsdfPdf / (lastBsdfPdf + effectivePdfLight + 1e-5f);
                    }

                    Lo += throughput * Le * misWeight;
                }
            }

            // Next event estimation
            if (!emitters.empty() && its.bsdf) {
                // Pick a light
                size_t lightIdx = std::min((size_t)(sampler->next1D() * emitters.size()), emitters.size() - 1);
                const Emitter *emitter = emitters[lightIdx];

                EmitterQueryRecord lRec;
                lRec.ref = its.p;
                float lightPdf; // Area Measure
                Color3f Le = emitter->sample(lRec, sampler->next2D(), lightPdf);

                if (!Le.isZero() && lightPdf > 0.0f) {
                    BSDFQueryRecord bRec(its.toLocal(-currentRay.d), its.toLocal(lRec.wi), ESolidAngle);
                    Color3f fr = its.bsdf->eval(bRec); 

                    if (!fr.isZero()) {
                        // Visibility Check
                        Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                        if (!scene->rayIntersect(shadowRay)) {
                            float cosAtShading = Frame::cosTheta(bRec.wo);
                            float cosAtLight   = std::abs(lRec.n.dot(-lRec.wi));
                            float distSq       = lRec.dist * lRec.dist;
                            float G            = cosAtLight / distSq;
                            float weightFactor = (float)emitters.size();

                            // MIS Calculation
                            float pdfBsdf = its.bsdf->pdf(bRec);
                            
                            // Convert Light PDF to Solid Angle (pdfSa = pdfArea / G)
                            float lightPdfSa = lightPdf / G;
                            float effectivePdfLight = lightPdfSa / weightFactor;

                            float misWeight = effectivePdfLight / (effectivePdfLight + pdfBsdf + 1e-5f);

                            // EMS Part: (throughput * fr * Le * cosAtShading * G * weightFactor / lightPdf)
                            // MIS Part: * misWeight
                            Lo += throughput * fr * Le * cosAtShading * G * weightFactor / lightPdf * misWeight;
                        }
                    }
                }
            }

            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(0.99f, throughput.maxCoeff());
                if (sampler->next1D() > q) break;
                throughput /= q;
            }

            // Indirect Illumination (BSDF Sampling)
            if (!its.bsdf) break;

            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f bsdfWeight = its.bsdf->sample(bRec, sampler->next2D());
            
            if (bsdfWeight.isZero()) break;

            // Update MIS tracking variables
            lastBsdfPdf = its.bsdf->pdf(bRec); 
            lastBounceSpecular = (bRec.measure == EDiscrete);
            
            throughput *= bsdfWeight;
            
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, INFINITY);
            depth++;
        }

        return Lo;
    }

    std::string toString() const override { return "PathMISIntegrator[]"; }
};

NORI_REGISTER_CLASS(PathMISIntegrator, "path_mis");

NORI_NAMESPACE_END