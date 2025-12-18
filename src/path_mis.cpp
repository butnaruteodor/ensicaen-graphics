#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/mesh.h>
#include <nori/ray.h>
#include <nori/common.h>
#include <limits>
#include <cmath>

NORI_NAMESPACE_BEGIN

class PathMISIntegrator : public Integrator {
public:
    PathMISIntegrator(const PropertyList &props) { }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Color3f Lo(0.0f);
        Color3f throughput(1.0f);
        auto isFiniteColor = [&](const Color3f &c) {
            return std::isfinite(c[0]) && std::isfinite(c[1]) && std::isfinite(c[2]);
        };
        Ray3f currentRay = ray;
        int depth = 0;
        
        // Tracks the probability of the BSDF sample from the PREVIOUS bounce.
        // Used to weight the contribution when we hit a light directly.
        float pdfBsdf = 0.0f;
        
        // Tracks if the previous bounce was a specular interaction (Dirac delta),
        // which prevents us from calculating a standard PDF.
        bool lastBounceSpecular = false;

        while (true) {
            Intersection its;
            if (!scene->rayIntersect(currentRay, its))
                break;

            // ====================================================================
            // 1. Direct Hit on Emitter (BSDF Sampling Strategy / MATS)
            // ====================================================================
            // If we hit a light source, we add its contribution, BUT weighted 
            // by the probability of having found it via this path.
            if (its.isEmitter()) {
                EmitterQueryRecord lRec;
                lRec.ref = currentRay.o;
                lRec.p = its.p;
                lRec.n = its.shFrame.n;
                lRec.wi = -currentRay.d;
                lRec.dist = its.t;

                Color3f Le = its.emitter->eval(lRec);

                float wMats = 1.0f;
                
                // If this is not the first hit, we need to balance with light sampling
                if (depth > 0 && !lastBounceSpecular) {
                    // Calculate what the PDF would have been if we had used Light Sampling (EMS)
                    // to pick this specific point on the light.
                    float pdfLightArea = its.emitter->pdf(lRec); // PDF w.r.t Area
                    
                    if (pdfLightArea > 1e-6f) {
                        // Convert Area PDF to Solid Angle PDF: pdf_SA = pdf_A * dist^2 / cos(theta_light)
                        float cosAtLight = std::max(0.0f, its.shFrame.n.dot(-currentRay.d));
                        float distSq = its.t * its.t;
                        float pdfLightSA = (cosAtLight > 1e-6f) ? (pdfLightArea * distSq / cosAtLight) : 0.0f;

                        // Adjust for the fact that we select 1 light out of N emitters
                        const std::vector<Emitter *> &emitters = scene->getEmitters();
                        pdfLightSA /= (float)emitters.size();

                        // Balance Heuristic: w = p_light / (p_bsdf + p_light)
                        // We used BSDF sampling (MATS), so weight by EMS probability
                        // pdfBsdf (from prev iteration) is already in Solid Angle
                        float pdfSum = pdfBsdf + pdfLightSA;
                        if (pdfSum > 1e-6f) {
                            wMats = pdfLightSA / pdfSum;
                        }
                    }
                }

                Color3f contrib = throughput * Le * wMats;
                if (isFiniteColor(contrib))
                    Lo += contrib;
                else
                    break; // stop the path if we produced invalid contribution
            }

            // Terminate if max depth is reached
            if (depth >= 100) break;

            // ====================================================================
            // 2. Russian Roulette (Unbiased Termination)
            // ====================================================================
            if (depth >= 3) {
                // Guard against invalid throughput
                if (!isFiniteColor(throughput)) break;
                float survivalProb = std::min(0.99f, throughput.maxCoeff());
                if (!(survivalProb > 1e-6f)) break;
                if (sampler->next1D() > survivalProb)
                    break;
                throughput /= survivalProb;
            }

            // ====================================================================
            // 3. Emitter Sampling Strategy (EMS) - Next Event Estimation
            // ====================================================================
            // We only do this if the surface is not purely specular (because specular 
            // surfaces can't "see" a light unless the reflection direction hits it perfectly).
            const std::vector<Emitter *> &emitters = scene->getEmitters();
            
            if (!emitters.empty() && its.bsdf->isDiffuse()) {
                // Select one emitter at random
                size_t lightIdx = std::min((size_t)(sampler->next1D() * emitters.size()), emitters.size() - 1);
                const Emitter *emitter = emitters[lightIdx];

                EmitterQueryRecord lRec;
                lRec.ref = its.p;
                
                float pdfLightArea;
                Color3f Le = emitter->sample(lRec, sampler->next2D(), pdfLightArea);

                if (!Le.isZero() && pdfLightArea > 0.0f) {
                    // Check visibility
                    Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
                    if (!scene->rayIntersect(shadowRay)) {
                        
                        // Evaluate BSDF
                        BSDFQueryRecord bRec(its.toLocal(lRec.wi), its.toLocal(-currentRay.d), ESolidAngle);
                        Color3f fr = its.bsdf->eval(bRec);
                        float cosTheta = std::max(0.0f, its.shFrame.n.dot(lRec.wi));

                        if (!fr.isZero() && cosTheta > 0.0f) {
                            // Calculate PDF of BSDF (Solid Angle)
                            float pdfBsdfSA = its.bsdf->pdf(bRec);

                            // Convert Light PDF from Area to Solid Angle
                            float cosAtLight = std::max(0.0f, lRec.n.dot(-lRec.wi));
                            float pdfLightSA = 0.0f;
                            if (cosAtLight > 1e-6f && pdfLightArea > 0.0f) {
                                pdfLightSA = pdfLightArea * (lRec.dist * lRec.dist) / cosAtLight;
                                pdfLightSA /= (float)emitters.size(); // Selection probability
                            }

                            // Balance Heuristic: w = p_light / (p_light + p_bsdf)
                            // Add guards against NaN
                            float wEms = 1.0f;
                            float pdfSum = pdfLightSA + pdfBsdfSA;
                            if (pdfSum > 1e-6f) {
                                wEms = pdfLightSA / pdfSum;
                            }
                            
                            // Nori's emitter->sample usually returns Le / pdfArea.
                            // However, we usually reconstruct the integral manually for clarity:
                            // Contribution = fr * Le * cosTheta * (1/pdf_choice) * weight
                            // Note: Le from sample() is (Radiance / pdfArea). 
                            // So: Le * pdfArea gives us Raw Radiance.
                            // But usually we just use the returned 'Le' and multiply by the geometry term G ourselves?
                            // No, relying on Nori's Le is safer. 
                            // Le is (Radiance * G) / pdfArea in some implementations, or just Radiance.
                            // Assuming Le = Radiance:
                            // We need: (Radiance * fr * cos * cos_light / dist^2) / pdf_total
                            
                            // Standard Nori safe formula:
                            // Le * fr * cosTheta / pdf_selection * weight
                            // But we need to be careful about the implicit G term.
                            
                            // Let's assume Le comes prescaled by 1/pdfArea.
                            // Standard Nori impl: Le = Radiance / pdfArea.
                            // So we just add: Le * fr * G_geometry * wEms?
                            // Actually, standard Nori sample returns: (Radiance * cos_light) / (dist^2 * pdfArea)?
                            // Let's stick to the Geometry term formula which is robust:
                            
                            float G = cosTheta * cosAtLight / (lRec.dist * lRec.dist);
                            
                            // Safe contribution: guard against pdfLightArea being zero
                            if (pdfLightArea > 1e-6f) {
                                Color3f contrib = throughput * fr * Le * G * wEms * (float)emitters.size() / pdfLightArea;
                                if (isFiniteColor(contrib))
                                    Lo += contrib;
                                else
                                    ; // skip invalid contribution
                            }
                        }
                    }
                }
            }

            // ====================================================================
            // 4. BSDF Sampling (prepare for next step)
            // ====================================================================
            if (!its.bsdf) break;

            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f bsdfWeight = its.bsdf->sample(bRec, sampler->next2D());

            // Guard against invalid BSDF sample
            if (bsdfWeight.isZero()) break;
            if (!isFiniteColor(bsdfWeight)) break;

            throughput *= bsdfWeight;
            if (!isFiniteColor(throughput)) break;
            
            // Store PDF for the NEXT iteration's weight calculation
            pdfBsdf = its.bsdf->pdf(bRec);
            if (!(std::isfinite(pdfBsdf) && pdfBsdf >= 0.0f)) pdfBsdf = 0.0f;
            
            // Handle specular case (delta distribution has no PDF)
            // If discrete, we can't do MIS, so we default to standard path tracing logic (weight = 1)
            if (its.bsdf->isDiffuse()) {
                lastBounceSpecular = false;
            } else {
                lastBounceSpecular = true;
                pdfBsdf = 1.0f; // Dummy value
            }

            // Update ray
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo), Epsilon, std::numeric_limits<float>::infinity());
            depth++;
        }

        return Lo;
    }

    std::string toString() const override {
        return "PathMISIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMISIntegrator, "path_mis");

NORI_NAMESPACE_END