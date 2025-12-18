#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AoIntegrator : public Integrator {
public:
    AoIntegrator(const PropertyList &props) {
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Point2f sample = sampler->next2D();
        Vector3f w_local = Warp::squareToCosineHemisphere(sample);

        // Transform into world space
        Vector3f w = its.shFrame.toWorld(w_local);

        // Shoot shadow ray for visibility
        Ray3f shadowRay(its.p, w, Epsilon, std::numeric_limits<float>::infinity());

        // If the ray hits something => occluded
        bool occluded = scene->rayIntersect(shadowRay);

        // AO returns visibility: 1 if visible, 0 if blocked
        float V = occluded ? 0.0f : 1.0f;

        return Color3f(V);
    }

    std::string toString() const {
        return "AoIntegrator[]";
    }
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END
