#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:
    SimpleIntegrator(const PropertyList &props) {
        // Read parameters from XML
        m_lightPos = props.getPoint("position", Point3f(0));
        m_energy   = props.getColor("energy",  Color3f(1.0f));
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        // Direction from shading point to light
        Vector3f lightDir = m_lightPos - its.p;
        float dist2 = lightDir.squaredNorm();
        float dist  = std::sqrt(dist2);
        lightDir.normalize();

        // cos_theta = n x w
        float cosTheta = its.shFrame.n.dot(lightDir);
        if (cosTheta <= 0.0f)
            return Color3f(0.0f); // light below surface

        // Shadow ray: origin = x, direction = w
        Ray3f shadowRay(its.p, lightDir, Epsilon, dist - Epsilon);

        // Visibility check
        bool occluded = scene->rayIntersect(shadowRay);
        if (occluded)
            return Color3f(0.0f);

        // Final radiance:
        // Phi / (4pi r_squared) * cos_theta
        return m_energy * (cosTheta / (4.0f * M_PI * M_PI * dist2));
    }

    std::string toString() const {
        return tfm::format(
            "SimpleIntegrator[\n"
            "  lightPos = %s,\n"
            "  energy = %s\n"
            "]",
            m_lightPos.toString(),
            m_energy.toString()
        );
    }

private:
    Point3f  m_lightPos;
    Color3f  m_energy;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END
