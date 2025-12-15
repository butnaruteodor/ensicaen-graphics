#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>   // for AreaLight + EmitterQueryRecord
#include <nori/bsdf.h>      // for BSDF + BSDFQueryRecord
#include <nori/sampler.h>   // for Sampler
#include <nori/mesh.h>      // for Mesh, samplePosition
#include <nori/ray.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) {
        // Read parameters from XML
        m_lightPos = props.getPoint("position", Point3f(0));
        m_energy   = props.getColor("energy",  Color3f(1.0f));
        
    }
Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f &ray) const override {
    return LiRecursive(scene, sampler, ray, 0);
}

private:
Color3f LiRecursive(const Scene* scene, Sampler* sampler, const Ray3f &ray, int depth) const {
    if (depth > 10)
        return Color3f(0.f);

    Intersection its;
    if (!scene->rayIntersect(ray, its))
        return Color3f(0.f); // background

    Color3f Lo(0.f);
    Vector3f wo = -ray.d;

    if (!its.bsdf->isDiffuse()) {
        BSDFQueryRecord bRec(its.shFrame.toLocal(wo));
        Point2f xi = sampler->next2D();

        Color3f f = its.bsdf->sample(bRec, xi);
        Ray3f reflRay(its.p, its.shFrame.toWorld(bRec.wo), Epsilon, std::numeric_limits<float>::infinity());

        if (xi.y() < 0.95f)
            Lo += f * LiRecursive(scene, sampler, reflRay, depth + 1) / 0.95f;

        return Lo;
    }

    // Diffuse surface: sample emitters
    for (uint32_t i = 0; i < scene->getAccel()->getMeshCount(); ++i) {
        const Mesh* mesh = scene->getAccel()->getMesh(i);
        if (!mesh->isEmitter())
            continue;

        const Emitter* emitter = mesh->getEmitter();
        EmitterQueryRecord lRec;
        lRec.ref = its.p;

        float pdf;
        Color3f Le = emitter->sample(lRec, sampler->next2D(), pdf);
        if (pdf == 0.f || Le.isZero())
            continue;

        Ray3f shadowRay(its.p, lRec.wi, Epsilon, lRec.dist - Epsilon);
        if (scene->rayIntersect(shadowRay))
            continue;

        BSDFQueryRecord bRec(its.shFrame.toLocal(lRec.wi),
                             its.shFrame.toLocal(-ray.d),
                             ESolidAngle);
        Color3f fr = its.bsdf->eval(bRec);

        float G = std::abs(its.shFrame.n.dot(lRec.wi)) *
                  std::abs(lRec.n.dot(-lRec.wi)) /
                  (lRec.dist * lRec.dist);

        Lo += fr * Le * G / pdf;
    }

    return Lo;
}





    std::string toString() const {
        return tfm::format(
            "WhittedIntegrator[\n"
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

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END
