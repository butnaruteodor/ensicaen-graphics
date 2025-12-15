#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/object.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
public:
    AreaLight(const PropertyList &props) {
        m_radiance = props.getColor("radiance", Color3f(1.f));
    }

    void setParent(NoriObject *obj) override {
        m_mesh = dynamic_cast<Mesh *>(obj);
        if (!m_mesh)
            throw NoriException("AreaLight must be attached to a mesh!");
    }

    Color3f sample(EmitterQueryRecord &lRec,
                   const Point2f &sample,
                   float &pdf) const override {
        // Sample a point on the mesh
        m_mesh->samplePosition(sample, lRec.p, lRec.n);

        Vector3f d = lRec.p - lRec.ref;
        lRec.dist = d.norm();
        lRec.wi = d / lRec.dist;

        pdf = 1.f / m_mesh->getTotalArea();

        if (lRec.n.dot(-lRec.wi) <= 0.f)
            return Color3f(0.f);

        return m_radiance;
    }

    float pdf(const EmitterQueryRecord &) const override {
        return 1.f / m_mesh->getTotalArea();
    }

    Color3f eval(const EmitterQueryRecord &lRec) const override {
        if (lRec.n.dot(lRec.wi) <= 0.f)
            return Color3f(0.f);
        return m_radiance;
    }

    std::string toString() const override {
        return tfm::format(
            "AreaLight[\n"
            "  radiance = %s\n"
            "]",
            m_radiance.toString()
        );
    }

private:
    Color3f m_radiance;
    Mesh *m_mesh = nullptr;
};

NORI_REGISTER_CLASS(AreaLight, "area")

NORI_NAMESPACE_END
