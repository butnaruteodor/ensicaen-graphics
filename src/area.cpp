#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
public:
    AreaLight(const PropertyList &props) {
        m_radiance = props.getColor("radiance", Color3f(0.0f));
    }

    Color3f eval(const EmitterQueryRecord &lRec) const {
        // Only emit on the front side
        return m_radiance;
    }

    // No direct "sample ray" here — mesh sampling provides (p,n,pdf)
    // But you must override isAreaLight():
    bool isAreaLight() const { return true; }

    std::string toString() const override {
        return tfm::format("AreaLight[radiance=%s]", m_radiance);
    }

private:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END