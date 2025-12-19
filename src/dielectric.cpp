#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        m_intIOR = propList.getFloat("intIOR", 1.5046f);
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        m_color = propList.getColor("color", Color3f(1.0f));
    }

    Color3f eval(const BSDFQueryRecord &) const override {
        return Color3f(0.0f); // Delta distribution
    }

    float pdf(const BSDFQueryRecord &) const override {
        return 0.0f; // Delta distribution
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        bRec.measure = EDiscrete;

        // 1. Check direction (Inside vs Outside)
        // Nori's local frame always has Normal = (0,0,1)
        float cosThetaI = Frame::cosTheta(bRec.wi);
        bool entering = cosThetaI > 0.0f;

        float etaI = entering ? m_extIOR : m_intIOR;
        float etaT = entering ? m_intIOR : m_extIOR;
        
        // 2. Compute Fresnel Term
        // Using "etaI / etaT" simplifies the math significantly
        float eta = etaI / etaT;
        float sinThetaI2 = std::max(0.0f, 1.0f - cosThetaI * cosThetaI);
        float sinThetaT2 = eta * eta * sinThetaI2;

        float Fr = 1.0f; // Default to Total Internal Reflection
        float cosThetaT = 0.0f;

        // IMPROVEMENT 2: Numerical Stability check
        // If sinThetaT2 > 1.0, we have Total Internal Reflection (TIR).
        if (sinThetaT2 < 1.0f) {
            cosThetaT = std::sqrt(std::max(0.0f, 1.0f - sinThetaT2));
            
            // Exact Dielectric Fresnel Equations
            // We use absolute cosines to ensure correct math regardless of direction
            float absCosI = std::abs(cosThetaI);
            
            float Rs = (etaI * absCosI - etaT * cosThetaT) / (etaI * absCosI + etaT * cosThetaT);
            float Rp = (etaT * absCosI - etaI * cosThetaT) / (etaT * absCosI + etaI * cosThetaT);
            Fr = 0.5f * (Rs * Rs + Rp * Rp);
        }

        // 3. Russian Roulette: Choose Reflection or Refraction based on Fr
        if (sample.x() < Fr) {
            // --- REFLECTION ---
            // In local coordinates, reflection is just (-x, -y, z)
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            bRec.eta = 1.0f; // No index change on reflection

            // Return Color * Weight. 
            // Weight = Fr / PDF. Since PDF = Fr, they cancel out -> 1.0
            return m_color; 
        } else {
            // --- REFRACTION ---
            bRec.eta = eta; // Record relative IOR for the integrator

            // Robust vector math for refraction in local frame
            // The sign of Z depends on whether we are entering or exiting
            float signZ = entering ? -1.0f : 1.0f;
            
            bRec.wo = Vector3f(
                -eta * bRec.wi.x(),
                -eta * bRec.wi.y(),
                signZ * cosThetaT
            );

            // CRITICAL: Radiance compression factor (eta^2)
            // Light concentrates when entering denser inputs.
            return m_color * (eta * eta);
        }
    }

    std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }

private:
    float m_intIOR, m_extIOR;
    Color3f m_color;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");

NORI_NAMESPACE_END