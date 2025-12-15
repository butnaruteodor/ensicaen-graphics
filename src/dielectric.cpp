/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        m_intIOR = propList.getFloat("intIOR", 1.5046f);
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const override {
        return Color3f(0.0f); // discrete BRDF
    }

    float pdf(const BSDFQueryRecord &) const override {
        return 0.0f; // discrete BRDF
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        // Determine entering/exiting
        bool entering = bRec.wi.z() > 0;
        float etaI = entering ? m_extIOR : m_intIOR;
        float etaT = entering ? m_intIOR : m_extIOR;
        float eta = etaI / etaT;

        // Normal pointing against incoming ray
        Vector3f n = entering ? Vector3f(0,0,1) : Vector3f(0,0,-1);

        float cosThetaI = bRec.wi.dot(n);
        float sinThetaTSq = eta*eta * (1 - cosThetaI*cosThetaI);

        bRec.measure = EDiscrete;
        bRec.eta = eta;

        // Total internal reflection
        if (sinThetaTSq > 1.0f) {
            // Reflect across normal
            bRec.wo = bRec.wi - 2.f * bRec.wi.dot(n) * n;
            return Color3f(1.0f);
        }

        float cosThetaT = std::sqrt(1 - sinThetaTSq);

        // Fresnel (Schlick)
        float R0 = (etaI - etaT) / (etaI + etaT);
        R0 *= R0;
        float R = R0 + (1 - R0) * std::pow(1 - cosThetaI, 5);

        if (sample.x() < R) {
            // Reflection
            bRec.wo = bRec.wi - 2.f * bRec.wi.dot(n) * n;
            return Color3f(R);
        } else {
            // Refraction
            bRec.wo = -eta * bRec.wi + (eta * cosThetaI - cosThetaT) * n;
            return Color3f(1.0f - R);
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
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");

NORI_NAMESPACE_END
