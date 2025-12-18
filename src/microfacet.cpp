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
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0) return Color3f(0.0f);

        Vector3f wi = bRec.wi;
        Vector3f wo = bRec.wo;
        Vector3f wh = (wi + wo).normalized();

        float cosThetaI = Frame::cosTheta(wi);
        float cosThetaO = Frame::cosTheta(wo);

        // Diffuse term
        Color3f diffuse = m_kd * INV_PI;

        // Specular term
        float D = Warp::squareToBeckmannPdf(wh, m_alpha);
        float F = fresnel(wi.dot(wh), m_extIOR, m_intIOR);
        float G = smithG1(wi, wh) * smithG1(wo, wh);

        Color3f specular(0.0f);
        float denom = 4.0f * cosThetaI * cosThetaO;
        if (denom > 0.0f)
            specular = Color3f(D * F * G / denom);

        return diffuse + m_ks * specular;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
    	if (Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        Vector3f wh = (bRec.wi + bRec.wo).normalized();

        float pdfSpec = 0.0f;
        float dotOH = bRec.wo.dot(wh);
        if (dotOH > 0.0f) {
            float Jh = 1.0f / (4.0f * dotOH);
            pdfSpec = Warp::squareToBeckmannPdf(wh, m_alpha) * Jh;
        }

        float pdfDiff =
            Warp::squareToCosineHemispherePdf(bRec.wo);

        return m_ks * pdfSpec + (1.0f - m_ks) * pdfDiff;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
    	if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        Point2f sample = _sample;

        // Choose component
        bool specular = sample.x() < m_ks;

        if (!specular) {
            // Diffuse
            sample.x() = (sample.x() - m_ks) / (1.0f - m_ks);
            bRec.wo = Warp::squareToCosineHemisphere(sample);
            bRec.eta = 1.0f;
        } else {
            // Specular
            sample.x() /= m_ks;

            Vector3f wh =
                Warp::squareToBeckmann(sample, m_alpha);

            bRec.wo = reflect(bRec.wi, wh);

            if (Frame::cosTheta(bRec.wo) <= 0)
                return Color3f(0.0f);

            bRec.eta = 1.0f;
        }

        bRec.measure = ESolidAngle;

        return eval(bRec) *
            Frame::cosTheta(bRec.wo) /
            pdf(bRec);

        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;

    float smithG1(const Vector3f &v, const Vector3f &wh) const {
        float cosThetaV = Frame::cosTheta(v);
        float dotVH = v.dot(wh);

        if (dotVH <= 0.0f || cosThetaV <= 0.0f)
            return 0.0f;

        float tanThetaV =
            std::sqrt(1.0f - cosThetaV * cosThetaV) / cosThetaV;

        float b = 1.0f / (m_alpha * tanThetaV);

        if (b < 1.6f)
            return (3.535f * b + 2.181f * b * b) /
                (1.0f + 2.276f * b + 2.577f * b * b);
        else
            return 1.0f;
    }
    static Vector3f reflect(const Vector3f &wi, const Vector3f &n) {
        return 2.0f * wi.dot(n) * n - wi;
    }
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
