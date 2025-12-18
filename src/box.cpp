#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/shape.h>
#include <iostream>

NORI_NAMESPACE_BEGIN
class Box : public Shape {
public:
  Box(const PropertyList &props) {
    // Axis-aligned unit cube by default [-1,1] in each axis
    m_min = Point3f(-1.f, -1.f, -1.f);
    m_max = Point3f( 1.f,  1.f,  1.f);
  }

  void activate() {
    if (!m_bsdf) {
      /* If no material was assigned, instantiate a diffuse BRDF */
      m_bsdf = static_cast<BSDF *>(
          NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }
  }

  bool rayIntersect(Ray3f &ray, Intersection &its,
                  bool shadowRay = false) const {
    float tmin = ray.mint;
    float tmax = ray.maxt;

    // For each axis, intersect with the slab
    for (int i = 0; i < 3; i++) {
        float invD = 1.0f / ray.d[i];
        float t0 = (m_min[i] - ray.o[i]) * invD;
        float t1 = (m_max[i] - ray.o[i]) * invD;

        if (invD < 0.f)
            std::swap(t0, t1);

        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);

        if (tmax <= tmin)
            return false; // missed the box
    }

    // We hit the box at entry point tmin
    float t = tmin;
    if (t < ray.mint || t > ray.maxt)
        return false;

    if (shadowRay)
        return true;

    // Compute normal of the hit face
    Point3f p = ray(t);
    Normal3f n(0.f, 0.f, 0.f);
    const float eps = 1e-4f;
    // Disabled: compute per-face normals to get uniform color
    // for (int i = 0; i < 3; i++) {
    //     if (std::abs(p[i] - m_min[i]) < eps) n[i] = -1;
    //     else if (std::abs(p[i] - m_max[i]) < eps) n[i] = 1;
    // }
    // Use a constant normal instead (pointing up/out):
    n = Normal3f(0.f, 1.f, 0.f);

    updateRayAndHit(ray, its, t, n);
    return true;
}

  /// Register a child object (e.g. a BSDF) with the mesh
  void addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
    case EBSDF:
      if (m_bsdf)
        throw NoriException(
            "Box: tried to register multiple BSDF instances!");
      m_bsdf = static_cast<BSDF *>(obj);
      break;

    case EEmitter: {
      Emitter *emitter = static_cast<Emitter *>(obj);
      if (m_emitter)
        throw NoriException(
            "Box: tried to register multiple Emitter instances!");
      m_emitter = emitter;
    } break;

    default:
      throw NoriException("Box::addChild(<%s>) is not supported!",
                          classTypeName(obj->getClassType()));
    }
  }

  std::string toString() const {
    return tfm::format(
        "Box[\n"
        "emitter = %s\n"
        "bsdf = %s\n"
        "]",
        (m_emitter) ? indent(m_emitter->toString()) : std::string("null"),
        (m_bsdf) ? indent(m_bsdf->toString()) : std::string("null"));
  }

private:
  void updateRayAndHit(Ray3f &ray, Intersection &its, float t,
                       const Normal3f &n) const {
    ray.maxt = its.t = t;
    its.p = ray(its.t);
    its.uv = Point2f(its.p.x(), its.p.z());
    its.bsdf = m_bsdf;
    its.emitter = m_emitter;
    its.geoFrame = its.shFrame = Frame(n);
  }

private:
  Point3f m_min, m_max; 
  BSDF *m_bsdf = nullptr;       ///< BSDF of the surface
  Emitter *m_emitter = nullptr; ///< Associated emitter, if any
};

NORI_REGISTER_CLASS(Box, "box");
NORI_NAMESPACE_END