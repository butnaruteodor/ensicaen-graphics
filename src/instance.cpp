#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/shape.h>
#include <nori/transform.h>
#include <iostream>
static int count = 0;
NORI_NAMESPACE_BEGIN
class Instance : public Shape {
public:
  Instance(const PropertyList &props) {
    // Get transform
    m_transform = props.getTransform("toWorld");
    //std::cout<<"Here\n"<<m_transform.toString();
  }

  void activate() {
    if (!m_shape) {
      /* If no material was assigned, instantiate a diffuse BRDF */
      m_bsdf = static_cast<BSDF *>(
          NoriObjectFactory::createInstance("diffuse", PropertyList()));
    }
  }

  bool rayIntersect(Ray3f &ray, Intersection &its,
                    bool shadowRay = false) const {
    // Inverse ray and call rayIntersect of child objects
    Ray3f localRay = m_transform.inverse()*ray;
    bool hit = m_shape->rayIntersect(localRay, its, shadowRay);
    if (hit) {
        // Transform intersection data back to world space
        its.p = m_transform*its.p;
        its.shFrame.n = m_transform*its.shFrame.n;
        //its.geoFrame.n = m_transform*its.geoFrame.n;
        ray.maxt = localRay.maxt;
    }
    return hit;
  }

  /// Register a child object (e.g. a BSDF) with the mesh
  void addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
    case EShape:
      if (m_shape)
          throw NoriException(
              "Sphere: tried to register multiple shape instances!");
      m_shape = static_cast<Shape *>(obj);
      break;

    default:
      throw NoriException("Instance::addChild(<%s>) is not supported!",
                          classTypeName(obj->getClassType()));
    }
  }

  std::string toString() const {
    return tfm::format(
        "Instance[\n"
        "emitter = %s\n"
        "bsdf = %s\n"
        "]",
        (m_emitter) ? indent(m_emitter->toString()) : std::string("null"),
        (m_bsdf) ? indent(m_bsdf->toString()) : std::string("null"));
  }

private:
  Transform m_transform;
  Shape *m_shape = nullptr;
  BSDF *m_bsdf = nullptr;       ///< BSDF of the surface
  Emitter *m_emitter = nullptr; ///< Associated emitter, if any
};

NORI_REGISTER_CLASS(Instance, "instance");
NORI_NAMESPACE_END