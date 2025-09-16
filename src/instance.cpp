#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/shape.h>
#include <nori/transform.h>
#include <iostream>

NORI_NAMESPACE_BEGIN
class Instance : public Shape {
public:
  Instance(const PropertyList &props) {
    // Get transform
    // Get shape
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

  }

  /// Register a child object (e.g. a BSDF) with the mesh
  void addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
    case EShape:
      if (m_bsdf)
        throw NoriException(
            "Instance: tried to register multiple BSDF instances!");
      m_bsdf = static_cast<BSDF *>(obj);
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
  Transform *m_transform = nullptr;
  Shape *m_shape = nullptr;
};

NORI_REGISTER_CLASS(Instance, "instance");
NORI_NAMESPACE_END