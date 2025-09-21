#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/shape.h>
#include <iostream>

NORI_NAMESPACE_BEGIN

class IFSShape : public Shape {
public:
    IFSShape(const PropertyList &props) {
        m_iterations = props.getInteger("iterations", 5); // recursion depth
        m_bounds = BoundingBox3f(Point3f(-1, -1, -1), Point3f(1, 1, 1)); // root cube
    }

    void activate() {
        if (!m_bsdf) {
            m_bsdf = static_cast<BSDF *>(
                NoriObjectFactory::createInstance("diffuse", PropertyList()));
        }
    }

    bool rayIntersect(Ray3f &ray, Intersection &its,
                      bool shadowRay = false) const {
        float tnear, tfar;
        if (!m_bounds.rayIntersect(ray, tnear, tfar)) {
            return false; // ray misses root bounding box
        }

        return intersectRecursive(ray, its, m_bounds, 0, shadowRay);
    }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException("IFSShape: multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter: {
            Emitter *emitter = static_cast<Emitter *>(obj);
            if (m_emitter)
                throw NoriException("IFSShape: multiple Emitters!");
            m_emitter = emitter;
        } break;

        default:
            throw NoriException("IFSShape::addChild(<%s>) not supported!",
                                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "IFSShape[\n"
            "iterations = %d\n"
            "emitter = %s\n"
            "bsdf = %s\n"
            "]",
            m_iterations,
            (m_emitter) ? indent(m_emitter->toString()) : std::string("null"),
            (m_bsdf) ? indent(m_bsdf->toString()) : std::string("null"));
    }

private:
    bool intersectRecursive(Ray3f &ray, Intersection &its,
                            const BoundingBox3f &bounds,
                            int depth, bool shadowRay) const {
        if (depth >= m_iterations) {
            // At recursion limit → treat as hit
            float tnear, tfar;
            if (!bounds.rayIntersect(ray, tnear, tfar))
                return false;

            if (tnear >= ray.mint && tnear <= ray.maxt) {
                if (shadowRay)
                    return true;

                updateRayAndHit(ray, its, tnear, bounds);
                return true;
            }
            return false;
        }

        // Generate IFS children (example: Menger sponge splits cube into 20 smaller ones)
        std::vector<BoundingBox3f> children = subdivide(bounds);

        bool hit = false;
        for (const auto &child : children) {
            float tnear, tfar;
            if (child.rayIntersect(ray, tnear, tfar)) {
                if (intersectRecursive(ray, its, child, depth + 1, shadowRay)) {
                    hit = true;
                    if (shadowRay) return true; // early exit
                }
            }
        }
        return hit;
    }

    /// Example subdivision rule (Menger sponge-like: keep 20 subcubes)
    std::vector<BoundingBox3f> subdivide(const BoundingBox3f &bounds) const {
    std::vector<BoundingBox3f> children;
    Vector3f size = bounds.getExtents() / 3.0f;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                if ((i == 1 && j == 1) || (i == 1 && k == 1) || (j == 1 && k == 1)) {
                    continue; // remove center cubes
                }
                Point3f min = bounds.min + Vector3f(i, j, k).cwiseProduct(size);
                Point3f max = min + size;
                children.push_back(BoundingBox3f(min, max));
            }
        }
    }
    return children;
}

    void updateRayAndHit(Ray3f &ray, Intersection &its, float t,
                         const BoundingBox3f &bounds) const {
        ray.maxt = its.t = t;
        its.p = ray(its.t);
        its.uv = Point2f(its.p.x(), its.p.z());
        its.bsdf = m_bsdf;
        its.emitter = m_emitter;

        // crude normal = face normal of bounding box
        Vector3f n(0, 1, 0);
        if (fabs(its.p.x() - bounds.min.x()) < 1e-4) n = Vector3f(-1, 0, 0);
        else if (fabs(its.p.x() - bounds.max.x()) < 1e-4) n = Vector3f(1, 0, 0);
        else if (fabs(its.p.y() - bounds.min.y()) < 1e-4) n = Vector3f(0, -1, 0);
        else if (fabs(its.p.y() - bounds.max.y()) < 1e-4) n = Vector3f(0, 1, 0);
        else if (fabs(its.p.z() - bounds.min.z()) < 1e-4) n = Vector3f(0, 0, -1);
        else if (fabs(its.p.z() - bounds.max.z()) < 1e-4) n = Vector3f(0, 0, 1);

        its.geoFrame = its.shFrame = Frame(n.normalized());
    }

private:
    BSDF *m_bsdf = nullptr;
    Emitter *m_emitter = nullptr;
    int m_iterations;
    BoundingBox3f m_bounds;
};

NORI_REGISTER_CLASS(IFSShape, "ifs")
NORI_NAMESPACE_END
