#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/shape.h>
#include <nori/transform.h>
#include <iostream>

NORI_NAMESPACE_BEGIN

struct IFSMorphism {
    Transform transform;
};

class IFSShape : public Shape {
public:
    IFSShape(const PropertyList &props) {
        m_iterations = props.getInteger("iterations", 5);
        int nMaps = props.getInteger("numMaps", 0);

        for (int i = 1; i <= nMaps; ++i) {
            std::string name = "map" + std::to_string(i);
            Transform T = props.getTransform(name);
            m_maps.push_back({ T.getMatrix() });
        }

        m_bounds = BoundingBox3f(Point3f(-1, -1, -1), Point3f(1, 1, 1));
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
        float tnear, tfar;
        if (!bounds.rayIntersect(ray, tnear, tfar))
            return false;

        if (tnear >= ray.mint && tnear <= ray.maxt) {
            if (shadowRay) return true;
            updateRayAndHit(ray, its, tnear, bounds);
            return true;
        }
        return false;
    }

    // Compute 8 corners of current bounding box
    Point3f corners[8];
    corners[0] = bounds.min;
    corners[1] = Point3f(bounds.max.x(), bounds.min.y(), bounds.min.z());
    corners[2] = Point3f(bounds.min.x(), bounds.max.y(), bounds.min.z());
    corners[3] = Point3f(bounds.min.x(), bounds.min.y(), bounds.max.z());
    corners[4] = Point3f(bounds.max.x(), bounds.max.y(), bounds.min.z());
    corners[5] = Point3f(bounds.max.x(), bounds.min.y(), bounds.max.z());
    corners[6] = Point3f(bounds.min.x(), bounds.max.y(), bounds.max.z());
    corners[7] = bounds.max;

    // Apply all IFS maps
    for (const auto &map : m_maps) {
        BoundingBox3f child;
        for (int i = 0; i < 8; ++i) {
            Point3f tc = map.transform * corners[i];
            child.expandBy(tc);
        }

        float tnear, tfar;
        if (child.rayIntersect(ray, tnear, tfar)) {
            if (intersectRecursive(ray, its, child, depth + 1, shadowRay))
                return true;
        }
    }
    return false;
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
    std::vector<IFSMorphism> m_maps;
};

NORI_REGISTER_CLASS(IFSShape, "ifs")
NORI_NAMESPACE_END
