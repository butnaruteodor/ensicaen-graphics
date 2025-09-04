/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2016 by Loic Simon

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

#if !defined(__NORI_SHAPE_H)
#define __NORI_SHAPE_H

#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>

NORI_NAMESPACE_BEGIN

class Shape;
/**
 * \brief Intersection data structure
 *
 * This data structure records local information about a ray-triangle intersection.
 * This includes the position, traveled ray distance, uv coordinates, as well
 * as well as two local coordinate frames (one that corresponds to the true
 * geometry, and one that is used for shading computations).
 */
struct Intersection {
    /// Position of the surface intersection
    Point3f p;
    /// Unoccluded distance along the ray
    float t;
    /// UV coordinates, if any
    Point2f uv;
    /// Shading frame (based on the shading normal)
    Frame shFrame;
    /// Geometric frame (based on the true geometry)
    Frame geoFrame;
    /// pointer to the associated bsdf
    const BSDF *bsdf;
    /// pointer to the associated emitter
    const Emitter *emitter;

    /// Create an uninitialized intersection record
    Intersection() : bsdf(nullptr), emitter(nullptr) { }

    /// Transform a direction vector into the local shading frame
    Vector3f toLocal(const Vector3f &d) const {
        return shFrame.toLocal(d);
    }

    /// Transform a direction vector from local to world coordinates
    Vector3f toWorld(const Vector3f &d) const {
        return shFrame.toWorld(d);
    }

    /// Is this intersection an area emitter?
    bool isEmitter() const { return emitter != nullptr; }

    /// Return a human-readable summary of the intersection record
    std::string toString() const;
};

/**
 * \brief Abstract shape
 *
 * This class is an interface for shapes
 */
class Shape : public NoriObject {
public:
    /// Release all memory
    virtual ~Shape() {}

    /**
     * \brief Intersect a ray against the shape
     *
     * Detailed information about the intersection, if any, will be
     * stored in the provided \ref Intersection data record. 
     *
     * The <tt>shadowRay</tt> parameter specifies whether this detailed
     * information is really needed. When set to \c true, the 
     * function just checks whether or not there is occlusion, but without
     * providing any more detail (i.e. \c its will not be filled with
     * contents). This is usually much faster.
     *
     * \return \c true If an intersection was found
     */
    virtual bool rayIntersect(Ray3f &ray, Intersection &its, 
        bool shadowRay = false) const = 0;

    //// Return an axis-aligned bounding box of the entire shape
    virtual const BoundingBox3f &getBoundingBox() const;

    /**
     * \brief Return the type of object (i.e. Shape/BSDF/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EShape; }

protected:
    /// Create an empty shape
    Shape() {}

    std::string m_name;                  ///< Identifying name
    static const BoundingBox3f invalidBbox;

};


NORI_NAMESPACE_END

#endif /* __NORI_SHAPE_H */

