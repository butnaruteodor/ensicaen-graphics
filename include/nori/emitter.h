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

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct EmitterQueryRecord {
    Point3f ref;   // shading point x
    Point3f p;     // sampled point y
    Normal3f n;    // normal at y
    Vector3f wi;   // direction x -> y
    float dist;    // |x - y|
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
    virtual Color3f eval(const EmitterQueryRecord &lRec) const = 0;

    virtual Color3f sample(EmitterQueryRecord &lRec,
                            const Point2f &sample,
                            float &pdf) const = 0;

    virtual float pdf(const EmitterQueryRecord &lRec) const = 0;
};

NORI_NAMESPACE_END
