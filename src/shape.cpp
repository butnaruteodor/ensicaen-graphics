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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN
const BoundingBox3f Shape::invalidBbox;

const BoundingBox3f &Shape::getBoundingBox() const {
    return Shape::invalidBbox;
}


std::string Intersection::toString() const {
    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  bsdf = %s\n"
        "  emitter = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        bsdf ? bsdf->toString() : std::string("null"),
        emitter ? emitter->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
