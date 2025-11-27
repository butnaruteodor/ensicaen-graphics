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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

float tentWarp(const float u) {
    if(u<0.5f){
        return sqrt(2*u)-1;
    }else{
        return 1-sqrt(2-2*u);
    }
}

Point2f Warp::squareToTent(const Point2f &sample) {
    return Point2f(tentWarp(sample.x()),tentWarp(sample.y()));
}

float Warp::squareToTentPdf(const Point2f &p) {
    float px = abs(p.x())<1 ? 1-abs(p.x()) : 0.0f;
    float py = abs(p.y())<1 ? 1-abs(p.y()) : 0.0f;

    return px * py;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float theta = 2*M_PI*sample.x();
    float r = sqrt(sample.y());
    return Point2f(r*cos(theta),r*sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    if (p.x()*p.x() + p.y()*p.y() <= 1.0f)
        return 1.0f / M_PI;
    else
        return 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float theta = 2*M_PI*sample.x();
    float phi = acos(2*sample.y()-1)-M_PI/2;
    float x = cos(phi)*cos(theta);
    float y = cos(phi)*sin(theta); 
    float z = sin(phi);
    return Vector3f(x,y,z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if (std::abs(v.squaredNorm() - 1.0f) > 1e-6f)
        return 0.0f;
    return 1/(4*M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float z = sample.y();                      // z ∈ [0,1]
    float r = std::sqrt(std::max(0.f, 1 - z*z));
    float phi = 2 * M_PI * sample.x();

    return Vector3f(
        r * std::cos(phi),
        r * std::sin(phi),
        z
    );
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    // Check that v is in the upper hemisphere AND normalized
    if (v.z() >= 0.0f && std::abs(v.norm() - 1.0f) < 1e-6f)
        return 1.0f / (2.0f * M_PI);
    return 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    // Sample uniform disk
    float r = std::sqrt(sample.y());
    float theta = 2 * M_PI * sample.x();

    float x = r * std::cos(theta);
    float y = r * std::sin(theta);
    float z = std::sqrt(std::max(0.f, 1 - x*x - y*y));

    return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    if (v.z() >= 0.0f && std::abs(v.norm() - 1.0f) < 1e-6f)
        return v.z() / M_PI;
    return 0.0f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float tan2Theta = -alpha * alpha * std::log(1.0f - sample.x());
    float phi = 2.0f * M_PI * sample.y();

    float cosTheta = 1.0f / std::sqrt(1.0f + tan2Theta);
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

    return Vector3f(
        sinTheta * std::cos(phi),
        sinTheta * std::sin(phi),
        cosTheta
    );
}
float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0.0f)
        return 0.0f;

    float cosTheta = m.z();
    float cosTheta2 = cosTheta * cosTheta;
    float cosTheta4 = cosTheta2 * cosTheta2;

    float tan2Theta = (1.0f - cosTheta2) / cosTheta2;

    float D = std::exp(-tan2Theta / (alpha * alpha)) /
              (M_PI * alpha * alpha * cosTheta4);

    return D * cosTheta;
}

NORI_NAMESPACE_END
