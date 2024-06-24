
#include "OpticalSystem.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Camera/Camera.h"
#include "FunctionLayer/Ray/Ray.h"
#include "ResourceLayer/Factory.h"
#include <__memory/shared_ptr.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

aperture::aperture(const Json &json) {
    this->thick = fetchOptional(json, "thick", 0.0) / 1000;
    this->ap = fetchOptional(json, "ap", 0.0) / 1000;
    this->dis2film = fetchOptional(json, "dis2Film", 0.0);
}
float aperture::get_ap() const { return this->ap; }

Vector3f aperture::getNormAt(Vector3f hitPoint) const {
    return normalize(lensDir);
}

float aperture::getHitTime(const Ray &ray) const {
    float T = (this->dis2film - dot(ray.origin.v3f(), lensDir)) /
              dot(ray.direction, lensDir);
    Vector3f hitPoint = ray.origin.v3f() + T * ray.direction;
    Vector3f V2A =
        hitPoint - normalize(lensDir) * dot(hitPoint, normalize(lensDir));
    if (T < 0 || V2A.length() > this->ap) {
        return INFINITY;
    } else {
        return T;
    }
}

sphereLensSurface::sphereLensSurface(const Json &json) {
    this->radius = fetchOptional(json, "radius", 0.0) / 1000;
    this->thick = fetchOptional(json, "thick", 0.0) / 1000;
    this->nd = fetchOptional(json, "nd", 1.000293120);
    this->V_no = fetchOptional(json, "V_no", 89.30);
    this->ap = fetchOptional(json, "ap", 0.0) / 1000;
    this->sphereCenter =
        (fetchOptional(json, "dis2Film", 0.0) - radius) * lensDir;
    this->theta = asin(this->ap / (2 * this->radius));
    /*
    std::cout << "\nradius : " << this->radius << "\nthick : " << this->thick
              << "\nnd : " << this->nd << "\nV_no : " << this->V_no
              << "\nap : " << this->ap << "\ntheta : " << this->theta << "\n";*/
}
float sphereLensSurface::get_ap() const { return this->ap; }
sphereLensSurface::sphereLensSurface(float radius, float thick, float nd,
                                     float V_no, float ap, float dis2Film) {
    this->radius = radius / 1000;
    this->thick = thick / 1000;
    this->nd = nd;
    this->V_no = V_no;
    this->ap = ap / 1000;
    this->sphereCenter = (dis2Film - radius) * lensDir;
    this->theta = asin(this->ap / (2 * this->radius));
}
float sphereLensSurface::getNd() const { return this->nd; };
float sphereLensSurface::getV_no() const { return this->V_no; };

float aperture::getNd() const { return AIR_ND; }
float aperture::getV_no() const { return AIR_V_NO; }

Vector3f getRefractDri(const Ray &rayIn, Vector3f norm, float nd_from,
                       float V_from, float nd_to, float V_to) {
    float n_from, n_to;
    if (rayIn.wavelength == 0) {
        n_from = nd_from;
        n_to = nd_to;
    } else {
        n_from = nd_from + ((nd_from - 1) / V_from) *
                               ((1 / (rayIn.wavelength * rayIn.wavelength)) -
                                (1 / (589.3 * 589.3)));
        n_to = nd_to + ((nd_to - 1) / V_to) *
                           ((1 / (rayIn.wavelength * rayIn.wavelength)) -
                            (1 / (589.3 * 589.3)));
    }
    Vector3f nNorm = normalize(norm);
    Vector3f directionIn = rayIn.direction;
    if (dot(nNorm, directionIn) < 0)
        nNorm = -nNorm;
    float cos_theta_in = dot(directionIn, nNorm) / directionIn.length();
    float sin_theta_in = sqrt(1 - (cos_theta_in * cos_theta_in));
    float sin_theta_out = (sin_theta_in * n_from) / n_to;
    float cos_theta_out = sqrt(1 - (sin_theta_out * sin_theta_out));
    return normalize(
        directionIn +
        nNorm * (((directionIn - nNorm * dot(nNorm, directionIn)).length() /
                  sin_theta_out) *
                     cos_theta_out -
                 dot(nNorm, directionIn)));
}

Vector3f aperture::sampleOnSurface(Vector2f sample) const {
    double u = sample[0];
    double v = sample[1];
    double theta = 2 * M_PI * u;
    double r = this->ap * sqrt(v);
    return Vector3f(this->dis2film, r * cos(theta), r * sin(theta));
}

float sphereLensSurface::getHitTime(const Ray &ray) const {
    float A = dot(ray.direction, ray.direction);
    float B = 2 * dot(ray.direction, (ray.origin.v3f() - sphereCenter));
    float C = dot((ray.origin.v3f() - sphereCenter),
                  (ray.origin.v3f() - sphereCenter)) -
              radius * radius;
    float delta = B * B - 4 * A * C;
    if (delta < 0) {
        return INFINITY;
    } else {
        float T0 = (-B - sqrt(delta)) / (2 * A);
        float T1 = (-B + sqrt(delta)) / (2 * A);
        Vector3f tempHitPoint0 = ray.origin.v3f() + T0 * ray.direction;
        Vector3f tempHitPoint1 = ray.origin.v3f() + T1 * ray.direction;
        if (dot(tempHitPoint0 - this->sphereCenter, lensDir) * this->radius < 0)
            T0 = INFINITY;
        if (dot(tempHitPoint1 - this->sphereCenter, lensDir) * this->radius < 0)
            T1 = INFINITY;
        if (T0 < 0)
            T0 = INFINITY;
        if (T1 < 0)
            T1 = INFINITY;
        float T = std::min(T0, T1);
        return T;
    }
}

Vector3f sphereLensSurface::getNormAt(Vector3f hitPoint) const {
    return normalize(hitPoint - this->sphereCenter);
}

Vector3f sphereLensSurface::sampleOnSurface(Vector2f sample) const {
    float sTheta = sin(sample[0] * asin(this->theta / PI)) * PI;
    float sPhi = sample[1] * 2 * PI;
    return this->sphereCenter + this->radius * cos(sTheta) * lensDir +
           Vector3f(0, this->radius * sin(sTheta) * sin(sPhi),
                    this->radius * sin(sTheta) * cos(sPhi));
}
//* 除镜片参数之外认为渲染器中1代表1米
//* 镜片参数中1代表1mm
OpticalSystem::OpticalSystem(const Json &json) : Camera(json) {
    // float lensScalingFactor = fetchOptional(json, "lensScalingFactor", 1.0);
    this->camPosition = fetchOptional(fetchOptional(json, "transform", Json(0)),
                                      "position", Vector3f(0));
    //* lensDir = Vector3f(1, 0, 0);
    this->camTo =
        normalize(fetchOptional(fetchOptional(json, "transform", Json(0)),
                                "lookAt", Vector3f(0)) -
                  camPosition);
    //* lensUp = Vector3f(0, 0, 1);
    this->camUp = normalize(
        fetchOptional(fetchOptional(json, "transform", Json(0)), "up", lensUp) -
        camTo *
            dot(camTo, fetchOptional(fetchOptional(json, "transform", Json(0)),
                                     "up", lensUp)));
    //* other Ax = (0, 1, 0)
    this->camY = normalize(cross(this->camUp, this->camTo));
    this->sampler = Factory::construct_class<Sampler>(json["sampler"]);
    this->diagonal =
        fetchOptional(fetchOptional(json, "film", Json(0)), "diagonal", 0.0443);
    float pixelLength =
        sqrt(pow(this->diagonal, 2) /
             (pow(this->film->size[0], 2) + pow(this->film->size[1], 2)));

    this->dx = Vector3f(0, 1, 0) * pixelLength;
    this->dy = lensUp * pixelLength;
    this->leftDownCorner = Vector3f(0, 0, 0) -
                           (0.5 * this->film->size[0] * this->dx) -
                           (0.5 * this->film->size[1] * this->dy);
    std::vector<Json> lens = fetchOptional(json, "lens", std::vector<Json>(0));
    float totalDis = 0;
    for (int i = lens.size() - 1; i >= 0; i--) {
        totalDis += fetchOptional(lens[i], "thick", 0.0) / 1000;
        // std::cout << fetchOptional(lens[i], "type", std::string("null"))
        //           << "\n";
        if (fetchOptional(lens[i], "type", std::string("null")) ==
            std::string("aperture"))
            this->apertureId = i;
        lens[i]["dis2Film"] = totalDis;
        // for (auto &[key, value] : lens[i].items()) {
        //     if (value.is_number()) {
        //         lens[i][key] =
        //             fetchOptional(lens[i], key, 0.0) * lensScalingFactor;
        //         std::cout << key << " : " << value << std::endl;
        //     }
        // }
    }
    for (int i = 0; i < lens.size(); i++) {
        this->lensSurfaceList.push_back(
            Factory::construct_class<lensSurface>(lens[i]));
    }
    std::cout << "has " << this->lensSurfaceList.size() << " lens\n";
    std::cout << "cam position : ";
    this->camPosition.debugPrint();
    std::cout << "aperture at " << this->apertureId << "\n";
    std::string focusMode =
        fetchOptional(json, "focusMode", std::string("Center"));
    if (focusMode == "Point") {
        this->focusMode = focusMode::Point;
        this->focusPoint = fetchOptional(json, "focusPoint", Vector3f(0, 0, 0));
    } else if (focusMode == "Center") {
        this->focusMode = focusMode::Center;
    } else {
        this->focusMode = focusMode::Point;
        this->focusPoint = Vector3f(0, 0, 0);
        this->focusOffset = 0;
    }
}

#define MAX_TRACE_INLENS 50
#define IS_ZERO_3f(x)                                                          \
    ((x[0] == 0 && x[1] == 0 && x[2] == 0) ||                                  \
     (std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2])))
#define IS_NAN_3f(x) (std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2]))
#define VALID_FLOAT(x) (not std::isnan(x) and not std::isinf(x))

float getReflectRatio(Vector3f norm, Vector3f rayDir, float nd_from,
                      float V_from, float nd_to, float V_to) {
    return -1;
}

Vector3f getReflectDir(Vector3f norm, Vector3f rayDir) {
    Vector3f NN = dot(norm, rayDir) > 0 ? -normalize(norm) : normalize(norm);
    return rayDir + NN * dot(NN, rayDir);
}

const int MAX_TRACE_TIMES = 50;

Ray OpticalSystem::getRayOut(const Ray &rayIn) const {
    Ray rayInLens = rayIn;
    float T = INFINITY;
    int nextLens = -1, thisLens = -1;
    int TRACE_TIMES = 0;
    while (TRACE_TIMES < MAX_TRACE_TIMES) {
        TRACE_TIMES++;
        bool hasHitted = 0;
        T = INFINITY;
        for (int i = 0; i < this->lensSurfaceList.size(); i++) {
            float t = this->lensSurfaceList[i]->getHitTime(rayInLens);
            // std::cout << i << " hit time : " << t << "\n";
            if (t < T && i != thisLens) {
                hasHitted = 1;
                T = t;
                nextLens = i;
            }
        }
        // std::cout << "is hit ? " << hasHitted << ", " << thisLens << " to "
        //           << nextLens << "\n";
        if (not hasHitted) {
            if (thisLens == 0 || thisLens == this->lensSurfaceList.size() - 1) {
                return rayInLens;
            } else {
                return Ray(Point3f(NAN), Vector3f(NAN));
            }
        } else {
            float nd_from, V_from, nd_to, V_to;
            if (dot(lensDir, rayInLens.direction) > 0) {
                nd_from = this->lensSurfaceList[nextLens]->getNd();
                V_from = this->lensSurfaceList[nextLens]->getV_no();
                if (nextLens == 0) {
                    nd_to = AIR_ND;
                    V_to = AIR_V_NO;
                } else {
                    nd_to = this->lensSurfaceList[nextLens - 1]->getNd();
                    V_to = this->lensSurfaceList[nextLens - 1]->getV_no();
                }
            } else {
                if (thisLens == -1) {
                    nd_from = AIR_ND;
                    V_from = AIR_V_NO;
                } else {
                    nd_from = this->lensSurfaceList[thisLens]->getNd();
                    V_from = this->lensSurfaceList[thisLens]->getV_no();
                }
                nd_to = this->lensSurfaceList[nextLens]->getNd();
                V_to = this->lensSurfaceList[nextLens]->getV_no();
            }
            Vector3f hitPoint =
                rayInLens.origin.v3f() + T * rayInLens.direction;
            Vector3f normAtHitPoint =
                normalize(this->lensSurfaceList[nextLens]->getNormAt(hitPoint));
            Vector3f nextDir;
            if (this->sampler->next1D() <
                getReflectRatio(normAtHitPoint, rayInLens.direction, nd_from,
                                V_from, nd_to, V_to)) {
                nextDir = getReflectDir(normAtHitPoint, rayInLens.direction);
            } else {
                nextDir = getRefractDri(rayInLens, normAtHitPoint, nd_from,
                                        V_from, nd_to, V_to);
            }
            float waveLength = rayInLens.wavelength;
            rayInLens = Ray(Point3f(hitPoint), nextDir);
            rayInLens.wavelength = waveLength;
        }
        thisLens = nextLens;
    }
    return Ray(Point3f(NAN), Vector3f(NAN));
}
void OpticalSystem::autoFocus(const Scene &scene) {
    std::cout << "autoFocus...\n";
    if (this->focusMode == focusMode::Point) {
        this->autoFocus(this->focusPoint);
    } else if (this->focusMode == focusMode::Center) {
        Ray centerRay = Ray(Point3f(this->camPosition), Vector3f(this->camTo));
        std::optional<Intersection> centerIntersection =
            scene.rayIntersect(centerRay);
        this->focusPoint = centerIntersection->position.v3f();
        std::cout << "Center focus point : ";
        this->focusPoint.debugPrint();
        this->autoFocus(this->focusPoint);
    } else {
        this->autoFocus(this->focusPoint);
    }
}
void OpticalSystem::autoFocus(Vector3f focusPoint) {
    float distance =
        dot(focusPoint - this->camPosition, normalize(this->camTo));
    std::cout << "focuc distance : " << distance << "\n";
    const int sampleTimes = 1000;
    for (int i = 0; i < sampleTimes; i++) {
    RETRY_FOCUS_SAMPLE:
        Point3f Origin =
            Point3f(normalize(lensDir) * dot(normalize(this->camTo),
                                             focusPoint - this->camPosition));
        Point3f Dest = Point3f(
            this->lensSurfaceList[0]->sampleOnSurface(this->sampler->next2D()));
        // Origin.debugPrint();
        Ray rayInLens = Ray(Origin, Dest);
        // rayInLens.origin.debugPrint();
        // rayInLens.direction.debugPrint();
        rayInLens.wavelength = 0;
        rayInLens = this->getRayOut(rayInLens);
        // rayInLens.origin.debugPrint();
        // rayInLens.direction.debugPrint();
        if (IS_NAN_3f(rayInLens.origin) || IS_NAN_3f(rayInLens.direction))
            goto RETRY_FOCUS_SAMPLE;
        Vector3f publicVerticalVec = cross(lensDir, rayInLens.direction);
        Vector3f surfadeVerticalVec = cross(lensDir, publicVerticalVec);
        float T = -(dot(rayInLens.origin.v3f(), surfadeVerticalVec) /
                    dot(rayInLens.direction, surfadeVerticalVec));
        this->focusOffset +=
            dot(normalize(lensDir),
                rayInLens.origin.v3f() + T * rayInLens.direction);
    }
    this->focusOffset /= sampleTimes;
    std::cout << "focucOffset : " << this->focusOffset << "\n";
}

Ray OpticalSystem::sampleRay(const CameraSample &sample, Vector2f NDC) const {
    Vector3f startFrom =
        this->leftDownCorner +
        this->dx * (NDC[0] * this->film->size[0] + sample.xy[0]) +
        this->dy * (NDC[1] * this->film->size[1] + sample.xy[1]) +
        this->focusOffset * normalize(lensDir);
    Vector3f pointOnLen0;
    Ray rayInLens;
RETRY_GENERATE:
    // pointOnLen0 =
    //     this->lensSurfaceList.back()->sampleOnSurface(this->sampler->next2D());
    pointOnLen0 = this->lensSurfaceList[this->apertureId]->sampleOnSurface(
        this->sampler->next2D());
    rayInLens = Ray(Point3f(startFrom), Point3f(pointOnLen0));
    float wavelength = rayInLens.wavelength;
    rayInLens.wavelength = 0;
    rayInLens = this->getRayOut(rayInLens);
    rayInLens.wavelength = wavelength;
    if (IS_NAN_3f(rayInLens.origin) || IS_NAN_3f(rayInLens.direction)) {
        goto RETRY_GENERATE;
    }
    Vector3f outDir = rayInLens.direction[0] * this->camTo +
                      rayInLens.direction[1] * this->camY +
                      rayInLens.direction[2] * this->camUp;
    Point3f outOri = this->camPosition + rayInLens.origin[0] * this->camTo +
                     rayInLens.origin[1] * this->camY +
                     rayInLens.origin[2] * this->camUp;
    Ray rayOut = Ray(outOri, outDir);
    rayOut.wavelength = rayInLens.wavelength;
    // if ((NDC - Vector2f(0.5, 0.5)).len() < 0.01) {
    //     rayInLens.origin.debugPrint();
    //     rayInLens.direction.debugPrint();
    // }
    return rayOut;
}
Ray OpticalSystem::sampleRayDifferentials(const CameraSample &sample,
                                          Vector2f NDC) const {
    return this->sampleRay(sample, NDC);
}

REGISTER_CLASS(OpticalSystem, "opticalSystem")
REGISTER_CLASS(sphereLensSurface, "sphereSurface")
REGISTER_CLASS(aperture, "aperture")
