
#include "OpticalSystem.h"
#include "CoreLayer/Math/Geometry.h"
#include "FunctionLayer/Camera/Camera.h"
#include "FunctionLayer/Ray/Ray.h"
#include "ResourceLayer/Factory.h"
#include <algorithm>
#include <cmath>
#include <memory>

aperture::aperture(const Json &json) {
    this->thick = fetchOptional(json, "thick", 0.0) / 1000;
    this->ap = fetchOptional(json, "ap", 0.0) / 1000;
    this->dis2film = fetchOptional(json, "dis2Film", 0.0);
}
float aperture::getV_ap() const { return this->ap; }
Ray aperture::getRefractRay(const Ray &rayIn, float outsideNd,
                            float outsideV_no) const {
    // std::cout << "aperture\n";
    float T = (this->dis2film - dot(rayIn.origin.v3f(), lensDir)) /
              dot(rayIn.direction, lensDir);
    Vector3f hitPoint = rayIn.origin.v3f() + T * rayIn.direction;
    if ((hitPoint - lensDir * dot(hitPoint, lensDir)).length() > this->ap) {
        return Ray(Point3f(0), Vector3f(0));
    } else {
        return rayIn;
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
float sphereLensSurface::getV_ap() const { return this->ap; }
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

Vector3f getRefractDri(const Ray &rayIn, Vector3f norm, float nd_in, float V_in,
                       float nd_out, float V_out) {
    float n_in, n_out;
    if (rayIn.wavelength == 0) {
        n_in = nd_in;
        n_out = nd_out;
    } else {
        n_in = nd_in + ((nd_in - 1) / V_in) *
                           ((1 / (rayIn.wavelength * rayIn.wavelength)) -
                            (1 / (589.3 * 589.3)));
        n_out = nd_out + ((nd_out - 1) / V_out) *
                             ((1 / (rayIn.wavelength * rayIn.wavelength)) -
                              (1 / (589.3 * 589.3)));
    }
    Vector3f nNorm = normalize(norm);
    if (dot(nNorm, rayIn.direction) < 0)
        nNorm = -nNorm;
    float sin_theta_in =
        (rayIn.direction - nNorm * dot(nNorm, rayIn.direction)).length() /
        rayIn.direction.length();
    float sin_theta_out = (sin_theta_in * n_in) / n_out;
    float cos_theta_out = 1 - (sin_theta_out * sin_theta_out);
    float tan_theta_out = sin_theta_out / cos_theta_out;
    return normalize(
        rayIn.direction +
        (((rayIn.direction - nNorm * dot(nNorm, rayIn.direction)).length() /
          tan_theta_out) -
         dot(nNorm, rayIn.direction)) *
            nNorm);
}
Ray aperture::getReflectRay(const Ray &rayIn, float outsideNd,
                            float outsideV_no) const {
    return Ray(Point3f(0), Vector3f(0));
}
float aperture::getReflectRatio(const Ray &rayIn, float outsideNd,
                                float outsideV_no) const {
    return 0;
}
Vector3f aperture::sampleOnSurface(Vector2f sample) const {
    double u = sample[0];
    double v = sample[1];
    double theta = 2 * M_PI * u;
    double r = this->ap * sqrt(v);
    return Vector3f(this->dis2film, r * cos(theta), r * sin(theta));
}
float sphereLensSurface::getReflectRatio(const Ray &rayIn, float outsideNd,
                                         float outsideV_no) const {
    return 0;
}

Ray sphereLensSurface::getReflectRay(const Ray &rayIn, float outsideNd,
                                     float outsideV_no) const {
    float A = dot(rayIn.direction, rayIn.direction);
    float B = 2 * dot(rayIn.direction, (rayIn.origin.v3f() - sphereCenter));
    float C = dot((rayIn.origin.v3f() - sphereCenter),
                  (rayIn.origin.v3f() - sphereCenter)) -
              radius * radius;
    float delta = B * B - 4 * A * C;
    if (delta < 0) {
        return Ray(Point3f(0), Vector3f(0));
    } else {
        float T0 = (-B - sqrt(delta)) / (2 * A);
        float T1 = (-B + sqrt(delta)) / (2 * A);
        if (T0 < 0 && T1 < 0) {
            return Ray(Point3f(0), Vector3f(0));
        } else {
            if (this->radius > 0) {
                if (dot(rayIn.origin.v3f() + (T0 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) > 0 &&
                    T0 > 0) {
                    ;
                } else {
                    T0 = INFINITY;
                }
                if (dot(rayIn.origin.v3f() + (T1 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) > 0 &&
                    T1 > 0) {
                    ;
                } else {
                    T1 = INFINITY;
                }
            } else {
                if (dot(rayIn.origin.v3f() + (T0 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) < 0 &&
                    T0 > 0) {
                    ;
                } else {
                    T0 = INFINITY;
                }
                if (dot(rayIn.origin.v3f() + (T1 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) < 0 &&
                    T1 > 0) {
                    ;
                } else {
                    T1 = INFINITY;
                }
            }
            float T = std::min(T0, T1);
            if (T == INFINITY) {
                return Ray(Point3f(0), Vector3f(0));
            }
            Vector3f hitPoint = rayIn.origin.v3f() + rayIn.direction * T;
            if ((hitPoint - dot(hitPoint, lensDir) * lensDir).length() >
                this->ap) {
                return Ray(Point3f(0), Vector3f(0));
            } else {
                Vector3f normAtHitPoint = hitPoint - sphereCenter;
                Vector3f outDir;
                Vector3f nNorm = normalize(
                    dot(normAtHitPoint, rayIn.direction) > 0 ? -normAtHitPoint
                                                             : normAtHitPoint);
                outDir = rayIn.direction + nNorm * dot(nNorm, rayIn.direction);
                Ray ret = Ray(Point3f(hitPoint), normalize(outDir));
                ret.wavelength = rayIn.wavelength;
                return ret;
            }
        }
    }
}
Ray sphereLensSurface::getRefractRay(const Ray &rayIn, float outsideNd,
                                     float outsideV_no) const {
    // std::cout << "sphereLensSurface\n";
    float A = dot(rayIn.direction, rayIn.direction);
    float B = 2 * dot(rayIn.direction, (rayIn.origin.v3f() - sphereCenter));
    float C = dot((rayIn.origin.v3f() - sphereCenter),
                  (rayIn.origin.v3f() - sphereCenter)) -
              radius * radius;
    float delta = B * B - 4 * A * C;
    if (delta < 0) {
        return Ray(Point3f(0), Vector3f(0));
    } else {
        float T0 = (-B - sqrt(delta)) / (2 * A);
        float T1 = (-B + sqrt(delta)) / (2 * A);
        if (T0 < 0 && T1 < 0) {
            return Ray(Point3f(0), Vector3f(0));
        } else {
            if (this->radius > 0) {
                if (dot(rayIn.origin.v3f() + (T0 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) > 0 &&
                    T0 > 0) {
                    ;
                } else {
                    T0 = INFINITY;
                }
                if (dot(rayIn.origin.v3f() + (T1 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) > 0 &&
                    T1 > 0) {
                    ;
                } else {
                    T1 = INFINITY;
                }
            } else {
                if (dot(rayIn.origin.v3f() + (T0 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) < 0 &&
                    T0 > 0) {
                    ;
                } else {
                    T0 = INFINITY;
                }
                if (dot(rayIn.origin.v3f() + (T1 * rayIn.direction) -
                            this->sphereCenter,
                        lensDir) < 0 &&
                    T1 > 0) {
                    ;
                } else {
                    T1 = INFINITY;
                }
            }
            float T = std::min(T0, T1);
            if (T == INFINITY) {
                return Ray(Point3f(0), Vector3f(0));
            }
            Vector3f hitPoint = rayIn.origin.v3f() + rayIn.direction * T;
            if ((hitPoint - dot(hitPoint, lensDir) * lensDir).length() >
                this->ap) {
                return Ray(Point3f(0), Vector3f(0));
            } else {
                Vector3f normAtHitPoint = hitPoint - sphereCenter;
                Vector3f outDir;
                if (dot(rayIn.direction, lensDir) > 0) {
                    outDir = getRefractDri(rayIn, normAtHitPoint, this->nd,
                                           this->V_no, outsideNd, outsideV_no);
                } else {
                    outDir = getRefractDri(rayIn, normAtHitPoint, outsideNd,
                                           outsideV_no, this->nd, this->V_no);
                }
                Ray ret = Ray(Point3f(hitPoint), outDir);
                ret.wavelength = rayIn.wavelength;
                return ret;
            }
        }
    }
}
Vector3f sphereLensSurface::sampleOnSurface(Vector2f sample) const {
    // std::cout << "sampleOnSurface : \n";
    float sTheta = sin(sample[0] * asin(this->theta / PI)) * PI;
    float sPhi = sample[1] * 2 * PI;
    //(this->sphereCenter + this->radius * cos(sTheta) * lensDir).debugPrint();
    // Vector3f(0, this->radius * sin(sTheta) * sin(sPhi),
    //         this->radius * sin(sTheta) * cos(sPhi))
    //    .debugPrint();
    //(this->sphereCenter + this->radius * cos(sTheta) * lensDir +
    // Vector3f(0, this->radius * sin(sTheta) * sin(sPhi),
    //          this->radius * sin(sTheta) * cos(sPhi)))
    //    .debugPrint();
    // std::cout << "----\n";
    //(this->sphereCenter + this->radius * cos(sTheta) * lensDir +
    // Vector3f(0, this->radius * sin(sTheta) * sin(sPhi),
    //          this->radius * sin(sTheta) * cos(sPhi)))
    //    .debugPrint();
    return this->sphereCenter + this->radius * cos(sTheta) * lensDir +
           Vector3f(0, this->radius * sin(sTheta) * sin(sPhi),
                    this->radius * sin(sTheta) * cos(sPhi));
}
//* 除镜片参数之外认为渲染器中1代表1米
//* 镜片参数中1代表1mm
OpticalSystem::OpticalSystem(const Json &json) : Camera(json) {
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
        lens[i]["dis2Film"] = totalDis;
    }
    for (int i = 0; i < lens.size(); i++) {
        this->lensSurfaceList.push_back(
            Factory::construct_class<lensSurface>(lens[i]));
    }
    std::cout << "has " << this->lensSurfaceList.size() << " lens\n";
    std::cout << "cam position : ";
    this->camPosition.debugPrint();
    this->focusPoint = fetchOptional(json, "focusPoint", Vector3f(NAN));
    if (this->focusPoint[0] != NAN) {
        this->autoFocus(this->focusPoint);
        std::cout << "focucOffset : " << this->focusOffset << "\n";
    } else {
        this->focusOffset = 0;
    }
}

#define MAX_TRACE_INLENS 50
#define IS_ZERO_3f(x)                                                          \
    ((x[0] == 0 && x[1] == 0 && x[2] == 0) ||                                  \
     (std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2])))
#define IS_NAN_3f(x) (std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2]))

void OpticalSystem::autoFocus(Vector3f focusPoint) {
    float distance =
        dot(focusPoint - this->camPosition, normalize(this->camTo));
    const int sampleTimes = 500;
    for (int i = 0; i < sampleTimes; i++) {
    RETRY_FOCUS_SAMPLE:
        Point3f Origin =
            Point3f(normalize(lensDir) * dot(normalize(this->camTo),
                                             focusPoint - this->camPosition));
        Point3f Dest = Point3f(
            this->lensSurfaceList[0]->sampleOnSurface(this->sampler->next2D()));
        // Origin.debugPrint();
        Ray rayInLens = Ray(Origin, Dest);
        // std::cout << this->lensSurfaceList[0]->getV_ap() << "\n";
        // Dest.debugPrint();
        // rayInLens.origin.debugPrint();
        // rayInLens.direction.debugPrint();
        rayInLens.wavelength = 0;
        float outsideNd = AIR_ND, outsideV_no = AIR_V_NO;
        for (int j = 0; j < this->lensSurfaceList.size(); j++) {
            rayInLens = this->lensSurfaceList[j]->getRefractRay(
                rayInLens, outsideNd, outsideV_no);
            if ((IS_ZERO_3f(rayInLens.direction) &&
                 IS_ZERO_3f(rayInLens.origin)) ||
                (IS_NAN_3f(rayInLens.direction) ||
                 IS_NAN_3f(rayInLens.origin))) {
                goto RETRY_FOCUS_SAMPLE;
            }
            outsideNd = this->lensSurfaceList[j]->getNd();
            // rayInLens.origin.debugPrint();
            // rayInLens.direction.debugPrint();
        }
        // rayInLens.origin.debugPrint();
        // rayInLens.direction.debugPrint();
        Vector3f publicVerticalVec = cross(lensDir, rayInLens.direction);
        Vector3f surfadeVerticalVec = cross(lensDir, publicVerticalVec);
        float T = -(dot(rayInLens.origin.v3f(), surfadeVerticalVec) /
                    dot(rayInLens.direction, surfadeVerticalVec));

        // std::cout << dot(normalize(lensDir),
        //                  rayInLens.origin.v3f() + T * rayInLens.direction)
        //           << "\n";
        //(rayInLens.origin.v3f() + T * rayInLens.direction).debugPrint();
        this->focusOffset +=
            dot(normalize(lensDir),
                rayInLens.origin.v3f() + T * rayInLens.direction);
    }
    this->focusOffset /= sampleTimes;
}

Ray OpticalSystem::sampleRay(const CameraSample &sample, Vector2f NDC) const {
    Vector3f startFrom =
        this->leftDownCorner +
        this->dx * (NDC[0] * this->film->size[0] + sample.xy[0]) +
        this->dy * (NDC[1] * this->film->size[1] + sample.xy[1]) +
        this->focusOffset * normalize(lensDir);
    Vector3f pointOnLen0;
    Ray rayInLens;
    int times;
    int atLens;
RETRY_GENERATE:
    pointOnLen0 =
        this->lensSurfaceList.back()->sampleOnSurface(this->sampler->next2D());
    atLens = this->lensSurfaceList.size() - 1;
    times = 0;
    rayInLens = Ray(Point3f(startFrom), Point3f(pointOnLen0));
    rayInLens.wavelength = 0;
    // std::cout << "\nin lens : \n";
    // std::cout << "at ";
    // rayInLens.origin.debugPrint();
    // std::cout << "to ";
    // rayInLens.direction.debugPrint();
    // assert(NDC == Vector2f(0, 0));
    while (times < MAX_TRACE_INLENS) {
        times++;
        // std::cout << times << " : " << atLens << "\n";
        int nextLens;
        if (dot(rayInLens.direction, lensDir) > 0) {
            nextLens = atLens - 1;
        } else {
            nextLens = atLens + 1;
        }
        if (nextLens < 0) {
            Vector3f outDir = rayInLens.direction[0] * this->camTo +
                              rayInLens.direction[1] * this->camY +
                              rayInLens.direction[2] * this->camUp;
            Point3f outOri = this->camPosition +
                             rayInLens.origin[0] * this->camTo +
                             rayInLens.origin[1] * this->camY +
                             rayInLens.origin[2] * this->camUp;
            Ray rayOut = Ray(outOri, outDir);
            /*
                        if ((NDC - Vector2f(0.5, 0.5)).len() < 0.01) {
                            std::cout << "out at ";
                            rayOut.origin.debugPrint();
                            std::cout << "to ";
                            rayOut.direction.debugPrint();
                        }
                        */
            return rayOut;
        } else if (nextLens >= this->lensSurfaceList.size()) {
            break;
        } else {
            float outsideNd =
                nextLens < atLens
                    ? (nextLens == 0
                           ? AIR_ND
                           : this->lensSurfaceList[nextLens - 1]->getNd())
                    : this->lensSurfaceList[atLens]->getNd();
            float outsideV_no =
                nextLens < atLens
                    ? (nextLens == 0
                           ? AIR_V_NO
                           : this->lensSurfaceList[nextLens - 1]->getV_no())
                    : this->lensSurfaceList[atLens]->getV_no();
            if (this->sampler->next1D() <
                this->lensSurfaceList[nextLens]->getReflectRatio(
                    rayInLens, outsideNd, outsideV_no)) {
                rayInLens = this->lensSurfaceList[nextLens]->getReflectRay(
                    rayInLens, outsideNd, outsideV_no);
            } else {
                rayInLens = this->lensSurfaceList[nextLens]->getRefractRay(
                    rayInLens, outsideNd, outsideV_no);
            }
            if (IS_ZERO_3f(rayInLens.origin)) {
                break;
            }
        }
        // std::cout << "at ";
        // rayInLens.origin.debugPrint();
        // std::cout << "to ";
        // rayInLens.direction.debugPrint();
        atLens = nextLens;
    }
    goto RETRY_GENERATE;
}
Ray OpticalSystem::sampleRayDifferentials(const CameraSample &sample,
                                          Vector2f NDC) const {
    return this->sampleRay(sample, NDC);
}

REGISTER_CLASS(OpticalSystem, "opticalSystem")
REGISTER_CLASS(sphereLensSurface, "sphereSurface")
REGISTER_CLASS(aperture, "aperture")
