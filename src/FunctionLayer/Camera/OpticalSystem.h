#pragma once

#include "../src/FunctionLayer/Scene/Scene.h"
#include "Camera.h"
#include "CoreLayer/Math/Geometry.h"
#include <FunctionLayer/Sampler/Sampler.h>
#include <memory>
#include <vector>

#define AIR_ND 1.00029312
#define AIR_V_NO 89.30
enum class focusMode { Center, Point };
//* outside指镜片表面左侧（即远离底片的方向）空间
class lensSurface {
  public:
    // lensSurface() = delete;

    //* 如果不能相交，请返回INF
    virtual float getHitTime(const Ray &ray) const = 0;
    virtual Vector3f getNormAt(Vector3f hitPoint) const = 0;
    virtual Vector3f sampleOnSurface(Vector2f sample) const = 0;
    virtual float getNd() const = 0;
    virtual float getV_no() const = 0;
    virtual float get_ap() const = 0;
};

class sphereLensSurface : public lensSurface {
  public:
    sphereLensSurface(const Json &json);
    sphereLensSurface(float radius, float thick, float nd, float V_no, float ap,
                      float dis2Film);
    virtual float getHitTime(const Ray &ray) const override;
    virtual Vector3f getNormAt(Vector3f hitPoint) const override;
    virtual Vector3f sampleOnSurface(Vector2f sample) const override;
    float getNd() const override;
    float getV_no() const override;
    virtual float get_ap() const override;

  private:
    //* thick参数为光轴上到右侧（即靠近底片）那片镜片的距离
    float radius, thick, nd, V_no, ap;
    float theta;
    Vector3f sphereCenter;
};

class aperture : public lensSurface {
  public:
    aperture(const Json &json);
    virtual float getHitTime(const Ray &ray) const override;
    virtual Vector3f getNormAt(Vector3f hitPoint) const override;
    virtual Vector3f sampleOnSurface(Vector2f sample) const override;
    float getNd() const override;
    float getV_no() const override;
    virtual float get_ap() const override;
    float thick, ap, dis2film;
};

const Vector3f lensDir = Vector3f(1, 0, 0);
const Vector3f lensUp = Vector3f(0, 0, 1);

//* 设光轴为从(0,0,0)开始的，方向为(1,0,0)的射线
//* 胶片上方向为(0,0,1)
//* 计算过后再根据相机方向调整出射光线方向
class OpticalSystem : public Camera {
  public:
    OpticalSystem() = delete;

    OpticalSystem(const Json &json);

    virtual Ray sampleRay(const CameraSample &sample,
                          Vector2f NDC) const override;

    virtual Ray sampleRayDifferentials(const CameraSample &sample,
                                       Vector2f NDC) const override;
    //* 自动对焦目前只支持纯折射系统
    void autoFocus(Vector3f focusPoint);
    void autoFocus(const Scene &scene) override;

  private:
    std::vector<std::shared_ptr<lensSurface>> lensSurfaceList;
    float diagonal; // 单位为米
    std::shared_ptr<Sampler> sampler;
    Vector3f dx, dy;
    Vector3f leftDownCorner;
    Vector3f camTo, camUp, camPosition, camY;
    float monitoring_range; // 单位为mm
    Vector3f focusPoint;
    float focusOffset;
    int apertureId;
    focusMode focusMode;
    Ray getRayOut(const Ray &rayIn) const;
};
