#pragma once
#include "CoreLayer/Math/Geometry.h"
#include "Shape.h"
// TODO 当前只有Transform中的translate对sphere生效
class Sphere : public Shape {
  public:
    Sphere() = delete;

    Sphere(const Json &json);

    virtual bool rayIntersectShape(Ray &ray, int *primID, float *u,
                                   float *v) const override;

    virtual void fillIntersection(float distance, int primID, float u, float v,
                                  Intersection *intersection) const override;
    virtual void uniformSampleOnSurface(Vector2f sample,
                                        Intersection *intersection,
                                        float *pdf) const override {
        float theta = sample[0] * 2 * PI;
        float phi = (sample[1] * 2 * PI) - PI;
        float phip = sin(phi) * PI;
        Vector3f relative_hitpoint =
            Vector3f(radius * sin(theta) * cos(phip),
                     radius * sin(theta) * sin(phip), radius * cos(theta));
        intersection->position = center + relative_hitpoint;
        intersection->normal = normalize(Vector3f(relative_hitpoint));
        *pdf = 1 / (4 * PI * pow(radius, 2));
        return;
    }
    virtual float getArea() const override { return 4 * PI * pow(radius, 2); }

  public:
    Point3f center;
    float radius;
};