#pragma once
#include "CoreLayer/Math/Geometry.h"
#include "Shape.h"
#include <FunctionLayer/Acceleration/Acceleration.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/Mesh.h>
#include <cmath>
#include <iostream>
#include <queue>
class TriangleMesh;

static struct faceSInseq {
    float left, right;
    int id;
    faceSInseq(float l, float r, int I) { left = l, right = r, id = I; }
};

class Triangle : public Shape {
  public:
    Triangle() = default;

    Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
             const TriangleMesh *_mesh);

    virtual bool rayIntersectShape(Ray &ray, int *primID, float *u,
                                   float *v) const override;

    virtual void fillIntersection(float distance, int primID, float u, float v,
                                  Intersection *intersection) const override;

    virtual void uniformSampleOnSurface(Vector2f sample,
                                        Intersection *intersection,
                                        float *pdf) const override {
        // TODO finish this
        return;
    }

  public:
    int primID;
    int vtx0Idx, vtx1Idx, vtx2Idx;
    const TriangleMesh *mesh = nullptr;
};

class TriangleMesh : public Shape {
  public:
    TriangleMesh() = default;

    TriangleMesh(const Json &json);

    //* 当使用embree时，我们使用embree内置的求交函数，故覆盖默认方法
    virtual RTCGeometry getEmbreeGeometry(RTCDevice device) const override;

    virtual bool rayIntersectShape(Ray &ray, int *primID, float *u,
                                   float *v) const override;

    virtual void fillIntersection(float distance, int primID, float u, float v,
                                  Intersection *intersection) const override;

    virtual void uniformSampleOnSurface(Vector2f sample,
                                        Intersection *intersection,
                                        float *pdf) const override {
        // TODO finish this
        *pdf = 1 / totalArea;
        int l = 0, r = meshData->faceCount, mid;
        float choise = sample[0] * totalArea;
        while (l < r) {
            mid = (l + r) / 2;
            if (facesS[mid].left <= choise && choise <= facesS[mid].right) {
                sampleInTriangle(sample, intersection, facesS[mid]);
                return;
            } else if (facesS[mid].left > choise) {
                r = mid - 1;
            } else if (facesS[mid].right < choise) {
                l = mid + 1;
            }
        }
        sampleInTriangle(sample, intersection, facesS[l]);
        return;
    }
    virtual float getArea() const override { return totalArea; }
    virtual void initInternalAcceleration() override;

    friend class Triangle;

  private:
    std::vector<faceSInseq> facesS;
    std::shared_ptr<MeshData> meshData;
    std::shared_ptr<Acceleration> acceleration;
    float totalArea;

    void sampleInTriangle(Vector2f sample, Intersection *intersection,
                          faceSInseq triangle) const {
        Point3f a = meshData->vertexBuffer[meshData->faceBuffer[triangle.id][0]
                                               .vertexIndex],
                b = meshData->vertexBuffer[meshData->faceBuffer[triangle.id][1]
                                               .vertexIndex],
                c = meshData->vertexBuffer[meshData->faceBuffer[triangle.id][2]
                                               .vertexIndex];
        float S = cross(b - a, c - a).length() / 2;
        float u = ((sample[0] * totalArea) - triangle.left) /
                  (triangle.right - triangle.left),
              v = sample[1];
        Point3f P = (a * std::sqrt(u) * (1 - v)) + (b * u * std::sqrt(v)) +
                    (c * (1 - std::sqrt(v)));
        Vector3f N = cross(b - a, c - a) / cross(b - a, c - a).length();
        intersection->position = P;
        intersection->normal = N;
    }

    void makeFacssSSeq() {
        float lastPoint = 0;
        for (int i = 0; i < meshData->faceBuffer.size(); i++) {
            Point3f
                a = meshData
                        ->vertexBuffer[meshData->faceBuffer[i][0].vertexIndex],
                b = meshData
                        ->vertexBuffer[meshData->faceBuffer[i][1].vertexIndex],
                c = meshData
                        ->vertexBuffer[meshData->faceBuffer[i][2].vertexIndex];
            float S = cross(b - a, c - a).length() / 2;
            facesS.push_back(faceSInseq(lastPoint, lastPoint + S, i));
            lastPoint += S;
        }
        totalArea = lastPoint;
    }
};