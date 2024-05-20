#include "WoodTexture.h"
#include "CoreLayer/Math/Geometry.h"
#include <ResourceLayer/Factory.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#define MIN(x, y) (x < y) ? x : y
#define MAX(x, y) (x > y) ? x : y

static Vector2f get_randomV2f() {
    Vector2f ret = {
        (static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) / 2)) - 1.f,
        (static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) / 2)) -
            1.f};
    while (ret.len() > 1.f) {
        ret = {
            (static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) / 2)) -
                1.f,
            (static_cast<float>(rand()) / (static_cast<float>(RAND_MAX) / 2)) -
                1.f};
    }
    return ret;
}
static float get_latticeCeil(float x) {
    return MIN(LATTICE_NUM, ceil(x * LATTICE_NUM) / LATTICE_NUM);
}
static float get_latticeFloor(float x) {
    return MIN(LATTICE_NUM, floor(x * LATTICE_NUM) / LATTICE_NUM);
}
static float get_latticeCeiladdr(float x) {
    return MIN(LATTICE_NUM, ceil(x * LATTICE_NUM));
}
static float get_latticeFlooraddr(float x) {
    return MIN(LATTICE_NUM, floor(x * LATTICE_NUM));
}

WoodTexture::WoodTexture(const Json &json) : Texture<Spectrum>() {
    std::srand(static_cast<unsigned>(time(0)));
    // std::srand(static_cast<unsigned>(211502008));
    for (int i = 0; i < LATTICE_NUM; i++) {
        for (int j = 0; j < LATTICE_NUM; j++) {
            lattice[i][j] = get_randomV2f();
            lattice[i][j] /= lattice[i][j].len();
        }
    }
}
Spectrum WoodTexture::evaluate(const Intersection &intersection) const {
    TextureCoord T;
    T.coord = intersection.texCoord;
    // return evaluate(T);
    float theta, phi, u, v;
    u = intersection.texCoord[0] * LATTICE_NUM;
    v = intersection.texCoord[1] * LATTICE_NUM;
    Vector2f d[4], p = Vector2f{u, v}, l[4];
    l[0] = Vector2f{floor(u), floor(v)};
    l[1] = Vector2f{ceil(u), floor(v)};
    l[2] = Vector2f{floor(u), ceil(v)};
    l[3] = Vector2f{ceil(u), ceil(v)};
    d[0] = p - l[0];
    d[1] = p - l[1];
    d[2] = p - l[2];
    d[3] = p - l[3];
    float dx = p[0] - l[0][0];
    float dy = p[1] - l[1][1];
    dx = dx * dx * (3 - 2 * dx);
    dy = dy * dy * (3 - 2 * dy);
    float f[4];
    f[0] = d[0].dot(lattice[(int)floor(u)][(int)floor(v)]);
    f[1] = d[1].dot(lattice[(int)ceil(u)][(int)floor(v)]);
    f[2] = d[2].dot(lattice[(int)floor(u)][(int)ceil(v)]);
    f[3] = d[3].dot(lattice[(int)ceil(u)][(int)ceil(v)]);
    float rx, ry;
    rx = dx / (l[1][0] - l[0][0]);
    ry = dy / (l[2][1] - l[1][1]);
    float x_low, x_high, F;
    x_low = (1 - rx) * f[3] + rx * f[2];
    x_high = (1 - rx) * f[0] + rx * f[1];
    F = (1 - ry) * x_low + ry * x_high;
    return ((F * 5) - floor(F * 5)) *
           Spectrum{164.f / 255.f, 116.f / 255.f, 73.f / 255.f}; // 164,116,73
}
Spectrum WoodTexture::evaluate(const TextureCoord &texCoord) const {
    Spectrum color = Spectrum{0.64313f, 0.4549f, 0.28627f};
    float u = texCoord.coord[0] * LATTICE_NUM;
    float v = texCoord.coord[1] * LATTICE_NUM;
    int u1 = floor(u);
    int v1 = floor(v);
    int u2 = ceil(u);
    int v2 = ceil(v);
    float du = u - u1;
    float dv = v - v1;
    du = du * du * (3 - 2 * du);
    dv = dv * dv * (3 - 2 * dv);
    Vector2f p = {u, v}, d[4];
    d[0] = p - lattice[u1 - 1][v2 - 1];
    d[1] = p - lattice[u2 - 1][v2 - 1];
    d[2] = p - lattice[u2 - 1][v1 - 1];
    d[3] = p - lattice[u1 - 1][v1 - 1];
    float f[4];
    f[0] = dot(d[0], lattice[u1 - 1][v2 - 1]);
    f[1] = dot(d[1], lattice[u2 - 1][v2 - 1]);
    f[2] = dot(d[2], lattice[u2 - 1][v1 - 1]);
    f[3] = dot(d[3], lattice[u1 - 1][v1 - 1]);
    float rx = (du) / (u2 - u1);
    float ry = (dv) / (v2 - v1);
    float x_low = (1 - rx) * f[3] + rx * f[2];
    float x_high = (1 - rx) * f[0] + rx * f[1];
    float F = (1 - ry) * x_low + ry * x_high;
    return (5 * F - floor(5 * F)) * color;
}
REGISTER_CLASS(WoodTexture, "woodTex")