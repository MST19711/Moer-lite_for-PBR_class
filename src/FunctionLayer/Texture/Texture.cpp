#include "Texture.h"
#include "CoreLayer/Math/Geometry.h"
#include <cmath>

TextureCoord UVMapping::map(const Intersection &intersection) const {

    Vector3f P =
        intersection.position.v3f() / intersection.position.v3f().length();
    float theta, phi, u, v;
    // 球面纹理映射 :
    /*
        phi = acos(P[2]);
        theta = atan(P[1] / P[0]);
        u = theta / (PI);
        v = phi / (PI);
        u = u + 0.5f;
        v = v;
        */
    // 圆柱体纹理映射 :

    phi = acos(P[2]);
    theta = atan(P[1] / P[0]);
    u = theta / (PI);
    u = u + 0.5f;
    v = P[2] >= 0 ? P[2] : P[2] + 1.f;

    // 平面纹理映射 :
    /*
        u = ((float)(intersection.position.v3f()[0] -
                     (int)intersection.position.v3f()[0]) /
             2.f) +
            0.5f;
        v = ((float)(intersection.position.v3f()[1] -
                     (int)intersection.position.v3f()[1]) /
             2.f) +
            0.5f;
    */
    return TextureCoord{Vector2f{u, v},
                        Vector2f{intersection.dudx, intersection.dvdx},
                        Vector2f{intersection.dudy, intersection.dvdy}};
    return TextureCoord{intersection.texCoord,
                        Vector2f{intersection.dudx, intersection.dvdx},
                        Vector2f{intersection.dudy, intersection.dvdy}};
}