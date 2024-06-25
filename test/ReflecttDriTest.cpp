#include <iostream>
using namespace std;
class Vector3f {
  public:
    Vector3f(float x, float y, float z) : x(x), y(y), z(z) {}
    Vector3f() : x(0), y(0), z(0) {}
    float x, y, z;
    Vector3f operator+(const Vector3f &v) const {
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }
    Vector3f operator-(const Vector3f &v) const {
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }
    Vector3f operator*(float s) const { return Vector3f(s * x, s * y, s * z); }
    Vector3f operator/(float s) const { return Vector3f(x / s, y / s, z / s); }
    Vector3f normalize() const {
        float invLen = 1 / sqrt(x * x + y * y + z * z);
        return Vector3f(x * invLen, y * invLen, z * invLen);
    }
    float length() const { return sqrt(x * x + y * y + z * z); }
};

float dot(const Vector3f &a, const Vector3f &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3f cross(const Vector3f &a, const Vector3f &b) {
    return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x);
}
Vector3f getRefractDri(const Vector3f &directionIn, Vector3f norm,
                       float nd_from, float V_from, float nd_to, float V_to) {
    float n_from, n_to;
    n_from = nd_from;
    n_to = nd_to;

    Vector3f nNorm = norm.normalize();
    if (dot(nNorm, directionIn) < 0)
        nNorm = nNorm * -1;
    float cos_theta_in = dot(directionIn, nNorm) / directionIn.length();
    float sin_theta_in = sqrt(1 - (cos_theta_in * cos_theta_in));
    float sin_theta_out = (sin_theta_in * n_from) / n_to;
    float cos_theta_out = sqrt(1 - (sin_theta_out * sin_theta_out));
    return (directionIn +
            nNorm * (((directionIn - nNorm * dot(nNorm, directionIn)).length() /
                      sin_theta_out) *
                         cos_theta_out -
                     dot(nNorm, directionIn)))
        .normalize();
}
Vector3f getReflectDir(Vector3f norm, Vector3f rayDir) {
    Vector3f NN = norm.normalize();
    return rayDir - NN * dot(NN, rayDir) * 2;
}
int main() {
    Vector3f directionIn(0.0, 0.5, -2.0);
    Vector3f norm(0.0, -1.0, -1.0);
    // 从空气射入水
    float nd_from = 1.0;
    float V_from = 0.0;
    float nd_to = 1.33;
    float V_to = 1.0;
    Vector3f refractDri = getReflectDir(norm, directionIn);
    cout << refractDri.x << " " << refractDri.y << " " << refractDri.z << endl;
}