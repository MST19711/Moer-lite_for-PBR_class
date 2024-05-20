# 图形绘制技术 Lab2 实验报告

>chengxili@smail.nju.edu.cn

## 光线微分

### 思路、实现和代码

该实验中，需要计算平面上的点在单位uv坐标变化下的变化：
$$
{\part P\over \part u} = 
\begin{bmatrix}
pu^\top, pv^\top, pw^\top
\end{bmatrix}{\part
\begin{bmatrix}
u'\\
v'\\
w'
\end{bmatrix} 
\over \part u}\\

\begin{bmatrix}
tu^\top, tv^\top, tw^\top
\end{bmatrix}{\part
\begin{bmatrix}
u'\\
v'\\
w'
\end{bmatrix} 
\over \part u} = 
\begin{bmatrix}
1\\
0
\end{bmatrix} \\
u'+v'=1-w'
$$
对v的微分同理。

使用

```cpp
auto solveLinearSystem2x2 = [](const Vector2f A0, const Vector2f A1,
                                   const Vector2f B, float &x0, float &x1) {
        float det = A0[0] * A1[1] - A0[1] * A1[0];
        if (std::abs(det) < 1e-10f)
            return false;
        x0 = (A1[1] * B[0] - A0[1] * B[1]) / det;
        x1 = (A0[0] * B[1] - A1[0] * B[0]) / det;
        if (std::isnan(x0) || std::isnan(x1))
            return false;
        return true;
    };
```

解方程，从而计算dpdu和dpdv：

```cpp
    float dUdu, dVdu;
    solveLinearSystem2x2((tw - tu), (tw - tv), Vector2f(1, 0), dUdu, dVdu);
    intersection->dpdu =
        dUdu * (pu.v3f() - pw.v3f()) + dVdu * (pv.v3f() - pw.v3f());

    float dUdv, dVdv;
    solveLinearSystem2x2((tw - tu), (tw - tv), Vector2f(0, 1), dUdv, dVdv);
    intersection->dpdv =
        dUdv * (pu.v3f() - pw.v3f()) + dVdv * (pv.v3f() - pw.v3f());
```

在`Ray.h`中打开光线微分：

```cpp
bool hasDifferentials = true;
```

### 实验

先在`Mipmap.cpp`中强制开启双线性滤波屏蔽光线微分：

```cpp
return bilinear(0, uv); // force bilinear
```

![texture_withBilinear](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab2/imgs/texture_withBilinear.png)

然后关闭强制双线性滤波启用光线微分：

```cpp
//return bilinear(0, uv); // force bilinear
```

![texture_withDifferentials](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab2/imgs/texture_withDifferentials.png)

可见光线微分可以快速逼近最优纹理过滤方法。

## 纹理映射

对于任意方向，先求归一化方向向量：

```cpp
Vector3f P =
        intersection.position.v3f() / intersection.position.v3f().length();
```

### 球面纹理映射

$$
\phi = \arccos(p_z), \theta = \arctan({p_y\over p_x})\\
u = {\theta\over 2\pi}, v = {\phi\over \pi}
$$
然后将uv线性映射到[0,1]上，代码实现如下：

```cpp
float theta, phi, u, v;
phi = acos(P[2]);
theta = atan(P[1] / P[0]);
u = theta / (PI);
v = phi / (PI);
u = u + 0.5f;
v = v;
```

球面纹理映射实验结果结果如下：

![bunny_texture_ball](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab2/imgs/bunny_texture_ball.png)

### 圆柱纹理映射

$$
\phi = \arccos(p_z), \theta = \arctan({p_y\over p_x})\\
u = {\theta\over 2\pi}, v = \left\{
\begin{aligned}
&p_z, &p_z\ge 0\\
&p_z + 1, &p_z< 0\\
\end{aligned}\right .
$$

然后将uv线性映射到[0,1]上，代码实现如下：

```cpp
phi = acos(P[2]);
theta = atan(P[1] / P[0]);
u = theta / (PI);
u = u + 0.5f;
v = P[2] >= 0 ? P[2] : P[2] + 1.f;
```

圆柱纹理映射实验结果结果如下：

![bunny_texture_cylinder](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab2/imgs/bunny_texture_cylinder.png)

### 平面纹理映射

$$
u = v_\alpha\cdot p, v = v_\beta\cdot p\\
v_\alpha = (1,0,0), v_\beta = (0,1,0)
$$

然后将uv线性映射到[0,1]上，代码实现如下：

```cpp
u = ((float)(intersection.position.v3f()[0] - (int)intersection.position.v3f()[0]) / 2.f) + 0.5f;
v = ((float)(intersection.position.v3f()[1] - (int)intersection.position.v3f()[1]) / 2.f) +å 0.5f;
```

圆柱纹理映射实验结果结果如下：

![bunny_texture_flatness](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab2/imgs/bunny_texture_flatness.png)

## 木纹

### 思路、实现和代码

使用Perlin噪声生成木纹。

首先先初始化格点上的随机向量：

```cpp
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
WoodTexture::WoodTexture(const Json &json) : Texture<Spectrum>() {
    std::srand(static_cast<unsigned>(time(0)));
    for (int i = 0; i < LATTICE_NUM; i++) {
        for (int j = 0; j < LATTICE_NUM; j++) {
            lattice[i][j] = get_randomV2f();
            lattice[i][j] /= lattice[i][j].len();
        }
    }
}
```

对于任何一个intersection，将其texCoord分离并送入`evaluate`计算：

```cpp
Spectrum WoodTexture::evaluate(const Intersection &intersection) const {
    TextureCoord T;
    T.coord = intersection.texCoord;
    return evaluate(T);
}
```

evaluate代码如下：

```cpp
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
```

### 实验

使用场景bunny-texture-wood:

![bunny_texture_wood](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/build/bunny_texture_wood.jpeg)
