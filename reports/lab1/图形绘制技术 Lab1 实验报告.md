# 图形绘制技术 Lab1 实验报告

>
> chengxili@smail.nju.edu.cn

## 恒定功率光源

### 思路、实现和代码

由于光源的功率可以被表示为$radiance*\cos\theta$对角度和面积的二重积分，且衡定功率的光源每个光鲜的radiance都相同，易得：
$$
\begin{aligned}
\int_{\text{Area}}\int_{\text{Hemisphere}} L\cos\theta\  d\omega dS &= P\\
L &={P\over S\pi}
\end{aligned}
$$
从而在光源的构造函数中使用该公式计算出radiance即可：
```cpp
		energy = fetchOptional<Spectrum>(json, "energy", 0.0f);
    power = fetchOptional<Spectrum>(json, "power", 0.0f);
    if (energy.isZero() &&
        !power.isZero()) { // 只有光源功率，将其转换为radiance
        power.debugPrint();
        energy = SpectrumRGB(power / (this->shape->getArea() * PI));
        energy.debugPrint();
    }
```

为了得到发光体的表面积，给`shape`类中添加接口`getArea()`:

```cpp
virtual float getArea() const { return 0.0f; }
```

为了在平行四边形光源的场景能有正常的行为，给平行四边形overwrite计算表面积的接口：

```cpp
float Parallelogram::getArea() const { return cross(edge0, edge1).length(); }
```

### 实验

先在不设置衡定功率光源的情况下改变光源的表面积：

![test2_0-1](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_0-1.png)

![test2_0-2](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_0-2.png)

![test2_0-3](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_0-3.png)

可见此时场景亮度与光源面积正相关；

然后设置衡定功率光源再进行相同的实验：

![test2_1-1](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_1-1.png)

![test2_1-2](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_1-2.png)

![test2_1-3](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_1-3.png)

可见恒定光源的功率后场景亮度与光源面积基本无关。

## 球面光源

### 思路、实现和代码

该实验需要使用两个[0,1]上均匀分布的随机变量(x, y)完成在球面上均匀采样。

不难想到，直接将两个变量分别映射为球面上的经度和纬度即可；但是显然这样会导致在两极附近的采样概率大于在赤道周围的采样概率。因此我们设采样落在$(x, y)$上的概率为常数$p(x,y) = c$，则有：
$$
\begin{aligned}
\int_{[0, 2\pi]}\int_{[-\pi, \pi]}c \ dxdy &= 1\\
{\part P(x, y)\over \part x} = {\part P(x, y)\over \part y} &= p(x,y) = c\\
p(x,y) = {\cos (y)\over 4\pi r^2}
\end{aligned}
$$
此处y为与赤道平面的夹角。

从而不难发现，对于一个[0,1]间均匀分布的x，$\sin(2\pi x - \pi)$落在某个维度Y上的概率恰好为$\cos(Y)$。

在代码中直接实现这个公式即可。具体来说，就是根据半径和经纬度算出相对球心的位置，以此取norm作为法向量，然后在加上球心的坐标就是采样到的点所在的坐标;pdf为面积的倒数：

```cpp
		float theta = sample[0] * 2 * PI;
   	float phi = (sample[1] * 2 * PI) - PI;
    float phip = sin(phi) * PI;
    Vector3f relative_hitpoint =
        Vector3f(radius * sin(theta) * cos(phip),
                 radius * sin(theta) * sin(phip), radius * cos(theta));
    intersection->position = center + relative_hitpoint;
    intersection->normal = normalize(Vector3f(relative_hitpoint));
		*pdf = 1 / (4 * PI * pow(radius, 2));
```

### 实验

没有添加相应代码：

![cornell_before](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/cornell_before.png)

添加了相关代码：

![cornell_after](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/cornell_after.png)

## 三角形网格光源

### 思路、实现和代码

需要完成在三角形网格上进行均匀采样的方法。

首先将所有三角面排列在一个vector中，vector中元素i的两个值l和r分别为$\sum_{j\in [0, i-1]}S_j$和$\sum_{j\in [0, i]}S_j$，然后再将两个[1,0]上均匀分布采样得到的随机数其中之一放大到[0, 三角形网格总面积]然后使用二分查找判断这个值落在vector中的那个区间上，从而实现在所有三角形中以与面积呈正比的概率随机选取一个三角形：

首先在构造函数中计算好用于排列三角形面积的vector和总面积：

```cpp
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
```

在采样时使用二分查找快速找到随机选中的三角形：

```cpp
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
```

得到三角形后，将x在三角形面积对应区间上的位置放大到[0,1]，然后在使用如下映射使将两个均匀分布的随机数映射到三角形上的均匀采样：
$$
P = \sqrt{v}(1-u)A + u\sqrt vB + (1 - \sqrt v)C
$$
代码如下：

```cpp
Point3f a = meshData->vertexBuffer[meshData->faceBuffer[triangle.id][0].vertexIndex],
				b = meshData->vertexBuffer[meshData->faceBuffer[triangle.id][1] .vertexIndex],
        c = meshData->vertexBuffer[meshData->faceBuffer[triangle.id][2].vertexIndex];
float S = cross(b - a, c - a).length() / 2;
float u = ((sample[0] * totalArea) - triangle.left) / (triangle.right - triangle.left),
      v = sample[1];
Point3f P = (a * std::sqrt(u) * (1 - v)) + (b * u * std::sqrt(v)) + (c * (1 - std::sqrt(v)));
Vector3f N = cross(b - a, c - a) / cross(b - a, c - a).length();
intersection->position = P;
intersection->normal = N;
```

### 实验

使用场景bunny-light：

编写接口实现前：

![bunny_before](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/bunny_before.png)

编写实现后：

![bunny-2](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/bunny-2.png)

## Non-Diffuse 光源

### 思路、实现和代码

我使用的光源分布为：
$$
p(\theta) = c\cos^{rank}\theta\\
\int_{0}^\pi p(\theta) = 1
$$
首先现在构造函数中使用蒙特卡洛积分算出c ：

```cpp
if (useRank) {
		float sum = 0;
		auto sampler = Factory::construct_class<Sampler>({{"type", "independent"}, {"xSamples", 4}, {"ySamples", 4}});
		for (int i = 0; i < MonteCarloTimes; i++) {
				float theta = (sampler->next1D() * PI) / 2;
				sum += pow(cos(theta), rank);
    }
		energyRatio = (this->shape->getArea() * MonteCarloTimes) / sum;
		// (sum / (S * MCtimes)) * k = power
    // k_r = power_r * (S * MCtimes) / sum
    // k_g = power_g * (S * MCtimes) / sum
    // k_b = power_b * (S * MCtimes) / sum
}
```

然后再在光源直射采样中计算不同夹角的radiance：
```cpp
if (!useRank) {
		return energy;
} else {
		return energyRatio * pow(abs(dot(normalize(intersection.normal), normalize(wo))), rank) *
           energy;
}
```

最后再在反射采样中计算radiance：

```cpp
if (!useRank) {
		retRadiance = energy;
} else {
		retRadiance = energyRatio * 
      						pow(abs(dot(normalize(shadingPoint.normal), normalize(shadingPoint2sample))), rank) * 
      						energy;
}
```

### 实验

使用不同的rank参数进行实验：

rk = 1

![test2_rank=1](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_rank=1.png)

rk = 3

![test2_rank=3](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_rank=3.png)

rk = 5

![test2_rank=5](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_rank=5.png)

rk = 10

![test2_rank=10](/Users/cx_li/Documents/Github/Moer-lite_for-PBR_class/reports/lab1/imgs/test2_rank=10.png)

