#include "AreaLight.h"
#include "CoreLayer/Math/Constant.h"
#include "CoreLayer/Math/Geometry.h"
#include <FunctionLayer/Sampler/Sampler.h>
#include <ResourceLayer/Factory.h>
#include <iostream>

#define MonteCarloTimes 10000
AreaLight::AreaLight(const Json &json) : Light(json) {

    type = LightType::AreaLight;
    shape = Factory::construct_class<Shape>(json["shape"]);
    energy = fetchOptional<Spectrum>(json, "energy", 0.0f);
    power = fetchOptional<Spectrum>(json, "power", 0.0f);
    if (energy.isZero() &&
        !power.isZero()) { // 只有光源功率，将其转换为radiance
        // W = \int_{face}\int_{\omega} L\cos \thets d\omega dA =
        // L\int_{face}\int_{\omega} \cos \thets d\omega dA
        // L = {W\over \int_{face}\int_{\omega} \cos \thets d\omega dA}
        // {W \over \pi S}
        power.debugPrint();
        energy = SpectrumRGB(power / (this->shape->getArea() * PI));
        energy.debugPrint();
    }
    if (!energy.isZero() && power.isZero()) {
        energy.debugPrint();
        power = SpectrumRGB(energy * (this->shape->getArea() * PI));
        power.debugPrint();
    }
    rank = fetchOptional<float>(json, "rank", 0.0f);
    std::cerr << "RANK : " << rank << "\n";
    if (rank != 0.0f)
        useRank = true;
    if (useRank) {
        float sum = 0;
        auto sampler = Factory::construct_class<Sampler>(
            {{"type", "independent"}, {"xSamples", 4}, {"ySamples", 4}});
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
}

Spectrum AreaLight::evaluateEmission(const Intersection &intersection,
                                     const Vector3f &wo) const {
    if (!useRank) {
        return energy;
    } else {
        return energyRatio *
               pow(abs(dot(normalize(intersection.normal), normalize(wo))),
                   rank) *
               energy;
    }
}

LightSampleResult AreaLight::sample(const Intersection &shadingPoint,
                                    const Vector2f &sample) const {
    Intersection sampleResult;
    float pdf;
    shape->uniformSampleOnSurface(sample, &sampleResult, &pdf);
    Vector3f shadingPoint2sample =
        sampleResult.position - shadingPoint.position;
    Spectrum retRadiance;
    if (!useRank) {
        retRadiance = energy;
    } else {
        retRadiance = energyRatio *
                      pow(abs(dot(normalize(shadingPoint.normal),
                                  normalize(shadingPoint2sample))),
                          rank) *
                      energy;
    }
    return {retRadiance,
            normalize(shadingPoint2sample),
            shadingPoint2sample.length() - EPSILON,
            sampleResult.normal,
            pdf,
            false,
            type};
}

REGISTER_CLASS(AreaLight, "areaLight")