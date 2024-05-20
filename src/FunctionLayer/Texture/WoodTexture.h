#pragma once
#include "CoreLayer/Math/Geometry.h"
#include "Texture.h"
#include <CoreLayer/ColorSpace/Spectrum.h>
#define LATTICE_NUM 5
class WoodTexture : public Texture<Spectrum> {
  public:
    WoodTexture() = delete;
    WoodTexture(const Json &json);
    virtual Spectrum evaluate(const Intersection &intersection) const override;
    virtual Spectrum evaluate(const TextureCoord &texCoord) const override;

  private:
    Vector2f lattice[LATTICE_NUM][LATTICE_NUM];
};
