#pragma once

#include "esphome/components/light/light_effect.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"

namespace esphome {
namespace feller_uni_taster {

class FellerUniTaster;

class FellerLed : public light::LightOutput {
 public:
  FellerLed(FellerUniTaster *parent, uint8_t index) : parent_(parent), index_(index) {}
  light::LightTraits get_traits() override {
    light::LightTraits traits;
    traits.set_supported_color_modes({light::ColorMode::RGB});
    return traits;
  }
  void write_state(light::LightState *state) override;

 protected:
  FellerUniTaster *parent_;
  uint8_t index_;
};

class FellerBlinkEffect : public light::LightEffect {
 public:
  explicit FellerBlinkEffect(const char *name) : LightEffect(name) {}
  void apply() override {}
};

}  // namespace feller_uni_taster
}  // namespace esphome
