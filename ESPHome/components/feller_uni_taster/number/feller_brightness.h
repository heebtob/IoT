#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace feller_uni_taster {

class FellerUniTaster;

class FellerBrightness : public number::Number, public Component {
 public:
  void set_parent(FellerUniTaster *parent) { this->parent_ = parent; }
  void set_restore_value(bool value) { this->restore_value_ = value; }
  void setup() override;

 protected:
  void control(float value) override;
  FellerUniTaster *parent_{nullptr};
  bool restore_value_{true};
  ESPPreferenceObject pref_;
};

}  // namespace feller_uni_taster
}  // namespace esphome
