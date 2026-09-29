#include "feller_brightness.h"

#include "../feller_uni_taster.h"

namespace esphome {
namespace feller_uni_taster {

void FellerBrightness::setup() {
  float value = 255;
  if (this->restore_value_) {
    this->pref_ = this->make_entity_preference<float>();
    if (this->pref_.load(&value) && (value < 0 || value > 255 || value != value))
      value = 255;
  }
  this->publish_state(value);
  this->parent_->set_brightness(static_cast<uint8_t>(value));
}

void FellerBrightness::control(float value) {
  this->publish_state(value);
  if (this->restore_value_)
    this->pref_.save(&value);
  this->parent_->set_brightness(static_cast<uint8_t>(value));
}

}  // namespace feller_uni_taster
}  // namespace esphome
