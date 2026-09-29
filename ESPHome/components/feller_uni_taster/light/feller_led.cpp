#include "feller_led.h"

#include "../feller_uni_taster.h"

namespace esphome {
namespace feller_uni_taster {

void FellerLed::write_state(light::LightState *state) {
  float red, green, blue;
  state->current_values_as_rgb(&red, &green, &blue);
  uint8_t color = 0;
  if (red > 0 || green > 0 || blue > 0) {
    color = red >= green && red >= blue ? 1 : green >= blue ? 2 : 3;
  }
  this->parent_->set_led(this->index_, color, color != 0 && state->get_current_effect_index() != 0);
}

}  // namespace feller_uni_taster
}  // namespace esphome
