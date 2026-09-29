#pragma once

#include <array>
#include <cstdint>

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace feller_uni_taster {

class FellerButton;
class FellerLed;
class FellerBrightness;

class FellerUniTaster : public Component, public uart::UARTDevice {
 public:
  void set_baud_rate(uint32_t baud) { this->baud_rate_ = baud; }
  void set_baud_code(uint8_t code) { this->baud_code_ = code; }
  void set_byte_timeout(uint8_t timeout) { this->byte_timeout_ = timeout; }
  void set_button_indication(bool enabled) { this->button_indication_ = enabled; }
  void set_system_state_indication(bool enabled) { this->system_state_indication_ = enabled; }
  void set_reset_on_boot(bool enabled) { this->reset_on_boot_ = enabled; }
  void set_poll_interval(uint32_t interval) { this->poll_interval_ = interval; }

  void register_button(FellerButton *button, uint8_t index);
  void register_led(FellerLed *led, uint8_t index);
  void register_brightness(FellerBrightness *brightness) { this->brightness_entity_ = brightness; }
  void set_led(uint8_t index, uint8_t color, bool blinking);
  void set_brightness(uint8_t brightness);

  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  enum class Phase : uint8_t { WAIT_RESET, WAIT_SEND_SETTINGS, WAIT_SETTINGS, WAIT_SWITCH, WAIT_VERIFY, RUNNING };
  void send_(const uint8_t *body, uint8_t length);
  void request_settings_();
  void handle_frame_(const uint8_t *body, uint8_t length);
  void receive_();
  void reset_parser_();
  void flush_leds_();
  void warn_indices_();
  uint32_t receive_timeout_ms_() const;

  uint32_t baud_rate_{9600};
  uint8_t baud_code_{0x08};
  uint8_t byte_timeout_{0};
  bool button_indication_{true};
  bool system_state_indication_{true};
  bool reset_on_boot_{true};
  uint32_t poll_interval_{0};
  Phase phase_{Phase::WAIT_RESET};
  uint32_t phase_at_{0};
  uint32_t last_poll_{0};
  uint32_t last_byte_at_{0};
  uint32_t last_tx_at_{0};
  std::array<uint8_t, 32> rx_{};
  uint8_t rx_length_{0};
  uint8_t rx_expected_{0};
  std::array<FellerButton *, 8> buttons_{};
  std::array<FellerLed *, 8> leds_{};
  std::array<uint8_t, 8> colors_{};
  std::array<bool, 8> blinking_{};
  bool leds_dirty_{false};
  bool brightness_dirty_{false};
  bool info_pending_{false};
  bool initial_buttons_pending_{false};
  uint8_t brightness_{255};
  FellerBrightness *brightness_entity_{nullptr};
  uint8_t button_count_{0};
  uint8_t led_count_{0};
};

}  // namespace feller_uni_taster
}  // namespace esphome
