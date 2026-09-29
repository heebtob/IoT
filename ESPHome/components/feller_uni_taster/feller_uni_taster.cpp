#include "feller_uni_taster.h"

#include "binary_sensor/feller_button.h"
#include "light/feller_led.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace feller_uni_taster {

static const char *const TAG = "feller_uni_taster";

constexpr uint8_t frame_header(uint8_t length) {
  uint8_t value = 0x20 | length;
  uint8_t bits = value;
  bits ^= bits >> 4;
  bits ^= bits >> 2;
  bits ^= bits >> 1;
  return value | ((bits & 1) << 7);
}

static_assert(frame_header(1) == 0x21 && frame_header(2) == 0x22 && frame_header(3) == 0xA3 &&
                  frame_header(5) == 0xA5 && frame_header(6) == 0xA6 && frame_header(8) == 0x28,
              "Feller frame headers must have even parity");

void FellerUniTaster::send_(const uint8_t *body, uint8_t length) {
  this->write_byte(frame_header(length));
  this->write_array(body, length);
  this->flush();
  this->last_tx_at_ = millis();
}

void FellerUniTaster::register_button(FellerButton *button, uint8_t index) {
  this->buttons_[index - 1] = button;
}

void FellerUniTaster::register_led(FellerLed *led, uint8_t index) {
  this->leds_[index - 1] = led;
}

void FellerUniTaster::set_led(uint8_t index, uint8_t color, bool blinking) {
  const uint8_t slot = index - 1;
  if (this->colors_[slot] == color && this->blinking_[slot] == blinking)
    return;
  this->colors_[slot] = color;
  this->blinking_[slot] = blinking;
  this->leds_dirty_ = true;
}

void FellerUniTaster::set_brightness(uint8_t brightness) {
  this->brightness_ = brightness;
  this->brightness_dirty_ = true;
}

void FellerUniTaster::setup() {
  this->phase_at_ = millis();
  this->brightness_dirty_ = this->brightness_entity_ != nullptr;
  // The insert always starts at 9600, even if the negotiated rate differs.
  this->parent_->set_baud_rate(9600);
  this->parent_->load_settings();
  if (this->reset_on_boot_) {
    this->write_byte(0xA0);
    this->flush();
    this->last_tx_at_ = millis();
  }
}

void FellerUniTaster::request_settings_() {
  const uint8_t body[] = {0x10, 0x00, this->baud_code_, this->byte_timeout_,
                          uint8_t(this->button_indication_), uint8_t(this->system_state_indication_)};
  this->send_(body, sizeof(body));
  this->phase_ = Phase::WAIT_SETTINGS;
  this->phase_at_ = millis();
}

uint32_t FellerUniTaster::receive_timeout_ms_() const {
  uint32_t factor = this->byte_timeout_ <= 1 ? 10 : this->byte_timeout_;
  uint32_t baud = this->phase_ == Phase::WAIT_RESET || this->phase_ == Phase::WAIT_SEND_SETTINGS ||
                          this->phase_ == Phase::WAIT_SETTINGS || this->phase_ == Phase::WAIT_SWITCH
                      ? 9600
                      : this->baud_rate_;
  // Millisecond clock resolution requires rounding up.
  uint32_t timeout = (factor * 10000UL + baud - 1) / baud;
  return timeout > 500 ? 500 : timeout;
}

void FellerUniTaster::reset_parser_() {
  this->rx_length_ = 0;
  this->rx_expected_ = 0;
}

void FellerUniTaster::receive_() {
  uint8_t byte;
  while (this->available()) {
    if (this->rx_expected_ && millis() - this->last_byte_at_ > this->receive_timeout_ms_()) {
      ESP_LOGW(TAG, "Inter-byte timeout: discarding incomplete frame");
      this->reset_parser_();
    }
    if (!this->read_byte(&byte))
      break;
    this->last_byte_at_ = millis();
    if (!this->rx_expected_) {
      if (byte == 0xA0) {
        ESP_LOGI(TAG, "Insert software reset indication");
        this->parent_->set_baud_rate(9600);
        this->parent_->load_settings();
        this->leds_dirty_ = true;
        this->brightness_dirty_ = this->brightness_entity_ != nullptr;
        this->phase_ = Phase::WAIT_SEND_SETTINGS;
        this->phase_at_ = millis();
        continue;
      }
      uint8_t bits = byte;
      bits ^= bits >> 4;
      bits ^= bits >> 2;
      bits ^= bits >> 1;
      if ((byte & 0x60) != 0x20 || (bits & 1) || (byte & 0x1F) == 0) {
        ESP_LOGW(TAG, "Invalid frame header 0x%02X", byte);
        continue;
      }
      this->rx_expected_ = byte & 0x1F;
      this->rx_length_ = 0;
    } else {
      this->rx_[this->rx_length_++] = byte;
      if (this->rx_length_ == this->rx_expected_) {
        this->handle_frame_(this->rx_.data(), this->rx_length_);
        this->reset_parser_();
      }
    }
  }
  if (this->rx_expected_ && millis() - this->last_byte_at_ > this->receive_timeout_ms_()) {
    ESP_LOGW(TAG, "Inter-byte timeout: discarding incomplete frame");
    this->reset_parser_();
  }
}

void FellerUniTaster::warn_indices_() {
  for (uint8_t i = this->button_count_; i < 8; i++)
    if (this->buttons_[i] != nullptr)
      ESP_LOGW(TAG, "Button %u exceeds hardware button count %u", i + 1, this->button_count_);
  for (uint8_t i = this->led_count_; i < 8; i++)
    if (this->leds_[i] != nullptr)
      ESP_LOGW(TAG, "LED %u exceeds hardware LED count %u", i + 1, this->led_count_);
}

void FellerUniTaster::handle_frame_(const uint8_t *body, uint8_t length) {
  const uint8_t service = body[0];
  switch (service) {
    case 0x11:
      if (length != 1 || this->phase_ != Phase::WAIT_SETTINGS)
        break;
      this->phase_ = Phase::WAIT_SWITCH;
      this->phase_at_ = millis();
      return;
    case 0x13:
      if (length != 6 || this->phase_ != Phase::WAIT_VERIFY)
        break;
      if (body[1] != 0 || body[2] != this->baud_code_ || body[3] != this->byte_timeout_ ||
          body[4] != uint8_t(this->button_indication_) || body[5] != uint8_t(this->system_state_indication_)) {
        ESP_LOGW(TAG, "Settings confirmation does not match requested settings; restarting negotiation");
        this->write_byte(0xA0);
        this->flush();
        this->last_tx_at_ = millis();
        this->phase_ = Phase::WAIT_RESET;
        this->phase_at_ = millis();
        return;
      }
      this->phase_ = Phase::RUNNING;
      this->last_poll_ = millis();
      this->info_pending_ = true;
      this->initial_buttons_pending_ = !this->button_indication_ || this->poll_interval_;
      ESP_LOGI(TAG, "Settings confirmed; insert ready");
      return;
    case 0x1D:
      if (length != 3)
        break;
      ESP_LOGI(TAG, "Software %u.%u, hardware variant 0x%02X", body[1] >> 4, body[1] & 0x0F, body[2]);
      if (body[2] > 3) {
        ESP_LOGW(TAG, "Unknown hardware variant 0x%02X", body[2]);
      } else {
        this->button_count_ = (body[2] & 1) ? 8 : 4;
        this->led_count_ = body[2] == 0 ? 0 : body[2] == 1 ? 0 : body[2] == 2 ? 6 : 8;
        this->warn_indices_();
      }
      return;
    case 0x41:
    case 0x42:
      if (length != 2)
        break;
      for (uint8_t i = 0; i < 8; i++)
        if (this->buttons_[i] != nullptr)
          this->buttons_[i]->publish_state(body[1] & (1 << i));
      return;
    case 0x19:
    case 0x1A:
      if (length != 2)
        break;
      if (body[1] != 0) {
        static const char *const errors[] = {
            "invalid service", "invalid sub-service", "header frame length mismatch", "service frame length mismatch",
            "invalid frame header"};
        const char *message = "unknown system error";
        if (body[1] >= 0x10 && body[1] <= 0x14)
          message = errors[body[1] - 0x10];
        else
          switch (body[1]) {
            case 0x17: message = "invalid software handshake"; break;
            case 0x18: message = "simultaneous send request discarded"; break;
            case 0x20: message = "settings change failed"; break;
            case 0x21: message = "SPI requires handshake"; break;
            case 0x22: message = "invalid baud rate"; break;
            case 0x23: message = "timeout capped at 500 ms"; break;
            case 0x30: case 0x32: message = "receive buffer overflow"; break;
            case 0x31: case 0x33: message = "send buffer overflow"; break;
            case 0x34: message = "byte receive error"; break;
            case 0x35: message = "byte send error"; break;
            case 0x36: message = "receive timeout"; break;
            case 0x37: message = "send timeout"; break;
            case 0x40: message = "USART frame error"; break;
            case 0x41: message = "USART parity error"; break;
            case 0x42: message = "USART overflow"; break;
            case 0x43: message = "USART break"; break;
          }
        ESP_LOGW(TAG, "Insert status 0x%02X: %s", body[1], message);
      }
      return;
    case 0x31:
      if (length == 2 && (body[1] == 0x11 || body[1] == 0x12))
        return;
      break;
    case 0x39:
      if (length == 2 && body[1] == 0x11)
        return;
      break;
    default:
      ESP_LOGW(TAG, "Unexpected service 0x%02X", service);
      return;
  }
  ESP_LOGW(TAG, "Invalid length or state for service 0x%02X (%u bytes)", service, length);
}

void FellerUniTaster::flush_leds_() {
  uint8_t body[] = {0x30, 0x12, 0, 0, 0, 0, 0, 0};
  for (uint8_t i = 0; i < 8; i++) {
    if (this->colors_[i] >= 1 && this->colors_[i] <= 3)
      body[2 + (this->blinking_[i] ? 3 : 0) + this->colors_[i] - 1] |= 1 << i;
  }
  this->send_(body, sizeof(body));
  this->leds_dirty_ = false;
}

void FellerUniTaster::loop() {
  this->receive_();
  uint32_t now = millis();
  uint32_t gap = this->receive_timeout_ms_();
  if (this->phase_ == Phase::WAIT_SEND_SETTINGS && now - this->last_byte_at_ > gap) {
    this->request_settings_();
    return;
  }
  if (this->phase_ == Phase::WAIT_SWITCH && now - this->last_byte_at_ > gap) {
    this->parent_->set_baud_rate(this->baud_rate_);
    this->parent_->load_settings();
    this->phase_ = Phase::WAIT_VERIFY;
    this->phase_at_ = now;
    const uint8_t request[] = {0x12};
    this->send_(request, sizeof(request));
    return;
  }
  if (this->phase_ != Phase::RUNNING) {
    if (now - this->phase_at_ > (this->phase_ == Phase::WAIT_VERIFY ? 1000UL : 1500UL)) {
      ESP_LOGW(TAG, "Insert startup timed out; requesting reset");
      this->parent_->set_baud_rate(9600);
      this->parent_->load_settings();
      this->reset_parser_();
      this->write_byte(0xA0);
      this->flush();
      this->last_tx_at_ = now;
      this->phase_ = Phase::WAIT_RESET;
      this->phase_at_ = now;
    }
    return;
  }
  if (now - this->last_tx_at_ <= gap || now - this->last_byte_at_ <= gap)
    return;
  if (this->info_pending_) {
    const uint8_t request[] = {0x1C};
    this->send_(request, sizeof(request));
    this->info_pending_ = false;
    return;
  }
  if (this->initial_buttons_pending_) {
    const uint8_t request[] = {0x40};
    this->send_(request, sizeof(request));
    this->initial_buttons_pending_ = false;
    this->last_poll_ = now;
    return;
  }
  if (this->brightness_dirty_) {
    const uint8_t body[] = {0x38, 0x11, this->brightness_};
    this->send_(body, sizeof(body));
    this->brightness_dirty_ = false;
    return;
  }
  if (this->leds_dirty_) {
    this->flush_leds_();
    return;
  }
  if (this->poll_interval_ && now - this->last_poll_ >= this->poll_interval_) {
    const uint8_t request[] = {0x40};
    this->send_(request, sizeof(request));
    this->last_poll_ = now;
  }
}

void FellerUniTaster::dump_config() {
  ESP_LOGCONFIG(TAG, "Feller UNI-Taster:");
  ESP_LOGCONFIG(TAG, "  Baud rate: %" PRIu32, this->baud_rate_);
  ESP_LOGCONFIG(TAG, "  Byte timeout factor: %u", this->byte_timeout_);
  ESP_LOGCONFIG(TAG, "  Button indications: %s", YESNO(this->button_indication_));
  ESP_LOGCONFIG(TAG, "  System state indications: %s", YESNO(this->system_state_indication_));
}

}  // namespace feller_uni_taster
}  // namespace esphome
