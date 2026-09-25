#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>

#include "esphome/components/socket/socket.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include <netinet/in.h>

namespace esphome {
namespace artnet_dmx {

class ArtNetDMX : public Component {
 public:
  void set_tx_pin(InternalGPIOPin *pin) { this->tx_pin_ = pin; }
  void set_de_pin(InternalGPIOPin *pin) { this->de_pin_ = pin; }
  void set_uart_num(uint8_t uart_num) { this->uart_num_ = uart_num; }
  void set_net(uint8_t net) { this->net_ = net; }
  void set_subnet(uint8_t subnet) { this->subnet_ = subnet; }
  void set_universe(uint8_t universe) { this->universe_ = universe; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_channels(uint16_t channels) { this->channels_ = channels; }
  void set_start_channel(uint16_t channel) { this->start_channel_ = channel; }
  void set_refresh_rate(float rate) { this->refresh_rate_ = rate; }
  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
  void set_timeout_blackout(bool blackout) { this->timeout_blackout_ = blackout; }
  void set_short_name(const char *name) {
    this->short_name_.fill(0);
    const size_t length = std::min(std::strlen(name), this->short_name_.size() - 1);
    std::memcpy(this->short_name_.data(), name, length);
  }
  void set_long_name(const char *name) {
    this->long_name_.fill(0);
    const size_t length = std::min(std::strlen(name), this->long_name_.size() - 1);
    std::memcpy(this->long_name_.data(), name, length);
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void on_shutdown() override;

 protected:
  static constexpr size_t ARTNET_PACKET_MAX = 530;
  static constexpr size_t POLL_REPLY_QUEUE_SIZE = 8;

  struct PendingPollReply {
    bool active{false};
    bool send_unicast{false};
    sockaddr_in poller{};
    uint64_t due_us{0};
  };

  bool setup_socket_();
  bool setup_uart_();
  void receive_packets_();
  void handle_packet_(const uint8_t *data, size_t length, const sockaddr_in &source);
  void handle_artdmx_(const uint8_t *data, size_t length);
  void queue_poll_reply_(const sockaddr_in *poller, bool send_unicast);
  void send_poll_reply_(const sockaddr_in *poller, bool send_unicast);
  void send_due_poll_replies_();
  void send_dmx_frame_();
  void update_timeout_();
  bool data_is_recent_(uint64_t now_ms) const;

  InternalGPIOPin *tx_pin_{nullptr};
  InternalGPIOPin *de_pin_{nullptr};
  std::unique_ptr<socket::Socket> socket_;
  std::array<uint8_t, 513> dmx_{};
  std::array<PendingPollReply, POLL_REPLY_QUEUE_SIZE> pending_replies_{};
  std::array<char, 18> short_name_{};
  std::array<char, 64> long_name_{};
  uint16_t port_{6454};
  uint16_t channels_{512};
  uint16_t start_channel_{1};
  uint32_t timeout_ms_{5000};
  uint64_t last_frame_us_{0};
  uint64_t last_packet_ms_{0};
  uint64_t min_frame_interval_us_{0};
  float refresh_rate_{40.0f};
  uint8_t uart_num_{1};
  uint8_t net_{0};
  uint8_t subnet_{0};
  uint8_t universe_{0};
  uint8_t last_sequence_{0};
  bool timeout_blackout_{false};
  bool have_sequence_{false};
  bool have_received_packet_{false};
  bool blackout_applied_{false};
  bool was_connected_{false};
};

}  // namespace artnet_dmx
}  // namespace esphome
