#include "artnet_dmx.h"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include "esphome/components/network/util.h"
#include "esphome/core/log.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>

namespace esphome {
namespace artnet_dmx {

static const char *const TAG = "artnet_dmx";
static constexpr uint8_t ARTNET_ID[8] = {'A', 'r', 't', '-', 'N', 'e', 't', 0};
static constexpr uint8_t ARTNET_OP_POLL = 0x00;
static constexpr uint8_t ARTNET_OP_DMX = 0x50;
static constexpr uint16_t ARTNET_OP_POLL_REPLY = 0x2100;
static constexpr size_t ARTNET_POLL_REPLY_SIZE = 239;

struct NetifAddress {
  esp_netif_ip_info_t info{};
};

static bool find_ipv4_netif_(esp_netif_t *netif, void *context) {
  auto *address = static_cast<NetifAddress *>(context);
  return esp_netif_get_ip_info(netif, &address->info) == ESP_OK && address->info.ip.addr != 0;
}

static uint64_t now_us_() { return static_cast<uint64_t>(esp_timer_get_time()); }

void ArtNetDMX::setup() {
  this->dmx_.fill(0);
  this->min_frame_interval_us_ =
      std::max<uint64_t>(132 + (static_cast<uint64_t>(this->channels_) + 1) * 44,
                         static_cast<uint64_t>(1000000.0f / this->refresh_rate_));

  if (this->uart_num_ == UART_NUM_0) {
    ESP_LOGW(TAG, "UART0 is commonly used by the ESPHome logger; ensure it is not shared");
  }
  if (!this->setup_uart_() || !this->setup_socket_()) {
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Art-Net listener ready on UDP port %u", static_cast<unsigned>(this->port_));
}

bool ArtNetDMX::setup_uart_() {
  const auto uart = static_cast<uart_port_t>(this->uart_num_);
  esp_err_t err = uart_driver_install(uart, 0, 0, 0, nullptr, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
    return false;
  }

  uart_config_t config{};
  config.baud_rate = 250000;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_2;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  config.rx_flow_ctrl_thresh = 0;

  err = uart_param_config(uart, &config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
    uart_driver_delete(uart);
    return false;
  }

  const int tx_pin = this->tx_pin_ == nullptr ? UART_PIN_NO_CHANGE : this->tx_pin_->get_pin();
  err = uart_set_pin(uart, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
    uart_driver_delete(uart);
    return false;
  }

  if (this->de_pin_ != nullptr) {
    this->de_pin_->setup();
    this->de_pin_->digital_write(false);
  }
  return true;
}

bool ArtNetDMX::setup_socket_() {
  this->socket_ = socket::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->socket_ == nullptr) {
    ESP_LOGE(TAG, "Unable to create Art-Net UDP socket");
    return false;
  }

  int enabled = 1;
  if (this->socket_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) != 0 ||
      this->socket_->setsockopt(SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled)) != 0) {
    ESP_LOGE(TAG, "Unable to configure Art-Net UDP socket: errno %d", errno);
    return false;
  }
  if (this->socket_->setblocking(false) != 0) {
    ESP_LOGE(TAG, "Unable to set Art-Net UDP socket non-blocking: errno %d", errno);
    return false;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(this->port_);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (this->socket_->bind(reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0) {
    ESP_LOGE(TAG, "Unable to bind Art-Net UDP port %u: errno %d", static_cast<unsigned>(this->port_), errno);
    return false;
  }
  return true;
}

void ArtNetDMX::loop() {
  this->receive_packets_();

  const bool connected = network::is_connected();
  if (connected && !this->was_connected_) {
    this->queue_poll_reply_(nullptr, false);
  }
  this->was_connected_ = connected;

  this->send_due_poll_replies_();
  this->update_timeout_();

  const uint64_t now = now_us_();
  if (this->last_frame_us_ == 0 || now - this->last_frame_us_ >= this->min_frame_interval_us_) {
    this->last_frame_us_ = now;
    this->send_dmx_frame_();
  }
}

void ArtNetDMX::receive_packets_() {
  if (this->socket_ == nullptr) {
    return;
  }

  uint8_t packet[ARTNET_PACKET_MAX];
  while (this->socket_->ready()) {
    sockaddr_in source{};
    socklen_t source_length = sizeof(source);
    const ssize_t length = this->socket_->recvfrom(packet, sizeof(packet), reinterpret_cast<sockaddr *>(&source),
                                                   &source_length);
    if (length < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        ESP_LOGW(TAG, "Art-Net UDP receive failed: errno %d", errno);
      }
      break;
    }
    if (length == 0 || source.sin_family != AF_INET) {
      continue;
    }
    this->handle_packet_(packet, static_cast<size_t>(length), source);
  }
}

void ArtNetDMX::handle_packet_(const uint8_t *data, size_t length, const sockaddr_in &source) {
  if (length < 10 || memcmp(data, ARTNET_ID, sizeof(ARTNET_ID)) != 0) {
    return;
  }
  if (data[8] == ARTNET_OP_POLL && data[9] == 0x20) {
    if (length >= 14) {
      this->queue_poll_reply_(&source, true);
    }
    return;
  }
  if (data[8] == ARTNET_OP_DMX && data[9] == 0x50) {
    this->handle_artdmx_(data, length);
  }
}

void ArtNetDMX::handle_artdmx_(const uint8_t *data, size_t length) {
  if (length < 18) {
    return;
  }
  const uint16_t protocol_version = (static_cast<uint16_t>(data[10]) << 8) | data[11];
  const uint16_t payload_length = (static_cast<uint16_t>(data[16]) << 8) | data[17];
  if (protocol_version < 14 || payload_length < 2 || payload_length > 512 || (payload_length & 1) != 0 ||
      length < 18 + payload_length) {
    return;
  }

  const uint8_t packet_net = data[15] & 0x7F;
  const uint8_t packet_subnet = data[14] >> 4;
  const uint8_t packet_universe = data[14] & 0x0F;
  if (packet_net != this->net_ || packet_subnet != this->subnet_ || packet_universe != this->universe_) {
    return;
  }

  const uint8_t sequence = data[12];
  if (sequence == 0) {
    this->have_sequence_ = false;
  } else if (this->have_sequence_) {
    const uint8_t delta = static_cast<uint8_t>(sequence - this->last_sequence_);
    if (delta == 0 || delta > 127) {
      ESP_LOGV(TAG, "Dropping out-of-order ArtDmx sequence %u", static_cast<unsigned>(sequence));
      return;
    }
  }
  if (sequence != 0) {
    this->last_sequence_ = sequence;
    this->have_sequence_ = true;
  }

  std::fill(this->dmx_.begin() + 1, this->dmx_.end(), 0);
  const size_t source_offset = this->start_channel_ - 1;
  const size_t available = payload_length > source_offset ? payload_length - source_offset : 0;
  const size_t copied = std::min<size_t>(this->channels_, available);
  if (copied > 0) {
    memcpy(this->dmx_.data() + 1, data + 18 + source_offset, copied);
  }
  this->last_packet_ms_ = static_cast<uint64_t>(esp_timer_get_time()) / 1000;
  this->have_received_packet_ = true;
  this->blackout_applied_ = false;
  ESP_LOGV(TAG, "Accepted ArtDmx packet with %u slots", static_cast<unsigned>(payload_length));
}

bool ArtNetDMX::data_is_recent_(uint64_t now_ms) const {
  return this->have_received_packet_ && now_ms - this->last_packet_ms_ < this->timeout_ms_;
}

void ArtNetDMX::update_timeout_() {
  if (!this->have_received_packet_ || this->data_is_recent_(static_cast<uint64_t>(esp_timer_get_time()) / 1000)) {
    return;
  }

  this->have_received_packet_ = false;
  this->have_sequence_ = false;
  if (this->timeout_blackout_ && !this->blackout_applied_) {
    std::fill(this->dmx_.begin() + 1, this->dmx_.end(), 0);
    this->blackout_applied_ = true;
    ESP_LOGI(TAG, "ArtDmx timeout; blacking out DMX output");
  } else {
    ESP_LOGI(TAG, "ArtDmx timeout; holding the last DMX frame");
  }
}

void ArtNetDMX::send_dmx_frame_() {
  const auto uart = static_cast<uart_port_t>(this->uart_num_);
  if (this->de_pin_ != nullptr) {
    this->de_pin_->digital_write(true);
  }

  uint8_t break_byte = 0;
  esp_err_t err = uart_set_baudrate(uart, 83333);
  if (err == ESP_OK) {
    if (uart_write_bytes(uart, &break_byte, 1) != 1) {
      err = ESP_FAIL;
    } else {
      err = uart_wait_tx_done(uart, pdMS_TO_TICKS(100));
    }
  }
  if (err == ESP_OK) {
    err = uart_set_baudrate(uart, 250000);
  }
  if (err == ESP_OK) {
    const int written = uart_write_bytes(uart, this->dmx_.data(), this->channels_ + 1);
    if (written != this->channels_ + 1) {
      err = ESP_FAIL;
    } else {
      err = uart_wait_tx_done(uart, pdMS_TO_TICKS(100));
    }
  }

  if (this->de_pin_ != nullptr) {
    this->de_pin_->digital_write(false);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "DMX UART transmission failed: %s", esp_err_to_name(err));
    this->mark_failed();
  }
}

void ArtNetDMX::queue_poll_reply_(const sockaddr_in *poller, bool send_unicast) {
  for (auto &reply : this->pending_replies_) {
    if (reply.active) {
      continue;
    }
    reply.active = true;
    reply.send_unicast = send_unicast;
    if (poller != nullptr) {
      reply.poller = *poller;
      reply.poller.sin_port = htons(6454);
    }
    reply.due_us = now_us_() + (esp_random() % 101) * 1000;
    return;
  }
  ESP_LOGW(TAG, "ArtPoll reply queue full; dropping discovery response");
}

void ArtNetDMX::send_due_poll_replies_() {
  const uint64_t now = now_us_();
  for (auto &reply : this->pending_replies_) {
    if (!reply.active || now < reply.due_us) {
      continue;
    }
    this->send_poll_reply_(reply.send_unicast ? &reply.poller : nullptr, reply.send_unicast);
    reply.active = false;
  }
}

void ArtNetDMX::send_poll_reply_(const sockaddr_in *poller, bool send_unicast) {
  if (this->socket_ == nullptr || !network::is_connected()) {
    return;
  }

  uint8_t packet[ARTNET_POLL_REPLY_SIZE]{};
  memcpy(packet, ARTNET_ID, sizeof(ARTNET_ID));
  packet[8] = static_cast<uint8_t>(ARTNET_OP_POLL_REPLY & 0xFF);
  packet[9] = static_cast<uint8_t>(ARTNET_OP_POLL_REPLY >> 8);
  packet[14] = static_cast<uint8_t>(6454 >> 8);
  packet[15] = static_cast<uint8_t>(6454 & 0xFF);
  packet[16] = 0;
  packet[17] = 1;
  packet[18] = this->net_ & 0x7F;
  packet[19] = this->subnet_ & 0x0F;
  packet[20] = 0xFF;
  packet[21] = 0xFF;
  packet[24] = 0;
  packet[25] = 0;

  memcpy(packet + 26, this->short_name_.data(), this->short_name_.size());
  memcpy(packet + 44, this->long_name_.data(), this->long_name_.size());
  const char *report = this->data_is_recent_(static_cast<uint64_t>(esp_timer_get_time()) / 1000)
                           ? "#0001 [0000] Art-Net DMX bridge: receiving ArtDmx"
                           : "#0001 [0000] Art-Net DMX bridge: waiting for ArtDmx";
  snprintf(reinterpret_cast<char *>(packet + 108), 64, "%s", report);

  packet[172] = 0;
  packet[173] = 1;
  packet[174] = 0x80;
  packet[182] = this->data_is_recent_(static_cast<uint64_t>(esp_timer_get_time()) / 1000) ? 0x80 : 0;
  packet[190] = this->universe_;
  packet[200] = 0;
  packet[211] = 1;
  const esp_err_t mac_result = esp_read_mac(packet + 201, ESP_MAC_WIFI_STA);
  if (mac_result != ESP_OK) {
    ESP_LOGW(TAG, "Unable to read device MAC address: %s", esp_err_to_name(mac_result));
  }

  sockaddr_in broadcast{};
  broadcast.sin_family = AF_INET;
  broadcast.sin_port = htons(6454);
  broadcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);

  NetifAddress address{};
  if (esp_netif_find_if(find_ipv4_netif_, &address) != nullptr) {
    memcpy(packet + 10, &address.info.ip.addr, sizeof(address.info.ip.addr));
    memcpy(packet + 207, &address.info.ip.addr, sizeof(address.info.ip.addr));
    const uint32_t ip_host = ntohl(address.info.ip.addr);
    const uint32_t mask_host = ntohl(address.info.netmask.addr);
    if (mask_host != 0 && mask_host != 0xFFFFFFFF) {
      broadcast.sin_addr.s_addr = htonl((ip_host & mask_host) | ~mask_host);
    }
  }

  if (send_unicast && poller != nullptr) {
    const ssize_t sent =
        this->socket_->sendto(packet, sizeof(packet), 0, reinterpret_cast<const sockaddr *>(poller), sizeof(*poller));
    if (sent != sizeof(packet)) {
      ESP_LOGW(TAG, "Unable to send unicast ArtPollReply: errno %d", errno);
    }
  }
  const ssize_t broadcast_sent = this->socket_->sendto(packet, sizeof(packet), 0,
                                                       reinterpret_cast<const sockaddr *>(&broadcast), sizeof(broadcast));
  if (broadcast_sent != sizeof(packet)) {
    ESP_LOGW(TAG, "Unable to broadcast ArtPollReply: errno %d", errno);
  }
}

void ArtNetDMX::dump_config() {
  ESP_LOGCONFIG(TAG, "Art-Net DMX bridge:");
  LOG_PIN("  TX Pin: ", this->tx_pin_);
  LOG_PIN("  DE Pin: ", this->de_pin_);
  ESP_LOGCONFIG(TAG,
                "  UART: %u\n"
                "  UDP port: %u\n"
                "  Port address: %u-%u-%u\n"
                "  Channels: %u starting at Art-Net channel %u\n"
                "  Refresh rate: %.1f Hz\n"
                "  Timeout: %u ms (%s)\n"
                "  Short name: %s\n"
                "  Long name: %s",
                static_cast<unsigned>(this->uart_num_), static_cast<unsigned>(this->port_),
                static_cast<unsigned>(this->net_), static_cast<unsigned>(this->subnet_),
                static_cast<unsigned>(this->universe_), static_cast<unsigned>(this->channels_),
                static_cast<unsigned>(this->start_channel_), this->refresh_rate_,
                static_cast<unsigned>(this->timeout_ms_),
                this->timeout_blackout_ ? "blackout" : "hold", this->short_name_.data(), this->long_name_.data());
}

void ArtNetDMX::on_shutdown() {
  if (uart_is_driver_installed(static_cast<uart_port_t>(this->uart_num_))) {
    uart_wait_tx_done(static_cast<uart_port_t>(this->uart_num_), pdMS_TO_TICKS(100));
    uart_driver_delete(static_cast<uart_port_t>(this->uart_num_));
  }
}

}  // namespace artnet_dmx
}  // namespace esphome
