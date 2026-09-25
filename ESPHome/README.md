# ESPHome Art-Net DMX bridge

`artnet_dmx` is an ESPHome external component for ESP32 devices using the ESP-IDF framework. It receives Art-Net DMX data over UDP and continuously outputs DMX512 on a UART connected to an RS-485 transceiver. It also answers ArtPoll discovery requests and announces itself when the network comes up. It does not create ESPHome light or output entities.

## Wiring

Connect an ESP32 UART TX pin to the transceiver's `DI` input and the configured DE pin to both `DE` and `/RE` (or use the transceiver's equivalent direction control). Connect the transceiver's differential output to DMX XLR pin 3 (Data+) and pin 2 (Data-); XLR pin 1 is ground. Use a proper isolated DMX transceiver and termination appropriate to the bus. Do not connect an RS-485 bus directly to ESP32 GPIO.

For the example below, connect GPIO17 to `DI` and GPIO16 to `DE` and `/RE`. The component holds DE high only while transmitting. UART0 is usually used by the ESPHome logger, so the example uses UART1 and disables logger serial output.

## External component

For the ESPHome builder or dashboard, add the repository's `ESPHome/components` directory as the external component source:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/heebtob/IoT
      ref: main
      path: ESPHome/components
    components: [artnet_dmx]
```

## Configuration

| Option | Type | Default | Description |
| --- | --- | --- | --- |
| `id` | ID | required | Component ID. |
| `uart_num` | `0`, `1`, or `2` | `1` | ESP-IDF UART port. UART0 is commonly reserved for logger output. |
| `tx_pin` | GPIO pin | required | DMX UART transmit pin. |
| `de_pin` | GPIO pin | none | Optional RS-485 driver-enable pin, driven high during a frame. |
| `net` | `0`–`127` | `0` | Art-Net Net address. |
| `subnet` | `0`–`15` | `0` | Art-Net Sub-Net address. |
| `universe` | `0`–`15` | `0` | Art-Net universe within the Sub-Net. |
| `port` | UDP port | `6454` | Art-Net UDP listen port. |
| `channels` | `1`–`512` | `512` | Number of DMX slots transmitted (in addition to the start code). |
| `start_channel` | `1`–`512` | `1` | First 1-based Art-Net channel copied to DMX slot 1. |
| `refresh_rate` | `1`–`44` Hz | `40` | Requested maximum DMX refresh rate. The frame's wire time imposes an additional upper limit. |
| `timeout` | time period | `5s` | Time without matching ArtDmx data before the timeout action is applied. |
| `timeout_action` | `hold` or `blackout` | `hold` | Keep the last frame, or clear all output slots to zero, after timeout. |
| `short_name` | string, max 17 chars | ESPHome device name | ArtPollReply short name. |
| `long_name` | string, max 63 chars | ESPHome device name | ArtPollReply long name. |

Both ArtPollReply names default to the ESPHome device name and are limited to the Art-Net field sizes. `start_channel` and `channels` are independently configurable; any output slots for which the incoming packet has no source channel are zero-filled.

## Complete example

```yaml
esphome:
  name: artnet-dmx-bridge
  friendly_name: Art-Net DMX Bridge

esp32:
  board: esp32dev
  framework:
    type: esp-idf

external_components:
  - source:
      type: git
      url: https://github.com/heebtob/IoT
      ref: main
      path: ESPHome/components
    components: [artnet_dmx]

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:

ota:
  - platform: esphome

logger:
  baud_rate: 0

artnet_dmx:
  id: dmx_bridge
  uart_num: 1
  tx_pin: GPIO17
  de_pin: GPIO16
  universe: 0
  channels: 512
```

## Limitations

- ESP32 with ESP-IDF is required; Arduino framework and other platforms are not supported.
- The component receives one configured Art-Net port address and drives one physical DMX output. It is designed for a single instance in v1.
- ArtPoll/ArtPollReply discovery is supported, including unicast and subnet-directed-broadcast replies. ArtAddress, ArtIpProg, RDM, and ArtSync handling are out of scope.
- The RS-485 transceiver, bus cabling, termination, and electrical isolation are external hardware.
