# Feller EDIZIOdue colore UNI-Taster 392x for ESPHome

This external component drives one or more Feller UNI-Taster inserts over a separate ESPHome UART per insert. It targets the **asynchronous protocol without handshake** (ComType 11, no resistor on pin 6), with 8N1 UART at 9600 baud by default (no resistor on pin 10). It exposes up to eight debounced buttons, eight RGB-snap LED lights with a hardware **Blink** effect (1 Hz, equal on/off time), and one global 0–255 LED brightness number. LEDs can only display pure red, green or blue, not mixed colours or per-LED brightness. Changes to several lights in one loop are sent as a single LED frame.

## Wiring

The example uses ESP32 `esp32dev` with the **ESP-IDF** framework. ESPHome's UART device API is also supported by the Arduino framework.

The 2x5 header (2.54 mm) has 5 V signal levels:

| Pin | Signal | Connection |
| --- | --- | --- |
| 1 | GND | ESP32 ground |
| 2 | RxD (insert to host) | ESP32 RX **through a 5 V to 3.3 V level shifter or divider** |
| 3 | CLK | Not connected in UART mode |
| 4 | TxD (host to insert) | ESP32 TX **through a 3.3 V to 5 V level shifter**; pull-up recommended |
| 5 | +5V | Regulated 5 V supply |
| 6 | COM TYPE | Leave unconnected for asynchronous protocol without handshake |
| 7 | CTS | Unused |
| 8 | NC | Not connected |
| 9 | RTS | Unused |
| 10 | BAUD RATE | Leave unconnected for 9600-baud startup |

Power consumption is about 1 mA without LEDs and up to about 19 mA with eight blue LEDs at 100% brightness. **Do not directly connect 5 V insert outputs to ESP32 GPIO.** RxD and CTS have pull-ups on the insert. Share a ground between the supply and ESP32.

## External component

Paste this source into ESPHome Builder; the complete ESP32 configuration is in [`ESPHome/feller-uni-taster.yaml`](../../feller-uni-taster.yaml). Until this change is merged into `main`, use the PR branch as `ref`.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/heebtob/IoT
      ref: main
      path: ESPHome/components
    components: [feller_uni_taster]
```

The example starts a Wi-Fi access point for initial setup; replace its `wifi:` block with your network credentials for normal Home Assistant access. Use `uart:` with `tx_pin: GPIO17`, `rx_pin: GPIO16`, `baud_rate: 9600`, `data_bits: 8`, `parity: NONE`, `stop_bits: 1`, then configure `feller_uni_taster:` with `uart_id` matching the UART. Add `binary_sensor:`, `light:`, and `number:` entities as shown in the complete example.

## Configuration

| Option | Type | Default | Description |
| --- | --- | --- | --- |
| `id` / `uart_id` | ID | required | Unique hub and ESPHome UART IDs. Multiple hubs need distinct UARTs. |
| `baud_rate` | baud rate | `9600` | Negotiated speed; set the same speed in `uart:`. Supported: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200. |
| `byte_timeout` | `0`–`255` | `0` | Insert timeout factor (0/1 automatic; 2–255 factor × 10 / baud seconds, maximum 500 ms). |
| `button_indication` | boolean | `true` | Request change-driven button events. |
| `system_state_indication` | boolean | `true` | Request error indications. |
| `reset_on_boot` | boolean | `true` | Send software reset on boot and wait for insert indication. |
| `poll_interval` | time period | `0s` | Optional button polling fallback. Enable with a positive interval when indications are disabled. |

Each button (`binary_sensor: - platform: feller_uni_taster`) and LED (`light: - platform: feller_uni_taster`) needs `index: 1` through `8`, and `feller_uni_taster_id` if multiple hubs exist. Hardware variant discovery warns about indexes unsupported by the insert. Each LED supports RGB colour (snapped to the dominant channel, ties red before green before blue) and the built-in `Blink` effect; transitions default to `0s` because the insert cannot fade. ESPHome software light effects are not supported. `number: - platform: feller_uni_taster` controls global LED brightness with optional `restore_value: true` (default).

The insert's 1/4 buttons use T1/L1 through T8/L8. The 1/2 layout uses T3/L3, T4/L4, T7/L7 and T8/L8. A 1/1 button actuates T3 and/or T7 on the left and T4 and/or T8 on the right.

## Complete example

See [`ESPHome/feller-uni-taster.yaml`](../../feller-uni-taster.yaml) for a paste-ready ESP32 configuration with a hub, eight buttons, eight lights, and the global brightness number.

## Protocol

Frames contain an even-parity header (`P 01 LLLLL`, where the low five bits encode the following byte count), a service and its payload. Invalid headers and incomplete frames are discarded. The insert sends `A0` at boot or after a software reset; the hub sends `SetSystemSettings` (`10`), receives confirmation (`11`) on the old UART settings, switches speed, then verifies via `GetSystemSettings` (`12`/`13`) within 10 seconds. It queries hardware/software info (`1C`/`1D`); receives `ButtonState` (`42`) and optionally polls `GetButtonState` (`40`/`41`); sends `SetLedState` variant 2 (`30`/`31`) and `SetLedBrightness` (`38`/`39`); and logs decoded `SystemState` errors (`1A`). No RTS/CTS or software handshake is used.

Protocol reference: Feller document 10.UNI3924-D.2104, *Applikationsbeschreibung UNI-Taster*.

## Limitations

- No software/RTS/CTS handshake or SPI mode; wire the insert for asynchronous protocol without handshake.
- LED colours snap to red, green or blue, with only global brightness. The `Blink` effect uses the insert's fixed 1 Hz cycle.
- A 4-button or 0/6-LED insert reports its hardware variant; entities for unavailable indexes are warned about, not removed.
