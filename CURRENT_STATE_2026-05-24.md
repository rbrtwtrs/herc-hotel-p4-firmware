# Herc Hotel P4 Current State - 2026-05-24

This document records the known-good state and the remaining operational caveat so the repo is not ambiguous.

## Device

- Hardware: ESP32-P4 Waveshare board with OV5647 / Raspberry Pi Camera B.
- Network identity: `10.1.70.181` / `hercbtl-hotel.nautilus.oet.org`.
- MQTT broker: `10.1.70.53:1883`.
- MQTT device name/topic prefix: `herc-hotel-p4`.

## Working On The Bench

- Ethernet boot and MQTT connection are working.
- Camera is working with browser endpoints:
  - `http://10.1.70.181/focus`
  - `http://10.1.70.181/snapshot.jpg`
  - `http://10.1.70.181/camera_status`
- The focus page uses repeated snapshots. The true MJPEG stream endpoint was removed because it could wedge the HTTP server.
- Snapshot capture is serialized and bounded so repeated browser refreshes do not leave the camera stuck indefinitely.
- MQTT reboot command is available on `herc-hotel-p4/cmd/reboot`.
- Hotel I2C sensors were moved off the camera SCCB bus:
  - I2C port: `I2C_NUM_1`
  - SDA: GPIO4
  - SCL: GPIO5
  - Verified devices: `0x48`, `0x49`, `0x4A`, `0x4B`, `0x77`.
- BME688, ADS1115/leak zone telemetry, and Home Assistant MQTT discovery are enabled.
- Bender UART remains on UART1:
  - TX: GPIO24
  - RX: GPIO22
- Status LEDs:
  - GPIO25
  - GPIO26
- Ring LED:
  - GPIO3
  - 18 LEDs
  - SK6812/GRBW RGBW
  - Default is full-brightness red blink at 2 Hz.
  - `white` MQTT command sets temporary solid white.
  - Command brightness can be set with payloads such as `red brightness=32` or `white brightness=32`.
  - Commands time out after 60 seconds and return to default red blink.

## MQTT Commands

- OTA URL: `herc-hotel-p4/cmd/ota`
- OTA state: `herc-hotel-p4/ota/state`
- Reboot: `herc-hotel-p4/cmd/reboot`
- Ring: `herc-hotel-p4/cmd/ring`
- Leak threshold: `herc-hotel-p4/cmd/leak_threshold`

Do not publish retained messages to command topics.

## Important OTA Caveat

The unit currently running on the bench was flashed before HTTP OTA was enabled in sdkconfig. It accepts MQTT commands, but a plain `http://.../herc_hotel_p4.bin` OTA URL does not fetch because `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP` was disabled in that running firmware.

This repo state enables HTTP OTA for the next build:

- `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y`
- `CONFIG_OTA_ALLOW_HTTP=y`

The patched source was build-verified locally as `hotel-ws-httpota-fullred-20260524` with ESP-IDF `v5.5.2`.

- App image: `herc_hotel_p4.bin`, SHA256 `A0485BC5B9D88F85BEE9C724DF1D06F675E0C243A164222BC9227739FAD9FAEE`
- Bootloader: `bootloader.bin`, SHA256 `F9EB0F92F7C6EDA4F0122BAD1E93730DC566D3CA07C0FF6CBE576B8DEDA2A09D`
- Partition table: `partition-table.bin`, SHA256 `F1E567915E11BC9C49E65FE894F631FF59EF8C971DA2AEE4437BB8E70A262300`
- App size: `0x104250`
- Smallest OTA app partition: `0x700000`

One physical flash of this repo state is required before Ethernet HTTP OTA can be honestly called verified. After that flash, OTA should be validated using MQTT only, with no serial dependency:

1. Serve `build/herc_hotel_p4.bin` from a reachable HTTP server.
2. Publish the URL to `herc-hotel-p4/cmd/ota`.
3. Watch `herc-hotel-p4/ota/state` for `started` and `success_restarting`.
4. Verify reboot, `herc-hotel-p4/status online`, `http://10.1.70.181/camera_status`, and `http://10.1.70.181/snapshot.jpg`.

## Known Boundaries

- Serial should not be assumed available in operation.
- Camera SCCB uses GPIO7/GPIO8; do not move Hotel I2C back to those pins.
- The current preferred browser workflow is `/focus`, not `/stream.mjpg`.
