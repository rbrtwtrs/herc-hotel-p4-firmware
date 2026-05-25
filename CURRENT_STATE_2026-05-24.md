# Herc Hotel P4 Current State - 2026-05-24

This document records the known-good state and the remaining operational caveat so the repo is not ambiguous.

## Device

- Hardware: ESP32-P4 Waveshare board with OV5647 / Raspberry Pi Camera B.
- Network identity: `10.1.70.181` / `hercbtl-hotel.nautilus.oet.org`.
- MQTT broker: `10.1.70.53:1883`.
- MQTT device name/topic prefix: `herc-hotel-p4`.

## Working On The Bench

- Ethernet boot and MQTT connection are working.
- Network OTA over Ethernet/MQTT is verified from the running firmware. Serial was used once only to recover/install the corrected PSRAM+HTTP-OTA base image.
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

## Verified Network OTA

The network OTA path has been verified without using serial for the update:

- Base recovery install: serial flash of `hotel-ws-psram-httpota-base-20260524`, required because the earlier running image did not have PSRAM and HTTP OTA enabled together.
- OTA verification image: `hotel-ws-ota-netverify-v2-20260524`.
- OTA command published to `herc-hotel-p4/cmd/ota`: `http://10.1.70.131:8007/herc_hotel_p4.bin`.
- Device requested the image over HTTP: `GET /herc_hotel_p4.bin`.
- MQTT OTA state sequence: `started` then `success_restarting`.
- After reboot, MQTT health reported:
  - `app_version`: `hotel-ws-ota-netverify-v2-20260`
  - `partition`: `ota_1`
- Post-OTA camera status:
  - `ready`: `true`
  - `last_error`: `ok`
  - `capture_errors`: `0`
  - `free_spiram`: about `27.9 MB`
- Post-OTA snapshot check: `HTTP 200`, `382427` bytes.
- Post-OTA MQTT I2C scan: `0x48`, `0x49`, `0x4A`, `0x4B`, `0x77`.
- Post-OTA ring state: default red blink, `brightness:255`, RGBW `255,0,0,0`.

The repo default `sdkconfig` now matches the Waveshare/PSRAM configuration and enables plain HTTP OTA:

- `CONFIG_SPIRAM=y`
- `CONFIG_SPIRAM_SPEED_200M=y`
- `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y`
- `CONFIG_OTA_ALLOW_HTTP=y`

Verified OTA app artifact:

- App image: `herc_hotel_p4.bin`, SHA256 `9B576A97BDC549422EECB82F0D8DC2B0280DE505A58E8EF07360255DE8EBC516`
- App size: `1079104` bytes

For future OTA updates:

1. Build with the repo default `sdkconfig`.
2. Serve `build/herc_hotel_p4.bin` from a reachable HTTP server.
3. Publish the URL to `herc-hotel-p4/cmd/ota` with `retain=false`.
4. Watch `herc-hotel-p4/ota/state` for `started` and `success_restarting`.
5. Verify reboot, `herc-hotel-p4/status online`, `herc-hotel-p4/health` app version/partition, `http://10.1.70.181/camera_status`, and `http://10.1.70.181/snapshot.jpg`.

## Known Boundaries

- Serial should not be assumed available in operation.
- Camera SCCB uses GPIO7/GPIO8; do not move Hotel I2C back to those pins.
- The current preferred browser workflow is `/focus`, not `/stream.mjpg`.
