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
  - Built-in fallback default is full-brightness red blink at 2 Hz.
  - Saved default mode is configurable over MQTT and stored in ESP32-P4 NVS flash.
  - Current saved default on the bench is steady RGBW white at brightness `100` on the 0-255 scale.
  - Named MQTT color commands are allowed: `red`, `white`, `green`, `blue`, `cyan`, `magenta`, and `yellow`.
  - Arbitrary RGB numeric commands remain disabled.
  - Command brightness can be set with payloads such as `red brightness=32`, `white brightness=32`, or `green brightness=26`.
  - Commands time out after 10 minutes and return to the saved default mode.
  - Any `/snapshot.jpg` request preserves the current LED ring setting and, if a command is active, resets that command timeout to 10 minutes.

## MQTT Commands

- OTA URL: `herc-hotel-p4/cmd/ota`
- OTA state: `herc-hotel-p4/ota/state`
- Reboot: `herc-hotel-p4/cmd/reboot`
- Ring: `herc-hotel-p4/cmd/ring`
- Ring default mode: `herc-hotel-p4/cmd/ring_default`
- Leak threshold: `herc-hotel-p4/cmd/leak_threshold`

Do not publish retained messages to command topics.

Ring default mode commands are saved in NVS and affect startup, `default`/`auto` ring commands, and command timeout fallback:

- `red_blink` returns the saved default to the original 2 Hz red blink.
- `white brightness=100` sets the saved default to steady RGBW white at brightness value `100` on the 0-255 scale.
- `white brightness=255` sets the saved default to steady RGBW white at full brightness.

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
- Later OTA update `hotel-ws-snapshotquiet-20260524` verified the 10-minute ring behavior:
  - OTA URL: `http://10.1.70.131:8008/herc_hotel_p4.bin`.
  - Device fetched the image over HTTP and rebooted into `ota_0`.
  - MQTT health app version: `hotel-ws-snapshotquiet-20260524`.
  - Explicit `/snapshot.jpg` returned `HTTP 200`, `397334` bytes, and published ring state `command`/`OFF` with `timeout_s` about `600`.
  - Explicit ring command `white brightness=32` published ring state `command`/`ON`, RGBW `0,0,0,255`, brightness `32`, with `timeout_s` about `600`.
- Later OTA update `hotel-ws-ringcolors-20260524` verified the named-color whitelist:
  - OTA URL: `http://10.1.70.131:8009/herc_hotel_p4.bin`.
  - Device fetched the image over HTTP and rebooted into `ota_1`.
  - MQTT health app version: `hotel-ws-ringcolors-20260524`.
  - `/camera_status` returned `HTTP 200` and ready.
  - `/snapshot.jpg` returned `HTTP 200` and published ring state `command`/`OFF` with `timeout_s` about `600`.
  - MQTT ring commands `green brightness=26`, `blue brightness=26`, and `yellow brightness=26` each published the expected RGBW state with `timeout_s` about `600`.
  - Final `off` command left the ring quiet/off for about 10 minutes.
- Later OTA update `hotel-ws-ringhold-min-20260524` verified snapshot timeout extension without changing the current LED setting:
  - OTA was triggered with a retained test URL during troubleshooting; command topics were cleared afterward. Do not use retained command messages in normal operation.
  - MQTT health app version: `hotel-ws-ringhold-min-20260524`.
  - MQTT health partition: `ota_1`.
  - MQTT ring command `green brightness=26` published the expected green RGBW state.
  - `/snapshot.jpg` returned `HTTP 200` and left the ring green while resetting `timeout_s` to about `600`.
  - MQTT ring command `off` followed by `/snapshot.jpg` left the ring off while resetting `timeout_s` to about `600`.
  - Final `default` command returned the ring to full-brightness default red blink.
  - Note: `/camera_status` still reported stale `last_error:"capture_failed:0xb006"` after rapid snapshot testing, but `/snapshot.jpg` returned valid JPEGs and health remained online.
- Later OTA update on 2026-06-01 added the persistent ring default mode command:
  - Build path: `C:\ocbuild\herc-hotel-p4-defaultmode-ota-20260601`.
  - OTA URL: `http://10.1.70.131:8010/herc_hotel_p4.bin`.
  - MQTT OTA state sequence: fresh `started` then fresh `success_restarting`.
  - Device rebooted and republished `herc-hotel-p4/status online`.
  - Post-OTA `/camera_status` returned `ready:true`, `last_error:"ok"`, and `capture_errors:0`.
  - Post-OTA `/snapshot.jpg` returned `HTTP 200` JPEG.
  - MQTT ring default command `white brightness=100` saved NVS default state `{"mode":"steady_white","white_brightness":100}`.
  - Final retained ring state: steady white, `brightness:100`, RGBW `0,0,0,255`.
  - MQTT command topics were published with `retain=false`; temporary HTTP server was stopped after OTA.

The repo default `sdkconfig` now matches the Waveshare/PSRAM configuration and enables plain HTTP OTA:

- `CONFIG_SPIRAM=y`
- `CONFIG_SPIRAM_SPEED_200M=y`
- `CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y`
- `CONFIG_OTA_ALLOW_HTTP=y`

Verified OTA app artifact:

- App image: `C:\ocbuild\herc-hotel-p4-defaultmode-ota-20260601\build\herc_hotel_p4.bin`
- SHA256: `293997061F5D0F55AE7636BD6B7911A1C680D095B1366FB498FA17A7C4720652`
- App size: `1094656` bytes

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
