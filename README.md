# ESP32 ADS-B Mini Radar

Shows live aircraft from [api.adsb.lol](https://api.adsb.lol) as a radar on a 1.8" ST7735S TFT (KMR-1.8, 160x128), centred on Quito / SEQM.

## Hardware
- ESP32 Dev Module
- KMR-1.8 SPI display (ST7735S). Wiring is listed in the header of `plane_tracker_esp32.ino`.

## Libraries
- Adafruit ST7735 and ST7789 Library
- Adafruit GFX Library
- ArduinoJson (v7)

## Setup
1. Copy `secrets.example.h` to `secrets.h` and enter your Wi-Fi details there (git ignores `secrets.h`).
2. Set the map centre and the range under `USER SETTINGS` in the sketch.
3. Select the board **ESP32 Dev Module** and upload.

## On-screen errors
- **ERR 426**: Upgrade Required. The API rejects HTTP/1.0, so the sketch uses HTTP/1.1.
- **ERR 429**: Too Many Requests. The sketch polls every 15 s and waits longer when rate-limited.
- **ERR -100**: The JSON response could not be parsed.
- **ERR < 0 (other)**: Connection or TLS failure (see the ESP32 `HTTPClient` error codes).
