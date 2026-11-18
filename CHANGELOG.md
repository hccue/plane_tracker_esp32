# Changelog

## [0.2.0]
### Fixed
- **426 Upgrade Required**: Switched from HTTP/1.0 to HTTP/1.1. The response body is now read in full before parsing, because HTTP/1.1 replies can arrive in chunks.
- **429 Too Many Requests**: Raised the poll interval from 5 s to 15 s. On a 429, the sketch now waits for the time in `Retry-After`, or doubles its wait, up to 120 s.

## [0.1.0]
### Added
- First version: an ADS-B radar on an ESP32 with an ST7735S display, using data from api.adsb.lol.
