# ESP32 UART Bridge

Wireless serial port for USB devices. An ESP32-S3 acts as a USB host for a
USB-serial adapter (CP210x, CH34x, FTDI, ...) and exposes it over BLE using the
**Nordic UART Service (NUS)** — the de-facto standard for BLE serial — extended
with characteristics for full serial control (DTR/RTS, baud rate, line status).

Typical uses: configuring headless gear over a console port without a cable,
and **flashing ESP32 targets over the air with unmodified esptool / PlatformIO**
through the included RFC2217 proxy.

## Features

- **Standard NUS data channel** — works out of the box with any NUS terminal
  app (iOS/Android/desktop). Transparent byte stream, no framing.
- **Full serial control over BLE** — DTR/RTS control lines, line coding
  (baud rate / data bits / parity / stop bits), and serial-state notifications,
  via three extension characteristics.
- **Timed control-line batches** — reset sequences (e.g. entering the ESP32
  bootloader) execute on the bridge with precise local timing, immune to BLE
  link latency and jitter.
- **RFC2217 network serial proxy** (`tools/nus2217.py`) — esptool, PlatformIO,
  and any pyserial program use the bridge natively, including the stub flasher
  (measured ~448 kbit/s upload).
- **WS2812 status LED** — connection and RX/TX activity at a glance.

## Hardware

- ESP32-S3 dev board (tested on ESP32-S3-DevKitC-1, 8 MB flash)
- A USB-serial adapter attached to the S3's USB-OTG port
  (some boards need a solder bridge to enable USB-OTG host power)
- WS2812 RGB LED on GPIO 48 (integrated on ESP32-S3-WROOM-1 devkits)

## GATT interface

Advertised name: `ESP32_BRIDGE`. No pairing required.

| Characteristic | UUID (`6E40xxxx-B5A3-F393-E0A9-E50E24DCCA9E`) | Access | Purpose |
|---|---|---|---|
| RX (client → device) | `0002` | Write / Write w/o resp | Bytes to the USB serial port |
| TX (device → client) | `0003` | Notify | Bytes from the USB serial port |
| Control lines | `0004` | Write / Read | DTR/RTS bitmask (1 B) or timed batch (2–64 B) |
| Line coding | `0005` | Write / Read | USB CDC layout, 7 B: baud (LE u32), stop, parity, data bits |
| Serial state | `0006` | Notify / Read | USB serial state bitmap (2 B LE) |

`0002`/`0003` are plain NUS; the `0004`–`0006` extensions are ignored by
generic NUS apps. USB side defaults to 115200 8N1 until changed via `0005`.
See [docs/ipad-integration.md](docs/ipad-integration.md) for the full protocol,
including the bootloader-reset batch sequences.

## Building

### PlatformIO

```sh
pio run -t upload            # env: esp32-s3-n8r2
```

### ESP-IDF (v5.3+)

```sh
idf.py set-target esp32s3
idf.py build flash
```

## Flashing ESP32 targets over BLE

```sh
./tools/nus2217.py           # scans for ESP32_BRIDGE, listens on localhost:4000

esptool --port rfc2217://localhost:4000 write-flash 0x10000 app.bin
pio run -t upload --upload-port rfc2217://localhost:4000
```

Details, tuning, and known issues: [tools/README.md](tools/README.md).

## LED status

| Color | Meaning |
|---|---|
| Purple | BLE connected, idle |
| Blue | USB serial connected, no BLE |
| Red / Green | RX / TX activity |
| Yellow | Activity in both directions |
| Off | Nothing connected |
