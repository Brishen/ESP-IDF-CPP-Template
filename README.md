# ESP32-P4 Audio Streamer

This project demonstrates how to stream audio from the **Waveshare ESP32-P4-NANO**'s onboard microphone (ES8311) over Wi-Fi using a simple HTTP webserver.

Since the ESP32-P4 does not have internal Wi-Fi, this project uses the **`espressif/esp_wifi_remote`** component to communicate with an external Wi-Fi coprocessor (typically an **ESP32-C6** or **ESP32-C5**) connected via SDIO or SPI.

## Features
- **Audio Capture**: Captures audio from the ES8311 codec via I2S (16kHz, 16-bit, Mono/Stereo).
- **Audio Streaming**: Serves a WAV audio stream (Mic input) via HTTP at `http://<IP>/stream`.
- **Audio Playback**: Plays WAV files (uploaded via POST) to the local speaker with automatic resampling.
- **Volume Control**: Adjustable output volume via API.
- **External Wi-Fi**: Uses the ESP-Hosted solution to provide Wi-Fi connectivity to the P4.

## Hardware Requirements
1.  **Host Board**: ESP32-P4 Development Board (e.g., Waveshare ESP32-P4-NANO).
2.  **Slave Module**: ESP32-C6 or ESP32-C5 module acting as the Wi-Fi network adapter.
    *   *Note: Many P4 boards come with a C6 wire-down on the board.*
3.  **Microphone**: ES8311 (or compatible I2S codec on the standard P4 I2S pins).

## Prerequisites
1.  **ESP-IDF**: Ensure you have a recent version of ESP-IDF (v5.3+ recommended).
2.  **Slave Firmware**: The external Wi-Fi module (e.g., ESP32-C6) **MUST** be flashed with the correct **ESP-Hosted Slave Firmware**.
    *   See [ESP-Hosted Documentation](https://github.com/espressif/esp-hosted) for instructions on flashing the slave.

## Build Instructions

1.  **Clone and Update Submodules**:
    This project relies on detailed ESP-IDF components. Ensure your ESP-IDF installation is healthy.
    ```bash
    cd $IDF_PATH
    git submodule update --init --recursive
    ```

2.  **Configure Project**:
    Set the target and open menuconfig.
    ```bash
    idf.py set-target esp32p4
    idf.py menuconfig
    ```
    *   **Project Build Configuration**: Set your Wi-Fi SSID and Password.
    *   **ESP Wi-Fi Remote**: Ensure `CONFIG_ESP_WIFI_REMOTE_ENABLED` is set (it is enabled by default in `sdkconfig.defaults`).

3.  **Build**:
    ```bash
    idf.py build
    ```

4.  **Flash and Monitor**:
    ```bash
    # Replace /dev/ttyACM0 with your P4's USB port
    idf.py -p /dev/ttyACM0 flash monitor
    ```

## Usage
1.  After flashing, the ESP32-P4 will boot and attempt to connect to the configured Wi-Fi network using the external slave.
2.  Watch the serial monitor for the IP address:
    ```
    I (3420) audio_stream: got ip:192.168.1.105
    I (3430) audio_stream: Starting Webserver
    ```
3.  Open a web browser on your computer/phone connected to the same network.
4.  Navigate to: `http://<IP_ADDRESS>/stream`
6.  **Play Audio**: Upload a WAV file to play:
    ```bash
    curl -X POST --data-binary @song.wav http://<IP>/play
    ```
7.  **Set Volume**: Adjust volume (0-100):
    ```bash
    curl -X POST "http://<IP>/volume?val=60"
    ```
