# Waveshare ESP32-P4-NANO Hardware Details

This document collects specific hardware details found during development, particularly regarding the Audio subsystem.

## Audio Subsystem

The board uses an **ES8311** codec connected via I2S and I2C.

### Pinout Configuration

| Signal      | ESP32-P4 GPIO | Description                         |
| :---        | :---          | :---                                |
| **I2C SDA** | GPIO 7        | Codec Control Data                  |
| **I2C SCL** | GPIO 8        | Codec Control Clock                 |
| **I2S MCLK**| GPIO 13       | Master Clock                        |
| **I2S BCLK**| GPIO 12       | Bit Clock                           |
| **I2S WS**  | GPIO 10       | Word Select / LRCK                  |
| **I2S DOUT**| GPIO 9        | Data Out (from ESP to Codec)        |
| **I2S DIN** | GPIO 11       | Data In (from Codec to ESP)         |
| **PA EN**   | GPIO 53       | **Power Amplifier Enable** (Active High)|

> [!IMPORTANT]
> **Power Amplifier (PA)**: GPIO 53 must be driven **HIGH** to enable the onboard audio amplifier. Without this, speaker output may be silent even if the codec is functioning.

> [!NOTE]
> **I2S Pin Naming**: Note that `I2S_DIN` (GPIO 11) is the input to the ESP32 (Microphone data), and `I2S_DOUT` (GPIO 9) is the output from the ESP32 (Speaker data).

## External Wi-Fi

The ESP32-P4 does not have internal Wi-Fi. This board typically mates with an **ESP32-C6-MINI** module.

*    **Interface**: SDIO / SPI
*   **Driver**: `espressif/esp_wifi_remote` (Host) + ESP-Hosted (Slave)

## Power
*   **USB-C**: Powers the board and provides JTAG/Serial interface.
