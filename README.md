# ESP32 Deadman Switch Firmware

This repo contains two Arduino sketches for a wireless deadman switch system:

- `tx_deadman_esp32.ino` – transmitter firmware for the handheld deadman handle (ESP32-C3).
- `rx_deadman_esp32.ino` – receiver firmware for the truck/cart module (ESP32 or ESP32-C3).
- `deadman_common.h` – shared packet/authentication definitions used by both sketches.

## How to build

Arduino IDE expects one sketch per folder. To build both sides, create **two folders** and place the shared header alongside each sketch:

1. Create a folder named `tx_deadman_esp32` and copy `tx_deadman_esp32.ino` **and** `deadman_common.h` into it. Open the `.ino` in Arduino IDE/PlatformIO and compile/flash for your ESP32-C3 board.
2. Create a folder named `rx_deadman_esp32` and copy `rx_deadman_esp32.ino` **and** `deadman_common.h` into it. Open the `.ino` and compile/flash for the receiver board.

Keeping `deadman_common.h` beside each `.ino` ensures both sides share the exact packet structure and MAC logic without manual edits. If you prefer a library-style layout, you can place `deadman_common.h` in a reusable library folder that is included in both sketches.

## Libraries

The sketches use the Arduino ESP32 core plus the `RF24` library for the E01C-ML01SP (nRF24L01+ compatible) module. Install `RF24` from the Arduino Library Manager or include it in your PlatformIO `lib_deps`.

## Testing notes

Refer to the README-style comment at the top of each `.ino` for wiring and bench-test steps. Both sketches default to a compiled `PAIR_ID` and `SECRET_KEY`; update them in code or via the receiver's serial config mode before deployment.
