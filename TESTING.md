# Firmware build verification status

The ESP32 firmware sketches in this repository target the Arduino framework. In this execution environment the Arduino/PlatformIO toolchains and the required RF24 library are not installed, so a full compile test could not be executed. To verify locally:

1. Install the Arduino IDE (with the ESP32 board support package) or PlatformIO.
2. Add the RF24 library by TMRh20.
3. Place `deadman_common.h` alongside each sketch in its own folder.
4. Compile `tx_deadman_esp32.ino` for an ESP32-C3 target and `rx_deadman_esp32.ino` for the chosen ESP32 board.

If toolchain access is added here later, re-run compilation using `arduino-cli compile` or a PlatformIO project to confirm there are no syntax errors.
