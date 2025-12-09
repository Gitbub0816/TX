/*
 * rx_deadman_esp32.ino - Wireless deadman receiver for ESP32 + E01C-ML01SP (nRF24L01+)
 *
 * Wiring (adjust pins for your board):
 *   RF24 CE  -> PIN_RF_CE (GPIO4 default)
 *   RF24 CSN -> PIN_RF_CSN (GPIO5 default)
 *   RF24 SCK -> PIN_SPI_SCK (GPIO18 default)
 *   RF24 MOSI-> PIN_SPI_MOSI (GPIO23 default)
 *   RF24 MISO-> PIN_SPI_MISO (GPIO19 default)
 *   DEADMAN_OUT drives relay/MOSFET for fueling solenoid (active HIGH)
 *   CONFIG_BUTTON held low during boot enters serial config menu
 *   LEDs on PIN_LED_GREEN / PIN_LED_RED for status, optional buzzer on PIN_BUZZER
 *
 * Behavior summary:
 *   - Listens for DeadmanPacket frames from its paired transmitter.
 *   - Validates PAIR_ID, MAC, and counter freshness before enabling output.
 *   - DEADMAN_OUT is asserted only while fresh, valid packets with triggerPressed=1 arrive within DEADMAN_TIMEOUT_MS.
 *   - On timeout or any validation failure, output is forced LOW and alarms are shown (blink/beep).
 *   - Config mode allows updating PAIR_ID and SECRET_KEY via serial; stored in NVS.
 *
 * Bench test:
 *   - Power TX and RX; ensure CONFIG_BUTTON released for normal mode.
 *   - Watch serial at 115200 for packet logs (enable DEBUG_SERIAL if desired).
 *   - Hold TX trigger: RX LED goes solid green, DEADMAN_OUT goes HIGH.
 *   - Release trigger or remove power from TX: after timeout, DEADMAN_OUT returns LOW and alarm blinks.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <RF24.h>
#include "deadman_common.h"

// ---------------- Hardware configuration ----------------
#define PIN_SPI_MOSI     23
#define PIN_SPI_MISO     19
#define PIN_SPI_SCK      18
#define PIN_RF_CE        4
#define PIN_RF_CSN       5

#define PIN_LED_GREEN    2
#define PIN_LED_RED      25
#define PIN_BUZZER       27
#define DEADMAN_OUT_PIN  15
#define CONFIG_BUTTON    0

// ---------------- Timing and thresholds ----------------
const uint32_t DEADMAN_TIMEOUT_MS = 400;   // Fail-safe timeout without packets
const uint32_t LED_BLINK_SLOW_MS  = 1000;
const uint32_t LED_BLINK_FAST_MS  = 120;
const uint16_t COUNTER_REPLAY_WINDOW = 500; // Reject counters older than this delta
const uint16_t TX_LOW_BATTERY_MV = 3400;    // Alert threshold based on transmitter report

// ---------------- Compile-time debug ----------------
//#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define DBG(...) Serial.printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

Preferences prefs;
SPIClass spiBus(HSPI);
RF24 radio(PIN_RF_CE, PIN_RF_CSN);

uint32_t pairId = DEFAULT_PAIR_ID;
uint32_t secretKey = DEFAULT_SECRET_KEY;
uint16_t lastCounter = 0xFFFF; // 0xFFFF indicates uninitialized
uint32_t lastPacketMs = 0;
uint16_t lastBatteryMv = 0;

// Forward declarations
bool initRadio();
bool loadOrInitKeys();
void handleConfigMode();
void updateOutputs(bool triggerPressed, bool linkFresh, bool lowTxBattery, bool radioError);

void setLed(bool greenOn, bool redOn) {
  digitalWrite(PIN_LED_GREEN, greenOn ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,   redOn ? HIGH : LOW);
}

void beep(uint16_t freq, uint16_t durationMs) {
#ifdef PIN_BUZZER
  if (freq == 0 || durationMs == 0) return;
  tone(PIN_BUZZER, freq, durationMs);
#endif
}

void setup() {
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  DBG("Deadman RX booting...\n");
#endif

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(DEADMAN_OUT_PIN, OUTPUT);
  pinMode(CONFIG_BUTTON, INPUT_PULLUP);
  digitalWrite(DEADMAN_OUT_PIN, LOW);

  if (digitalRead(CONFIG_BUTTON) == LOW) {
    handleConfigMode();
  }

  if (!loadOrInitKeys()) {
    DBG("Failed to init NVS keys\n");
  }

  if (!initRadio()) {
    DBG("Radio init failed\n");
    while (true) {
      setLed(false, true);
      beep(1600, 60);
      delay(200);
      setLed(false, false);
      delay(200);
    }
  }
}

void loop() {
  uint32_t now = millis();
  bool linkFresh = (now - lastPacketMs) <= DEADMAN_TIMEOUT_MS;
  bool lowTxBattery = (lastBatteryMv > 0 && lastBatteryMv < TX_LOW_BATTERY_MV);

  if (radio.available()) {
    DeadmanPacket pkt;
    radio.read(&pkt, sizeof(pkt));

    if (pkt.pairId != pairId) {
      DBG("Pair mismatch\n");
    } else if (!validateMac(pkt, secretKey)) {
      DBG("MAC failed\n");
    } else if (isCounterTooOld(lastCounter, pkt.counter, COUNTER_REPLAY_WINDOW)) {
      DBG("Counter too old prev=%u new=%u\n", lastCounter, pkt.counter);
    } else {
      lastPacketMs = now;
      lastBatteryMv = pkt.batteryMv;
      if ((uint16_t)(pkt.counter - lastCounter) < 0x8000) {
        lastCounter = pkt.counter;
      }
      linkFresh = true;
      lowTxBattery = pkt.batteryMv < TX_LOW_BATTERY_MV;

      bool triggerPressed = pkt.flags & 0x01;
      updateOutputs(triggerPressed, linkFresh, lowTxBattery, false);
      return; // handled this iteration
    }
  }

  // No packet or failed validation: enforce timeout behavior
  updateOutputs(false, linkFresh, lowTxBattery, !linkFresh);
}

bool loadOrInitKeys() {
  if (!prefs.begin("deadman", false)) {
    return false;
  }
  pairId = prefs.getULong("pair", DEFAULT_PAIR_ID);
  secretKey = prefs.getULong("secret", DEFAULT_SECRET_KEY);
  prefs.putULong("pair", pairId);
  prefs.putULong("secret", secretKey);
  prefs.end();
  DBG("PAIR_ID=0x%08X SECRET=0x%08X\n", pairId, secretKey);
  return true;
}

bool initRadio() {
  spiBus.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  if (!radio.begin(&spiBus)) {
    return false;
  }
  radio.setChannel(90);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(5, 15);
  radio.setPayloadSize(sizeof(DeadmanPacket));
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();

  uint8_t address[5] = { (uint8_t)(pairId >> 24), (uint8_t)(pairId >> 16), (uint8_t)(pairId >> 8), (uint8_t)pairId, 0xD3 };
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening();
  return true;
}

void handleConfigMode() {
  Serial.begin(115200);
  Serial.println("\nCONFIG MODE - Update PAIR_ID and SECRET_KEY");
  Serial.println("Press Enter to keep current value.");

  if (!prefs.begin("deadman", false)) {
    Serial.println("NVS open failed; rebooting...");
    delay(500);
    ESP.restart();
  }

  uint32_t currentPair = prefs.getULong("pair", DEFAULT_PAIR_ID);
  uint32_t currentSecret = prefs.getULong("secret", DEFAULT_SECRET_KEY);

  Serial.printf("Current PAIR_ID: 0x%08X\n", currentPair);
  Serial.printf("Enter new PAIR_ID (hex): ");
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() > 0) {
    currentPair = strtoul(input.c_str(), nullptr, 16);
  }

  Serial.printf("Current SECRET_KEY: 0x%08X\n", currentSecret);
  Serial.printf("Enter new SECRET_KEY (hex): ");
  input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() > 0) {
    currentSecret = strtoul(input.c_str(), nullptr, 16);
  }

  prefs.putULong("pair", currentPair);
  prefs.putULong("secret", currentSecret);
  prefs.end();

  Serial.println("Saved. Rebooting...");
  delay(500);
  ESP.restart();
}

void updateOutputs(bool triggerPressed, bool linkFresh, bool lowTxBattery, bool radioError) {
  static uint32_t lastBlink = 0;
  static bool blinkState = false;
  uint32_t now = millis();

  // Fail-safe: default output is OFF unless explicitly enabled
  bool outputOn = triggerPressed && linkFresh && !radioError;
  digitalWrite(DEADMAN_OUT_PIN, outputOn ? HIGH : LOW);

  // LED/Buzzer logic
  if (!linkFresh || radioError) {
    if (now - lastBlink >= LED_BLINK_FAST_MS) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    setLed(false, blinkState);
    if (blinkState) beep(2000, 40);
    return;
  }

  if (lowTxBattery) {
    if (now - lastBlink >= LED_BLINK_SLOW_MS) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    setLed(blinkState, !blinkState); // yellow-ish by alternating
    if (blinkState) beep(1200, 40);
    return;
  }

  if (outputOn) {
    setLed(true, false); // solid green
  } else {
    if (now - lastBlink >= LED_BLINK_SLOW_MS) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    setLed(blinkState, false); // slow blink green when idle but linked
  }
}

