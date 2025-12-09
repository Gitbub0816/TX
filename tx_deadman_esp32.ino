/*
 * tx_deadman_esp32.ino - Wireless deadman transmitter for ESP32-C3 + E01C-ML01SP (nRF24L01+)
 *
 * Wiring (adjust pins if using a different board):
 *   RF24 CE  -> PIN_RF_CE (GPIO8 default)
 *   RF24 CSN -> PIN_RF_CSN (GPIO7 default)
 *   RF24 SCK -> PIN_SPI_SCK (GPIO6 default)
 *   RF24 MOSI-> PIN_SPI_MOSI (GPIO5 default)
 *   RF24 MISO-> PIN_SPI_MISO (GPIO4 default)
 *   Deadman switch (NO) between SWITCH_PIN and GND (internal pull-up enabled)
 *   Battery sense to PIN_BATTERY_ADC via divider (to stay within 0-3.3V)
 *   LEDs on PIN_LED_GREEN / PIN_LED_RED (or use a single RGB LED with resistors)
 *   Optional buzzer on PIN_BUZZER
 *
 * Behavior summary:
 *   - Continuously sends authenticated DeadmanPacket frames when the trigger is held (~20 Hz).
 *   - Sends slower keep-alive packets when released (~5 Hz) so the receiver can differentiate idle vs. lost link.
 *   - Sleeps after 30 seconds of inactivity; wakes on switch press via GPIO wakeup.
 *   - LED patterns: solid green when linked + pressed; slow green blink when linked idle; fast red blink on low battery;
 *     distinctive red error blink when radio/ACK failures occur.
 *   - All safety failures (radio init, missing ACKs, bad MAC) are treated as "fueling OFF" on the receiver side.
 *
 * Bench test:
 *   - Power TX and RX, open serial at 115200 for debug.
 *   - Hold the trigger: RX should energize output and both devices show linked indication.
 *   - Release trigger: RX de-energizes; TX eventually sleeps after 30s.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <RF24.h>
#include "deadman_common.h"
#include "esp_sleep.h"

// ---------------- Hardware configuration (adjust for your board) ----------------
// ESP32-C3 default SPI pins for the Seeed Studio board
#define PIN_SPI_MOSI    5
#define PIN_SPI_MISO    4
#define PIN_SPI_SCK     6
#define PIN_RF_CE       8
#define PIN_RF_CSN      7

#define PIN_LED_GREEN   2
#define PIN_LED_RED     3
#define PIN_BUZZER      10
#define SWITCH_PIN      9     // Deadman trigger input (NO to GND)
#define PIN_BATTERY_ADC 0     // Analog pin connected to battery divider

// ---------------- Timing and thresholds ----------------
const uint32_t SEND_INTERVAL_PRESSED_MS   = 50;   // ~20 Hz when trigger held
const uint32_t SEND_INTERVAL_RELEASED_MS  = 200;  // ~5 Hz when idle
const uint32_t SLEEP_AFTER_RELEASE_MS     = 30000; // Enter deep sleep after 30s idle
const uint32_t LED_BLINK_SLOW_MS          = 1000;
const uint32_t LED_BLINK_FAST_MS          = 150;
const uint16_t LOW_BATTERY_MV             = 3400;  // Low battery threshold
const uint8_t  MAX_TX_RETRIES             = 5;     // Consecutive failed packets before showing error

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

struct LinkState {
  uint32_t lastSendMs = 0;
  uint32_t lastAckMs = 0;
  uint32_t lastReleaseMs = 0;
  uint16_t counter = 0;
  uint8_t failedTx = 0;
  bool lastSwitchState = false;
};

LinkState link;
uint32_t pairId = DEFAULT_PAIR_ID;
uint32_t secretKey = DEFAULT_SECRET_KEY;

// Forward declarations
void updateLeds(bool triggerPressed, bool acked, bool lowBattery, bool radioError);
uint16_t readBatteryMv();
void enterDeepSleep();
bool initRadio();
bool loadOrInitKeys();
bool sendPacket(bool triggerPressed, uint16_t batteryMv);

// Simple LED/buzzer helper for non-blocking blinking
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
  DBG("Deadman TX booting...\n");
#endif

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(PIN_BATTERY_ADC, INPUT);

  if (!loadOrInitKeys()) {
    DBG("Failed to init NVS keys\n");
  }

  if (!initRadio()) {
    DBG("Radio init failed\n");
    // Blink error pattern indefinitely; deep sleep avoided so issue is visible.
    while (true) {
      setLed(false, true);
      beep(1800, 80);
      delay(150);
      setLed(false, false);
      delay(150);
    }
  }

  link.lastReleaseMs = millis();
}

void loop() {
  uint32_t now = millis();
  bool triggerPressed = digitalRead(SWITCH_PIN) == LOW; // Active low with pull-up

  // Debounce: simple stable check every loop
  static bool lastSample = triggerPressed;
  static uint32_t lastDebounceMs = now;
  if (triggerPressed != lastSample) {
    lastDebounceMs = now;
    lastSample = triggerPressed;
  }
  if ((now - lastDebounceMs) < 20) {
    triggerPressed = link.lastSwitchState; // keep previous stable
  } else {
    link.lastSwitchState = triggerPressed;
  }

  if (!triggerPressed) {
    link.lastReleaseMs = now;
  }

  // Enter deep sleep if idle long enough
  if (!triggerPressed && (now - link.lastReleaseMs) > SLEEP_AFTER_RELEASE_MS) {
    enterDeepSleep();
  }

  uint32_t sendInterval = triggerPressed ? SEND_INTERVAL_PRESSED_MS : SEND_INTERVAL_RELEASED_MS;
  if ((now - link.lastSendMs) >= sendInterval) {
    link.lastSendMs = now;
    uint16_t batt = readBatteryMv();
    bool acked = sendPacket(triggerPressed, batt);
    if (acked) {
      link.lastAckMs = now;
      link.failedTx = 0;
    } else {
      if (link.failedTx < 255) link.failedTx++;
    }
    bool lowBatt = batt < LOW_BATTERY_MV;
    bool radioErr = link.failedTx >= MAX_TX_RETRIES;
    updateLeds(triggerPressed, acked, lowBatt, radioErr);
  }
}

bool loadOrInitKeys() {
  if (!prefs.begin("deadman", false)) {
    return false;
  }
  pairId = prefs.getULong("pair", DEFAULT_PAIR_ID);
  secretKey = prefs.getULong("secret", DEFAULT_SECRET_KEY);
  // Ensure defaults are present if keys were missing
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
  radio.setChannel(90); // 2.490 GHz away from Wi-Fi overlap
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(5, 15); // delay * 250us, count
  radio.setPayloadSize(sizeof(DeadmanPacket));
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();

  // Derive address from pairId to reduce cross-talk between multiple systems
  uint8_t address[5] = { (uint8_t)(pairId >> 24), (uint8_t)(pairId >> 16), (uint8_t)(pairId >> 8), (uint8_t)pairId, 0xD3 };
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.stopListening();
  return true;
}

uint16_t readBatteryMv() {
  // Calibrated scale depends on resistor divider ratio; assume 2:1 divider to read up to 8.4V.
  const float dividerRatio = 2.0f; // Vbattery = ADC * dividerRatio
  const float adcMaxMv = 3300.0f;  // ESP32 ADC reference ~3.3V
  uint16_t raw = analogRead(PIN_BATTERY_ADC);
  // ESP32 ADC default 12-bit
  float mv = (raw / 4095.0f) * adcMaxMv * dividerRatio;
  return (uint16_t)mv;
}

bool sendPacket(bool triggerPressed, uint16_t batteryMv) {
  DeadmanPacket pkt = {};
  pkt.pairId = pairId;
  pkt.counter = ++link.counter;
  pkt.flags = triggerPressed ? 0x01 : 0x00;
  pkt.batteryMv = batteryMv;
  pkt.mac = computeMac(pkt, secretKey);

  radio.stopListening();
  bool ok = radio.write(&pkt, sizeof(pkt));
  DBG("TX cnt=%u trig=%d batt=%umV ack=%d\n", pkt.counter, triggerPressed, batteryMv, ok);
  radio.startListening(); // Keep listening for ACK payloads if used in future
  return ok;
}

void updateLeds(bool triggerPressed, bool acked, bool lowBattery, bool radioError) {
  static uint32_t lastBlink = 0;
  static bool blinkState = false;
  uint32_t now = millis();

  if (radioError) {
    if (now - lastBlink >= LED_BLINK_FAST_MS) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    setLed(false, blinkState);
    if (blinkState) beep(1800, 50);
    return;
  }

  if (lowBattery) {
    if (now - lastBlink >= LED_BLINK_FAST_MS) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    setLed(false, blinkState);
    if (blinkState) beep(1200, 40);
    return;
  }

  if (triggerPressed && acked) {
    setLed(true, false); // solid green
    return;
  }

  // Linked idle: slow blink green if recent ACKs, else both off
  bool linkFresh = (now - link.lastAckMs) < 2000;
  if (linkFresh) {
    if (now - lastBlink >= LED_BLINK_SLOW_MS) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    setLed(blinkState, false);
  } else {
    setLed(false, false);
  }
}

void enterDeepSleep() {
  DBG("Entering deep sleep after inactivity\n");
  setLed(false, false);
  beep(1000, 40);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SWITCH_PIN, 0); // Wake when switch pulls low
  delay(50);
  esp_deep_sleep_start();
}

