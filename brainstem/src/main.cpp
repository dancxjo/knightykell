// KnightyKell Brainstem: Pro Micro (ATmega32U4) firmware
// Bridges USB Serial (cerebellum) and Serial1 (iRobot Create 1 OI)

#include <Arduino.h>
#include <ArduinoJson.h>

// ---------- Config ----------
static const uint32_t USB_BAUD = 115200;    // host ↔ brainstem
static const uint32_t OI_BAUD  = 57600;     // Create 1 default

// Watchdog: stop if no drive command within this window (ms)
static const uint32_t DRIVE_WATCHDOG_MS = 500;

// Sensor polling interval (ms)
static const uint32_t SENSOR_POLL_MS = 100;

// ---------- Helpers ----------
inline void oiWrite(uint8_t b) { Serial1.write(b); }

inline void oiCommand(uint8_t opcode) { Serial1.write(opcode); }

inline void oiCommand(uint8_t opcode, const uint8_t* data, size_t len) {
  Serial1.write(opcode);
  if (len) Serial1.write(data, len);
}

inline void oiWriteI16BE(int16_t v) {
  Serial1.write((uint8_t)((v >> 8) & 0xFF));
  Serial1.write((uint8_t)(v & 0xFF));
}

static int16_t clampSpeed(int32_t v, int16_t minV = -500, int16_t maxV = 500) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return (int16_t)v;
}

// ---------- State ----------
static int16_t lastLeft = 0;
static int16_t lastRight = 0;
static uint32_t lastDriveCmdMs = 0;
static uint32_t lastSensorPollMs = 0;

// ---------- OI primitives ----------
void oiStartSafe() {
  oiCommand(128); // START
  delay(20);
  oiCommand(131); // SAFE
}

void oiFull() { oiCommand(132); }

void oiDriveDirect(int16_t left, int16_t right) {
  // OI opcode 145: drive direct. Order is Right, Left (signed, mm/s)
  Serial1.write((uint8_t)145);
  oiWriteI16BE(right);
  oiWriteI16BE(left);
}

void oiStop() { oiDriveDirect(0, 0); }

void oiLeds(bool advance, bool play, uint8_t color, uint8_t intensity) {
  // OI opcode 139: LEDs. bits: | | | | | | adv | play |
  uint8_t leds = 0;
  if (advance) leds |= (1 << 3);
  if (play)    leds |= (1 << 0);
  uint8_t payload[3] = { leds, color, intensity };
  oiCommand(139, payload, sizeof(payload));
}

// ---------- JSON I/O ----------
static const size_t JSON_CAP = 512; // adjust as needed
StaticJsonDocument<JSON_CAP> doc;

void sendJsonAck(const char* cmd, bool ok, const char* msg = nullptr) {
  StaticJsonDocument<128> ack;
  ack["ack"] = cmd;
  ack["ok"] = ok;
  if (msg) ack["msg"] = msg;
  serializeJson(ack, Serial);
  Serial.println();
}

void sendJsonEventBump(uint8_t raw) {
  StaticJsonDocument<128> evt;
  evt["evt"] = "bump";
  evt["raw"] = raw;
  evt["right"] = (raw & 0x01) ? 1 : 0;     // bit0 right bump
  evt["left"] = (raw & 0x02) ? 1 : 0;      // bit1 left bump
  evt["right_drop"] = (raw & 0x04) ? 1 : 0;// bit2 right wheel drop
  evt["left_drop"] = (raw & 0x08) ? 1 : 0; // bit3 left wheel drop
  serializeJson(evt, Serial);
  Serial.println();
}

void sendReady() {
  StaticJsonDocument<128> ready;
  ready["status"] = "ready";
  ready["usb_baud"] = USB_BAUD;
  ready["oi_baud"] = OI_BAUD;
  serializeJson(ready, Serial);
  Serial.println();
}

// ---------- Command handling ----------
void handleCommand(JsonDocument& j) {
  const char* cmd = j["cmd"] | "";

  if (!strcmp(cmd, "start")) {
    oiStartSafe();
    sendJsonAck(cmd, true);
    return;
  }
  if (!strcmp(cmd, "safe")) {
    oiCommand(131);
    sendJsonAck(cmd, true);
    return;
  }
  if (!strcmp(cmd, "full")) {
    oiFull();
    sendJsonAck(cmd, true);
    return;
  }
  if (!strcmp(cmd, "drive_direct")) {
    int32_t l = j["left"].as<int32_t>();
    int32_t r = j["right"].as<int32_t>();
    lastLeft = clampSpeed(l);
    lastRight = clampSpeed(r);
    oiDriveDirect(lastLeft, lastRight);
    lastDriveCmdMs = millis();
    sendJsonAck(cmd, true);
    return;
  }
  if (!strcmp(cmd, "stop")) {
    lastLeft = lastRight = 0;
    oiStop();
    sendJsonAck(cmd, true);
    return;
  }
  if (!strcmp(cmd, "leds")) {
    bool adv = j["advance"].as<bool>();
    bool ply = j["play"].as<bool>();
    uint8_t color = j["color"].as<uint8_t>();
    uint8_t intensity = j["intensity"].as<uint8_t>();
    oiLeds(adv, ply, color, intensity);
    sendJsonAck(cmd, true);
    return;
  }
  if (!strcmp(cmd, "sensors")) {
    uint8_t packet = j["packet"].as<uint8_t>();
    uint8_t payload[1] = { packet };
    oiCommand(142, payload, 1);
    sendJsonAck(cmd, true);
    return;
  }

  sendJsonAck(cmd, false, "unknown_cmd");
}

// ---------- Line buffer ----------
static const size_t LINE_MAX = 256;
static char lineBuf[LINE_MAX];
static size_t lineLen = 0;

void processSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      // parse lineBuf[0..lineLen)
      DeserializationError err = deserializeJson(doc, lineBuf, lineLen);
      if (!err) {
        handleCommand(doc);
      } else {
        sendJsonAck("parse", false, err.c_str());
      }
      lineLen = 0;
      continue;
    }
    if (lineLen + 1 < LINE_MAX) {
      lineBuf[lineLen++] = c;
    } else {
      // overflow -> reset
      lineLen = 0;
      sendJsonAck("parse", false, "line_too_long");
    }
  }
}

// ---------- Sensors ----------
void pollSensors() {
  uint32_t now = millis();
  if (now - lastSensorPollMs < SENSOR_POLL_MS) return;
  lastSensorPollMs = now;

  // Request packet 7 (Bumps and Wheel Drops, 1 byte)
  uint8_t pkt = 7;
  oiCommand(142, &pkt, 1);

  // Non-blocking read: allow a brief window for the byte to arrive
  uint32_t t0 = millis();
  while ((millis() - t0) < 5) {
    if (Serial1.available() >= 1) {
      uint8_t raw = (uint8_t)Serial1.read();
      sendJsonEventBump(raw);
      break;
    }
  }
}

// ---------- Safety ----------
void driveWatchdog() {
  uint32_t now = millis();
  if ((lastLeft != 0 || lastRight != 0) && (now - lastDriveCmdMs) > DRIVE_WATCHDOG_MS) {
    lastLeft = lastRight = 0;
    oiStop();
    sendJsonAck("watchdog", true, "stopped");
  }
}

// ---------- Arduino entry ----------
void setup() {
  // USB serial for host
  Serial.begin(USB_BAUD);
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 3000) {
    ; // wait up to 3s for USB
  }

  // Create OI UART
  Serial1.begin(OI_BAUD);
  delay(50);

  oiStartSafe();
  lastDriveCmdMs = millis();

  sendReady();
}

void loop() {
  processSerialInput();
  pollSensors();
  driveWatchdog();
}

