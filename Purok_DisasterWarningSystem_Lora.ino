/*
  Disaster Warning System — Purok Node (Merged LoRa + Siren Controller)
  - Preserves main siren engine, amplifier wake, and LCD (20x4)
  - Adds two-way LoRa: sends local events to BARANGAY_HALL and responds to remote commands
*/

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

// =============== CONFIG ===============
#define DEVICE_ADDRESS "PUROK1"        // change per device
#define CENTRAL_ADDRESS "BARANGAY_HALL"
#define LORA_BAND 915E6                // set region as needed (433E6/868E6/915E6)

// If your LoRa module uses non-default pins, uncomment and set accordingly
// #define LORA_SS   18
// #define LORA_RST  14
// #define LORA_DIO0 26

LiquidCrystal_I2C lcd(0x27, 16, 2);   // 20x4 LCD

// ======= TONE PROFILE STRUCT =======
struct ToneProfile {
  int lowHz;               // Starting frequency
  int highHz;              // Peak frequency
  int sweepDurationMs;     // Time (ms) to sweep up or down
  int holdHighMs;          // Hold time at high frequency
  int holdLowMs;           // Hold time at low frequency
};

// ======= TONE PROFILES (from base) =======
ToneProfile fireTone = { 500, 1500, 3000, 500, 500 };
ToneProfile floodTone = { 800, 1400, 1000, 200, 200 };
ToneProfile earthquakeTone = { 500, 600, 200, 50, 50 };

ToneProfile tones[] = { fireTone, floodTone, earthquakeTone };

const String modeLabels[] = { "FIRE", "FLOOD", "EARTHQUAKE" };

// Rotary switch durations (ms)
const unsigned long durations[] = {15000, 30000, 60000, 90000, 120000, 180000};
const String durationLabels[] = {"15sec", "30sec", "1min", "1:30min", "2min", "3min"};

// ======= PINS (keep from base file) =======
const int pinWail     = 3;   // FIRE
const int pinYelp     = 4;   // FLOOD
const int pinHiLo     = 5;   // EARTHQUAKE
const int pinMute     = 6;

const int pinDuration1 = A0;
const int pinDuration2 = A1;
const int pinDuration3 = A2;
const int pinDuration4 = A3;
const int pinDuration5 = A4;
const int pinDuration6 = A5;

const int pinBuzzer = 7;

// ======= SYSTEM VARIABLES =======
bool isPlaying = false;
bool isMuted = false;
unsigned long stopTime = 0;
unsigned long lastToneChange = 0;
unsigned long holdStartTime = 0;

int currentHz = 0;
int toneDirection = 1; // 1 = up, -1 = down, 0 = hold
int currentModeIndex = -1;
bool holding = false;
bool atHigh = false;
bool atLow = false;

// ======= LCD CONTROL =======
unsigned long lastDisplayUpdate = 0;
bool displayNeedsUpdate = true;

// Amplifier wake management
unsigned long lastAmpWakeTime = 0;
const unsigned long ampWakeInterval = 86400000UL; // 24 hours? (original file used 1 hour comment mismatch). Keep as 86400000UL = 24hr. Change if needed.

// LoRa status
bool loRaEnabled = false;

// ======= SETUP =======
void setup() {
  pinMode(pinWail,    INPUT_PULLUP);
  pinMode(pinYelp,    INPUT_PULLUP);
  pinMode(pinHiLo,    INPUT_PULLUP);
  pinMode(pinMute,    INPUT_PULLUP);

  pinMode(pinDuration1, INPUT_PULLUP);
  pinMode(pinDuration2, INPUT_PULLUP);
  pinMode(pinDuration3, INPUT_PULLUP);
  pinMode(pinDuration4, INPUT_PULLUP);
  pinMode(pinDuration5, INPUT_PULLUP);
  pinMode(pinDuration6, INPUT_PULLUP);

  pinMode(pinBuzzer, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.begin(115200);

  // Initialize LoRa
  Serial.println("Initializing LoRa...");
  // If using custom pins uncomment and set them before LoRa.begin()
  // LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (LoRa.begin(LORA_BAND)) {
    LoRa.setSyncWord(0xF3);
    loRaEnabled = true;
    Serial.println("LoRa OK");
    lcd.setCursor(0,0);
    lcd.print("LoRa Ready");
    delay(800);
    lcd.clear();
  } else {
    loRaEnabled = false;
    Serial.println("LoRa init failed - continuing without LoRa.");
    lcd.setCursor(0,0);
    lcd.print("LoRa Init Fail");
    delay(1000);
    lcd.clear();
  }

}

// ======= MAIN LOOP =======
void loop() {
  // ---------- Receive check (LoRa) ----------
  if (loRaEnabled) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String msg = "";
      while (LoRa.available()) msg += (char)LoRa.read();
      handleLoRaMessage(msg);
    }
  }

  handleButtons();

  if (isPlaying) {
    updateSiren();
    displayState();
    displayNeedsUpdate = true;
  } else if (!isMuted && displayNeedsUpdate) {
    displaySelection();
    displayNeedsUpdate = false;
  }

  static int lastDurationIndex = -1;
  int currentDurationIndex = readDurationIndex();
  if (currentDurationIndex != lastDurationIndex) {
    displayNeedsUpdate = true;
    lastDurationIndex = currentDurationIndex;
  }

  // ===== PERIODIC AMPLIFIER WAKE PULSE =====
  if (!isPlaying && !isMuted) {
    unsigned long now = millis();
    if (now - lastAmpWakeTime >= ampWakeInterval) {
      wakeAmplifier();           // Send short inaudible tone
      lastAmpWakeTime = now;     // Reset timer
      Serial.println("Amplifier keep-alive tone sent.");
    }
  }
}

// ======= BUTTON HANDLER (merged with LoRa send) =======
void handleButtons() {
  if (digitalRead(pinMute) == LOW) {
    if (!isMuted) {
      noTone(pinBuzzer);
      isMuted = true;
      isPlaying = false;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MUTED");
      delay(300);
      displayNeedsUpdate = true;

      // Notify central about mute
      if (loRaEnabled) sendLoRaEvent("MUTE");
    }
    return;
  } else {
    isMuted = false;
  }

  if (!isPlaying) {
    if (digitalRead(pinWail) == LOW) {
      if (loRaEnabled) sendLoRaEvent("FIRE");
      startSiren(0);
    }
    else if (digitalRead(pinYelp) == LOW) {
      if (loRaEnabled) sendLoRaEvent("FLOOD");
      startSiren(1);
    }
    else if (digitalRead(pinHiLo) == LOW) {
      if (loRaEnabled) sendLoRaEvent("EARTHQUAKE");
      startSiren(2);
    }
  }
  return;
}

// ======= START SIREN (local) =======
void startSiren(int modeIndex) {
  isPlaying = true;
  isMuted = false;
  currentModeIndex = modeIndex;

  ToneProfile& tp = tones[currentModeIndex];
  currentHz = tp.lowHz;
  toneDirection = 1; // start going up
  holding = false;
  atHigh = false;
  atLow = true;

  stopTime = millis() + readDuration();
  lastToneChange = millis();
  holdStartTime = 0;
}

// ======= START REMOTE SIREN (called by handleLoRaMessage) =======
void startRemoteSiren(int mode, unsigned long durMs) {
  isPlaying = true;
  isMuted = false;
  currentModeIndex = mode;

  ToneProfile& tp = tones[currentModeIndex];
  currentHz = tp.lowHz;
  toneDirection = 1;
  holding = false;
  atHigh = false;
  atLow = true;

  stopTime = millis() + durMs;
  lastToneChange = millis();
  holdStartTime = 0;
}

// ======= UPDATE SIREN =======
void updateSiren() {
  unsigned long now = millis();
  if (now >= stopTime) {
    isPlaying = false;
    noTone(pinBuzzer);
    return;
  }

  ToneProfile& tp = tones[currentModeIndex];
  float hzRange = tp.highHz - tp.lowHz;
  float freqChangePerMs = hzRange / tp.sweepDurationMs;

  if (!holding) {
    // Sweep logic (up or down)
    if (now - lastToneChange >= 10) {
      int step = round(freqChangePerMs * 10);
      currentHz += step * toneDirection;
      tone(pinBuzzer, currentHz);
      lastToneChange = now;

      // Reached high
      if (toneDirection == 1 && currentHz >= tp.highHz) {
        currentHz = tp.highHz;
        holding = true;
        atHigh = true;
        atLow = false;
        holdStartTime = now;
      }
      // Reached low
      else if (toneDirection == -1 && currentHz <= tp.lowHz) {
        currentHz = tp.lowHz;
        holding = true;
        atHigh = false;
        atLow = true;
        holdStartTime = now;
      }
    }
  } else {
    // Handle hold timing
    if (atHigh && now - holdStartTime >= tp.holdHighMs) {
      toneDirection = -1;
      holding = false;
    } else if (atLow && now - holdStartTime >= tp.holdLowMs) {
      toneDirection = 1;
      holding = false;
    }
  }
}

// ======= DURATION READING =======
unsigned long readDuration() {
  if (digitalRead(pinDuration1) == LOW) return durations[0];
  if (digitalRead(pinDuration2) == LOW) return durations[1];
  if (digitalRead(pinDuration3) == LOW) return durations[2];
  if (digitalRead(pinDuration4) == LOW) return durations[3];
  if (digitalRead(pinDuration5) == LOW) return durations[4];
  if (digitalRead(pinDuration6) == LOW) return durations[5];
  return durations[0];
}

int readDurationIndex() {
  if (digitalRead(pinDuration1) == LOW) return 0;
  if (digitalRead(pinDuration2) == LOW) return 1;
  if (digitalRead(pinDuration3) == LOW) return 2;
  if (digitalRead(pinDuration4) == LOW) return 3;
  if (digitalRead(pinDuration5) == LOW) return 4;
  if (digitalRead(pinDuration6) == LOW) return 5;
  return 0;
}

// ======= DISPLAY =======
void displaySelection() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SIREN READY");
  lcd.setCursor(0, 1);
  lcd.print("Duration:");
  lcd.print(durationLabels[readDurationIndex()]);
}

void displayState() {
  unsigned long now = millis();
  if (now - lastDisplayUpdate >= 1000) {
    unsigned long remaining = stopTime > now ? stopTime - now : 0;
    unsigned long remainingSeconds = remaining / 1000;
    unsigned long minutes = remainingSeconds / 60;
    unsigned long seconds = remainingSeconds % 60;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(modeLabels[currentModeIndex]);
    lcd.print(" ");
    lcd.print(durationLabels[readDurationIndex()]);

    lcd.setCursor(0, 1);
    char buffer[10];
    sprintf(buffer, "%02lu:%02lu", minutes, seconds);
    lcd.print("Time: ");
    lcd.print(buffer);

    lastDisplayUpdate = now;
  }
}

// ======= AMPLIFIER WAKE =======
void wakeAmplifier() {
  const int wakeFreq = 10;   // 10Hz (mostly inaudible)
  const int wakeDuration = 200; // 200 ms pulse
  tone(pinBuzzer, wakeFreq);
  delay(wakeDuration);
  noTone(pinBuzzer);
  delay(100); // let amp stabilize
}

// ======= LoRa: send event =======
void sendLoRaEvent(String event) {
  if (!loRaEnabled) return;
  String payload = String(DEVICE_ADDRESS) + "|" + CENTRAL_ADDRESS + "|" + event + "|" + durationLabels[readDurationIndex()];
  Serial.print("TX -> "); Serial.println(payload);
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
}

// ======= LoRa: handle incoming messages =======
void handleLoRaMessage(String msg) {
  Serial.print("RX <- "); Serial.println(msg);
  // Expected format: FROM|TO|EVENT|DURATION
  int i1 = msg.indexOf('|');
  int i2 = msg.indexOf('|', i1 + 1);
  int i3 = msg.indexOf('|', i2 + 1);
  if (i1 < 0 || i2 < 0 || i3 < 0) return;

  String from = msg.substring(0, i1);
  String to = msg.substring(i1 + 1, i2);
  String event = msg.substring(i2 + 1, i3);
  String durStr = msg.substring(i3 + 1);

  // Only accept messages coming from CENTRAL_ADDRESS
  if (from != CENTRAL_ADDRESS) return;
  // Check if message addressed to this device or broadcast to ALL
  if (to != DEVICE_ADDRESS && to != "ALL") return;

  // Determine duration index from durStr
  int durIndex = 0;
  for (int i = 0; i < 6; i++) {
    if (durStr == durationLabels[i]) { durIndex = i; break; }
  }
  unsigned long durMs = durations[durIndex];

  // Map event to mode
  int mode = -1;
  if (event == "FIRE") mode = 0;
  else if (event == "FLOOD") mode = 1;
  else if (event == "EARTQUAKE" || event == "EARTHQUAKE") mode = 2;
  else if (event == "MUTE") {
    // Central commanded a mute for this node
    isMuted = true;
    isPlaying = false;
    noTone(pinBuzzer);
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Remote MUTE");
    displayNeedsUpdate = true;
    return;
  } else return; // unknown event

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Remote Cmd: ");
  lcd.print(event);
  lcd.setCursor(0, 1);
  lcd.print("Dur: ");
  lcd.print(durStr);

  // Start remote siren
  startRemoteSiren(mode, durMs);
  displayNeedsUpdate = true;
}