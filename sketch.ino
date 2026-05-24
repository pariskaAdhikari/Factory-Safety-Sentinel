// Factory Safety Sentinel
// Group: Nawid Hassan, Pariska Adhikari, Crystal Quach
// Option A - Embedded Intelligence (Wokwi)

#include "esp_task_wdt.h"

// Pins
const int GAS_PIN    = 34;
const int TRIG_PIN   = 5;
const int ECHO_PIN   = 18;
const int BUZZER     = 25;
const int LED_GREEN  = 26;
const int LED_YELLOW = 27;
const int LED_RED    = 14;
const int LED_PURPLE   = 32;
const int RESET_BTN  = 0;

// States
enum State { SAFE, CAUTION, DANGER, FAILSAFE };
State currentState = SAFE;

// Gas thresholds (using two values so it doesn't flicker between states)
const int GAS_CAUTION_HIGH = 2500;
const int GAS_CAUTION_LOW  = 2200;
const int GAS_DANGER_HIGH  = 3500;
const int GAS_DANGER_LOW   = 3200;
const int GAS_FUSION       = 2000;   // mild gas + person nearby = caution
const int NEAR_CM          = 100;    // person counted as "near" if closer than this

// Polling speeds - we check faster when things get worse
const int POLL_SAFE    = 500;
const int POLL_CAUTION = 200;
const int POLL_DANGER  = 100;
const int STABLE_TIME  = 3000;       // wait 3 seconds before downgrading state

// Timing variables (these change while the program runs)
unsigned long lastPoll = 0;
unsigned long stableSince = 0;
unsigned long lastBeep = 0;
bool buzzerOn = false;

// Sensor health
int badReadings = 0;
const int MAX_BAD = 5;
int bootGrace = 15;                  // ignore the first few sensor reads on startup
bool resetPressed = false;


void setup() {
  Serial.begin(115200);
  Serial.println("Factory Safety Sentinel starting...");

  // Hardware watchdog - resets the chip if our code freezes
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 3000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_PURPLE, OUTPUT);
  pinMode(RESET_BTN, INPUT_PULLUP);

  // Start in SAFE state - turn on green LED
  digitalWrite(LED_GREEN, HIGH);
}


void loop() {
  esp_task_wdt_reset();   // tell the watchdog we're still alive

  // Check if user pressed the reset button (BOOT button on ESP32)
  if (digitalRead(RESET_BTN) == LOW) {
    resetPressed = true;
  }

  // Only sense and think every X milliseconds (depends on state)
  int pollSpeed = getPollSpeed();
  if (millis() - lastPoll >= pollSpeed) {
    lastPoll = millis();

    // SENSE
    int gas = analogRead(GAS_PIN);
    float distance = readDistance();
    bool sensorsOk = checkSensors(gas, distance);

    // THINK - figure out what state we should be in
    State newState = decideState(gas, distance, sensorsOk);

    // ACT - if state changed, update LEDs
    if (newState != currentState) {
      changeState(newState);
    }

    // Print status to serial monitor
    Serial.print("[");
    Serial.print(stateName(currentState));
    Serial.print("] gas=");
    Serial.print(gas);
    Serial.print(" dist=");
    Serial.print(distance);
    Serial.println("cm");
  }

  // Buzzer pattern runs every loop so beeping is smooth
  updateBuzzer();
}


// Read distance from HC-SR04 ultrasonic sensor
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;       // sensor didn't respond
  return duration * 0.0343 / 2;       // convert time to cm
}


// Check if sensor readings make sense
bool checkSensors(int gas, float distance) {
  // Give sensors time to warm up at startup
  if (bootGrace > 0) {
    bootGrace--;
    badReadings = 0;
    return true;
  }

  bool gasOk = (gas >= 0 && gas <= 4095);
  bool distOk = (distance > 0 && distance <= 450);

  if (gasOk && distOk) {
    badReadings = 0;
    return true;
  }

  // Don't trip failsafe on a single bad reading - need 5 in a row
  badReadings++;
  return (badReadings < MAX_BAD);
}


// The brain - decides which state we should be in
State decideState(int gas, float distance, bool sensorsOk) {
  // While booting, just stay safe
  if (bootGrace > 0) return SAFE;

  // If sensors are broken, go to failsafe (highest priority)
  if (!sensorsOk) return FAILSAFE;

  switch (currentState) {

    case SAFE:
      // Sensor fusion: trigger caution if gas is high alone,
      // OR if gas is mild AND a person is near
      if (gas > GAS_CAUTION_HIGH) return CAUTION;
      if (gas > GAS_FUSION && distance < NEAR_CM) return CAUTION;
      return SAFE;

    case CAUTION:
      if (gas > GAS_DANGER_HIGH) return DANGER;
      // Only drop back to SAFE after gas has been low for 3 seconds
      if (gas < GAS_CAUTION_LOW) {
        if (stableSince == 0) stableSince = millis();
        if (millis() - stableSince >= STABLE_TIME) {
          stableSince = 0;
          return SAFE;
        }
      } else {
        stableSince = 0;
      }
      return CAUTION;

    case DANGER:
      // Same idea - need 3 seconds of safer readings before downgrading
      if (gas < GAS_DANGER_LOW) {
        if (stableSince == 0) stableSince = millis();
        if (millis() - stableSince >= STABLE_TIME) {
          stableSince = 0;
          return CAUTION;
        }
      } else {
        stableSince = 0;
      }
      return DANGER;

    case FAILSAFE:
      // Operator must press BOOT button to confirm hazard is cleared
      if (resetPressed && badReadings == 0) {
        resetPressed = false;
        Serial.println("Reset accepted - back to SAFE");
        return SAFE;
      }
      resetPressed = false;
      return FAILSAFE;
  }

  return currentState;
}


// Turn LEDs on/off when state changes
void changeState(State newState) {
  Serial.print("STATE: ");
  Serial.print(stateName(currentState));
  Serial.print(" -> ");
  Serial.println(stateName(newState));

  currentState = newState;
  stableSince = 0;

  // Turn all LEDs off first
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_PURPLE, LOW);
  digitalWrite(BUZZER, LOW);

  // Turn on the right LED for the new state
  if (newState == SAFE)     digitalWrite(LED_GREEN, HIGH);
  if (newState == CAUTION)  digitalWrite(LED_YELLOW, HIGH);
  if (newState == DANGER)   digitalWrite(LED_RED, HIGH);
  if (newState == FAILSAFE) digitalWrite(LED_PURPLE, HIGH);
}


// Returns how often we should check sensors based on current state
int getPollSpeed() {
  if (currentState == SAFE)    return POLL_SAFE;
  if (currentState == CAUTION) return POLL_CAUTION;
  return POLL_DANGER;   // DANGER and FAILSAFE both poll fast
}


// Buzzer behaviour - beeps in DANGER, solid in FAILSAFE, silent otherwise
// Buzzer behaviour - beeps in DANGER, solid in FAILSAFE, silent otherwise
void updateBuzzer() {
  if (currentState == DANGER) {
    // Beep on/off every 250ms
    if (millis() - lastBeep >= 250) {
      buzzerOn = !buzzerOn;
      if (buzzerOn) {
        tone(BUZZER, 1000);   // play 1000Hz tone
      } else {
        noTone(BUZZER);       // silence
      }
      lastBeep = millis();
    }
  } else if (currentState == FAILSAFE) {
    tone(BUZZER, 1000);       // solid tone, no toggling
  } else {
    noTone(BUZZER);           // silent in SAFE and CAUTION
    buzzerOn = false;
  }
}



// Helper to print state names
const char* stateName(State s) {
  if (s == SAFE)     return "SAFE";
  if (s == CAUTION)  return "CAUTION";
  if (s == DANGER)   return "DANGER";
  if (s == FAILSAFE) return "FAILSAFE";
  return "?";
}