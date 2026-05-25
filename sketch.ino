// Factory Safety Sentinel
// Programming for Robotics Final Project

// Group members: Nawid Hassan, Pariska Adhikari, Crystal Quach
// Specialisation: Option A - Embedded Intelligence (Wokwi)

// Description: Monitors hazardous gas levels and nearby workers 
// using sensor fusion. The system uses a Finite State Machine (FSM)
// to transition between SAFE, CAUTION, DANGER and FAILSAFE states 
// and activates visual and audible warnings when risks are detected.
#include "esp_task_wdt.h"

// Hardware pin assignments for sensors and outputs
const int GAS_PIN    = 34;
const int TRIG_PIN   = 5;
const int ECHO_PIN   = 18;
const int BUZZER     = 25;
const int LED_GREEN  = 26;
const int LED_YELLOW = 27;
const int LED_RED    = 14;
const int LED_PURPLE   = 32;

// Finite State Machine(FSM) States used by the system
enum State { SAFE, CAUTION, DANGER, FAILSAFE };
State currentState = SAFE;

// Gas thresholds (Seperate two values so it doesn't flicker between states)
const int GAS_CAUTION_HIGH = 2500;
const int GAS_CAUTION_LOW  = 2200;
const int GAS_DANGER_HIGH  = 3500;
const int GAS_DANGER_LOW   = 3200;
const int GAS_FUSION       = 2300;   // mild gas + person nearby = caution
const int NEAR_CM          = 100;    // person counted as "near" if closer than this

// Adaptive monitoring rates: The system increases sensor polling
// frequency as risk levels increase to improve reponse time.
const int POLL_SAFE    = 500;
const int POLL_CAUTION = 200;
const int POLL_DANGER  = 100;
const int STABLE_TIME  = 3000;    // wait 3 seconds before downgrading state

// Variables used for timing state transitions
// sensor polling and buzzer control
unsigned long lastPoll = 0;
unsigned long stableSince = 0;
unsigned long lastBeep = 0;
bool buzzerOn = false;

// Sensor validation and Failsafe monitoring variables
int badReadings = 0;
const int MAX_BAD = 5;
//Ignore the first few readings while sensors stabilise after startup
//This prevents false arams caused by warmup noise or invalid initial measurements.
int bootGrace = 15; 
//(this explains why when you start the program is says safe at a high gas reading)


//System Initialisation
// Start serial communication
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

// configure sensor and output pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_PURPLE, OUTPUT);
// Begin in SAFE state
  digitalWrite(LED_GREEN, HIGH);
}

// Main control loop implementing (Sense-Think-Act) Architecture
void loop() {
  esp_task_wdt_reset();

  int pollSpeed = getPollSpeed();
  if (millis() - lastPoll >= pollSpeed) {
    lastPoll = millis();

    // SENSE: Read environmental data from sensors
    int gas = analogRead(GAS_PIN);
    float distance = readDistance();
    bool sensorsOk = checkSensors(gas, distance);

    // THINK: Determine the appropriate FSM state
    State newState = decideState(gas, distance, sensorsOk);

    // ACT: Update outputs if the FSM state changes
    if (newState != currentState) {
      changeState(newState);
    }

    Serial.print("[");
    Serial.print(stateName(currentState));
    Serial.print("] gas=");
    Serial.print(gas);
    Serial.print(" dist=");
    Serial.print(distance);
    Serial.println("cm");
  }

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
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}


// validate sensor readings
bool checkSensors(int gas, float distance) {
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

  badReadings++;
  return (badReadings < MAX_BAD);
}


// Core FSM decision Logic. Determines state transitions using gas levels,
//proximity data and sensor health information
State decideState(int gas, float distance, bool sensorsOk) {
  if (bootGrace > 0) return SAFE;

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
      // No automatic recovery - operator must restart the system
      // after inspecting and clearing the fault. Matches industrial
      // safety standards: a human must verify before resuming.
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

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_PURPLE, LOW);
  digitalWrite(BUZZER, LOW);

  if (newState == SAFE)     digitalWrite(LED_GREEN, HIGH);
  if (newState == CAUTION)  digitalWrite(LED_YELLOW, HIGH);
  if (newState == DANGER)   digitalWrite(LED_RED, HIGH);
  if (newState == FAILSAFE) digitalWrite(LED_PURPLE, HIGH);
}

//Adaptive behaviour: Returns the sensor polling interval based on 
// thecurrent risk level
int getPollSpeed() {
  if (currentState == SAFE)    return POLL_SAFE;
  if (currentState == CAUTION) return POLL_CAUTION;
  return POLL_DANGER;
}

// Audible warning behaviour
void updateBuzzer() {
  if (currentState == DANGER) {
    if (millis() - lastBeep >= 250) {
      buzzerOn = !buzzerOn;
      if (buzzerOn) {
        tone(BUZZER, 1000);
      } else {
        noTone(BUZZER);
      }
      lastBeep = millis();
    }
  } else if (currentState == FAILSAFE) {
    tone(BUZZER, 1000);
  } else {
    noTone(BUZZER);
    buzzerOn = false;
  }
}

//Convert FSM state values into readable text for serial monitor output
const char* stateName(State s) {
  if (s == SAFE)     return "SAFE";
  if (s == CAUTION)  return "CAUTION";
  if (s == DANGER)   return "DANGER";
  if (s == FAILSAFE) return "FAILSAFE";
  return "?";
}