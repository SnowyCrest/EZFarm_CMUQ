/*
  =========================================================================
  Actuator Node - Arduino Nano
  =========================================================================
  REQUIRED LIBRARIES (Arduino Library Manager):
    - "RF24" by TMRh20

    for wiring, refer to circuit schem

  IMPORTANT - CHECK RELAY MODULE'S LOGIC LEVEL:
    Most cheap 4-channel relay boards are ACTIVE-LOW: the relay energizes
    when the Arduino pin is driven LOW, not HIGH. RELAY_ACTIVE_LOW below
    defaults to true for that reason. If relays turn ON when
    expect OFF, flip this to false.
  =========================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// ---------------------------------------------------------------------
// NETWORK CONFIGURATION - change NODE_ID for every actuator Nano
// ---------------------------------------------------------------------
#define NODE_ID               101   // <-- UNIQUE across the WHOLE network
                                     // (sensor Nanos and actuator Nanos
                                     // share one numbering space so logs/
                                     // MQTT topics stay unambiguous, even
                                     // though they use separate radio pipes)

// Dedicated pipes for actuator traffic - intentionally different from the
// sensor Nanos' UPLINK_ADDR/DOWNLINK_ADDR, so this traffic never collides
// with, or gets accidentally parsed by, a sensor node.
// NOTE: the actuator uplink address's top 4 bytes (0xF0F0F0F0) match the
// sensor uplink address on purpose - see the ESP32 gateway comments for
// why (an NRF24L01 hardware requirement for pipes 2-5).
const uint64_t ACTUATOR_UPLINK_ADDR   = 0xF0F0F0F0E2LL; // actuator -> ESP32
const uint64_t ACTUATOR_DOWNLINK_ADDR = 0xF0F0F0F0D3LL; // ESP32 -> actuator

// ---------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------
#define RELAY1_PIN 5
#define RELAY2_PIN 6
#define RELAY3_PIN 7
#define RELAY4_PIN 8

#define LED_GREEN_PIN 2
#define LED_YELLOW_PIN 3
#define LED_RED_PIN 4

#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

const bool RELAY_ACTIVE_LOW = true;
// ---------------------------------------------------------------------
// TIMING
// ---------------------------------------------------------------------
const unsigned long HEARTBEAT_INTERVAL_MS = 10000;
const unsigned long RESPONSE_TIMEOUT_MS = 1500;
const unsigned long YELLOW_BLINK_MS = 300;

// ---------------------------------------------------------------------
// PROTOCOL - must match the ESP32 gateway exactly
// ---------------------------------------------------------------------
enum ActuatorStatus : uint8_t {
  ACT_GREEN_OK    = 0,
  ACT_YELLOW_WARN = 1,
  ACT_RED_ERROR   = 2
};

struct __attribute__((packed)) ActuatorHeartbeat {
  uint8_t  nodeId;
  uint32_t timestamp;
};

struct __attribute__((packed)) RelayCommandPacket {
  uint8_t nodeId;
  uint8_t relayMask;
  uint8_t status;
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

unsigned long lastHeartbeatTime = 0;
unsigned long lastYellowToggle  = 0;
bool yellowState = false;

ActuatorStatus currentStatus = ACT_RED_ERROR; // pessimistic
uint8_t currentRelayMask = 0x00;

void setup() {
  Serial.begin(9600);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  applyRelayMask(0x00); // everything off at boot, failsafe

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  if (!radio.begin()) {
    Serial.println(F("NRF24L01 not detected - check wiring"));
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.openWritingPipe(ACTUATOR_UPLINK_ADDR);
  radio.openReadingPipe(1, ACTUATOR_DOWNLINK_ADDR);
  radio.startListening();

  applyLedState(ACT_RED_ERROR); // no exchange with the gateway has happened yet
}

void loop() {
  unsigned long now = millis();

  if (currentStatus == ACT_YELLOW_WARN && now - lastYellowToggle >= YELLOW_BLINK_MS) {
    yellowState = !yellowState;
    digitalWrite(LED_YELLOW_PIN, yellowState ? HIGH : LOW);
    lastYellowToggle = now;
  }

  if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatTime = now;
    runHeartbeatCycle();
  }
}

void runHeartbeatCycle() {
  ActuatorHeartbeat hb;
  hb.nodeId = NODE_ID;
  hb.timestamp = millis();

  radio.stopListening();
  bool sent = radio.write(&hb, sizeof(hb));
  radio.startListening();

  if (!sent) {
    applyLedState(ACT_RED_ERROR); // couldn't even reach the gateway
    return;

  }

  RelayCommandPacket cmd;
  if (waitForCommand(cmd)) {
    applyRelayMask(cmd.relayMask);
    applyLedState((ActuatorStatus)cmd.status);
  } else {
    // Heartbeat went out, but no reply came back in time.
    applyLedState(ACT_RED_ERROR);

  }
}

bool waitForCommand(RelayCommandPacket &outCmd) {
  unsigned long start = millis();
  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    if (radio.available()) {
      RelayCommandPacket cmd;
      radio.read(&cmd, sizeof(cmd));
      if (cmd.nodeId == NODE_ID) {
        outCmd = cmd;
        return true;
      }
      // else: meant for a different node, keep waiting
    }
  }
  return false;
}

void applyRelayMask(uint8_t mask) {
  currentRelayMask = mask;
  writeRelay(RELAY1_PIN, mask & 0x01);
  writeRelay(RELAY2_PIN, mask & 0x02);
  writeRelay(RELAY3_PIN, mask & 0x04);
  writeRelay(RELAY4_PIN, mask & 0x08);
}

void writeRelay(uint8_t pin, bool energize) {
  bool pinHigh = RELAY_ACTIVE_LOW ? !energize : energize;
  digitalWrite(pin, pinHigh ? HIGH : LOW);
}

void applyLedState(ActuatorStatus state) {
  currentStatus = state;

  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  yellowState = false;
  lastYellowToggle = millis();

  switch (state) {
    case ACT_GREEN_OK:
      digitalWrite(LED_GREEN_PIN, HIGH);
      break;
    case ACT_YELLOW_WARN:
      // handled by the non-blocking blink logic in loop()
      break;
    case ACT_RED_ERROR:
      digitalWrite(LED_RED_PIN, HIGH);
      break;
  }
}
