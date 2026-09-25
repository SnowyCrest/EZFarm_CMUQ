/*
  =========================================================================
  ARDUINO NANO SENSOR EDGE NODE
  =========================================================================
  req libs:
    - "DHT sensor library" by Adafruit
    - "Adafruit Unified Sensor" (dependency of the above)
    - "BH1750" by Christopher Laws (claws)
    - "RF24" by TMRh20
    - "LiquidCrystal I2C" by Frank de Brabander (for the LCD)

  refer to schematic in the repo for the wiring
*/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// ---------------------------------------------------------------------
// NETWORK CONFIG
// ---------------------------------------------------------------------
#define NODE_ID 1

const uint64_t UPLINK_ADDR    = 0xF0F0F0F0E1LL;  // nano to ESP32
const uint64_t DOWNLINK_ADDR  = 0xF0F0F0F0D2LL;  // ESP32 to nano

// ---------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------
#define DHTPIN 2
#define DHTTYPE DHT11

#define WATER_LEVEL_PIN A7
#define WATER_LEVEL_VCC_PIN 7

#define MQ135_PIN A1
#define SOIL_MOISTURE_PIN A0

#define LED_GREEN_PIN 3
#define LED_YELLOW_PIN 4
#define LED_RED_PIN 5

#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

// ---------------------------------------------------------------------
// LCD (I2C) - see the assumption note above if it doesn't light up
// ---------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

const unsigned long LCD_CYCLE_INTERVAL_MS = 2000; // 2s per sensor slide
const unsigned long LCD_EVENT_DURATION_MS = 3000; // 3s for a status overlay

// ---------------------------------------------------------------------
// TIMING
// ---------------------------------------------------------------------
const unsigned long SAMPLE_INTERVAL_MS     = 10000; // 10 seconds
const unsigned long WATER_SENSOR_WARMUP_MS = 50;    // settle time after power-up
const unsigned long RESPONSE_TIMEOUT_MS    = 1500;  // wait for ESP32 reply
const unsigned long YELLOW_BLINK_MS        = 300;

// ---------------------------------------------------------------------
// PROTOCOL
// ---------------------------------------------------------------------
enum SensorType : uint8_t {
  SENSOR_TEMPERATURE = 0,
  SENSOR_HUMIDITY = 1,
  SENSOR_WATER_LEVEL = 2,
  SENSOR_AIR_QUALITY = 3,
  SENSOR_SOIL_MOISTURE = 4,
  SENSOR_LIGHT_INTENSITY = 5,
  SENSOR_COUNT = 6
};

enum LedCommand : uint8_t {
  CMD_GREEN_OK = 0,
  CMD_YELLOW_WARN = 1,
  CMD_RED_ERROR = 2
};

struct __attribute__((packed)) SensorPacket {
  uint8_t  nodeId;
  uint8_t  sensorType;
  float    value;
  uint32_t timestamp;
};

//gateway
struct __attribute__((packed)) CommandPacket {
  uint8_t nodeId;   
  uint8_t command;
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;

unsigned long lastSampleTime   = 0;
unsigned long lastYellowToggle = 0;
bool yellowState = false;

LedCommand currentStatus = CMD_RED_ERROR; // pessimistic

// Latest known reading per sensor, kept around so the LCD can show
// something anytime, not just right after a sample cycle runs.
float latestValues[SENSOR_COUNT];
bool  latestValid[SENSOR_COUNT] = {false, false, false, false, false, false};

// ---------------------------------------------------------------------
// LCD STATE MACHINE
// Two modes: normally it CYCLES through the 6 sensor slides, 2s each.
// Whenever applyLedState() runs (i.e. a connection/status update just
// happened), it switches to EVENT mode for 3s to show that update, then
// automatically drops back into cycling where it left off.
// ---------------------------------------------------------------------
enum LcdMode : uint8_t { LCD_CYCLING, LCD_EVENT };
LcdMode lcdMode = LCD_CYCLING;
unsigned long lcdModeStartTime = 0;
uint8_t lcdCycleIndex = 0;

void setup() {
  Serial.begin(9600);

  pinMode(WATER_LEVEL_VCC_PIN, OUTPUT);
  digitalWrite(WATER_LEVEL_VCC_PIN, LOW); // sensor stays OFF except while reading

  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  dht.begin();

  Wire.begin();
  lightMeter.begin();

  lcd.init();
  lcd.backlight();
  lcdModeStartTime = millis();
  showCycleSlide(lcdCycleIndex);

  if (!radio.begin()) {
    Serial.println(F("NRF24L01 not detected"));
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);  
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.openWritingPipe(UPLINK_ADDR);
  radio.openReadingPipe(1, DOWNLINK_ADDR);
  radio.startListening();

  applyLedState(CMD_RED_ERROR); // no exchange with the gateway has happened yet
                                 // (this also puts the LCD into its first event overlay)
}

void loop() {
  unsigned long now = millis();

  if (currentStatus == CMD_YELLOW_WARN && now - lastYellowToggle >= YELLOW_BLINK_MS) {
    yellowState = !yellowState;
    digitalWrite(LED_YELLOW_PIN, yellowState ? HIGH : LOW);
    lastYellowToggle = now;
  }

  updateLcd(now);

  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    runSampleCycle();
  }
}

void runSampleCycle() {
  float temperature = dht.readTemperature();
  float humidity= dht.readHumidity();
  float waterLevel = readWaterLevel();
  float airQuality = analogRead(MQ135_PIN);
  float soilMoisture = analogRead(SOIL_MOISTURE_PIN);
  float lux = lightMeter.readLightLevel();

  bool dhtOk = !isnan(temperature) && !isnan(humidity);

  // Keep the LCD's "latest known" values fresh regardless of whether the
  // radio send below succeeds - display shouldn't depend on connectivity.
  latestValues[SENSOR_TEMPERATURE] = temperature;
  latestValid[SENSOR_TEMPERATURE]  = dhtOk;
  latestValues[SENSOR_HUMIDITY] = humidity;
  latestValid[SENSOR_HUMIDITY]  = dhtOk;
  latestValues[SENSOR_WATER_LEVEL] = waterLevel;
  latestValid[SENSOR_WATER_LEVEL]  = true;
  latestValues[SENSOR_AIR_QUALITY] = airQuality;
  latestValid[SENSOR_AIR_QUALITY]  = true;
  latestValues[SENSOR_SOIL_MOISTURE] = soilMoisture;
  latestValid[SENSOR_SOIL_MOISTURE]  = true;
  latestValues[SENSOR_LIGHT_INTENSITY] = lux;
  latestValid[SENSOR_LIGHT_INTENSITY]  = (lux >= 0); // BH1750 lib returns negative on error

  radio.stopListening();

  bool allSent = true;
  if (dhtOk) {
    allSent &= sendReading(SENSOR_TEMPERATURE, temperature);
    allSent &= sendReading(SENSOR_HUMIDITY, humidity);
  } else {
    allSent = false; 
  }
  allSent &= sendReading(SENSOR_WATER_LEVEL, waterLevel);
  allSent &= sendReading(SENSOR_AIR_QUALITY, airQuality);
  allSent &= sendReading(SENSOR_SOIL_MOISTURE, soilMoisture);
  allSent &= sendReading(SENSOR_LIGHT_INTENSITY, lux);

  radio.startListening();

  if (!allSent) {
    // when cant connect
    applyLedState(CMD_RED_ERROR);
    return;
  }

  LedCommand response;
  if (waitForCommand(response)) {
    applyLedState(response);
  } else {
    // reply too late
    applyLedState(CMD_RED_ERROR);
  }
}

float readWaterLevel() {
  digitalWrite(WATER_LEVEL_VCC_PIN, HIGH);
  delay(WATER_SENSOR_WARMUP_MS);
  int reading = analogRead(WATER_LEVEL_PIN);
  digitalWrite(WATER_LEVEL_VCC_PIN, LOW);
  return (float)reading;
}

bool sendReading(SensorType type, float value) {
  SensorPacket packet;
  packet.nodeId = NODE_ID;
  packet.sensorType = type;
  packet.value = value;
  packet.timestamp = millis();
  return radio.write(&packet, sizeof(packet));
}

bool waitForCommand(LedCommand &outCommand) {
  unsigned long start = millis();
  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    if (radio.available()) {
      CommandPacket cmd;
      radio.read(&cmd, sizeof(cmd));
      if (cmd.nodeId == NODE_ID) {
        outCommand = (LedCommand)cmd.command;
        return true;
      }
      // not for us
    }
  }
  return false;
}

void applyLedState(LedCommand state) {
  currentStatus = state;

  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  yellowState = false;
  lastYellowToggle = millis();

  switch (state) {
    case CMD_GREEN_OK:
      digitalWrite(LED_GREEN_PIN, HIGH);
      triggerLcdEvent(F("Status: OK"), F("All sensors normal"));
      break;
    case CMD_YELLOW_WARN:
      triggerLcdEvent(F("Status: WARNING"), F("Value out of range"));
      break;
    case CMD_RED_ERROR:
      digitalWrite(LED_RED_PIN, HIGH);
      triggerLcdEvent(F("Status: ERROR"), F("No connection"));
      break;
  }
}

// ---------------------------------------------------------------------
// LCD HELPERS
// ---------------------------------------------------------------------

// Advances/refreshes whichever LCD mode is currently active. Called every
// loop() iteration; does nothing most of the time (only acts once its
// current mode's timer has elapsed), so it's safe to call unconditionally.
void updateLcd(unsigned long now) {
  if (lcdMode == LCD_EVENT) {
    if (now - lcdModeStartTime >= LCD_EVENT_DURATION_MS) {
      lcdMode = LCD_CYCLING;
      lcdModeStartTime = now;
      showCycleSlide(lcdCycleIndex); // resume cycling right where it left off
    }
    return;
  }

  // LCD_CYCLING
  if (now - lcdModeStartTime >= LCD_CYCLE_INTERVAL_MS) {
    lcdCycleIndex = (lcdCycleIndex + 1) % SENSOR_COUNT;
    lcdModeStartTime = now;
    showCycleSlide(lcdCycleIndex);
  }
}

void showCycleSlide(uint8_t idx) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(sensorLabel(idx));
  lcd.setCursor(0, 1);
  if (latestValid[idx]) {
    char valBuf[16];
    formatSensorValue(idx, valBuf, sizeof(valBuf));
    lcd.print(valBuf);
  } else {
    lcd.print(F("N/A"));
  }
}

void triggerLcdEvent(const __FlashStringHelper *line1, const __FlashStringHelper *line2) {
  lcdMode = LCD_EVENT;
  lcdModeStartTime = millis();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

const __FlashStringHelper *sensorLabel(uint8_t type) {
  switch (type) {
    case SENSOR_TEMPERATURE:     return F("Temperature");
    case SENSOR_HUMIDITY:        return F("Humidity");
    case SENSOR_WATER_LEVEL:     return F("Water Level");
    case SENSOR_AIR_QUALITY:     return F("Air Quality");
    case SENSOR_SOIL_MOISTURE:   return F("Soil Moisture");
    case SENSOR_LIGHT_INTENSITY: return F("Light");
    default:                     return F("Unknown");
  }
}

// Writes a short human-readable value+unit into buf (e.g. "23.4 C").
void formatSensorValue(uint8_t type, char *buf, size_t bufSize) {
  float v = latestValues[type];
  char valStr[8];
  switch (type) {
    case SENSOR_TEMPERATURE:
      dtostrf(v, 4, 1, valStr);
      snprintf(buf, bufSize, "%s C", valStr);
      break;
    case SENSOR_HUMIDITY:
      dtostrf(v, 4, 1, valStr);
      snprintf(buf, bufSize, "%s %%", valStr);
      break;
    case SENSOR_WATER_LEVEL:
    case SENSOR_AIR_QUALITY:
    case SENSOR_SOIL_MOISTURE:
      snprintf(buf, bufSize, "%d", (int)v);
      break;
    case SENSOR_LIGHT_INTENSITY:
      snprintf(buf, bufSize, "%d lx", (int)v);
      break;
    default:
      buf[0] = '\0';
  }
}
