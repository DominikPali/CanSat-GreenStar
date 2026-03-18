// ============================================================================
// GREEN STAR CANSAT - MAIN FLIGHT SOFTWARE
// ============================================================================
//
// Mission: Deploy cellulose seed capsules at ~100m AGL during descent,
//          while continuously logging and transmitting environmental data.
//
// Hardware:
//   - ATSAMD21G18A microcontroller (CanSat Kit)
//   - BMP280 pressure/temperature sensor (I2C)
//   - DS18B20 external temperature sensor (OneWire on pin 2)
//   - SparkFun SAM-M10Q GPS module (UART on Serial, pins 0/1)
//   - LoRa SX1278 radio transceiver (SPI)
//   - SG-90 micro servo motor (PWM on pin 7)
//   - RGB LED common-anode (pins 4, 6, 8 — LOW = ON)
//   - MicroSD card (SPI, CS on pin 11)
//
// Flight phases:
//   1. GROUND — sensors initialise, LED sequence confirms health
//   2. ASCENT (flight_up) — rocket is climbing, altitude increasing
//   3. DESCENT — CanSat ejected, altitude decreasing at ~expected rate
//   4. DEPLOY — altitude <= 100 m AGL while descending → servo opens
//   5. POST-DEPLOY — continue logging/transmitting until recovery
//
// Authors : Green Star Team
// Version : 2.0
// ============================================================================

#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <CanSatKit.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define ONE_WIRE_BUS 2
#define GPS_SERIAL Serial

#define LED_R 13
#define LED_G 6
#define LED_B 8

#define SERVO_PIN 7

const int chipSelect = 11;

// ============================================================================
// LIBRARY NAMESPACE
// ============================================================================
using namespace CanSatKit;

// ============================================================================
// GLOBAL SENSOR OBJECTS
// ============================================================================

BMP280 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dsSensors(&oneWire);
File logFile;
Servo containerServo;

// ============================================================================
// SERVO STATE MACHINE
// ============================================================================

bool openContainer  = false;
bool servoDone      = false;
bool servoMoving    = false;

// Waypoints are globals — a static index into a local array is UB and causes
// garbage writes (twitching). Keep everything together at global scope.
const int SERVO_WAYPOINTS[]   = { 0, 30, 65, 90, 119 };
const int SERVO_NUM_WAYPOINTS = 5;
int       servoWaypointIndex  = 0;

// ============================================================================
// TEST MODE
// ============================================================================

bool test = true;

// ============================================================================
// LOOP TIMING (non-blocking)
// ============================================================================

unsigned long previousLoopTime      = 0;
unsigned long previousServoStepTime = 0;  // independent timer — decoupled from sensors

const unsigned long loopInterval      = 500; // ms between sensor reads
const unsigned long servoStepInterval = 500; // ms between each waypoint step

bool ledBlinkOn = false;
unsigned long previousBlinkTime = 0;
const unsigned long blinkInterval = 500;

// ============================================================================
// ALTITUDE RING-BUFFER PARAMETERS
// ============================================================================

static const int   ALT_RING_SIZE          = 6;
static const int   K_DESCENT              = 3;
static const int   K_ASCENT               = 3;
static const float EXPECTED_FALL_VELOCITY = 2.0f;
static const float VEL_MARGIN             = 0.4f;
static const float MIN_DT_S               = 0.2f;

static const float EXPECTED_ROCKET_VELOCITY = 10.0f;
static const float ROCKET_VEL_MARGIN        = 3.0f;

static const float DEPLOY_ALTITUDE_THRESHOLD = 100.0f;

// ============================================================================
// WATCHDOG TIMER
// ============================================================================

void wdt_init(uint8_t period) {
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2)
                     | GCLK_GENCTRL_GENEN
                     | GCLK_GENCTRL_SRC_OSCULP32K
                     | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT
                     | GCLK_CLKCTRL_CLKEN
                     | GCLK_CLKCTRL_GEN_GCLK2;
  while (GCLK->STATUS.bit.SYNCBUSY);

  WDT->CTRL.reg = 0;
  while (WDT->STATUS.bit.SYNCBUSY);

  WDT->CONFIG.reg = period;

  WDT->CTRL.reg = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);
}

void wdt_reset() {
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);
}

// ============================================================================
// LED HELPER
// ============================================================================

void setLED(const char* color) {
  bool r = HIGH, g = HIGH, b = HIGH;

  if      (strcmp(color, "red")    == 0) { r = LOW; }
  else if (strcmp(color, "green")  == 0) { g = LOW; }
  else if (strcmp(color, "blue")   == 0) { b = LOW; }
  else if (strcmp(color, "yellow") == 0) { r = LOW; g = LOW; }
  else if (strcmp(color, "purple") == 0) { r = LOW; b = LOW; }
  else if (strcmp(color, "cyan")   == 0) { g = LOW; b = LOW; }

  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

// ============================================================================
// NMEA CHECKSUM VALIDATOR
// ============================================================================

bool validateNMEAChecksum(const char* sentence) {
  if (sentence[0] != '$') return false;

  uint8_t calculated = 0;
  int i = 1;

  while (sentence[i] != '\0' && sentence[i] != '*') {
    calculated ^= (uint8_t)sentence[i];
    i++;
  }

  if (sentence[i] != '*') return true;

  char hexStr[3] = { sentence[i + 1], sentence[i + 2], '\0' };
  uint8_t expected = (uint8_t)strtol(hexStr, NULL, 16);

  return (calculated == expected);
}

// ============================================================================
// MAIN CANSAT CLASS
// ============================================================================

class GreenStar {
public:
  double temperatureIn;
  double temperatureOut;
  double pressure;

  double latitude;
  double longitude;
  double gpsAltitude;
  bool   gpsFixed;

  bool descent   = false;
  bool flight_up = false;

  const float P_ground_ref = 1004.56f;
  float currentHeightAG;

  float         alt_buffer[ALT_RING_SIZE];
  unsigned long t_buffer[ALT_RING_SIZE];
  int  buf_index   = 0;
  bool buffer_full = false;

private:
  static const int NMEA_BUF_SIZE = 120;
  char nmeaBuf[NMEA_BUF_SIZE];
  int  nmeaLen;

  Radio radio = Radio(Pins::Radio::ChipSelect,
                      Pins::Radio::DIO0,
                      433.0,
                      Bandwidth_125000_Hz,
                      SpreadingFactor_9,
                      CodingRate_4_8);
  Frame frame;

public:
  GreenStar() {
    nmeaLen         = 0;
    latitude        = 0.0;
    longitude       = 0.0;
    gpsAltitude     = 0.0;
    gpsFixed        = false;
    temperatureIn   = 0.0;
    temperatureOut  = 0.0;
    pressure        = 0.0;
    currentHeightAG = 0.0;

    for (int i = 0; i < ALT_RING_SIZE; i++) {
      alt_buffer[i] = 0.0f;
      t_buffer[i]   = 0;
    }
  }

  void initializeSensors() {
    dsSensors.begin();
    dsSensors.setResolution(10);
    bmp.setOversampling(16);
  }

  void initializeRadio() {
    radio.begin();
  }

  void readTemperatureOut() {
    dsSensors.requestTemperatures();
    temperatureOut = dsSensors.getTempCByIndex(0);
  }

  void readTemperatureInAndPressure() {
    bmp.measureTemperatureAndPressure(temperatureIn, pressure);
  }

  void altitude_from_pressure() {
    currentHeightAG = 44330.0f * (1.0f - powf(pressure / P_ground_ref, 0.1903f));
  }

  void updateAltitudeSampleAndCheckPhase() {
    unsigned long t_now = millis();

    alt_buffer[buf_index] = currentHeightAG;
    t_buffer[buf_index]   = t_now;

    buf_index++;
    if (buf_index >= ALT_RING_SIZE) {
      buf_index   = 0;
      buffer_full = true;
    }

    if (!buffer_full) {
      descent   = false;
      flight_up = false;
      return;
    }

    int start         = buf_index;
    int descent_votes = 0;
    int ascent_votes  = 0;

    for (int i = 1; i < ALT_RING_SIZE; i++) {
      int idx_prev = (start + i - 1) % ALT_RING_SIZE;
      int idx_cur  = (start + i)     % ALT_RING_SIZE;

      float dh = alt_buffer[idx_cur] - alt_buffer[idx_prev];
      float dt = (t_buffer[idx_cur]  - t_buffer[idx_prev]) / 1000.0f;

      if (dt < MIN_DT_S) continue;

      float v = dh / dt;

      if (v < -(EXPECTED_FALL_VELOCITY - VEL_MARGIN))          descent_votes++;
      if (v >  (EXPECTED_ROCKET_VELOCITY - ROCKET_VEL_MARGIN)) ascent_votes++;
    }

    descent   = (descent_votes >= K_DESCENT);
    flight_up = (ascent_votes  >= K_ASCENT);
  }

  void readGPS() {
    while (GPS_SERIAL.available()) {
      char c = GPS_SERIAL.read();

      if (c == '\n') {
        nmeaBuf[nmeaLen] = '\0';
        if (validateNMEAChecksum(nmeaBuf)) {
          processNMEALine(nmeaBuf);
        }
        nmeaLen = 0;
      }
      else if (c != '\r') {
        if (nmeaLen < NMEA_BUF_SIZE - 1) {
          nmeaBuf[nmeaLen++] = c;
        } else {
          nmeaLen = 0;
        }
      }
    }
  }

  void logData() {
    if (!logFile) return;

    unsigned long ms = millis();

    logFile.print(ms);                  logFile.print(',');
    logFile.print(temperatureIn, 2);    logFile.print(',');
    logFile.print(temperatureOut, 2);   logFile.print(',');
    logFile.print(pressure, 2);         logFile.print(',');
    logFile.print(latitude, 6);         logFile.print(',');
    logFile.print(longitude, 6);        logFile.print(',');
    logFile.print(currentHeightAG, 3);  logFile.print(',');
    logFile.print(gpsAltitude, 2);      logFile.print(',');
    logFile.print(descent ? 1 : 0);     logFile.print(',');
    logFile.print(flight_up ? 1 : 0);   logFile.print(',');
    logFile.print(servoDone ? 1 : 0);   logFile.print(',');
    logFile.print(servoMoving ? 1 : 0); logFile.print(',');
    logFile.println(openContainer ? 1 : 0);
    logFile.flush();

    SerialUSB.print("T_out: ");      SerialUSB.print(temperatureOut);
    SerialUSB.print(" | P: ");       SerialUSB.print(pressure);
    SerialUSB.print(" | T_in: ");    SerialUSB.print(temperatureIn);
    SerialUSB.print(" | Lat: ");     SerialUSB.print(latitude, 6);
    SerialUSB.print(" | Lon: ");     SerialUSB.print(longitude, 6);
    SerialUSB.print(" | hAGL: ");    SerialUSB.print(currentHeightAG, 2);
    SerialUSB.print(" | GPS_alt: "); SerialUSB.print(gpsAltitude, 2);
    SerialUSB.print(" | desc: ");    SerialUSB.print(descent ? "Y" : "N");
    SerialUSB.print(" | up: ");      SerialUSB.print(flight_up ? "Y" : "N");
    SerialUSB.print(" | servo: ");
    SerialUSB.println(servoDone ? "DONE" : (servoMoving ? "MOVING" : "WAIT"));

    SerialUSB.print("  alt_buf: ");
    for (int i = 0; i < ALT_RING_SIZE; i++) {
      SerialUSB.print(alt_buffer[i], 2);
      SerialUSB.print(" ");
    }
    SerialUSB.println();
  }

  void sendRadioBundle() {
    unsigned long ms = millis();

    frame.print(ms);                  frame.print(',');
    frame.print(temperatureIn, 2);    frame.print(',');
    frame.print(temperatureOut, 2);   frame.print(',');
    frame.print(pressure, 2);         frame.print(',');
    frame.print(latitude, 6);         frame.print(',');
    frame.print(longitude, 6);        frame.print(',');
    frame.print(currentHeightAG, 3);  frame.print(',');
    frame.print(gpsAltitude, 2);      frame.print(',');
    frame.print(descent ? 1 : 0);     frame.print(',');
    frame.print(flight_up ? 1 : 0);   frame.print(',');
    frame.print(servoDone ? 1 : 0);   frame.print(',');
    frame.print(servoMoving ? 1 : 0); frame.print(',');
    frame.print(openContainer ? 1 : 0);

    radio.transmit(frame);

    SerialUSB.print("RADIO TX: ");
    SerialUSB.println(frame);

    frame.clear();
  }

private:
  void processNMEALine(const char* line) {
    int lineLen = strlen(line);
    if (lineLen < 7) return;
    if (line[0] != '$') return;

    char type[4];
    type[0] = line[3];
    type[1] = line[4];
    type[2] = line[5];
    type[3] = '\0';

    bool isGGA = (strcmp(type, "GGA") == 0);
    bool isRMC = (strcmp(type, "RMC") == 0);
    if (!isGGA && !isRMC) return;

    char buf[NMEA_BUF_SIZE];
    strncpy(buf, line, NMEA_BUF_SIZE);
    buf[NMEA_BUF_SIZE - 1] = '\0';

    char* fields[20];
    int   fieldCount = 0;
    char* p = buf;
    fields[fieldCount++] = p;
    while (*p && fieldCount < 20) {
      if (*p == ',') {
        *p = '\0';
        fields[fieldCount++] = p + 1;
      }
      p++;
    }

    if (isGGA) {
      if (fieldCount > 10 && strlen(fields[2]) >= 4 && strlen(fields[4]) >= 4) {
        int fixQuality = atoi(fields[6]);
        if (fixQuality == 0) { gpsFixed = false; return; }

        double lat = convertNMEADeg(fields[2]);
        if (fields[3][0] == 'S') lat = -lat;
        double lon = convertNMEADeg(fields[4]);
        if (fields[5][0] == 'W') lon = -lon;

        latitude  = lat;
        longitude = lon;
        gpsFixed  = true;

        if (strlen(fields[9]) > 0) gpsAltitude = atof(fields[9]);
      }
    }
    else if (isRMC) {
      if (fieldCount > 6 && strlen(fields[3]) >= 4 && strlen(fields[5]) >= 4) {
        if (fields[2][0] != 'A') return;

        double lat = convertNMEADeg(fields[3]);
        if (fields[4][0] == 'S') lat = -lat;
        double lon = convertNMEADeg(fields[5]);
        if (fields[6][0] == 'W') lon = -lon;

        latitude  = lat;
        longitude = lon;
        gpsFixed  = true;
      }
    }
  }

  double convertNMEADeg(const char* raw) {
    double v       = atof(raw);
    int    deg     = (int)(v / 100);
    double minutes = v - (deg * 100);
    return deg + (minutes / 60.0);
  }
};

// ============================================================================
// GLOBAL CANSAT INSTANCE
// ============================================================================
GreenStar cansat;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED("off");

  SerialUSB.begin(115200);
  GPS_SERIAL.begin(9600);

  containerServo.attach(SERVO_PIN);
  containerServo.write(0);
  delay(500);

  setLED("green");
  delay(2000);
  setLED("off");

  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("SD init failed, stopping.");
    while (1) { setLED("red"); delay(500); setLED("off"); delay(500); }
  }

  logFile = SD.open("log.csv", FILE_WRITE);
  if (!logFile) {
    SerialUSB.println("Failed to open log.csv for writing.");
    while (1) { setLED("red"); delay(500); setLED("off"); delay(500); }
  }

  if (logFile.size() == 0) {
    logFile.println(
      "timestamp_ms,temperatureIn,temperatureOut,pressure,"
      "latitude,longitude,altitude_AGL,gps_altitude,"
      "descent,flight_up,servoDone,servoMoving,openContainer"
    );
    logFile.flush();
  }

  if (!bmp.begin()) {
    SerialUSB.println("BMP init failed!");
    while (1) { setLED("red"); delay(500); setLED("off"); delay(500); }
  }

  cansat.initializeSensors();

  cansat.readTemperatureInAndPressure();
  bool sensorsOk = (cansat.pressure >= 300.0 && cansat.pressure <= 1100.0) &&
                   (cansat.temperatureIn > -40.0 && cansat.temperatureIn <= 85.0);
  if (sensorsOk) {
    setLED("yellow"); delay(2000); setLED("off");
  } else {
    setLED("yellow"); delay(500);
    for (int i = 0; i < 5; i++) {
      setLED("red"); delay(300); setLED("off"); delay(200);
    }
  }

  cansat.initializeRadio();

  setLED("blue");
  delay(2000);
  setLED("off");

  SerialUSB.println("=== Setup complete, logging started ===");

  previousLoopTime      = millis();
  previousBlinkTime     = millis();
  previousServoStepTime = millis();

  wdt_init(WDT_CONFIG_PER_4K);
  SerialUSB.println("WDT enabled (~4 s timeout)");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  wdt_reset();

  // Read GPS every iteration to prevent serial buffer overflow
  cansat.readGPS();

  // ================================================================
  // SENSOR READ — every 500 ms
  // ================================================================
  if (now - previousLoopTime >= loopInterval) {
    previousLoopTime = now;

    cansat.readTemperatureOut();
    cansat.readTemperatureInAndPressure();
    cansat.altitude_from_pressure();
    cansat.updateAltitudeSampleAndCheckPhase();
    cansat.logData();
    cansat.sendRadioBundle();
  }

  // ================================================================
  // TEST MODE — servo triggers 10 s after boot
  // ================================================================
  if (test && !openContainer && now >= 10000) {
    openContainer = true;
    SerialUSB.println("TEST: triggering servo after 10 s");
  }

  // ================================================================
  // FLIGHT DEPLOYMENT LOGIC
  // ================================================================
  static bool rocketFlew = false;
  if (cansat.flight_up) rocketFlew = true;

  if (!test        &&
      !openContainer &&
      !servoDone     &&
      rocketFlew     &&
      cansat.descent &&
      cansat.currentHeightAG <= DEPLOY_ALTITUDE_THRESHOLD) {
    openContainer = true;
    SerialUSB.println("DEPLOY: conditions met — opening container!");
  }

  // ================================================================
  // NON-BLOCKING SERVO STEP SEQUENCE
  // ================================================================
  // Steps through waypoints one per servoStepInterval (500 ms).
  // Uses its own independent timer (previousServoStepTime) so it is
  // fully decoupled from sensor reads, SD writes, and radio TX.
  // On first entry, angle 0 is written immediately and the timer is
  // started — subsequent waypoints fire every 500 ms after that.

  if (openContainer && !servoDone) {
    if (!servoMoving) {
      // First entry: write waypoint 0 immediately and start the step timer
      servoMoving           = true;
      previousServoStepTime = now;
      containerServo.write(SERVO_WAYPOINTS[0]);
      servoWaypointIndex    = 1;
      SerialUSB.print("Servo: moving to ");
      SerialUSB.println(SERVO_WAYPOINTS[0]);
    }
    else if (now - previousServoStepTime >= servoStepInterval) {
      previousServoStepTime = now;

      if (servoWaypointIndex < SERVO_NUM_WAYPOINTS) {
        containerServo.write(SERVO_WAYPOINTS[servoWaypointIndex]);
        SerialUSB.print("Servo: moving to ");
        SerialUSB.println(SERVO_WAYPOINTS[servoWaypointIndex]);
        servoWaypointIndex++;
      } else {
        servoMoving = false;
        servoDone   = true;
        SerialUSB.println("Servo: sweep complete (160°)");
      }
    }
    // No else — if neither condition fires, do nothing. No write() spam.
  }

  // ================================================================
  // NON-BLOCKING LED — GPS STATUS INDICATOR
  // ================================================================

  if (!cansat.gpsFixed) {
    if (now - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = now;
      ledBlinkOn = !ledBlinkOn;
    }
    setLED(ledBlinkOn ? "cyan" : "off");
  } else {
    setLED("cyan");
  }
}
