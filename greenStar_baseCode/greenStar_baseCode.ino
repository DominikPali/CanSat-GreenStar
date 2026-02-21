#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <CanSatKit.h>

// Pin where the DS18B20 data line is connected
#define ONE_WIRE_BUS 2
#define GPS_SERIAL Serial

// RGB LED pins (common anode - LOW = ON, HIGH = OFF)
#define LED_R 1
#define LED_G 3
#define LED_B 5

// Servo pin
#define SERVO_PIN 9

using namespace CanSatKit;

BMP280 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int chipSelect = 11;
File logFile;

// Servo
Servo containerServo;
bool openContainer = false;
bool servoDone = false;
bool servoMoving = false;
unsigned long servoStartTime = 0;
int servoSweepDuration = 2000; // duration of 0° -> 180° sweep in ms (modifiable)

// Test mode: if true, servo activates 10s after boot
bool test = true;

// Non-blocking loop timing
unsigned long previousLoopTime = 0;
const unsigned long loopInterval = 1000; // 1 second between sensor reads

// Non-blocking LED blink state
bool ledBlinkOn = false;
unsigned long previousBlinkTime = 0;
const unsigned long blinkInterval = 500; // 500ms on / 500ms off

// LED helper: common anode - LOW activates color
void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

class GreenStar {
public:
  double temperatureIn;
  double temperatureOut;
  double pressure;
  float latitude;
  float longitude;
  bool descent = false;

  const float P_ground_ref = 1004.56f;
  float currentHeightAG;

  static const int N_SAMPLES = 5;
  float alt_buffer[N_SAMPLES];
  int buf_index = 0;
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
    nmeaLen = 0;
    latitude = 0.0f;
    longitude = 0.0f;
    for (int i = 0; i < N_SAMPLES; i++) {
      alt_buffer[i] = 0.0f;
    }
  }

  void initializeSensors() {
    sensors.begin();
    bmp.setOversampling(16);
  }

  void initializeRadio() {
    radio.begin();
  }

  void readTemperatureOut() {
    sensors.requestTemperatures();
    temperatureOut = sensors.getTempCByIndex(0);
  }

  void readTemperatureInAndPressure() {
    bmp.measureTemperatureAndPressure(temperatureIn, pressure);
  }

  void altitude_from_pressure() {
    currentHeightAG = 44330.0f * (1.0f - powf(pressure / P_ground_ref, 0.1903f));
  }

  void updateAltitudeSampleAndCheckDescent() {
    alt_buffer[buf_index] = currentHeightAG;
    buf_index++;
    if (buf_index >= N_SAMPLES) {
      buf_index = 0;
      buffer_full = true;
    }

    if (buffer_full) {
      bool all_decreasing = true;
      int start = buf_index;
      for (int i = 1; i < N_SAMPLES; i++) {
        int idx_prev = (start + i - 1) % N_SAMPLES;
        int idx_cur  = (start + i) % N_SAMPLES;
        if (!(alt_buffer[idx_cur] < alt_buffer[idx_prev])) {
          all_decreasing = false;
          break;
        }
      }
      descent = all_decreasing;
    } else {
      descent = false;
    }
  }

  void readGPS() {
    while (GPS_SERIAL.available()) {
      char c = GPS_SERIAL.read();
      if (c == '\n') {
        nmeaBuf[nmeaLen] = '\0';
        processNMEALine(nmeaBuf);
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

    logFile.print(ms);                logFile.print(',');
    logFile.print(temperatureIn, 2);  logFile.print(',');
    logFile.print(temperatureOut, 2); logFile.print(',');
    logFile.print(pressure, 2);       logFile.print(',');
    logFile.print(latitude, 6);       logFile.print(',');
    logFile.print(longitude, 6);      logFile.print(',');
    logFile.print(currentHeightAG, 3); logFile.print(',');
    logFile.println(descent);

    logFile.flush();

    SerialUSB.print("Temperature out: ");
    SerialUSB.print(temperatureOut);
    SerialUSB.println(" °C");

    SerialUSB.print("Pressure: ");
    SerialUSB.print(pressure);
    SerialUSB.println(" hPa");

    SerialUSB.print("Temperature in: ");
    SerialUSB.print(temperatureIn);
    SerialUSB.println(" °C");

    SerialUSB.print("Lat: ");
    SerialUSB.println(latitude, 6);
    SerialUSB.print("Lon: ");
    SerialUSB.println(longitude, 6);

    SerialUSB.print("Height AGL: ");
    SerialUSB.println(currentHeightAG, 6);

    SerialUSB.print("Descent: ");
    SerialUSB.println(descent ? "YES" : "NO");

    for (int i = 0; i < N_SAMPLES; i++) {
      SerialUSB.print(alt_buffer[i], 3);
      SerialUSB.print(" ");
    }
    SerialUSB.println("");

    SerialUSB.println("Logged data to SD");
  }

  void sendRadioBundle() {
    unsigned long ms = millis();

    frame.print(ms);                frame.print(',');
    frame.print(temperatureIn, 2);  frame.print(',');
    frame.print(temperatureOut, 2); frame.print(',');
    frame.print(pressure, 2);       frame.print(',');
    frame.print(latitude, 6);       frame.print(',');
    frame.print(longitude, 6);      frame.print(',');
    frame.print(currentHeightAG, 3); frame.print(',');
    frame.print(descent ? 1 : 0);

    radio.transmit(frame);

    SerialUSB.print("RADIO TX: ");
    SerialUSB.println(frame);

    frame.clear();
  }

private:
  void processNMEALine(const char *line) {
    if (!(strncmp(line, "$GNGGA", 6) == 0 ||
          strncmp(line, "$GNRMC", 6) == 0)) {
      return;
    }

    char buf[NMEA_BUF_SIZE];
    strncpy(buf, line, NMEA_BUF_SIZE);
    buf[NMEA_BUF_SIZE - 1] = '\0';

    char *fields[20];
    int idx = 0;
    char *p = buf;
    fields[idx++] = p;
    while (*p && idx < 20) {
      if (*p == ',') {
        *p = '\0';
        fields[idx++] = p + 1;
      }
      p++;
    }

    if (idx > 6 && strlen(fields[3]) >= 4 && strlen(fields[5]) >= 4) {
      float lat = convertNMEADeg(fields[3]);
      float lon = convertNMEADeg(fields[5]);
      if (fields[4][0] == 'S') lat = -lat;
      if (fields[6][0] == 'W') lon = -lon;
      latitude = lat;
      longitude = lon;
    }
  }

  float convertNMEADeg(const char *raw) {
    float v = atof(raw);
    int deg = (int)(v / 100);
    float minutes = v - (deg * 100);
    return deg + (minutes / 60.0);
  }
};

GreenStar cansat;

void setup() {
  // Initialize LED pins
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(false, false, false);

  SerialUSB.begin(9600);
  GPS_SERIAL.begin(9600);

  // Initialize servo to starting position (0 degrees)
  containerServo.attach(SERVO_PIN);
  containerServo.write(0);
  delay(500);

  // RED (2s) - Arduino is powered
  setLED(true, false, false);
  delay(2000);
  setLED(false, false, false);

  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("SD init failed, stopping.");
    while (1);
  }

  logFile = SD.open("log.csv", FILE_WRITE);
  if (!logFile) {
    SerialUSB.println("Failed to open log.csv for writing.");
    while (1);
  }

  if (logFile.size() == 0) {
    logFile.println("timestamp_ms,temperatureIn,temperatureOut,pressure,latitude,longitude,altitude_AGL,descent");
    logFile.flush();
  }

  if (!bmp.begin()) {
    SerialUSB.println("BMP init failed!");
    while (1);
  }

  cansat.initializeSensors();

  // YELLOW (2s) - pressure and temperature sensors working properly
  cansat.readTemperatureInAndPressure();
  bool sensorsOk = (cansat.pressure >= 300.0 && cansat.pressure <= 1100.0) &&
                   (cansat.temperatureIn >= -40.0 && cansat.temperatureIn <= 85.0);
  if (sensorsOk) {
    setLED(true, true, false);
    delay(2000);
    setLED(false, false, false);
  }

  cansat.initializeRadio();

  // BLUE (2s) - radio initialized and working
  setLED(false, false, true);
  delay(2000);
  setLED(false, false, false);

  SerialUSB.println("Setup done, logging started.");

  previousLoopTime = millis();
  previousBlinkTime = millis();
}

void loop() {
  unsigned long now = millis();

  // ===================== SENSOR READ EVERY 1s =====================
  if (now - previousLoopTime >= loopInterval) {
    previousLoopTime = now;

    cansat.readTemperatureOut();
    cansat.readTemperatureInAndPressure();
    cansat.altitude_from_pressure();
    cansat.updateAltitudeSampleAndCheckDescent();
    cansat.readGPS();
    cansat.logData();
    cansat.sendRadioBundle();
  }
  // ================================================================

  // ===================== TEST MODE =====================
  if (test && !openContainer && now >= 10000) {
    openContainer = true;
    SerialUSB.println("TEST: triggering servo opening after 10s");
  }
  // =====================================================

  // ===================== NON-BLOCKING SERVO SWEEP =====================
  if (openContainer && !servoDone) {
    if (!servoMoving) {
      servoMoving = true;
      servoStartTime = now;
    }

    unsigned long elapsed = now - servoStartTime;

    if (elapsed >= (unsigned long)servoSweepDuration) {
      containerServo.write(180);
      delay(300); // brief hold to ensure servo reaches final position
      containerServo.detach(); // release timer to restore LED PWM
      servoMoving = false;
      servoDone = true;
      SerialUSB.println("Servo: reached 180°, detached");
    } else {
      int angle = (int)((float)elapsed / (float)servoSweepDuration * 180.0f);
      containerServo.write(angle);
    }
  }
  // ====================================================================

  // ===================== NON-BLOCKING LED GPS STATUS =====================
  bool gpsFixed = !(cansat.latitude == 0.0f && cansat.longitude == 0.0f);

  if (!gpsFixed) {
    // PURPLE blinking — toggle every 500ms
    if (now - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = now;
      ledBlinkOn = !ledBlinkOn;
    }
    if (ledBlinkOn) {
      setLED(true, false, true); // PURPLE = R+B
    } else {
      setLED(false, false, false);
    }
  } else {
    // GREEN constant — all systems working
    setLED(false, true, false);
  }
  // ======================================================================
}
