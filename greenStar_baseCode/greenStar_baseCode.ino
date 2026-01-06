#include <SPI.h>
#include <SD.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <CanSatKit.h>

// Pin where the DS18B20 data line is connected
#define ONE_WIRE_BUS 2
#define GPS_SERIAL Serial

using namespace CanSatKit;

BMP280 bmp;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

const int chipSelect = 11;
File logFile;

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
  int buf_index = 0;      // next position to write
  bool buffer_full = false;

private:
  // NMEA buffer
  static const int NMEA_BUF_SIZE = 120;
  char nmeaBuf[NMEA_BUF_SIZE];
  int  nmeaLen;

public:
  GreenStar() {
    nmeaLen = 0;
    latitude = 0.0f;
    longitude = 0.0f;
    // initialize buffer (optional)
    for (int i = 0; i < N_SAMPLES; i++) {
      alt_buffer[i] = 0.0f;
    }
  }

  void initializeSensors() {
    sensors.begin();
    bmp.setOversampling(16);
  }

  void readTemperatureOut() {
    sensors.requestTemperatures();
    temperatureOut = sensors.getTempCByIndex(0);
  }

  void readTemperatureInAndPressure() {
    bmp.measureTemperatureAndPressure(temperatureIn, pressure);
  }

  void altitude_from_pressure() {
    // barometric formula: h = 44330 * (1 - (pressure / P0)^(0.1903))
    currentHeightAG = 44330.0f * (1.0f - powf(pressure / P_ground_ref, 0.1903f));
  }

  // Call this every second (just after updating currentHeightAG)
  void updateAltitudeSampleAndCheckDescent() {
    // store current height in buffer
    alt_buffer[buf_index] = currentHeightAG;
    buf_index++;
    if (buf_index >= N_SAMPLES) {
      buf_index = 0;
      buffer_full = true;
    }

    if (buffer_full) {
      // check if strictly decreasing
      bool all_decreasing = true;
      // we need to check samples in time order: oldest → newest
      // compute start index of the oldest sample
      int start = buf_index;  // next write pos is oldest (oldest was overwritten), so buffer content is from start
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
          // buffer overflow — reset
          nmeaLen = 0;
        }
      }
    }
  }

  void logData() {
    if (!logFile) return;

    unsigned long ms = millis();

    logFile.print(ms);     logFile.print(',');
    logFile.print(temperatureIn, 2);  logFile.print(',');
    logFile.print(temperatureOut, 2); logFile.print(',');
    logFile.print(pressure, 2);       logFile.print(',');
    logFile.print(latitude, 6);       logFile.print(',');
    logFile.print(longitude, 6);    logFile.print(',');
    logFile.print(currentHeightAG, 3);    logFile.print(',');
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

    for(int i = 0; i < N_SAMPLES; i++){
      SerialUSB.print(alt_buffer[i], 3);
      SerialUSB.print(" ");
    }
    SerialUSB.println("");

    SerialUSB.println("Logged data to SD");
  }

private:
  void processNMEALine(const char *line) {
    if (!(strncmp(line, "$GNGGA", 6) == 0 ||
          strncmp(line, "$GNRMC", 6) == 0)) {
      return;
    }

    char buf[NMEA_BUF_SIZE];
    strncpy(buf, line, NMEA_BUF_SIZE);
    buf[NMEA_BUF_SIZE-1] = '\0';

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
  SerialUSB.begin(9600);
  GPS_SERIAL.begin(9600);

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
  SerialUSB.println("Setup done, logging started.");
}

void loop() {
  cansat.readTemperatureOut();
  cansat.readTemperatureInAndPressure();
  cansat.altitude_from_pressure();
  cansat.updateAltitudeSampleAndCheckDescent();
  cansat.readGPS();
  cansat.logData();
  delay(1000);
}