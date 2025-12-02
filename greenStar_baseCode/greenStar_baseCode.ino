#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#define GPS_SERIAL Serial1 
#define BMP280_I2C_ADDRESS 0x76
#define ONE_WIRE_BUS 2

Adafruit_BMP280 bmp;  // global BMP280 object
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

class GreenStar {
public:
  double temperatureIn, temperatureOut;
  double pressure;

  GreenStar() {
    // Do NOT do hardware stuff here
    temperatureIn = 0;
    temperatureOut = 0;
    pressure = 0;
  }

  // Call this from setup()
  void begin() {
    Serial.println("START");
    InitializePressureSensor();
    sensors.begin();
  }

  void readTemperature() {
    sensors.requestTemperatures();
    temperatureOut = sensors.getTempCByIndex(0);
  }

  void readPressure() {
    temperatureIn = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0; // hPa
  }

  void saveDataToSD() {
    Serial.print("Pressure: ");
    Serial.println(pressure, 2);
    Serial.print("Temp_in (BMP): ");
    Serial.println(temperatureIn, 2);
    Serial.print("Temp_out (OneWire): ");
    Serial.println(temperatureOut, 2);
    Serial.println("---");
  }

  void readGPS(){
 
  }

  void sendDataToGround(){
 
  }
 
  void checkForDescent(){
 
  }
 
  void checkForDesiredAltitude(){
 
  }

private:
  void InitializePressureSensor() {
    Serial.println(F("BMP280 + Arduino Mega test"));

    if (!bmp.begin(BMP280_I2C_ADDRESS)) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
      while (1) {
        delay(10);
      }
    }

    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,    // temperature
      Adafruit_BMP280::SAMPLING_X16,   // pressure
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500
    );

    Serial.println(F("BMP280 initialized."));
  }
};

GreenStar cansat;

void setup() {
  Serial.begin(9600);
  // For boards with native USB (e.g. Leonardo, Micro, etc.)
  while (!Serial) {
    ; // wait for serial port to connect
  }

  Wire.begin();       // Good practice for I2C
  delay(100);         // Small delay
  cansat.begin();     // Now it is safe to initialize sensors
}

void loop() {
  cansat.readTemperature();
  cansat.readPressure();
  cansat.saveDataToSD();
  delay(1000);
}