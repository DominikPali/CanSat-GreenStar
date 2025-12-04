#include <OneWire.h>
#include <DallasTemperature.h>
#include <CanSatKit.h>
#define ONE_WIRE_BUS 2
 
 
using namespace CanSatKit;
BMP280 bmp;
// Pin where the DS18B20 data line is connected
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
 
 
 
class GreenStar {
 
public:
 
  double temperatureIn, temperatureOut;
  double pressure;
  float latitude, longitude, altitude;
  bool descent = false;
 
 
  GreenStar() {
    SerialUSB.println("START ");
    SerialUSB.begin(9600);
    InitializePressureSensor();
  }
 
  void readTemperature(){
    sensors.requestTemperatures();
    temperatureOut = sensors.getTempCByIndex(0);
  }
 
  void readPressure(){
    bmp.measureTemperatureAndPressure(temperatureIn, pressure);
 
  }
 
  void readGPS(){
 
  }
 
  void saveDataToSD() {
    SerialUSB.println(pressure, 2);
    SerialUSB.println(temperatureOut, 2);
    SerialUSB.println(temperatureIn, 2);
  }
 
  void sendDataToGround(){
 
  }
 
  void checkForDescent(){
 
  }
 
  void checkForDesiredAltitude(){
 
  }
 
 
private:
 
 
  void InitializePressureSensor()
  {
      if(!bmp.begin()) {
        // if connection failed - print message to the user
        SerialUSB.println("BMP init failed!");
        // the program will be 'halted' here and nothing will happen till restart
        while(1);
      } else {
        // print message to the user if everything is OK
        SerialUSB.println("BMP init success!");
      }
    // setOversampling function allows to set resolution of the pressure sensor
    // possible values of setOversampling:
    //  1 -- 2.62 Pa
    //  2 -- 1.31 Pa
    //  4 -- 0.66 Pa
    //  8 -- 0.33 Pa
    // 16 -- 0.16 Pa
    bmp.setOversampling(16);
 
    sensors.begin();
  }
};
 
 
GreenStar cansat;
 
 
void setup() {
}
 
void loop() {
  cansat.readTemperature();
  cansat.readPressure();
  cansat.saveDataToSD();
 
  delay(1000);
}