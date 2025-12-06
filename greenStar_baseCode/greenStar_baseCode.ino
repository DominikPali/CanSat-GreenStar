#include <OneWire.h>
#include <DallasTemperature.h>
#include <CanSatKit.h>

// Pin where the DS18B20 data line is connected
#define ONE_WIRE_BUS 2

using namespace CanSatKit;
BMP280 bmp;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

class GreenStar{
  public:
    double temperatureIn, temperatureOut, pressure;
    float latitude, longitude, altitude;
    bool descent = false;

    GreenStar(){
      SerialUSB.begin(9600);
      initializeSensors();
    }

    void readTemperature(){
      sensors.requestTemperatures();
      temperatureOut = sensors.getTempCByIndex(0);
    }
    
    void readPressure(){
      bmp.measureTemperatureAndPressure(temperatureIn, pressure);
    }

    void saveDataToSD(){
      SerialUSB.print("Temperature out: ");
      SerialUSB.print(temperatureOut);
      SerialUSB.println(" °C");

      SerialUSB.print("Pressure: ");
      SerialUSB.print(pressure);
      SerialUSB.println(" hPa");

      SerialUSB.print("Temperature in: ");
      SerialUSB.print(temperatureIn);
      SerialUSB.println(" °C");
    }

    private:
      void initializeSensors(){
        sensors.begin();
        bmp.setOversampling(16);
      }
  
};

//Initialization of GreenStar statelite
GreenStar cansat;
void setup() {
  if(!bmp.begin()) {
          // if connection failed - print message to the user
          //SerialUSB.println("BMP init failed!");
          // the program will be 'halted' here and nothing will happen till restart
          while(1);
        }
  else {
  // print message to the user if everything is OK
  SerialUSB.println("BMP init success!");
  }
}

void loop() {
  cansat.readTemperature();
  cansat.readPressure();
  cansat.saveDataToSD();
  delay(1000);
}





