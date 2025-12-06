#include <OneWire.h>
#include <DallasTemperature.h>
#include <CanSatKit.h>

// Pin where the DS18B20 data line is connected
#define ONE_WIRE_BUS 2
#define GPS_SERIAL Serial

using namespace CanSatKit;
BMP280 bmp;
String nmea = "";

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

class GreenStar{
  public:
    double temperatureIn, temperatureOut, pressure;
    float latitude, longitude, altitude;
    bool descent = false;

    GreenStar(){
    }

    void readTemperature(){
      sensors.requestTemperatures();
      temperatureOut = sensors.getTempCByIndex(0);
    }
    
    void readPressure(){
      bmp.measureTemperatureAndPressure(temperatureIn, pressure);
    }

    void readGPS(){
      while (GPS_SERIAL.available()) {
        char c = GPS_SERIAL.read();

        if (c == '\n') {
          processNMEA(nmea);
          nmea = "";
        } 
        else if (c != '\r') {
          nmea += c;
        }
      }
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

      SerialUSB.print("Lat: ");
      SerialUSB.println(latitude, 6);
      SerialUSB.print("Lon: ");
      SerialUSB.println(longitude, 6);
    }

    void initializeSensors(){
        sensors.begin();
        bmp.setOversampling(16);
    }

    private:
      void processNMEA(String line) {

        // Only use GGA or RMC (both contain lat/lon)
        if (!line.startsWith("$GNGGA") && !line.startsWith("$GNRMC"))
          return;

        // Split fields by comma
        String fields[20];
        int idx = 0;
        int last = 0;

        for (int i = 0; i < line.length(); i++) {
          if (line[i] == ',' || line[i] == '*') {
            fields[idx++] = line.substring(last, i);
            last = i + 1;
          }
        }

        // Extract fields
        String latStr = fields[3];
        String latDir = fields[4];
        String lonStr = fields[5];
        String lonDir = fields[6];

        if (latStr.length() < 3 || lonStr.length() < 3)
          return;

        latitude = convert(latStr);
        longitude = convert(lonStr);

        if (latDir == "S") latitude = -latitude;
        if (lonDir == "W") longitude = -longitude;
      }

      // Convert ddmm.mmmm → decimal degrees
      double convert(String raw) {
        double val = raw.toFloat();
        int deg = (int)(val / 100);
        double minutes = val - (deg * 100);
        return deg + (minutes / 60.0);
      }

      
  
};

//Initialization of GreenStar statelite
GreenStar cansat;
void setup() {

  SerialUSB.begin(9600);
  GPS_SERIAL.begin(9600);
  
  if(!bmp.begin()) {
          // if connection failed - print message to the user
          //SerialUSB.println("BMP init failed!");
          // the program will be 'halted' here and nothing will happen till restart
          while(1);
        }
  else {
  // print message to the user if everything is OK
  cansat.initializeSensors();
  SerialUSB.println("BMP init success!");
  }
}

void loop() {
  cansat.readTemperature();
  cansat.readPressure();
  cansat.readGPS();
  cansat.saveDataToSD();
  delay(1000);
}