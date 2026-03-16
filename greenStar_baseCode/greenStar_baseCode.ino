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

// DS18B20 external temperature sensor data line
#define ONE_WIRE_BUS 2

// GPS is connected to the hardware UART (Serial) on the SAMD21.
// On the CanSat Kit, Serial uses pins 0 (RX) and 1 (TX).
// IMPORTANT: Do NOT use pins 0 or 1 for anything else (e.g. LEDs).
#define GPS_SERIAL Serial

// RGB LED pins — common-anode wiring means LOW turns the colour ON.
// FIX #15: Moved LED pins away from 1 and 3 to avoid conflict with
// Serial (pin 0/1) and potential SERCOM issues on pin 3.
// New pins: 4, 6, 8 — all are safe GPIOs with no UART conflict.
#define LED_R 13
#define LED_G 6
#define LED_B 8

// Servo motor control pin (PWM-capable)
#define SERVO_PIN 7

// SD card chip-select pin
const int chipSelect = 11;

// ============================================================================
// LIBRARY NAMESPACE
// ============================================================================
using namespace CanSatKit;

// ============================================================================
// GLOBAL SENSOR OBJECTS
// ============================================================================

// BMP280 pressure + internal temperature sensor (from CanSatKit library)
BMP280 bmp;

// DS18B20 external temperature sensor (OneWire protocol)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dsSensors(&oneWire);

// SD card log file handle
File logFile;

// Servo motor object for seed-dispersal mechanism
Servo containerServo;

// ============================================================================
// SERVO STATE MACHINE
// ============================================================================
// The servo opens the seed container.  We track its state with booleans
// so that the opening sequence runs exactly once, non-blocking.

bool openContainer  = false;   // true when deployment conditions are met
bool servoDone      = false;   // true after servo has finished its sweep
bool servoMoving    = false;   // true while the servo is actively sweeping
unsigned long servoStartTime = 0;
int servoSweepDuration = 3000; // time for 0° → 180° sweep (ms) — tuneable

// ============================================================================
// TEST MODE
// ============================================================================
// When 'test' is true the servo will activate 10 seconds after boot,
// bypassing the flight-phase checks.  Set to false for actual flight!

bool test = true;

// ============================================================================
// LOOP TIMING (non-blocking)
// ============================================================================

unsigned long previousLoopTime  = 0;

// FIX #22: Faster sample rate.
// BMP280 at 16× oversampling takes ~38 ms per measurement.
// DS18B20 at default 12-bit resolution takes ~750 ms.
// Radio TX takes ~50-100 ms depending on payload.
// A 500 ms interval is achievable if we keep DS18B20 reads at lower
// resolution or alternate reads.  We use 500 ms as a compromise:
// fast enough for accurate deployment, slow enough for all sensors.
const unsigned long loopInterval = 500; // 500 ms between sensor reads

// LED blink timing (for GPS-no-fix indication)
bool ledBlinkOn = false;
unsigned long previousBlinkTime = 0;
const unsigned long blinkInterval = 500; // 500 ms on / 500 ms off

// ============================================================================
// ALTITUDE RING-BUFFER PARAMETERS
// ============================================================================
// These constants control the descent/ascent detection algorithm.
// A ring buffer of N_SAMPLES altitude readings is maintained.  Between
// consecutive samples we compute vertical velocity.  If enough samples
// show velocity consistent with descent (or ascent), the flag is set.

static const int   ALT_RING_SIZE         = 6;    // ring-buffer capacity (>= 3)
static const int   K_DESCENT             = 3;    // votes needed to confirm descent
static const int   K_ASCENT              = 3;    // votes needed to confirm ascent
static const float EXPECTED_FALL_VELOCITY = 2.0f; // m/s (positive = downward)
static const float VEL_MARGIN            = 0.4f;  // noise margin (m/s)
static const float MIN_DT_S              = 0.2f;  // ignore tiny dt (glitches)

// Rocket ascent detection: expected upward velocity.
// Typical hobby rockets reach 30-80 m/s; even at low thrust, > 5 m/s.
static const float EXPECTED_ROCKET_VELOCITY = 10.0f; // m/s upward
static const float ROCKET_VEL_MARGIN        = 3.0f;  // noise margin

// Deployment altitude threshold (meters above ground level)
static const float DEPLOY_ALTITUDE_THRESHOLD = 100.0f;

// ============================================================================
// WATCHDOG TIMER (FIX #23)
// ============================================================================
// The SAMD21 has a hardware Watchdog Timer (WDT).  If the main loop
// hangs (e.g. I2C lockup, SD stall), the WDT will reset the MCU after
// the configured timeout, restoring operation automatically.
//
// How it works:
//   1. In setup(), we configure the WDT with a timeout (~4 seconds).
//   2. Every iteration of loop(), we "pet" (reset) the WDT counter.
//   3. If loop() ever stalls for longer than ~4 s, the WDT fires a
//      system reset — the MCU reboots and setup() runs again.
//   4. After reboot, the SD log file is reopened in append mode,
//      so data continuity is preserved.
//
// The WDT uses the SAMD21's internal ultra-low-power 32 kHz oscillator
// (OSCULP32K) as its clock source, independent of the main CPU clock.
// This means it keeps ticking even if the CPU is stuck.

// Helper: set up the WDT clock source and enable it
void wdt_init(uint8_t period) {
  // ---------- Clock setup for WDT ----------
  // Use Generic Clock Generator 2 with the internal 32 kHz oscillator
  // as the source for the WDT peripheral.

  // Configure GCLK2 to use OSCULP32K (ultra-low-power 32 kHz)
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2)
                     | GCLK_GENCTRL_GENEN
                     | GCLK_GENCTRL_SRC_OSCULP32K
                     | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Route GCLK2 to the WDT peripheral
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT
                     | GCLK_CLKCTRL_CLKEN
                     | GCLK_CLKCTRL_GEN_GCLK2;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // ---------- WDT configuration ----------
  // Disable WDT first (required before changing settings)
  WDT->CTRL.reg = 0;
  while (WDT->STATUS.bit.SYNCBUSY);

  // Set the timeout period
  WDT->CONFIG.reg = period;

  // Enable the WDT
  WDT->CTRL.reg = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);
}

// Helper: "pet" the watchdog — must be called regularly to prevent reset
void wdt_reset() {
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;  // Magic value 0xA5
  while (WDT->STATUS.bit.SYNCBUSY);
}

// ============================================================================
// LED HELPER  (FIX #19: use const char* instead of String)
// ============================================================================
// Common-anode RGB LED: setting a pin LOW turns that colour ON.
// Accepts colour names as C-strings to avoid heap-fragmenting String objects.

void setLED(const char* color) {
  // Default: all off
  bool r = HIGH, g = HIGH, b = HIGH;

  if      (strcmp(color, "red")    == 0) { r = LOW; }
  else if (strcmp(color, "green")  == 0) { g = LOW; }
  else if (strcmp(color, "blue")   == 0) { b = LOW; }
  else if (strcmp(color, "yellow") == 0) { r = LOW; g = LOW; }
  else if (strcmp(color, "purple") == 0) { r = LOW; b = LOW; }
  else if (strcmp(color, "cyan")   == 0) { g = LOW; b = LOW; }
  // "off" or any unrecognised string → all HIGH (LED off)

  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

// ============================================================================
// NMEA CHECKSUM VALIDATOR (FIX #20)
// ============================================================================
// NMEA sentences end with *XX where XX is a two-digit hexadecimal checksum.
// The checksum is the XOR of all characters between '$' and '*' (exclusive).
// Returns true if the checksum is valid or if no checksum is present
// (some truncated sentences).

bool validateNMEAChecksum(const char* sentence) {
  // Sentence must start with '$'
  if (sentence[0] != '$') return false;

  uint8_t calculated = 0;
  int i = 1; // skip the leading '$'

  // XOR every character until we hit '*' or end-of-string
  while (sentence[i] != '\0' && sentence[i] != '*') {
    calculated ^= (uint8_t)sentence[i];
    i++;
  }

  // If we hit end-of-string without finding '*', skip validation
  // (sentence was truncated — let the parser try anyway)
  if (sentence[i] != '*') return true;

  // Parse the two hex digits after '*'
  char hexStr[3] = { sentence[i + 1], sentence[i + 2], '\0' };
  uint8_t expected = (uint8_t)strtol(hexStr, NULL, 16);

  return (calculated == expected);
}

// ============================================================================
// MAIN CANSAT CLASS
// ============================================================================

class GreenStar {
public:
  // --- Sensor readings ---
  double temperatureIn;   // BMP280 internal temperature (°C)
  double temperatureOut;  // DS18B20 external temperature (°C)
  double pressure;        // BMP280 atmospheric pressure (hPa)

  // --- GPS data (FIX #21: use double for better precision) ---
  double latitude;
  double longitude;
  double gpsAltitude;     // FIX #12: altitude from GPS GGA sentence (m MSL)
  bool   gpsFixed;        // true when GPS has a valid position fix

  // --- Flight phase flags ---
  bool descent   = false;  // true when CanSat is descending
  bool flight_up = false;  // true when rocket is ascending (FIX #8)

  // --- Barometric altitude ---
  // P_ground_ref: reference pressure at ground level (hPa).
  // This should be calibrated on launch day for accurate AGL readings.
  // For now it is set to a typical sea-level value; it will be updated
  // before the actual mission.
  const float P_ground_ref = 1004.56f;
  float currentHeightAG;  // computed altitude above ground (m)

  // --- Altitude ring buffer for descent/ascent detection ---
  // FIX #1: added t_buffer[] for timing alongside alt_buffer[]
  // FIX #3: removed duplicate N_SAMPLES; use ALT_RING_SIZE from globals
  float         alt_buffer[ALT_RING_SIZE];
  unsigned long t_buffer[ALT_RING_SIZE];
  int  buf_index   = 0;
  bool buffer_full = false;

private:
  // NMEA parsing buffer
  static const int NMEA_BUF_SIZE = 120;
  char nmeaBuf[NMEA_BUF_SIZE];
  int  nmeaLen;

  // LoRa radio (CanSatKit library)
  Radio radio = Radio(Pins::Radio::ChipSelect,
                      Pins::Radio::DIO0,
                      433.0,
                      Bandwidth_125000_Hz,
                      SpreadingFactor_9,
                      CodingRate_4_8);
  Frame frame;

public:
  // ---- Constructor ----
  GreenStar() {
    nmeaLen      = 0;
    latitude     = 0.0;
    longitude    = 0.0;
    gpsAltitude  = 0.0;
    gpsFixed     = false;
    temperatureIn  = 0.0;
    temperatureOut = 0.0;
    pressure       = 0.0;
    currentHeightAG = 0.0;

    // Zero-fill ring buffers
    for (int i = 0; i < ALT_RING_SIZE; i++) {
      alt_buffer[i] = 0.0f;
      t_buffer[i]   = 0;
    }
  }

  // ------------------------------------------------------------------
  // Sensor Initialisation
  // ------------------------------------------------------------------
  void initializeSensors() {
    dsSensors.begin();

    // Set DS18B20 to 10-bit resolution (~187 ms conversion)
    // instead of default 12-bit (~750 ms).  This allows faster reads
    // while still providing 0.25°C resolution — sufficient for our mission.
    dsSensors.setResolution(10);

    bmp.setOversampling(16);
  }

  // ------------------------------------------------------------------
  // Radio Initialisation
  // ------------------------------------------------------------------
  void initializeRadio() {
    radio.begin();
  }

  // ------------------------------------------------------------------
  // Read DS18B20 external temperature
  // ------------------------------------------------------------------
  void readTemperatureOut() {
    dsSensors.requestTemperatures();
    temperatureOut = dsSensors.getTempCByIndex(0);
  }

  // ------------------------------------------------------------------
  // Read BMP280 internal temperature and pressure
  // ------------------------------------------------------------------
  void readTemperatureInAndPressure() {
    bmp.measureTemperatureAndPressure(temperatureIn, pressure);
  }

  // ------------------------------------------------------------------
  // Compute barometric altitude above ground level
  // Uses the hypsometric formula:
  //   h = 44330 × (1 − (P / P_ref)^0.1903)
  // where P_ref is the pressure at ground level.
  // ------------------------------------------------------------------
  void altitude_from_pressure() {
    currentHeightAG = 44330.0f * (1.0f - powf(pressure / P_ground_ref, 0.1903f));
  }

  // ------------------------------------------------------------------
  // FIX #4/#5/#8: Update altitude ring buffer & detect flight phase
  // ------------------------------------------------------------------
  // This function:
  //   1. Stores the current altitude + timestamp in the ring buffer.
  //   2. Once the buffer is full, computes velocity between consecutive
  //      samples and counts "ascent votes" and "descent votes".
  //   3. Sets flight_up = true if the rocket is climbing fast enough.
  //   4. Sets descent = true if the CanSat is falling at the expected rate.
  //
  // ASCENT detection uses EXPECTED_ROCKET_VELOCITY: the rocket climbs
  // much faster than the CanSat descends, so we can distinguish the two.
  // DESCENT detection uses EXPECTED_FALL_VELOCITY: the parachute descent
  // rate, typically ~2 m/s for CanSat missions.

  void updateAltitudeSampleAndCheckPhase() {
    unsigned long t_now = millis();

    // Write new sample into ring buffer
    alt_buffer[buf_index] = currentHeightAG;
    t_buffer[buf_index]   = t_now;

    // Advance the write pointer; wrap around when full
    buf_index++;
    if (buf_index >= ALT_RING_SIZE) {
      buf_index = 0;
      buffer_full = true;
    }

    // Need a full buffer before we can make reliable decisions
    if (!buffer_full) {
      descent   = false;
      flight_up = false;
      return;
    }

    // ---- Analyse velocity between consecutive samples ----
    // buf_index now points to the OLDEST element (next write position).
    int start = buf_index;

    int descent_votes = 0;
    int ascent_votes  = 0;

    for (int i = 1; i < ALT_RING_SIZE; i++) {
      int idx_prev = (start + i - 1) % ALT_RING_SIZE;
      int idx_cur  = (start + i)     % ALT_RING_SIZE;

      float dh = alt_buffer[idx_cur] - alt_buffer[idx_prev]; // metres
      float dt = (t_buffer[idx_cur]  - t_buffer[idx_prev]) / 1000.0f; // seconds

      // Skip pairs with suspiciously small time gaps (timer glitches)
      if (dt < MIN_DT_S) continue;

      float v = dh / dt; // m/s: positive = going UP, negative = going DOWN

      // --- Descent vote ---
      // v is negative when falling; check if magnitude exceeds threshold
      if (v < -(EXPECTED_FALL_VELOCITY - VEL_MARGIN)) {
        descent_votes++;
      }

      // --- Ascent vote (FIX #8) ---
      // v is positive when climbing; check if magnitude exceeds rocket threshold
      if (v > (EXPECTED_ROCKET_VELOCITY - ROCKET_VEL_MARGIN)) {
        ascent_votes++;
      }
    }

    descent   = (descent_votes >= K_DESCENT);
    flight_up = (ascent_votes  >= K_ASCENT);
  }

  // ------------------------------------------------------------------
  // FIX #10/#11: GPS Reader — runs every loop() iteration
  // ------------------------------------------------------------------
  // Reads all available bytes from the GPS serial buffer and assembles
  // them into complete NMEA sentences.  This must be called as often
  // as possible to prevent the 64-byte SAMD21 serial buffer from
  // overflowing and losing data.
  //
  // FIX #11: SparkFun SAM-M10Q (u-blox M10) GPS Module Notes
  // =========================================================
  // The SAM-M10Q is a multi-GNSS receiver that concurrently tracks
  // GPS, GLONASS, Galileo, and BeiDou.  According to the u-blox M10
  // interface description (UBX-20053845):
  //
  //   - Default main Talker ID is "GN" (multi-GNSS).
  //   - Sentences: $GNGGA, $GNRMC, $GNGLL, $GNVTG, $GNGSA, etc.
  //   - However, u-blox also supports $GPGGA if the talker ID is changed
  //     via setMainTalkerID(), or if only GPS constellation is enabled.
  //   - GSV messages use constellation-specific IDs: $GPGSV, $GLGSV, etc.
  //
  // To be robust against configuration changes, we match the sentence
  // TYPE (GGA, RMC) regardless of the two-letter talker prefix.
  // This handles $GNGGA, $GPGGA, $GLGGA, $GBGGA, $GAGGA, etc.
  //
  // The module's default UART baud rate is 9600 at 1 Hz update.
  // At 9600 baud, ~960 bytes/s can be received.  A full NMEA cycle
  // (GGA+RMC+GSA+GSV etc.) is typically 400-600 bytes, so 1 Hz is fine.

  void readGPS() {
    while (GPS_SERIAL.available()) {
      char c = GPS_SERIAL.read();

      if (c == '\n') {
        // End of sentence — null-terminate and process
        nmeaBuf[nmeaLen] = '\0';

        // FIX #20: Only parse sentences with valid checksum
        if (validateNMEAChecksum(nmeaBuf)) {
          processNMEALine(nmeaBuf);
        }

        nmeaLen = 0;
      }
      else if (c != '\r') {
        // Accumulate characters (skip carriage returns)
        if (nmeaLen < NMEA_BUF_SIZE - 1) {
          nmeaBuf[nmeaLen++] = c;
        } else {
          // Buffer overflow — discard this sentence
          nmeaLen = 0;
        }
      }
    }
  }

  // ------------------------------------------------------------------
  // FIX #9: Log data to SD card
  // Now includes all fields: flight_up, servoDone, servoMoving, openContainer
  // ------------------------------------------------------------------
  void logData() {
    if (!logFile) return;

    unsigned long ms = millis();

    // CSV format: timestamp,tempIn,tempOut,pressure,lat,lon,altAGL,gpsAlt,
    //             descent,flight_up,servoDone,servoMoving,openContainer
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

    // --- Debug output to SerialUSB (visible via USB cable) ---
    SerialUSB.print("T_out: ");   SerialUSB.print(temperatureOut);
    SerialUSB.print(" | P: ");    SerialUSB.print(pressure);
    SerialUSB.print(" | T_in: "); SerialUSB.print(temperatureIn);
    SerialUSB.print(" | Lat: ");  SerialUSB.print(latitude, 6);
    SerialUSB.print(" | Lon: ");  SerialUSB.print(longitude, 6);
    SerialUSB.print(" | hAGL: "); SerialUSB.print(currentHeightAG, 2);
    SerialUSB.print(" | GPS_alt: "); SerialUSB.print(gpsAltitude, 2);
    SerialUSB.print(" | desc: "); SerialUSB.print(descent ? "Y" : "N");
    SerialUSB.print(" | up: ");   SerialUSB.print(flight_up ? "Y" : "N");
    SerialUSB.print(" | servo: ");
    SerialUSB.println(servoDone ? "DONE" : (servoMoving ? "MOVING" : "WAIT"));

    // Print altitude ring buffer for debugging
    SerialUSB.print("  alt_buf: ");
    for (int i = 0; i < ALT_RING_SIZE; i++) {
      SerialUSB.print(alt_buffer[i], 2);
      SerialUSB.print(" ");
    }
    SerialUSB.println();
  }

  // ------------------------------------------------------------------
  // FIX #7/#2: Send telemetry via LoRa radio
  // Added comma separators between ALL fields and fixed missing semicolon.
  // ------------------------------------------------------------------
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
    frame.print(descent ? 1 : 0);     frame.print(',');  // FIX #7: comma added
    frame.print(flight_up ? 1 : 0);   frame.print(',');  // FIX #7: comma added
    frame.print(servoDone ? 1 : 0);   frame.print(',');  // FIX #7: comma added
    frame.print(servoMoving ? 1 : 0); frame.print(',');  // FIX #7: comma added
    frame.print(openContainer ? 1 : 0);                  // FIX #2: semicolon added

    radio.transmit(frame);

    SerialUSB.print("RADIO TX: ");
    SerialUSB.println(frame);

    frame.clear();
  }

private:
  // ------------------------------------------------------------------
  // FIX #11: NMEA sentence processor — handles any talker ID
  // ------------------------------------------------------------------
  // Instead of checking for exact prefixes like "$GNGGA", we now look
  // at the 3rd-5th characters of the sentence to match the sentence
  // TYPE: "GGA" or "RMC".  This handles all possible talker IDs:
  //   $GNGGA, $GPGGA, $GLGGA, $GBGGA, $GAGGA  (GGA variants)
  //   $GNRMC, $GPRMC, $GLRMC                   (RMC variants)
  //
  // NMEA sentence format reference:
  //   Position 0:   '$'
  //   Position 1-2: Talker ID  (GP, GN, GL, GA, GB, etc.)
  //   Position 3-5: Sentence type (GGA, RMC, GSA, GSV, etc.)
  //
  // GGA sentence structure (fields separated by commas):
  //   $--GGA,hhmmss.ss,llll.lll,a,yyyyy.yyy,a,x,xx,x.x,x.x,M,x.x,M,,*hh
  //   Field 0:  Sentence ID ($GNGGA)
  //   Field 1:  UTC time (hhmmss.ss)
  //   Field 2:  Latitude (ddmm.mmmm)
  //   Field 3:  N/S indicator
  //   Field 4:  Longitude (dddmm.mmmm)
  //   Field 5:  E/W indicator
  //   Field 6:  Fix quality (0=invalid, 1=GPS, 2=DGPS, ...)
  //   Field 7:  Number of satellites
  //   Field 8:  HDOP
  //   Field 9:  Altitude above MSL (metres)  ← FIX #12: now extracted
  //   Field 10: Altitude units (M)
  //   Field 11: Geoid separation
  //   ...
  //
  // RMC sentence structure:
  //   $--RMC,hhmmss.ss,A,llll.lll,a,yyyyy.yyy,a,x.x,x.x,ddmmyy,x.x,a*hh
  //   Field 2: Latitude
  //   Field 3: N/S
  //   Field 4: Longitude
  //   Field 5: E/W

  void processNMEALine(const char* line) {
    // Minimum valid sentence: "$XXYYY,..." = at least 7 chars
    int lineLen = strlen(line);
    if (lineLen < 7) return;

    // Must start with '$'
    if (line[0] != '$') return;

    // Extract sentence type from positions 3-5
    char type[4];
    type[0] = line[3];
    type[1] = line[4];
    type[2] = line[5];
    type[3] = '\0';

    bool isGGA = (strcmp(type, "GGA") == 0);
    bool isRMC = (strcmp(type, "RMC") == 0);

    if (!isGGA && !isRMC) return; // not a sentence we care about

    // --- Split the sentence into comma-separated fields ---
    char buf[NMEA_BUF_SIZE];
    strncpy(buf, line, NMEA_BUF_SIZE);
    buf[NMEA_BUF_SIZE - 1] = '\0';

    char* fields[20];
    int fieldCount = 0;
    char* p = buf;
    fields[fieldCount++] = p;
    while (*p && fieldCount < 20) {
      if (*p == ',') {
        *p = '\0';
        fields[fieldCount++] = p + 1;
      }
      p++;
    }

    // --- Parse GGA sentence ---
    if (isGGA) {
      // Need at least 10 fields to extract lat/lon and altitude
      // Fields: 0=type, 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W,
      //         6=quality, 7=numSats, 8=HDOP, 9=altitude, 10=altUnit
      if (fieldCount > 10 && strlen(fields[2]) >= 4 && strlen(fields[4]) >= 4) {

        // Check fix quality (field 6): 0 = no fix
        int fixQuality = atoi(fields[6]);
        if (fixQuality == 0) {
          gpsFixed = false;
          return;
        }

        // Parse latitude: format is ddmm.mmmm
        double lat = convertNMEADeg(fields[2]);
        if (fields[3][0] == 'S') lat = -lat;

        // Parse longitude: format is dddmm.mmmm
        double lon = convertNMEADeg(fields[4]);
        if (fields[5][0] == 'W') lon = -lon;

        latitude  = lat;
        longitude = lon;
        gpsFixed  = true;

        // FIX #12: Extract GPS altitude (field 9) — metres above MSL
        if (strlen(fields[9]) > 0) {
          gpsAltitude = atof(fields[9]);
        }
      }
    }

    // --- Parse RMC sentence ---
    else if (isRMC) {
      // RMC fields: 0=type, 1=time, 2=status, 3=lat, 4=N/S, 5=lon, 6=E/W
      // Status (field 2): 'A' = active/valid, 'V' = void/invalid
      if (fieldCount > 6 && strlen(fields[3]) >= 4 && strlen(fields[5]) >= 4) {

        // Check status
        if (fields[2][0] != 'A') {
          // Data is void — don't update position
          return;
        }

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

  // ------------------------------------------------------------------
  // FIX #21: NMEA coordinate conversion — uses double for precision
  // ------------------------------------------------------------------
  // NMEA format:  ddmm.mmmm (lat) or dddmm.mmmm (lon)
  // Converts to decimal degrees:  dd.dddddd
  //
  // Using double (~15 significant digits) instead of float (~7 digits)
  // avoids precision loss.  For longitude like 02101.1234, float would
  // only give ~5 decimal-degree digits; double gives the full ~10.

  double convertNMEADeg(const char* raw) {
    double v = atof(raw);           // e.g. 5213.1234 → 5213.1234
    int deg = (int)(v / 100);       // e.g. 52
    double minutes = v - (deg * 100); // e.g. 13.1234
    return deg + (minutes / 60.0);  // e.g. 52.218723...
  }
};

// ============================================================================