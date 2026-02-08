#include <SPI.h>
#include <RadioLib.h>

// === CONFIGURE PINS FOR YOUR LORA MODULE ===
// Example for SX1278 / RFM95 style
#define LORA_CS    10   // LoRa chip select
#define LORA_DIO0   2   // LoRa interrupt
#define LORA_RST    9   // LoRa reset

SX1278 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST);

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB) delay(10);

  SerialUSB.println("Serial ready, initializing LoRa...");

  // begin with freq 433MHz, BW=125kHz, SF=9, CR=4/8
  int state = radio.begin(433.0, 125.0, 9, 7, RADIOLIB_SX127X_SYNC_WORD, 10);
  if (state != RADIOLIB_ERR_NONE) {
    SerialUSB.print("Radio init failed, code ");
    SerialUSB.println(state);
    while (true) delay(100);
  }

  SerialUSB.println("LoRa receiver initialized.");
}

void loop() {
  String packet;
  
  // Try to receive (blocking)
  int state = radio.receive(packet);

  if (state == RADIOLIB_ERR_NONE) {
    SerialUSB.println("=== PACKET RECEIVED ===");
    SerialUSB.print("Data: ");
    SerialUSB.println(packet);

    SerialUSB.print("RSSI: ");
    SerialUSB.print(radio.getRSSI());
    SerialUSB.println(" dBm");

    SerialUSB.print("SNR: ");
    SerialUSB.print(radio.getSNR());
    SerialUSB.println(" dB");

    SerialUSB.println("------------------------");
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // No packet received within timeout
    // You can comment this out if it clutters Serial
    SerialUSB.println("RX timeout (no packet)");
  }
  else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    SerialUSB.println("CRC error (bad packet)");
  }
  else {
    SerialUSB.print("Receive failed, code ");
    SerialUSB.println(state);
  }

  delay(100); 
}
