#include <CanSatKit.h>

using namespace CanSatKit;

// Radio configuration (must match transmitter)
Radio radio(
  Pins::Radio::ChipSelect,
  Pins::Radio::DIO0,
  433.0,
  Bandwidth_125000_Hz,
  SpreadingFactor_9,
  CodingRate_4_8
);

// Frame to store received data
Frame frame;

void setup() {
  SerialUSB.begin(115200);

  radio.begin();
  SerialUSB.println("Ground station ready. Listening...");
}

void loop() {
  // Check if a packet was received
  if (radio.receive(frame)) {
    SerialUSB.println("Received:");
    SerialUSB.println(frame);

    // Clear frame for next packet
    frame.clear();
  }
}