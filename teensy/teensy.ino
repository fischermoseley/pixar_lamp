#include "VescUart.h"

VescUart UART;
float current = 1;

void setup() {

  // Setup Serial port between PC and Teensy to display data
  Serial.begin(9600);

  // Setup UART port between Teensy and VESC
  Serial1.begin(115200);
  
  while (!Serial) {;}

  // Define which ports to use as UART
  UART.setSerialPort(&Serial1);
}

void loop() {
  
  // Call the function getVescValues() to acquire data from VESC
  if ( UART.getVescValues() ) {
    Serial.println(UART.data.rpm);
    Serial.println(UART.data.inpVoltage);
    Serial.println(UART.data.ampHours);
    Serial.println(UART.data.tachometerAbs);
  }
  
  else {
    Serial.println("Failed to get data!");
  }

  UART.setCurrent(current);
  delay(50);
}
