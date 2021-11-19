#include "VescUart.h"
#include "trajectory.h"

VescUart shoulder;
VescUart elbow;

VescUart::dataPackage shoulder_telemetry[trajectory_length];
VescUart::dataPackage elbow_telemetry[trajectory_length];


void setup() {
  // Setup PC serial port
  Serial.begin(115200);

  // Setup VESC serial ports
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  while (!Serial) {;}

  // Define which ports to use as UART
  shoulder.setSerialPort(&Serial1);
  elbow.setSerialPort(&Serial2);
}

void loop() {
  // send trajectory, obtain telemetry
  Serial.println("Executing trajectory...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    shoulder.getVescValues();
    elbow.getVescValues();

    shoulder_telemetry[i] = shoulder.data;
    elbow_telemetry[i] = elbow.data;

    shoulder.setCurrent(shoulder_trajectory[i]);
    elbow.setCurrent(elbow_trajectory[i]);
    delay(100);
  }

  // hey babe, send telemetry ;)
  VescUart::dataPackage data; 
  Serial.println("Trajectory complete. Dumping data...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    data = shoulder_telemetry[i];
    Serial.print(i);
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.print(data.dutyCycleNow);

    data = elbow_telemetry[i];
    Serial.print(i);
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.println(data.dutyCycleNow);
  }

  Serial.println("Trajectroy sent. Breaking.");
  while(1);
}
