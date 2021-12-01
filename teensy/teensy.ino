#include "VescUart.h"
#include "trajectory.h"
#include <Encoder.h>

VescUart shoulder;
VescUart elbow;

VescUart::dataPackage shoulder_telemetry[trajectory_length];
VescUart::dataPackage elbow_telemetry[trajectory_length];

Encoder shoulder_encoder(7,8);
Encoder elbow_encoder(5,6);

int32_t initial_shoulder_angle = 0;
int32_t initial_elbow_angle = 0;
int32_t last_shoulder_angle = 0;
int32_t last_elbow_angle = 0; 

int32_t current_shoulder_angle = 0;
int32_t current_elbow_angle = 0;

float shoulder_velocity = 0;
float elbow_velocity = 0;

float k = 0.04; //0.04;
float d = 1;

float shoulder_current = 0;
float elbow_current = 0;


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
  /*
  // send trajectory, obtain telemetry
  Serial.println("Executing trajectory...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    shoulder.getVescValues();
    elbow.getVescValues();

    shoulder_telemetry[i] = shoulder.data;
    elbow_telemetry[i] = elbow.data;

    shoulder.setCurrent(14*shoulder_trajectory[i]);
    elbow.setCurrent(-14*elbow_trajectory[i]);
    delay(10);
  }

  // hey babe, send telemetry ;)
  VescUart::dataPackage data; 
  Serial.println("Trajectory complete. Dumping data...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    data = shoulder_telemetry[i];
    Serial.print(i);
    Serial.print(',');
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.print(data.dutyCycleNow);

    data = elbow_telemetry[i];
    Serial.print(i);
    Serial.print(',');
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.println(data.dutyCycleNow);
  }

  Serial.println("Trajectroy sent. Breaking.");
  */

  shoulder.getVescValues();
  elbow.getVescValues();
  initial_shoulder_angle = shoulder.data.tachometer;
  initial_elbow_angle = elbow.data.tachometer;
  last_shoulder_angle = initial_shoulder_angle;
  last_elbow_angle = initial_elbow_angle; 

  current_shoulder_angle = initial_shoulder_angle;
  current_elbow_angle = initial_elbow_angle;
  //for(uint16_t i=0; i < 300; i++){
  while(1){
    // log telemetry
    shoulder.getVescValues();
    elbow.getVescValues();

    //shoulder_telemetry[i] = shoulder.data;
    //elbow_telemetry[i] = elbow.data;
    
    // get positions and velocities
    current_shoulder_angle = shoulder_encoder.read();
    current_elbow_angle = elbow_encoder.read();
    
    shoulder_velocity = 0.01*(current_shoulder_angle - last_shoulder_angle);
    elbow_velocity    = 0.01*(current_elbow_angle - last_elbow_angle);

    last_shoulder_angle = current_shoulder_angle;
    last_elbow_angle = current_elbow_angle;

    // make impedance controller
    shoulder_current = k*(initial_shoulder_angle - current_shoulder_angle) + d*(-shoulder_velocity);
    elbow_current = k*(initial_elbow_angle - current_elbow_angle) + d*(-elbow_velocity);

    if(shoulder_current > 70) shoulder_current = 70;
    if(shoulder_current < -70) shoulder_current = -70;
    if(elbow_current > 70) elbow_current = 70;
    if(elbow_current < -70) elbow_current = -70;

    shoulder.setCurrent(-shoulder_current);    
    elbow.setCurrent(-elbow_current);
    Serial.print(initial_shoulder_angle);
    Serial.print(",");
    Serial.print(current_shoulder_angle);
    Serial.print(",");
    Serial.print(shoulder_velocity);
    Serial.print(",");
    Serial.print(shoulder_current);
    Serial.print(",");
    Serial.println(shoulder.data.avgMotorCurrent);
    delay(10);
  }
  while(1);

  /*
  VescUart::dataPackage data;
  Serial.println("Impedance control run complete. Dumping data...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    data = shoulder_telemetry[i];
    Serial.print(i);
    Serial.print(',');
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.print(data.dutyCycleNow);

    data = elbow_telemetry[i];
    Serial.print(i);
    Serial.print(',');
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.println(data.dutyCycleNow);
  }
  */
}
