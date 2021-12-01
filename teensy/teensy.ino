#include "VescUart.h"
#include "trajectory.h"
#include <Encoder.h>

// th1 is the shoulder, th2 is the elbow

VescUart th1_vesc;
VescUart th2_vesc;

VescUart::dataPackage th1_telemetry[trajectory_length];
VescUart::dataPackage th2_telemetry[trajectory_length];

Encoder th1_encoder(7,8);
Encoder th2_encoder(5,6);

int32_t th1_desired = 0;
int32_t th2_desired = 0;
int32_t previous_th1 = 0;
int32_t previous_th2 = 0; 

int32_t th1 = 0;
int32_t th2 = 0;

float dth1 = 0;
float dth2 = 0;

float k = 0.04; //0.04;
float d = 0;

float th1_current = 0;
float th2_current = 0;

void send_telemetry_to_computer() {
  VescUart::dataPackage data;
  Serial.println("Sending telemetry...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    data = th1_telemetry[i];
    Serial.print(i);
    Serial.print(',');
    Serial.print(data.tachometerAbs);
    Serial.print(',');
    Serial.print(data.rpm);
    Serial.print(',');
    Serial.print(data.inpVoltage);
    Serial.print(',');
    Serial.print(data.dutyCycleNow);

    data = th2_telemetry[i];
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
  Serial.println("Telemetry sent.");
}


void setup() {
  // Setup PC serial port
  Serial.begin(115200);

  // Setup VESC serial ports
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  while (!Serial) {;}

  // Define which ports to use as UART
  th1_vesc.setSerialPort(&Serial1);
  th2_vesc.setSerialPort(&Serial2);
}

void loop() {
  /*
  // send trajectory, obtain telemetry
  Serial.println("Executing trajectory...");
  for(uint16_t i=0; i < trajectory_length; i++) {
    th1_vesc.getVescValues();
    th2_vesc.getVescValues();

    th1_telemetry[i] = th1_vesc.data;
    th2_telemetry[i] = th2_vesc.data;

    th1_vesc.setCurrent(th1_trajectory[i]);
    th2_vesc.setCurrent(th2_trajectory[i]);
    delay(10);
  }
  */

  while(1){
    // log telemetry
    th1_vesc.getVescValues();
    th2_vesc.getVescValues();
    
    // get positions and velocities
    th1 = th1_encoder.read();
    th2 = th2_encoder.read();
    
    dth1 = 0.01*(th1 - previous_th1);
    dth2 = 0.01*(th2 - previous_th2);

    previous_th1 = th1;
    previous_th2 = th2;

    // make impedance controller
    th1_current = k*(th1_desired - th1) + d*(-dth1);
    th2_current = k*(th2_desired - th2) + d*(-dth2);

    if(th1_current >  60) th1_current =  60;
    if(th1_current < -60) th1_current = -60;
    if(th2_current > 60)  th2_current =  60;
    if(th2_current < -60) th2_current = -60;

    th1_vesc.setCurrent(-th1_current);    
    th2_vesc.setCurrent(-th2_current);

    Serial.print("th1:");
    Serial.print(th1);
    Serial.print(",");

    Serial.print("dth1:");
    Serial.print(dth1);
    Serial.print(",");

    Serial.print("th1_current:");
    Serial.println(th1_current);
    delay(10);
  }
  while(1);
}
