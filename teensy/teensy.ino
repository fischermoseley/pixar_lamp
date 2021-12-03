#include "VescUart.h"
#include "trajectory.h"
#include <Encoder.h>

// th1 is the shoulder, th2 is the elbow

VescUart th1_vesc;
VescUart th2_vesc;

float current_limit = 60;

VescUart::dataPackage th1_telemetry[dynamic_trajectory_length];
VescUart::dataPackage th2_telemetry[dynamic_trajectory_length];

Encoder th1_encoder(7,8);
Encoder th2_encoder(5,6);

float th1_desired = 0;
float th2_desired = 0;
float th1 = 0;
float th2 = 0;
float previous_th1 = 0;
float previous_th2 = 0;

float dth1_desired = 0;
float dth2_desired = 0;
float dth1 = 0;
float dth2 = 0;

float th1_current = 0;
float th2_current = 0;

float k = 40;// 0.5;
float d = 0.7; // 0.07

void send_telemetry_to_computer() {
  VescUart::dataPackage data;
  Serial.println("Sending telemetry...");
  for(uint16_t i=0; i < dynamic_trajectory_length; i++) {
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
  // if commanded to start the trajectory, then do so
  if(Serial.available() > 0) {
    int received = Serial.read();
    for(uint16_t i=0; i < dynamic_trajectory_length; i++) {
      // log telemetry
      th1_vesc.getVescValues();
      th2_vesc.getVescValues();

      // get positions
      th1 = th1_encoder.read()/5796.0; // heh this was totally experimental lol
      th2 = th2_encoder.read()/5796.0;

      // get velocities, use exponential moving average filter
      dth1 = 100*(th1 - previous_th1);
      dth2 = 100*(th2 - previous_th2);
      previous_th1 = th1;
      previous_th2 = th2;

      // add weighting function, because bullshit
      dth1 = dth1 * exp(-pow(dth1, 2)/100);
      dth2 = dth2 * exp(-pow(dth2, 2)/25);

      th1_desired = th1_trajectory[i];
      th2_desired = th2_trajectory[i];

      // make impedance controller
      th1_current = k*(th1_desired - th1) + d*(-dth1);
      th2_current = k*(th2_desired - th2) + d*(-dth2);

      // set hard currnet limit
      if(th1_current >  current_limit) th1_current =  current_limit;
      if(th1_current < -current_limit) th1_current = -current_limit;
      if(th2_current >  current_limit) th2_current =  current_limit;
      if(th2_current < -current_limit) th2_current = -current_limit;

      // write current
      th1_vesc.setCurrent(-th1_current);
      th2_vesc.setCurrent(-th2_current);

      // print interesting bits
      Serial.print("th1:");
      Serial.print(th1);
      Serial.print(",");

      Serial.print("th1_desired:");
      Serial.print(th1_desired);
      Serial.print(",");

      Serial.print("dth1:");
      Serial.print(dth1);
      Serial.print(",");

      Serial.print("th1_current:");
      Serial.print(th1_current);
      Serial.print(",");
      
      Serial.print("th2:");
      Serial.print(th2);
      Serial.print(",");

      Serial.print("th2_desired:");
      Serial.print(th2_desired);
      Serial.print(",");

      Serial.print("dth2:");
      Serial.print(dth2);
      Serial.print(",");

      Serial.print("th2_current:");
      Serial.println(th2_current);

      delay(10);
    }
  }

  // otherwise just do normal impedance control
  // log telemetry
  th1_vesc.getVescValues();
  th2_vesc.getVescValues();

  // get positions
  th1 = th1_encoder.read()/5796.0; // heh this was totally experimental lol
  th2 = th2_encoder.read()/5796.0;

  // get velocities, use exponential moving average filter
  dth1 = 100*(th1 - previous_th1);
  dth2 = 100*(th2 - previous_th2);
  previous_th1 = th1;
  previous_th2 = th2;

  // add weighting function, because bullshit
  dth1 = dth1 * exp(-pow(dth1, 2)/100);
  dth2 = dth2 * exp(-pow(dth2, 2)/25);

  // make impedance controller
  th1_current = k*(th1_desired - th1) + d*(-dth1);
  th2_current = k*(th2_desired - th2) + d*(-dth2);

  // set hard currnet limit
  if(th1_current >  current_limit) th1_current =  current_limit;
  if(th1_current < -current_limit) th1_current = -current_limit;
  if(th2_current >  current_limit) th2_current =  current_limit;
  if(th2_current < -current_limit) th2_current = -current_limit;

  // write current
  th1_vesc.setCurrent(-th1_current);
  th2_vesc.setCurrent(-th2_current);

  // print interesting bits
  Serial.print("th1:");
  Serial.print(th1);
  Serial.print(",");

  Serial.print("th1_desired:");
  Serial.print(th1_desired);
  Serial.print(",");

  Serial.print("dth1:");
  Serial.print(dth1);
  Serial.print(",");

  Serial.print("th1_current:");
  Serial.print(th1_current);
  Serial.print(",");
  
  Serial.print("th2:");
  Serial.print(th2);
  Serial.print(",");

  Serial.print("th2_desired:");
  Serial.print(th2_desired);
  Serial.print(",");

  Serial.print("dth2:");
  Serial.print(dth2);
  Serial.print(",");

  Serial.print("th2_current:");
  Serial.println(th2_current);

  delay(10);
}
