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
float previous_th1 = 0;
float previous_th2 = 0;

float th1 = 0;
float th2 = 0;

float dth1_desired = 0;
float dth2_desired = 0;
float dth1 = 0;
float dth2 = 0;
float new_dth1 = 0;
float new_dth2 = 0;
float dth_alpha = 0.05;

float k = 0; //0.04;
float d = 0.0005;

float th1_current = 0;
float th2_current = 0;

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
  if(Serial.available()) {
    if(Serial.read() == 't') {
      // execute trajectory
      for(uint16_t i=0; i < dynamic_trajectory_length; i++) {
        th1_vesc.getVescValues();
        th2_vesc.getVescValues();

        // get positions
        th1 = th1_encoder.read()*6.28/2048;
        th2 = th2_encoder.read()*6.28/2048;

        // get velocities, use exponential moving average filter
        //new_dth1 = 100*(th1 - previous_th1);
        //new_dth2 = 100*(th2 - previous_th2);
        //dth1 = (dth_alpha * new_dth1) + (1.0 - dth_alpha) * dth1;
        //dth2 = (dth_alpha * new_dth2) + (1.0 - dth_alpha) * dth2;

        dth1 = 100*(th1 - previous_th1);
        dth2 = 100*(th2 - previous_th2);

        previous_th1 = th1;
        previous_th2 = th2;

        // make impedance controller
        th1_desired = th1_trajectory[i];
        dth1_desired = dth1_trajectory[i];
        th2_desired = th2_trajectory[i];
        dth2_desired = dth2_trajectory[i];

        th1_current = k*(th1_desired - th1) + d*(dth1_desired - dth1);
        th2_current = k*(th2_desired - th2) + d*(dth2_desired - dth2);

        // set hard currnet limit
        if(th1_current >  current_limit) th1_current =  current_limit;
        if(th1_current < -current_limit) th1_current = -current_limit;
        if(th2_current >  current_limit) th2_current =  current_limit;
        if(th2_current < -current_limit) th2_current = -current_limit;

        //th1_vesc.setCurrent(-th1_current);
        th2_vesc.setCurrent(-th2_current);

        Serial.print("th2:");
        Serial.print(th2);
        Serial.print(",");

        Serial.print("dth2:");
        Serial.print(dth2);
        Serial.print(",");

        Serial.print("th2_current:");
        Serial.print(th2_current);
        Serial.print(",");

        Serial.print("th2_current_k_contribution:");
        Serial.print(k*(th2_desired - th2));
        Serial.print(",");

        Serial.print("th2_current_d_contribution:");
        Serial.println(d*(-dth2));


        delay(10);
      }
    }
  }

  // otherwise just do normal impedance control
  while(1){
    // log telemetry
    th1_vesc.getVescValues();
    th2_vesc.getVescValues();

    // get positions
    th1 = th1_encoder.read();
    th2 = th2_encoder.read();

    // get velocities, use exponential moving average filter
    dth1 = 100*(th1 - previous_th1);
    dth2 = 100*(th2 - previous_th2);
    previous_th1 = th1;
    previous_th2 = th2;

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
    Serial.print("th2:");
    Serial.print(th2);
    Serial.print(",");

    Serial.print("dth2:");
    Serial.print(dth2);
    Serial.print(",");

    Serial.print("th2_current:");
    Serial.print(th2_current);
    Serial.print(",");

    Serial.print("th2_current_k_contribution:");
    Serial.print(k*(th2_desired - th2));
    Serial.print(",");

    Serial.print("th2_current_d_contribution:");
    Serial.println(d*(-dth2));

    delay(10);
  }
}
