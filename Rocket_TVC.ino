// Haven't yet added the codes of getting the MPU6050 sensor readings (Coz I haven't get my hands on the sensor yet)

#include <Servo.h>

//// Global Parameters

// PID Gains (Haven't yet tuned)
float Kp = 0.1;
float Ki = 0.1;
float Kd = 0.1;

// Setpoint & Sensor Measurement
double SETPOINT = 0;
double sensorValue = 5;  // In Degrees  // Temporary

// PID Error Parameters
double pastError1 = 0;
double pastError2 = 0;
double integralError1 = 0;
double integralError2 = 0;

// Define the 2 servos
Servo servo1;
Servo servo2;

// Define the 2 Angles (In degrees)
double angle1;
double angle2;

// Time
unsigned long currentTime;
unsigned long pastTime;
float timeInterval = 100;



void setup() {
  Serial.begin(9600);

  servo1.attach(5);
  servo2.attach(6);

  pastTime = millis();
}

void loop() {
  currentTime = millis();
  unsigned long timeElasped = currentTime - pastTime;

  // do the PID around once every timeInterval milliseconds
  if (timeElasped >= timeInterval) {
    angle1 = PID(SETPOINT, sensorValue, timeElasped, &pastError1, &integralError1);
    angle2 = PID(SETPOINT, sensorValue, timeElasped, &pastError2, &integralError2);

    servo1.write(angle1);
    servo2.write(angle2);

    // Update pastTime
    pastTime = currentTime;
  }
}



// Apply PID Control for each of the 2 axis involved
// Return the angle as an output
double PID(double setPoint, double currentPoint, unsigned long timeChange, double *pastError, double *integralError) {
  double error = setPoint - currentPoint;
  double deriviativeError = (error - *pastError) / timeChange;
  *integralError += error;

  double outputAngle = Kp * error + Ki * *integralError + Kd * deriviativeError;

  *pastError = error;

  return outputAngle;
}