
//https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/

#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>
// #include <Kalman.h>

#define OUTPUT_READABLE_ACCELGYRO

MPU6050 mpu;
// Kalman_h kal(0.001, 0.003, 0.03);

int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax_, ay_, az_;

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

//PID
float kp = 8;
float ki = 0.1;
float kd = 0.075;


float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

int16_t  targetAngle = 40, error, PID, previousError,  Speed = 100;

float time, timePrev, elapsedTime;
float accPitch, kalPitch;

// Filtr Kalmana - zmienne stanu
float angle = 0; // oszacowany kąt
float bias = 0;  // bias żyroskopu
float P[2][2] = {{1, 0}, {0, 1}}; // macierz błędu

// Parametry filtru
float Q_angle = 0.001;  // Niepewność modelu kąta
float Q_bias = 0.003;   // Niepewność biasu żyroskopu
float R_measure = 0.03; // Niepewność pomiaru akcelerometru

float K[2]; // Wzmocnienia Kalmana
float y;    // Różnica pomiaru
float S;    // Szacowana niepewność

float gyroRate, filteredAngle;

float KalmanFilter(float newAngle, float newRate, float dt) {
    // Przewidywanie (Prediction)
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Aktualizacja (Correction)
    S = P[0][0] + R_measure;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return angle;
}

void Forwards()
{
  analogWrite(enA, Speed);
	analogWrite(enB, Speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
}

void Backwards()
{
  analogWrite(enA, Speed);
	analogWrite(enB, Speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
}

void Stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
	analogWrite(enB, 0);
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  mpu.initialize();
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }

  mpu.setXAccelOffset(0); //Set your accelerometer offset for axis X
  mpu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
  mpu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
  mpu.setXGyroOffset(0);  //Set your gyro offset for axis X
  mpu.setYGyroOffset(0);  //Set your gyro offset for axis Y
  mpu.setZGyroOffset(0);  //Set your gyro offset for axis Z
 
	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}



void loop() {

  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //mpu.getAcceleration(&ax, &ay, &az);
  //mpu.getRotation(&gx, &gy, &gz);

  ax_ = ax / 16384.0; // dla ±2g
  ay_ = ay / 16384.0;
  az_ = az / 16384.0;

  /*Print the obtained data on the defined format*/
  accPitch = atan2(ay_, sqrt(ax_ * ax_ + az_ * az_)) * 180.0 / M_PI;
  gyroRate = gx / 131.0; // Konwersja surowych danych żyroskopu (dla zakresu ±250°/s)
  filteredAngle = KalmanFilter(accPitch, gyroRate, elapsedTime);


  error = (int16_t)filteredAngle - targetAngle;

  //Proportional Error
  pid_p = kp * error;

  //Integral Error
  pid_i += (ki * error);

  //Differential Error
  pid_d = kd * ((error - previousError) / elapsedTime);

  //Total PID Value
  PID = pid_p + pid_i + pid_d;
  

  //Get modulous of PID value for speed of motors
  Speed = max(abs(PID) , 50);
  Speed = min(abs(PID) , 255);


  previousError = error;

  if (filteredAngle > 40)
  {Forwards();}
  
  if (filteredAngle < 40)
  {Backwards();} 

  if (filteredAngle < 20 || filteredAngle > 55)
  {Stop();} 


}

