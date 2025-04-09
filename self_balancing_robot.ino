//https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/

#include <Arduino_FreeRTOS.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

#define OUTPUT_READABLE_ACCELGYRO

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
 float ax_g, ay_g, az_g;

// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

void Engines(void *param);
void Accelerometer(void* param);


TaskHandle_t TaskHandler;
TaskHandle_t TaskHandler2;

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
  xTaskCreate(Engines, "engines", 256, NULL, 2, &TaskHandler);
  xTaskCreate(Accelerometer, "Accelerometer", 256, NULL, 1, &TaskHandler2);

  Serial.print("Minimalny pozostały stos dla Engines: ");
  Serial.println(uxTaskGetStackHighWaterMark(TaskHandler));

  Serial.print("Minimalny pozostały stos dla Accelerometer: ");
  Serial.println(uxTaskGetStackHighWaterMark(TaskHandler2));
}

void loop() {
}


void Engines(void *param){
  (void) param;
  analogWrite(enA, 120);
	analogWrite(enB, 120);

  while(1){
    // Turn on motor A & B
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    vTaskDelay(2000/ portTICK_PERIOD_MS);
   
    
    // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    vTaskDelay(2000/ portTICK_PERIOD_MS);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    vTaskDelay(2000/ portTICK_PERIOD_MS);

    // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    vTaskDelay(2000/ portTICK_PERIOD_MS);
  }	
}

void Accelerometer(void *param){
  (void) param;
  float roll, pitch;
  while(1){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("a/g:\t");
    ax_g = ax / 16384.0;
    ay_g = ay / 16384.0;
    az_g = az / 16384.0;
    roll = atan2(ax_g, az_g);
    pitch = atan2(ay_g, az_g);
    roll *= 180.0 / M_PI;
    pitch *= 180.0 / M_PI;
    Serial.print(roll); Serial.print("\n");
    Serial.print(pitch); Serial.print("\n");
     Serial.print("\n");

    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
}