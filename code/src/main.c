#define FOSC 16000000
#define BAUD 9600

#define F_CPU 16000000UL

#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <math.h>
#include <avr/interrupt.h>

#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "i2c.h"
#include "uart.h"
#include "KalmanFilter.h"
#include "engines.h"

#include <stdlib.h> 

int max_int(int a, int b) {
	if (a>b) return a;
    return b;
}

int min(int a, int b) {
    return (a < b) ? a : b;
}

void timer_setup();
//uint32_t get_time();
void get_time(double * dt);
volatile double count;

int main(void){
	SerialInit(FOSC, BAUD, 8, 2, 0);

	

	sei();  
	i2c_init();
	_delay_ms(200);

	int16_t accel_buff[3], gyro_buff[3];
	double accelX, accelY, accelZ, gyroY;
	double accPitch; // odchylenie
	double dt;
	int speed;

	double error, prev_error = 0;
	double pid_p, pid_i = 0, pid_d, pid;

	float kp = 40;
	float kd = 0.05;
	float ki = 200;
	

	uint8_t ret = i2c_start(MPU6050_ADDRESS+I2C_WRITE);
	if (ret == 1) SerialPrint("I2C cannot initialize!");
	else SerialPrint("I2C initialized :)\r\n");

	KalmanFilter kalman;
	initKalmanFilter(&kalman, 0.001, 0.003, 0.03);

	mpu6050_init();
	timer_setup();

	initEngines();

	while(1){
		get_time(&dt);

		mpu6050_read_accel_ALL(accel_buff);
		mpu6050_read_gyro_ALL(gyro_buff);

		accelX = accel_buff[0]*9.8*2/32768;
		accelY = accel_buff[1]*9.8*2/32768;
		accelZ = accel_buff[2]*9.8*2/32768;
		gyroY = (gyro_buff[1] / 131.0) * (M_PI / 180.0);

		accPitch = -(atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))*180.0)/M_PI;
		update(&kalman, accPitch, gyroY, dt);

		
		
		error = kalman.K_angle;

		//Proportional Error
		pid_p = kp * error;

		//Integral Error
		pid_i += (ki * error);

		//Differential Error
		pid_d = kd * ((error - prev_error) / dt);

		//Total PID Value
		pid = pid_p + pid_i + pid_d;

		prev_error = error;
		pid = abs(pid);
		//speed = max(abs((int)pid) , 50);
		speed = (int)abs(pid);
		if (speed < 100) speed = 100;
		if (speed > 255) speed = 255;
		
			SerialPrintInt(speed);
			SerialPrint("\r\n");
		if (error < -3) {
			forward(speed);
		} else if (error > 3) {
			backward(speed);
		} else {
			stop();
		}
			

		if (error < -90 || error > 90) {
			stop();
		}
	}
		
}

void timer_setup(){
    TCCR1A = 0x00;
    TIMSK1 |= _BV(TOIE1);
    TCCR1B |= _BV(CS10); // preskaler = 1 (16 MHz / 65536 ~ 244 Hz)
    TCCR1B &= ~( _BV(CS12)  | _BV(CS11));
}


void get_time(double * dt){
	cli();
		uint8_t l = TCNT1L;
		uint8_t h = TCNT1H;
		uint16_t step = h<<8 | l;
		*dt = (double)step / 16000000.0 + count * (65536.0 / 16000000.0);
		count = 0;
		TCNT1 = 0;
	sei();
}


// timer 1 overflow interrupt handler
SIGNAL(TIMER1_OVF_vect){
	count += 1;
	//TCNT1H = 0x00;
	//TCNT1L = 0x00;

}