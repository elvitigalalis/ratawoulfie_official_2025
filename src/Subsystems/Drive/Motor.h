#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <cmath>
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Motor {
   public:
	/**
     * Motor pins (for H-Bridge PWM outputs) + encoder pins (for encoder inputs).
     * Encoer events per revolution = 360 degrees by default.
     */
	Motor(int motorPin1, int motorPin2, int encoderPin1, int encoderPin2, float eventsPerRev = 360.0f, float maxRPM = 200.0f);

	void setPIDVariables(float Kp, float Ki, float Kd);	 // All PID Constants.

	void start();
	void stop();

	void setThrottle(float throttle);  // -1 (reverse) to 1 (forward).
	void setPosition(int pos);		   // Meters.

	float getTargetThrottle() const;
	float getRPM() const;
	int getTargetPosition() const;
	int getCurrentPosition() const;

	void updateEncoder();
	void updatePWM();

   private:
	int motorPin1, motorPin2, encoderPin1, encoderPin2;

	float maxRPM;
	float eventsPerRev;	 // Before quadrature division (so 360 degrees per rev).

	uint pwm_slice;
	uint channelForward, channelReverse;

	volatile float targetThrottle;	// [-1 (max reverse speed), 0 (off), 1 (max forward speed)].

	volatile int targetPosition, currentPosition;
	volatile float currentRPM;
	volatile bool motorOn;

	float Kp, kI, Kd;
	volatile float integral;
	volatile float derivative, lastError;
	absolute_time_t lastPIDTime;

	volatile int32_t prevEncoderCount;
	absolute_time_t prevEncoderTime;  // For RPM calcs.

	void setUp();
	void initializePWM();

	int32_t readEncoderCount();
};
#endif