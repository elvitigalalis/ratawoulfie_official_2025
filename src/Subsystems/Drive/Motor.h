#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <string>
#include "../../Constants/Logger.h"
#include "../Sensors/Encoder.h"
#include "hardware/adc.h"
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
    Motor(int motorPin1, int motorPin2, int encoderPin1, int encoderPin2, float eventsPerRev = 360.0f, float maxRPM = 200.0f, bool isReversed = false);

    void setPIDVariables(float Kp, float Ki, float Kd) {
        kP = Kp;
        kI = Ki;
        kD = Kd;
    }  // All PID Constants.

    void start();
    void stop();

    void setRPM(float RPM);
    float getDesiredRPM();
    void setPosition(int pos);  // Meters.
    void setVoltage(float volts, bool direction);
    int pwm_compensated(float desired_voltage, float battery_voltage);

    float getTargetThrottle() const;
    float getCurrentRPM();
    int getTargetPosition() const;
    int32_t getCurrentPosition();
    Encoder* getEncoder();
    float getMaxRPM() const;
    float voltage();
    void setMotorPWM(int motorPWM, bool forward = true);

    void updateEncoder();
    void updatePWM();
    void getFeedforwardValue(float i, std::string motorName);

   private:
    int motorPin1, motorPin2, encoderPin1, encoderPin2;
    bool isReversed;

    float maxRPM;
    float eventsPerRev;  // Before quadrature division (so 360 degrees per rev).

    uint pwm_slice;
    uint channelForward, channelReverse;

    volatile float desiredRPM;  // [-1 (max reverse speed), 0 (off), 1 (max forward speed)].

    volatile int targetPosition;
    volatile bool motorOn;

    volatile float m_motor_voltage;  // Volts.

    float kP, kI, kD;
    volatile float integral;
    volatile float derivative, lastError;
    absolute_time_t lastPIDTime;

    Encoder encoder;

    float clamp(float value, float min, float max);

    void setUp();

    // const float feedforwardLConstant = 648.0f / 200.0f;
    // const float feedforwardLSlope = 1.2f / 200.0f;
    // const float feedforwardRConstant = 641.0f / 200.0f;
    // const float feedforwardRSlope = 1.16f / 200.0f;
    // const float feedforwardLConstant = 3.381507466f;
    // const float feedforwardLSlope = 0.006180486071f;
    // const float feedforwardRConstant = 3.163033433f;
    // const float feedforwardRSlope = 0.0060479671f;
    // const float feedforwardLConstant = 0.81613f;  // 3.381507466f;
    // const float feedforwardLSlope = 0.015257f;    // 0.006180486071f;
    // const float feedforwardRConstant = 1.17717f;  // 3.163033433f;
    // const float feedforwardRSlope = 0.01418f;     // 0.0060479671f;
    const float feedforwardLConstant = 1.258043f;
    // const float feedforwardLSlope = 0.014981f;
    const float feedforwardLSlope = 0.013f;
    const float feedforwardRConstant = 1.274472f;
    // const float feedforwardRSlope = 0.014868f;
    const float feedforwardRSlope = 0.013f;

    const float MAX_MOTOR_VOLTAGE = 5.0f;  // Volts.
    const int MAX_MOTOR_PWM = 999;

    const int BATTERY_ADC_PIN = 26;
};
#endif