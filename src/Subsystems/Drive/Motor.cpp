#include "Motor.h"

Motor::Motor(int motorPin1, int motorPin2, int encoderPin1, int encoderPin2, float eventsPerRev, float maxRPM, bool isReversed)
    // Constructor for the Motor class.
    // Initializes the motor and encoder pins, sets up PWM, and initializes PID variables.
    // Parameters:
    // - motorPin1: GPIO pin for motor control (forward).
    // - motorPin2: GPIO pin for motor control (reverse).
    // - encoderPin1: GPIO pin for encoder (A).
    // - encoderPin2: GPIO pin for encoder (B).
    // - eventsPerRev: Number of encoder events per revolution.
    // - maxRPM: Maximum RPM of the motor.
    : encoder(encoderPin1, encoderPin2, eventsPerRev, 3U, isReversed) {
    // Sets variables for motor.
    this->motorPin1 = motorPin1;
    this->motorPin2 = motorPin2;
    this->encoderPin1 = encoderPin1;
    this->encoderPin2 = encoderPin2;
    this->eventsPerRev = eventsPerRev;
    this->maxRPM = maxRPM;
    this->isReversed = isReversed;

    desiredRPM = 0.0f;
    targetPosition = 0;
    motorOn = true;
    kP = kI = kD = 0.0f;
    integral = 0.0f;
    derivative = lastError = 0.0f;
    lastPIDTime = get_absolute_time();

    setUp();
}

void Motor::setUp() {
    // Initializes motors.
    gpio_set_function(motorPin1, GPIO_FUNC_PWM);
    gpio_set_function(motorPin2, GPIO_FUNC_PWM);

    // Initializes PWM.
    channelForward = pwm_gpio_to_channel(motorPin1);
    channelReverse = pwm_gpio_to_channel(motorPin2);  // Controls duty cycle of PWM signal (in turn the voltage provided).
    pwm_slice = pwm_gpio_to_slice_num(motorPin1);

    pwm_config config = pwm_get_default_config();
    pwm_init(pwm_slice, &config, false);   // Slice = two channels because H-Bridge;default config for PWM + won't start immediately.
    pwm_set_wrap(pwm_slice, 999);          // PWM counter (0 --> 999) thus 1000 discrete steps for duty cycle adjustments.
    pwm_set_both_levels(pwm_slice, 0, 0);  // Sets duty cycle of both channels to 0 (no power to both motors).
    pwm_set_enabled(pwm_slice, true);      // Enables PWM to genrate signals based on config.

    // Initializes the voltage.
    adc_init();
    // adc_gpio_init();
    adc_select_input(0);
}

void Motor::start() {
    motorOn = true;
    updatePWM();
}

void Motor::stop() {
    motorOn = false;
    pwm_set_both_levels(pwm_slice, 0, 0);
}

void Motor::setVoltage(float volts, bool direction) {
    // Sets the voltage for the motor.
    volts = clamp(volts, -MAX_MOTOR_VOLTAGE, MAX_MOTOR_VOLTAGE);
    m_motor_voltage = volts;
    float m_battery_voltage = MAX_MOTOR_VOLTAGE;
    // printf("Motor Voltage: %f\n", m_motor_voltage);

    // printf("Battery Voltage: %f\n", m_battery_voltage);

    int motorPWM = pwm_compensated(volts, m_battery_voltage);
    setMotorPWM(motorPWM, direction);
}

int Motor::pwm_compensated(float desired_voltage, float battery_voltage) {
    int pwm = MAX_MOTOR_PWM * fabs(desired_voltage) / battery_voltage;
    return pwm;
}

void Motor::setMotorPWM(int motorPWM, bool direction) {
    motorPWM = clamp(motorPWM, 0.0f, 999.0f);  // Because there are only 1000 different possible "levels" of motor power.
    // printf("PWM Expected: %i\n", motorPWM);

    pwm_set_chan_level(pwm_slice, direction ? channelForward : channelReverse, (uint16_t)motorPWM);
    pwm_set_chan_level(pwm_slice, direction ? channelReverse : channelForward, 0);
}

float Motor::voltage() {
    uint16_t raw = adc_read();
    float voltage = (raw * 3.3f) / 4096.0f;  // Convert to volts.
    // printf("Voltage: %f\n", voltage);
    return voltage;
}

void Motor::setRPM(float RPM) {
    desiredRPM = RPM;
    start();
}

void Motor::setPosition(int targetPosition) {
    this->targetPosition = targetPosition;
}

float Motor::getCurrentRPM() {
    return encoder.getRPM();
}

float Motor::getDesiredRPM() {
    return desiredRPM;
}

int Motor::getTargetPosition() const {
    return targetPosition;
}

int32_t Motor::getCurrentPosition() {
    return encoder.getCount();
}

Encoder* Motor::getEncoder() {
    return &encoder;
}

float Motor::getMaxRPM() const {
    return maxRPM;
}

void Motor::updateEncoder() {
    encoder.update();
}

void Motor::updatePWM() {
    if (!motorOn)
        return;
    updateEncoder();

    // Δt calculations (for PWM cycles)
    absolute_time_t currentPIDTime = get_absolute_time();
    float timeDelta = absolute_time_diff_us(lastPIDTime, currentPIDTime) / 1e6f;

    if (timeDelta < 0.1f) {
        return;
    }
    lastPIDTime = currentPIDTime;

    // if (timeDelta <= 0)
    //     timeDelta = 0.001f;  // Avoid divide by zero.  // Throttle -> RPM calc.
    //                          // printf("Desired RPM: %f\n", desiredRPM);
    if (std::fabs(desiredRPM) < 1e-6) {
        stop();
        return;
    }

    // PID calcs.
    float currentRPM = encoder.getRPM();
    float error = desiredRPM - currentRPM;  // Uses float absolute value.

    integral = clamp(integral + error * timeDelta, -100.0f, 100.0f);
    derivative = (error - lastError) / timeDelta;
    lastError = error;
    // Feedforward calculations:
    /* y = mx + b
*  b constant (537.0f) = baseline PWM level required to overcome static friction / system losses.
                "min power needed to start moving the motor"
m constant (0.8f) = scaling factor; maps desiredRPM to another PWM value. Higher the RPM,
                the more additional power you need, so this is proportional.
*/

    // Run motor at diff speeds (with no PID) -> test PWM values needed to maintain steady RPM -> make a regression until feedforward term predicts PWM needed.
    // Look at motor characteristics too + friction, load, battery, voltage, and H-Bridge characteristics
    // Then fine-tune using the kP term first, then add the integral and derivative terms for any residual errors.
    float feedForward;
    if (isReversed) {
        // LOG_DEBUG("Left:" + std::to_string(feedforwardLConstant + feedforwardLSlope * desiredRPM));
        feedForward = feedforwardLConstant + feedforwardLSlope * desiredRPM;
    } else {
        // LOG_DEBUG("Right:" + std::to_string(feedforwardRConstant + feedforwardRSlope * desiredRPM));
        feedForward = feedforwardRConstant + feedforwardRSlope * desiredRPM;
    }
    // LOG_DEBUG("Kp = " + std::to_string(kP) + ", Ki = " + std::to_string(kI) + ", Kd = " + std::to_string(kD));
    // LOG_DEBUG("Error: " + std::to_string(error) + "; kP * error: " + std::to_string(kP * error) + "; Voltage Output" + std::to_string(feedForward + kP * error + kI * integral + kD * derivative));
    float voltageOutput = feedForward + kP * error + kI * integral + kD * derivative;
    // printf("kP * error: %f\n", kP * error);
    // printf("Feedforward: %f\n", feedForward);

    // printf("Error: %f\n", error);
    // printf("Voltage Output: %f\n", voltageOutput);
    // LOG_DEBUG("Voltage Sign: " + std::to_string(!std::signbit(voltageOutput)));
    setVoltage(voltageOutput, !std::signbit(voltageOutput));
}

// void Motor::getFeedforwardValue(float i, std::string motorName) {
// 	float desiredRPM = 10;	// STUB

// 	float PIDOutput = i;

// 	printf("PWM: %f\n", i);

// 	pwm_set_chan_level(pwm_slice, (desiredRPM >= 0 ? channelForward : channelReverse), (uint16_t)PIDOutput);
// 	pwm_set_chan_level(pwm_slice, (desiredRPM >= 0 ? channelReverse : channelForward), 0);

// 	sleep_ms(100);
// 	printf("RPM: %f\n", getCurrentRPM());
// }

float Motor::clamp(float value, float min, float max) {
    return std::max(std::min(value, max), min);
}
