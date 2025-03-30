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

	targetThrottle = 0.0f;
	targetPosition = 0;
	motorOn = true;
	kP = kI = kD = 0.0f;
	integral = 0.0f;
	derivative = lastError = 0.0f;
	lastPIDTime = get_absolute_time();

	// Sets up motor.
	setUp();
	initializePWM();
}

void Motor::setUp() {
	gpio_set_function(motorPin1, GPIO_FUNC_PWM);
	gpio_set_function(motorPin2, GPIO_FUNC_PWM);
}

void Motor::initializePWM() {
	channelForward = pwm_gpio_to_channel(motorPin1);
	channelReverse = pwm_gpio_to_channel(motorPin2);  // Controls duty cycle of PWM signal (in turn the voltage provided).
	pwm_slice = pwm_gpio_to_slice_num(motorPin1);

	pwm_config config = pwm_get_default_config();
	pwm_init(pwm_slice, &config, false);   // Slice = two channels because H-Bridge;default config for PWM + won't start immediately.
	pwm_set_wrap(pwm_slice, 999);		   // PWM counter (0 --> 999) thus 1000 discrete steps for duty cycle adjustments.
	pwm_set_both_levels(pwm_slice, 0, 0);  // Sets duty cycle of both channels to 0 (no power to both motors).
	pwm_set_enabled(pwm_slice, true);	   // Enables PWM to genrate signals based on config.
}

void Motor::setPIDVariables(float kP, float kI, float kD) {
	this->kP = kP;
	this->kI = kI;
	this->kD = kD;
}

void Motor::start() {
	motorOn = true;
}

void Motor::stop() {
	motorOn = false;
	pwm_set_both_levels(pwm_slice, 0, 0);
}

void Motor::setThrottle(float targetThrottle) {
	this->targetThrottle = clamp(targetThrottle, -1.0f, 1.0f);

	integral = 0.0f;
	derivative = lastError = 0.0f;
	start();
}

void Motor::setPosition(int targetPosition) {
	this->targetPosition = targetPosition;
}

float Motor::getTargetThrottle() const {
	return targetThrottle;
}

float Motor::getCurrentRPM() {
	return encoder.getRPM();
}

int Motor::getTargetPosition() const {
	return targetPosition;
}

int32_t Motor::getCurrentPosition() {
	return encoder.getCount();
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
	lastPIDTime = currentPIDTime;

	if (timeDelta <= 0)
		timeDelta = 0.001f;	 // Avoid divide by zero.

	float desiredRPM = targetThrottle * maxRPM;	 // Throttle -> RPM calc.
	if (std::fabs(desiredRPM) < 1e-6) {
		// printf("Zero RPM Requested\n");
		pwm_set_both_levels(pwm_slice, 0, 0);
		return;
	}

	// PID calcs.
	float currentRPM = encoder.getRPM();
	float error = std::fabs(desiredRPM) - std::fabs(currentRPM);  // Uses float absolute value.

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
	float feedForward = 537.0f + 0.8f * std::fabs(desiredRPM);

	float PIDOutput = kP * error + kI * integral + kD * derivative + feedForward;
	PIDOutput = clamp(PIDOutput, 0.0f, 999.0f);	 // Because there are only 1000 different possible "levels" of motor power.

	pwm_set_chan_level(pwm_slice, (desiredRPM >= 0 ? channelForward : channelReverse), (uint16_t)PIDOutput);
	pwm_set_chan_level(pwm_slice, (desiredRPM >= 0 ? channelReverse : channelForward), 0);
}

float Motor::clamp(float value, float min, float max) {
	return std::max(std::min(value, max), min);
}
