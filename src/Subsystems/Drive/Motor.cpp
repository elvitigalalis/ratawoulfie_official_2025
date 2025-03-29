#include "Motor.h"

Motor::Motor(int motorPin1, int motorPin2, int encoderPin1, int encoderPin2, float eventsPerRev, float maxRPM) {
	// Sets variables for motor.
	this->motorPin1 = motorPin1;
	this->motorPin2 = motorPin2;
	this->encoderPin1 = encoderPin1;
	this->encoderPin2 = encoderPin2;
	this->eventsPerRev = eventsPerRev;
	this->maxRPM = maxRPM;
	targetThrottle = 0.0f;
	targetPosition = currentPosition = 0;
	currentRPM = 0;
	motorOn = false;
	kP = kI = kD = 0.0f;
	integral = 0.0f;
	derivative = lastError = 0.0f;

	// Sets variables for encoder.
	prevEncoderCount = 0;
	prevEncoderTime = get_absolute_time();
	lastPIDTime = get_absolute_time();

	// Sets up motor.
	setUp();
	initializePWM();
}

void Motor::setUp() {
	gpio_set_function(motorPin1, GPIO_FUNC_PWM);
	gpio_set_function(motorPin2, GPIO_FUNC_PWM);

	gpio_set_input_enabled(encoderPin1, true);
	gpio_set_input_enabled(encoderPin2, true);
	gpio_set_dir(encoderPin1, false);
	gpio_set_dir(encoderPin2, false);  // Input (not output)
	gpio_pull_up(encoderPin1);
	gpio_pull_up(encoderPin2);	// Prevents voltage fluctuation for reading during state.
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

float Motor::getCurrentRPM() const {
	return currentRPM;
}

int Motor::getTargetPosition() const {
	return targetPosition;
}

int Motor::getCurrentPosition() const {
	return currentPosition;
}

void Motor::updateEncoder() {
    int32_t currentCount = readEncoderCount();
    absolute_time_t currentTime = get_absolute_time();

	// Δt calculations.
	float deltaTime = absolute_time_diff_us(prevEncoderTime, currentTime) / 1e6f;  // Converts µs to s.
	if (deltaTime <= 0) {
        deltaTime = 1e-6f;
    }

	// Δ(Encoder counts) calculations.
	int32_t deltaCount = currentCount - prevEncoderCount;
    float effectiveCounts = eventsPerRev / 4.0f; // To account for quadrature. Essentially # of counts per quadrature.

    currentRPM = (deltaCount / effectiveCounts) * (60.0f / deltaTime);
}

float Motor::clamp(float value, float min, float max) {
	return std::max(std::min(value, max), min);
}
