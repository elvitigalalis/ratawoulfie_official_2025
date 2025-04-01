#include "Encoder.h"
#include <stdio.h>
#include <algorithm>
#include <cmath>

Encoder::Encoder(uint encoderPin1, uint encoderPin2, float eventsPerRev, uint smoothingWindow, bool isReversed)
	: encoderPin1(encoderPin1),
	  encoderPin2(encoderPin2),
	  eventsPerRev(eventsPerRev),
	  smoothingWindow(smoothingWindow),
	  pioInstance(pio0),
	  currentCount(0),
	  oldEncoderCount(0),
	  currentRPM(0.0f),
	  bufferIndex(0),
	  sampleCount(0),

	  isReversed(isReversed) {

	assert(encoderPin2 == encoderPin1 + 1);
	// Initialize smoothing buffer
	deltaTimesBuffer = new float[smoothingWindow]();

	// Setup PIO
	stateMachine = pio_claim_unused_sm(pioInstance, true);
	pio_add_program(pioInstance, &quadrature_encoder_program);
	// uint offset = pio_add_program(pioInstance, &quadrature_encoder_program);
	quadrature_encoder_program_init(pioInstance, stateMachine, encoderPin1, 0);

	// Initial reading
	oldEncoderCount = getCount();
	lastUpdateTime = get_absolute_time();
}

Encoder::~Encoder() {
	delete[] deltaTimesBuffer;
	pio_sm_set_enabled(pioInstance, stateMachine, false);
}

void Encoder::update() {
	absolute_time_t now = get_absolute_time();
	int deltaTime = absolute_time_diff_us(lastUpdateTime, now);
	double newDeltaTime = deltaTime / 1000000.0;
	currentCount = getCount();
	double deltaCount = currentCount - oldEncoderCount;

	if (newDeltaTime > 0.2) {
		if (abs(deltaCount) > 5) {
			oldEncoderCount = currentCount;
			lastUpdateTime = now;

			double currentRPM = (deltaCount / 360.0) * (60 / newDeltaTime);
			setRPM(currentRPM);

			int32_t RPM = getRPM();
			int32_t Pos = getCount();
			printf("RPM=%d P=%d\n", RPM, Pos);
		}
		setRPM(0.8f * getRPM());
		if (getRPM() < 1) {
			setRPM(0);
		}
	}
}

int32_t Encoder::getCount() const {
	return (isReversed ? -quadrature_encoder_get_count(pioInstance, stateMachine) : quadrature_encoder_get_count(pioInstance, stateMachine));
}

void Encoder::setRPM(double RPM) {
	currentRPM = RPM;
}

float Encoder::getPosition() const {
	return getCount() / 3376.24604581f;
}  // 1 ft = 3376.24604581 encoder counts (1/0.1066273*360).

float Encoder::getRPM() const {
	return currentRPM;
}

void Encoder::reset() {
	oldEncoderCount = 0;
	currentRPM = 0.0f;
	lastUpdateTime = get_absolute_time();
	bufferIndex = sampleCount = 0;
	std::fill(deltaTimesBuffer, deltaTimesBuffer + smoothingWindow, 0.0f);
}

// FIXME: Make more efficient (O(n) to O(1))
float Encoder::getAverageDeltaTime(float newDelta) {
	deltaTimesBuffer[bufferIndex] = newDelta;
	bufferIndex = (bufferIndex + 1) % smoothingWindow;	// Creates a circular array.
	if (sampleCount < smoothingWindow) {
		sampleCount++;
	}
	float sum = 0.0f;
	for (uint i = 0; i < sampleCount; i++) {
		sum += deltaTimesBuffer[i];
	}
	return sum / sampleCount;
}