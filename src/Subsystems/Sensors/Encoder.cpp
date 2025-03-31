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
	float deltaTime = std::max(absolute_time_diff_us(lastUpdateTime, now) / 1e6f, 0.001f);
	// printf("Delta Time: %f\n", deltaTime);
	lastUpdateTime = now;

	currentCount = getCount();
	// printf("Current Count: %d\n", currentCount);
	int32_t deltaCount = currentCount - oldEncoderCount;
	// printf("Delta Count: %d\n", deltaCount);
	oldEncoderCount = currentCount;

	// Calculate RPM with noise filtering
	if (abs(deltaCount) > 0) {	// Deadband for small movements
		float averageDelta = getAverageDeltaTime(deltaTime);
		currentRPM = (deltaCount / eventsPerRev) * (60.0f / averageDelta);
	} else {
		currentRPM *= 0.9f;
        if (fabs(currentRPM) < 0.1f) {
            currentRPM = 0.0f;
        }
	}
}

int32_t Encoder::getCount() const {
	return (isReversed ? -quadrature_encoder_get_count(pioInstance, stateMachine) : quadrature_encoder_get_count(pioInstance, stateMachine));
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