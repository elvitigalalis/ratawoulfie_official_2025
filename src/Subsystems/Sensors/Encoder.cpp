#include "Encoder.h"

Encoder::Encoder(uint encoderPin1, uint encoderPin2, float eventsPerRev, uint smoothingWindow) {
	assert(encoderPin2 == encoderPin1 + 1);

	// Set up encoder variables
	this->encoderPin1 = encoderPin1;
	this->encoderPin2 = encoderPin2;
	this->eventsPerRev = eventsPerRev;
	this->smoothingWindow = smoothingWindow;
	currentRPM = 0.0f;
	bufferIndex = sampleCount = 0;
	lastEncoderUpdateTime = get_absolute_time();
	deltaTimesBuffer = new float[smoothingWindow];
	std::fill(deltaTimesBuffer, deltaTimesBuffer + smoothingWindow, 0.0f);

	// Set up PIO program
	pioInstance = pio0;
	stateMachine = pio_claim_unused_sm(pioInstance, true);

	// Load PIO program and get offset
	// uint offset = pio_add_program(pioInstance, &quadrature_encoder_program);
	// Initialize PIO program with the correct offset
	quadrature_encoder_program_init(pioInstance, stateMachine, encoderPin1, 0);

	// Read initial encoder count after PIO is initialized
	oldEncoderCount = readEncoderCount();
	// printf("PIO SM: %d, PinA: %d, PinB: %d\n", stateMachine, encoderPin1, encoderPin2);
}

Encoder::~Encoder() {
	delete[] deltaTimesBuffer;
	pio_sm_set_enabled(pioInstance, stateMachine, false);
}

void Encoder::update() {
	absolute_time_t currentTime = get_absolute_time();

	// float deltaTime = absolute_time_diff_us(lastEncoderUpdateTime, currentTime) / 1e6f;
	// if (deltaTime <= 0) {
	// 	deltaTime = 1e-6f;
	// }
	lastEncoderUpdateTime = currentTime;

	int32_t newEncoderCount = readEncoderCount();
	printf("Raw Count: %ld\n", newEncoderCount);

	int32_t deltaCount = newEncoderCount - oldEncoderCount;
	oldEncoderCount = newEncoderCount;

	float effectiveCounts = eventsPerRev / 4.0f;  // Assuming eventsPerRev is cycles per revolution
	// float instantaneousRPM = (deltaCount / effectiveCounts) * (60.0f / deltaTime);

	// float averageDelta = getAverageDeltaTime(deltaTime);
	// currentRPM = (averageDelta > 0) ? ((deltaCount / effectiveCounts) * (60.0f / averageDelta)) : instantaneousRPM;
}

int32_t Encoder::getCount() const {
	return oldEncoderCount;
}

float Encoder::getRPM() const {
	return currentRPM;
}

void Encoder::reset() {
	oldEncoderCount = 0;
	currentRPM = 0.0f;
	lastEncoderUpdateTime = get_absolute_time();
	bufferIndex = 0;
	sampleCount = 0;
	std::fill(deltaTimesBuffer, deltaTimesBuffer + smoothingWindow, 0.0f);
}

int32_t Encoder::readEncoderCount() {
	uint32_t raw = quadrature_encoder_get_count(pioInstance, stateMachine);
	return static_cast<int32_t>(raw);
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
