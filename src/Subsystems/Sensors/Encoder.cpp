#include "Encoder.h"

Encoder::Encoder(uint encoderPin1, uint encoderPin2, float eventsPerRev, uint smoothingWindow) {
	// Set up encoder variables.
	this->encoderPin1 = encoderPin1;
	this->encoderPin2 = encoderPin2;
	this->eventsPerRev = eventsPerRev;
	this->smoothingWindow = smoothingWindow;
	currentEncoderCount = 0;
	currentRPM = 0.0f;
	bufferIndex = sampleCount = 0;
	lastEncoderUpdateTime = get_absolute_time();
	deltaTimesBuffer = new float[smoothingWindow];
	std::fill(deltaTimesBuffer, deltaTimesBuffer + smoothingWindow, 0.0f); // Initializes a block of "memory;" as in everything is 0.0f.

	// Set up encoder GPIO pins.
	gpio_init(encoderPin1);
	gpio_init(encoderPin2);
	gpio_set_dir(encoderPin1, GPIO_IN);
	gpio_set_dir(encoderPin2, GPIO_IN);
	gpio_pull_up(encoderPin1);
	gpio_pull_up(encoderPin2);
}

Encoder::~Encoder() {
	delete[] deltaTimesBuffer;
}

void Encoder::update() {
	absolute_time_t currentTime = get_absolute_time();

	float deltaTime = absolute_time_diff_us(lastEncoderUpdateTime, currentEncoderCount) / 1e6f;
	if (deltaTime <= 0) {
		deltaTime = 1e-6f;
	}
	lastEncoderUpdateTime = currentTime;

	int32_t newEncoderCount = readEncoderCount();
	int32_t deltaCount = newEncoderCount - currentEncoderCount;
	currentEncoderCount = newEncoderCount;

	float effectiveCounts = eventsPerRev / 4.0f;
    float instantaneousRPM = (deltaCount / effectiveCounts) * (60.0f / deltaTime);

    float averageDelta = getAverageDeltaTime(deltaTime);
    if (averageDelta > 0) {
        currentRPM = (deltaCount / effectiveCounts) * (60.0f / averageDelta);
    } else {
        currentRPM = instantaneousRPM;
    }
}

int32_t Encoder::getCount() const {
    return currentEncoderCount;
}

float Encoder::getRPM() const {
    return currentRPM;
}

void Encoder::reset() {
    currentEncoderCount = 0;
    currentRPM = 0.0f;
    lastEncoderUpdateTime = get_absolute_time();
    bufferIndex = 0;
    sampleCount = 0;
    std::fill(deltaTimesBuffer, deltaTimesBuffer + smoothingWindow, 0.0f);
}

int32_t Encoder::readEncoderCount() {
    return 0;
}

// FIXME: Make more efficient (O(n) to O(1))
float Encoder::getAverageDeltaTime(float newDelta) {
    deltaTimesBuffer[bufferIndex] = newDelta;
    bufferIndex = bufferIndex + 1 % smoothingWindow; // Creates a circular array.
    if (sampleCount < smoothingWindow) {
        sampleCount++;
    }
    float sum = 0.0f;
    for (uint i = 0; i < sampleCount; i++) {
        sum += deltaTimesBuffer[i];
    }
    return sum / sampleCount;
}
