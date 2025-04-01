#pragma once

#include "../../../quadrature_encoder.pio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

class Encoder {
   public:
	Encoder(uint encoderPin1, uint encoderPin2, float eventsPerRev, uint smoothingWindow = 100, bool isReversed = false);
	~Encoder();

	void update();
	int32_t getCount() const;
	float getPosition() const;	// ft
	float getRPM() const;
    void setRPM(double RPM);
	void reset();

   private:
	float getAverageDeltaTime(float newDelta);

	// Configuration
	const uint encoderPin1;
	const uint encoderPin2;
	const float eventsPerRev;
	const uint smoothingWindow;
	const bool isReversed;

	// PIO State (mutable for hardware access)
	mutable PIO pioInstance;
	mutable uint stateMachine;

	// Tracking State
	int32_t currentCount;
	int32_t oldEncoderCount;
	float currentRPM;
	absolute_time_t lastUpdateTime;

	// Smoothing
	float* deltaTimesBuffer;
	uint bufferIndex;
	uint sampleCount;
};