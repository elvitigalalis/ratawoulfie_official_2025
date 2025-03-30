#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include "../../../quadrature_encoder.pio.h"
#include "pico/stdlib.h"

class Encoder {
   public:
	Encoder(uint encoderPin1, uint encoderPin2, float eventsPerRev, uint smoothingWindow = 3);

	~Encoder();

	void update();

	int32_t getCount();

	float getRPM();

	void reset();

   private:
	uint encoderPin1;
	uint encoderPin2;
	float eventsPerRev;
	uint smoothingWindow;  //  # of samples to average for encoder readings.

	PIO pioInstance;
	uint stateMachine;

	float currentRPM;
	int32_t oldEncoderCount;
	absolute_time_t lastEncoderUpdateTime;

	float* deltaTimesBuffer;
	uint bufferIndex;
	uint sampleCount;

	int32_t readEncoderCount();

	float getAverageDeltaTime(float newDelta);	// Adds a new delta time to the buffer -> creates new average.
};
// pioasm -o c-sdk quaddec_encoder.pio quaddec_encoder.pio.h
#endif