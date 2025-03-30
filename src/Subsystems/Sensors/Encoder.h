#ifndef ENCODER_H
#define ENCODER_H

#include <algorithm>
#include <cstdint>
#include <cstring>
#include "pico/stdlib.h"

class Encoder {
   public:
	Encoder(uint encoderPin1, uint encoderPin2, float eventsPerRev, uint smoothingWindow = 3);

	~Encoder();

	void update();

	int32_t getCount() const;

	float getRPM() const;

	void reset();

   private:
	uint encoderPin1;
	uint encoderPin2;
	float eventsPerRev;

	int32_t currentEncoderCount;
	float currentRPM;
	absolute_time_t lastEncoderUpdateTime;

	uint smoothingWindow;  //  # of samples to average for encoder readings.
	float* deltaBuffer;
	uint bufferIndex;
	uint sampleCount;

	int32_t readEncoderCount();

	float getAverageDeltaTime(float newDelta); // Adds a new delta time to the buffer -> creates new average.
};
#endif
