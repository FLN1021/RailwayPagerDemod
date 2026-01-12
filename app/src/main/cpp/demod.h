#ifndef DEMOD_H
#define DEMOD_H

#include <complex>
#include <stdbool.h>
#include <stdint.h>
#include <string>
#include <iostream>
#include <memory>

#include "dsp/phasediscri.h"
#include "dsp/firfilter.h"
#include "util/movingaverage.h"
#include "util/qtypes.h"
#include "dsp/decimatorsu.h"

#define SDR_RX_SCALED 32768.0
#define SAMPLE_RATE 48000.0
#define BAUD_RATE 1200.0
#define DEVIATION 4500.0
#define SAMPLES_PER_SYMBOL (SAMPLE_RATE / BAUD_RATE)
#define POCSAG_SYNCCODE 0x7CD215D8
#define POCSAG_SYNCCODE_INV ~0x7CD215D8
#define PAGERDEMOD_POCSAG_IDLECODE 0x7A89C197
#define PAGERDEMOD_BATCH_WORDS 17
#define PAGERDEMOD_FRAMES_PER_BATCH 8
#define PAGERDEMOD_CODEWORDS_PER_FRAME 2

extern bool is_message_ready;

struct pocsag_msg {
    uint32_t i;
    uint32_t addr;
    uint16_t func;
    std::string numeric;
    std::string alpha;
};
extern std::unique_ptr<std::vector<pocsag_msg>> msg;

extern PhaseDiscriminators phaseDiscri;
extern Lowpass<double> lowpassBaud;
extern MovingAverageUtil<double, double, 2048> preambleMovingAverage;
extern double magsqRaw;

void processOneSample(float i, float q);

#endif