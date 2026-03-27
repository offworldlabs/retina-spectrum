// Adapted from blah2-arm/src/process/spectrum/SpectrumAnalyser.h
// Changes: float/fftwf, no IqData, single FFT per step, correct DSP order

#pragma once

#include "config.h"

#include <array>
#include <complex>
#include <vector>

// Process one captured step.
// samples: N_FFT complex<float> from stream_a_callback
// fc_mhz:  centre frequency of this step
// Returns: N_DISPLAY power bins in dBFS
std::array<float, N_DISPLAY> process_step(
    float fc_mhz,
    const std::vector<std::complex<float>>& samples);

// Frequency axis for a step — centre MHz of each display bin
// Useful for building the JSON payload
std::array<float, N_DISPLAY> freq_axis(float fc_mhz);
