// Adapted from blah2-arm/src/process/spectrum/SpectrumAnalyser.h
// Changes: float/fftwf, no IqData, single FFT per step, correct DSP order

#pragma once

#include "config.h"

#include <array>
#include <complex>
#include <vector>

// Process one captured step — sweep mode (N_DISPLAY bins, 125 kHz/bin)
std::array<float, N_DISPLAY> process_step(
    float fc_mhz,
    const std::vector<std::complex<float>>& samples);

// Process one captured step — focus mode (N_DISPLAY_FOCUS bins, 7.8 kHz/bin)
std::array<float, N_DISPLAY_FOCUS> process_step_focus(
    float fc_mhz,
    const std::vector<std::complex<float>>& samples);

// Frequency axis — centre MHz of each display bin
std::array<float, N_DISPLAY> freq_axis(float fc_mhz);
