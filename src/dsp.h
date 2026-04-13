// Adapted from blah2-arm/src/process/spectrum/SpectrumAnalyser.h
// Changes: float/fftwf, no IqData, single FFT per step, correct DSP order

#pragma once

#include "config.h"

#include <array>
#include <complex>
#include <cmath>
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

// ── Channel peak detection ────────────────────────────────────────────────────

struct ChannelPeak {
    float freq_mhz;  // absolute frequency of this peak
    float power_db;
    bool  is_pilot;  // true if within PILOT_TOL_MHZ of the expected pilot frequency
};

// Returns pointer to the full-resolution N_FFT-length dBFS array populated by
// the last call to process_step() or process_step_focus(). Valid until the next
// DSP call. Use this with find_channel_peaks().
const float* get_raw_db();

// Find the top num_peaks local-maximum peaks within channel [ch_lo_mhz, ch_hi_mhz].
//   raw_db      — N_FFT array from get_raw_db()
//   step_fc_mhz — frequency the SDR was tuned to for this step
//   pilot_mhz   — absolute ATSC pilot frequency to check against (0.0 = FM: skip check)
//   num_peaks   — max peaks to return (typically NUM_CHANNEL_PEAKS)
// Returns peaks sorted by power descending.
std::vector<ChannelPeak> find_channel_peaks(
    const float* raw_db,
    int          n_fft,
    float        step_fc_mhz,
    float        ch_lo_mhz,
    float        ch_hi_mhz,
    float        pilot_mhz,
    float        tol_mhz,
    int          num_peaks);

// ── FM channel metrics ────────────────────────────────────────────────────────

struct FmChannelMetrics {
    float snr_db;    // mean channel power relative to per-step noise floor (dB)
    float occupancy; // fraction of channel bins > noise_floor+10dB (0=empty/CW, 1=full wideband)
};

// 25th percentile of raw_db[0..n_fft-1] converted to linear, O(n) via nth_element,
// returned in dB. Used as the per-step noise floor reference.
float estimate_step_noise_floor(const float* raw_db, int n_fft);

// Compute SNR + occupancy for one FM channel slot [ch_lo_mhz, ch_hi_mhz].
//   noise_db — from estimate_step_noise_floor() for this step
FmChannelMetrics compute_fm_metrics(
    const float* raw_db,
    int          n_fft,
    float        step_fc_mhz,
    float        ch_lo_mhz,
    float        ch_hi_mhz,
    float        noise_db);
