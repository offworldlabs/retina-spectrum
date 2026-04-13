// Adapted from blah2-arm/src/process/spectrum/SpectrumAnalyser.cpp
// Changes from blah2:
//   - double/fftw_ → float/fftwf_
//   - Hanning window added before FFT (SpectrumAnalyser has no window)
//   - fftshift logic copied verbatim: (k + nfft/2 + 1) % nfft
//   - No averaging — single FFT per step is sufficient for broadcast towers
//   - Correct DSP order: stay in linear power domain, convert to dBFS LAST
//   - Frequency axis from actual fc_mhz (blah2 hardcodes 204640000 Hz offset)

#include "dsp.h"

#include <algorithm>
#include <cmath>
#include <fftw3.h>
#include <iostream>

// Static resources — allocated once, reused across steps.
// Safe: only the sweep thread ever calls process_step().
static fftwf_complex *s_fft_buf   = nullptr;
static fftwf_plan     s_plan      = nullptr;
static float          s_window[N_FFT];
static float          s_linear[N_FFT];    // per-FFT fftshifted magnitude², heap via static
static float          s_raw_acc[N_FFT];   // accumulator across N_AVG FFTs (linear power)
static float          s_raw_db[N_FFT];    // averaged dBFS — exposed via get_raw_db()
static bool           s_init      = false;

static void dsp_init()
{
    s_fft_buf = fftwf_alloc_complex(N_FFT);
    if (!s_fft_buf)
    {
        std::cerr << "Error: fftwf_alloc_complex failed" << std::endl;
        exit(1);
    }

    // in-place forward DFT — FFTW_MEASURE benchmarks algorithms at startup (~seconds)
    // for a faster plan on this specific hardware; output is mathematically identical.
    s_plan = fftwf_plan_dft_1d(N_FFT, s_fft_buf, s_fft_buf, FFTW_FORWARD, FFTW_MEASURE);
    if (!s_plan)
    {
        std::cerr << "Error: fftwf_plan_dft_1d failed" << std::endl;
        exit(1);
    }

    // Blackman window — computed once
    // −74 dB sidelobe suppression (vs −32 dB Hanning): prevents strong FM stations
    // from masking weaker neighbours through spectral leakage
    for (int n = 0; n < N_FFT; n++)
        s_window[n] = 0.42f
                    - 0.5f  * std::cos(2.0f * M_PI * n / (N_FFT - 1))
                    + 0.08f * std::cos(4.0f * M_PI * n / (N_FFT - 1));

    s_init = true;
}

std::array<float, N_DISPLAY> process_step(
    float fc_mhz,
    const std::vector<std::complex<float>>& samples)
{
    (void)fc_mhz; // used only for logging — frequency axis computed separately

    if (!s_init) dsp_init();

    constexpr int GROUP = N_FFT / N_DISPLAY;
    std::array<float, N_DISPLAY> acc{};  // display bin accumulator across N_AVG FFTs
    for (int k = 0; k < N_FFT; k++) s_raw_acc[k] = 0.0f;  // reset raw accumulator

    for (int avg = 0; avg < N_AVG; avg++)
    {
        int offset = avg * N_FFT;

        // ── 1. Load samples + apply Blackman window ───────────────────────
        // Normalise by 2048: RSPduo at Zero-IF 8MS/s = 12-bit ADC (RSP manual p.21).
        // API delivers right-justified samples in 16-bit shorts → full-scale peak = 2^11 = 2048.
        // /32768 would read 24 dB too low.
        for (int n = 0; n < N_FFT; n++)
        {
            s_fft_buf[n][0] = (samples[offset + n].real() / 2048.0f) * s_window[n];
            s_fft_buf[n][1] = (samples[offset + n].imag() / 2048.0f) * s_window[n];
        }

        // ── 2. FFT ────────────────────────────────────────────────────────
        fftwf_execute(s_plan);

        // ── 3. fftshift + magnitude squared + raw accumulate ─────────────
        for (int k = 0; k < N_FFT; k++)
        {
            int ks = (k + N_FFT / 2 + 1) % N_FFT;
            float re = s_fft_buf[ks][0];
            float im = s_fft_buf[ks][1];
            s_linear[k]   = re * re + im * im;
            s_raw_acc[k] += s_linear[k];  // full-resolution accumulation for peak detection
        }

        // ── 4. Decimate: peak-detect within each display bin group ───────
        // Take the strongest FFT bin in each group of GROUP=128 bins.
        // Averaging would suppress a narrowband FM carrier by ~21 dB (10*log10(128)).
        // Peak detection ensures signals always report at correct level (RSP manual).
        for (int d = 0; d < N_DISPLAY; d++)
        {
            float peak = 0.0f;
            for (int g = 0; g < GROUP; g++)
                if (s_linear[d * GROUP + g] > peak)
                    peak = s_linear[d * GROUP + g];
            acc[d] += peak;
        }
    }

    // ── 5. Average across N_AVG FFTs, then dBFS ──────────────────────────────
    //    dBFS = 10*log10(power / N_FFT²),  norm accounts for FFT scaling
    const float norm = (float)N_FFT * (float)N_FFT;
    for (int k = 0; k < N_FFT; k++)
    {
        float p = (s_raw_acc[k] / N_AVG) / norm;
        s_raw_db[k] = (p > 0.0f) ? 10.0f * log10f(p) : -120.0f;
    }
    std::array<float, N_DISPLAY> display;
    for (int d = 0; d < N_DISPLAY; d++)
    {
        float p = (acc[d] / N_AVG) / norm;
        display[d] = (p > 0.0f) ? 10.0f * log10f(p) : -120.0f;
    }

    return display;
}

// Focus mode — same pipeline as process_step but N_DISPLAY_FOCUS bins (7.8 kHz/bin)
std::array<float, N_DISPLAY_FOCUS> process_step_focus(
    float fc_mhz,
    const std::vector<std::complex<float>>& samples)
{
    (void)fc_mhz;

    if (!s_init) dsp_init();

    constexpr int GROUP = N_FFT / N_DISPLAY_FOCUS;  // = 8 FFT bins per display bin
    std::array<float, N_DISPLAY_FOCUS> acc{};
    for (int k = 0; k < N_FFT; k++) s_raw_acc[k] = 0.0f;

    for (int avg = 0; avg < N_AVG; avg++)
    {
        int offset = avg * N_FFT;

        for (int n = 0; n < N_FFT; n++)
        {
            s_fft_buf[n][0] = (samples[offset + n].real() / 2048.0f) * s_window[n];
            s_fft_buf[n][1] = (samples[offset + n].imag() / 2048.0f) * s_window[n];
        }

        fftwf_execute(s_plan);

        for (int k = 0; k < N_FFT; k++)
        {
            int ks = (k + N_FFT / 2 + 1) % N_FFT;
            float re = s_fft_buf[ks][0];
            float im = s_fft_buf[ks][1];
            s_linear[k]   = re * re + im * im;
            s_raw_acc[k] += s_linear[k];
        }

        for (int d = 0; d < N_DISPLAY_FOCUS; d++)
        {
            float peak = 0.0f;
            for (int g = 0; g < GROUP; g++)
                if (s_linear[d * GROUP + g] > peak)
                    peak = s_linear[d * GROUP + g];
            acc[d] += peak;
        }
    }

    const float norm = (float)N_FFT * (float)N_FFT;
    for (int k = 0; k < N_FFT; k++)
    {
        float p = (s_raw_acc[k] / N_AVG) / norm;
        s_raw_db[k] = (p > 0.0f) ? 10.0f * log10f(p) : -120.0f;
    }
    std::array<float, N_DISPLAY_FOCUS> display;
    for (int d = 0; d < N_DISPLAY_FOCUS; d++)
    {
        float p = (acc[d] / N_AVG) / norm;
        display[d] = (p > 0.0f) ? 10.0f * log10f(p) : -120.0f;
    }

    return display;
}

// Frequency axis — centre MHz of each display bin for a given step
std::array<float, N_DISPLAY> freq_axis(float fc_mhz)
{
    std::array<float, N_DISPLAY> freqs;
    const float bin_width = 8.0f / N_DISPLAY; // MHz per display bin
    const float f_start   = fc_mhz - 4.0f;    // left edge of 8 MHz window
    for (int d = 0; d < N_DISPLAY; d++)
        freqs[d] = f_start + (d + 0.5f) * bin_width;
    return freqs;
}

// ── Channel peak detection ────────────────────────────────────────────────────

const float* get_raw_db()
{
    return s_raw_db;
}

// ── FM channel metrics ────────────────────────────────────────────────────────

float estimate_step_noise_floor(const float* raw_db, int n_fft)
{
    // Convert all bins to linear power, find 25th percentile via nth_element O(n).
    // In a dense FM market <20% of bins are occupied so the 25th percentile always
    // lands on noise-only bins regardless of how many stations are active.
    std::vector<float> lin(n_fft);
    for (int k = 0; k < n_fft; k++)
        lin[k] = std::pow(10.0f, raw_db[k] / 10.0f);

    int p25 = n_fft / 4;
    std::nth_element(lin.begin(), lin.begin() + p25, lin.end());
    return (lin[p25] > 0.0f) ? 10.0f * std::log10(lin[p25]) : -120.0f;
}

FmChannelMetrics compute_fm_metrics(
    const float* raw_db,
    int          n_fft,
    float        step_fc_mhz,
    float        ch_lo_mhz,
    float        ch_hi_mhz,
    float        noise_db)
{
    const float bin_mhz = (float)SAMPLE_RATE_HZ / 1e6f / n_fft;
    auto freq_to_idx = [&](float f_mhz) -> int {
        int idx = n_fft / 2 + (int)roundf((f_mhz - step_fc_mhz) / bin_mhz);
        return std::max(0, std::min(n_fft - 1, idx));
    };

    int lo = freq_to_idx(ch_lo_mhz);
    int hi = freq_to_idx(ch_hi_mhz);
    int n  = hi - lo;
    if (n <= 0) return {0.0f, 0.0f};

    // Count occupied bins (> noise_floor + 10 dB) and accumulate mean power.
    const float sig_threshold_db = noise_db + 10.0f;
    float sum_lin  = 0.0f;
    int n_occupied = 0;
    for (int k = lo; k < hi; k++) {
        sum_lin += std::pow(10.0f, raw_db[k] / 10.0f);
        if (raw_db[k] > sig_threshold_db) n_occupied++;
    }

    float arith_mean = sum_lin / n;
    float noise_lin  = std::pow(10.0f, noise_db / 10.0f);
    float snr_db     = (arith_mean > noise_lin && noise_lin > 0.0f)
                     ? 10.0f * std::log10(arith_mean / noise_lin)
                     : 0.0f;
    float occupancy  = (float)n_occupied / n;

    return {snr_db, occupancy};
}

// Find the top num_peaks local-maximum peaks within [ch_lo_mhz, ch_hi_mhz].
// Operates on the full-resolution N_FFT dBFS array from the last DSP step.
// Peaks are sorted by power descending. pilot_mhz=0.0 skips the pilot check.
std::vector<ChannelPeak> find_channel_peaks(
    const float* raw_db,
    int          n_fft,
    float        step_fc_mhz,
    float        ch_lo_mhz,
    float        ch_hi_mhz,
    float        pilot_mhz,
    float        tol_mhz,
    int          num_peaks)
{
    // Bin width in MHz
    const float bin_mhz = (float)SAMPLE_RATE_HZ / 1e6f / n_fft;  // ~0.000977 MHz

    // Convert channel edges to indices in the fftshifted raw_db array.
    // raw_db[n_fft/2] ≈ DC (step_fc_mhz). Positive offset → higher index.
    auto freq_to_idx = [&](float f_mhz) -> int {
        int idx = n_fft / 2 + (int)roundf((f_mhz - step_fc_mhz) / bin_mhz);
        return std::max(1, std::min(n_fft - 2, idx));
    };

    int lo = freq_to_idx(ch_lo_mhz);
    int hi = freq_to_idx(ch_hi_mhz);
    if (lo >= hi) return {};

    // Collect local maxima: raw_db[k] > both neighbours
    struct Candidate { int idx; float power; };
    std::vector<Candidate> candidates;
    for (int k = lo + 1; k < hi; k++)
        if (raw_db[k] > raw_db[k - 1] && raw_db[k] > raw_db[k + 1])
            candidates.push_back({k, raw_db[k]});

    // Sort by power descending, keep top num_peaks
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b){ return a.power > b.power; });
    if ((int)candidates.size() > num_peaks)
        candidates.resize(num_peaks);

    // Convert to ChannelPeak results
    std::vector<ChannelPeak> result;
    result.reserve(candidates.size());
    for (auto& c : candidates)
    {
        float freq_mhz = step_fc_mhz + (c.idx - n_fft / 2) * bin_mhz;
        bool  is_pilot = (pilot_mhz > 0.0f)
                      && (fabsf(freq_mhz - pilot_mhz) <= tol_mhz)
                      && (c.power >= PEAK_MIN_DBF);
        result.push_back({freq_mhz, c.power, is_pilot});
    }
    return result;
}
