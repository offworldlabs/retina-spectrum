// Adapted from blah2-arm/src/process/spectrum/SpectrumAnalyser.cpp
// Changes from blah2:
//   - double/fftw_ → float/fftwf_
//   - Hanning window added before FFT (SpectrumAnalyser has no window)
//   - fftshift logic copied verbatim: (k + nfft/2 + 1) % nfft
//   - No averaging — single FFT per step is sufficient for broadcast towers
//   - Correct DSP order: stay in linear power domain, convert to dBFS LAST
//   - Frequency axis from actual fc_mhz (blah2 hardcodes 204640000 Hz offset)

#include "dsp.h"

#include <cmath>
#include <fftw3.h>
#include <iostream>

// Static resources — allocated once, reused across steps.
// Safe: only the sweep thread ever calls process_step().
static fftwf_complex *s_fft_buf = nullptr;
static fftwf_plan     s_plan    = nullptr;
static float          s_window[N_FFT];
static bool           s_init    = false;

static void dsp_init()
{
    s_fft_buf = fftwf_alloc_complex(N_FFT);
    if (!s_fft_buf)
    {
        std::cerr << "Error: fftwf_alloc_complex failed" << std::endl;
        exit(1);
    }

    // in-place forward DFT
    s_plan = fftwf_plan_dft_1d(N_FFT, s_fft_buf, s_fft_buf, FFTW_FORWARD, FFTW_ESTIMATE);
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
    float linear[N_FFT];
    std::array<float, N_DISPLAY> acc{};  // linear power accumulator across N_AVG FFTs

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

        // ── 3. fftshift + magnitude squared ──────────────────────────────
        for (int k = 0; k < N_FFT; k++)
        {
            int ks = (k + N_FFT / 2 + 1) % N_FFT;
            float re = s_fft_buf[ks][0];
            float im = s_fft_buf[ks][1];
            linear[k] = re * re + im * im;
        }

        // ── 4. Decimate: peak-detect within each display bin group ───────
        // Take the strongest FFT bin in each group of GROUP=128 bins.
        // Averaging would suppress a narrowband FM carrier by ~21 dB (10*log10(128)).
        // Peak detection ensures signals always report at correct level (RSP manual).
        for (int d = 0; d < N_DISPLAY; d++)
        {
            float peak = 0.0f;
            for (int g = 0; g < GROUP; g++)
                if (linear[d * GROUP + g] > peak)
                    peak = linear[d * GROUP + g];
            acc[d] += peak;
        }
    }

    // ── 5. Average across N_AVG FFTs, then dBFS ──────────────────────────────
    //    dBFS = 10*log10(power / N_FFT²),  norm accounts for FFT scaling
    const float norm = (float)N_FFT * (float)N_FFT;
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
    float linear[N_FFT];
    std::array<float, N_DISPLAY_FOCUS> acc{};

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
            linear[k] = re * re + im * im;
        }

        for (int d = 0; d < N_DISPLAY_FOCUS; d++)
        {
            float peak = 0.0f;
            for (int g = 0; g < GROUP; g++)
                if (linear[d * GROUP + g] > peak)
                    peak = linear[d * GROUP + g];
            acc[d] += peak;
        }
    }

    const float norm = (float)N_FFT * (float)N_FFT;
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
