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

    // Hanning window — computed once
    for (int n = 0; n < N_FFT; n++)
        s_window[n] = 0.5f * (1.0f - std::cos(2.0f * M_PI * n / (N_FFT - 1)));

    s_init = true;
}

std::array<float, N_DISPLAY> process_step(
    float fc_mhz,
    const std::vector<std::complex<float>>& samples)
{
    (void)fc_mhz; // used only for logging — frequency axis computed separately

    if (!s_init) dsp_init();

    // ── 1. Load samples + apply Hanning window ────────────────────────────────
    for (int n = 0; n < N_FFT; n++)
    {
        s_fft_buf[n][0] = (samples[n].real() / 32768.0f) * s_window[n];
        s_fft_buf[n][1] = (samples[n].imag() / 32768.0f) * s_window[n];
    }

    // ── 2. FFT ────────────────────────────────────────────────────────────────
    fftwf_execute(s_plan);

    // ── 3. fftshift — verbatim from blah2 SpectrumAnalyser.cpp ───────────────
    //    (k + nfft/2 + 1) % nfft  maps output so DC is centred
    float linear[N_FFT];
    for (int k = 0; k < N_FFT; k++)
    {
        int ks = (k + N_FFT / 2 + 1) % N_FFT;
        float re = s_fft_buf[ks][0];
        float im = s_fft_buf[ks][1];

        // ── 4. Magnitude squared — stay in LINEAR power domain ───────────────
        linear[k] = re * re + im * im;
    }

    // ── 5. Decimate: average N_FFT/N_DISPLAY linear-power bins → 1 display bin
    //    Must stay linear here — averaging dB values is wrong
    constexpr int GROUP = N_FFT / N_DISPLAY; // = 128
    std::array<float, N_DISPLAY> display;
    for (int d = 0; d < N_DISPLAY; d++)
    {
        float sum = 0.0f;
        for (int g = 0; g < GROUP; g++)
            sum += linear[d * GROUP + g];
        display[d] = sum / GROUP;
    }

    // ── 6. dBFS conversion LAST ───────────────────────────────────────────────
    //    power = |X[k]|²,  dBFS = 10*log10(power / N_FFT²)
    //    equivalent to 20*log10(|X[k]| / N_FFT)
    const float norm = (float)N_FFT * (float)N_FFT;
    for (int d = 0; d < N_DISPLAY; d++)
    {
        float p = display[d] / norm;
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
