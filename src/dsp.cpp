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
static float          s_raw_acc[N_FFT];        // accumulator across N_AVG FFTs (linear power)
static float          s_raw_db[N_FFT];         // averaged dBFS — exposed via get_raw_db()
static float          s_raw_linear_out[N_FFT]; // normalised linear power — exposed via get_raw_linear()
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

const float* get_raw_linear()
{
    const float norm = (float)N_FFT * (float)N_FFT;
    for (int k = 0; k < N_FFT; k++)
        s_raw_linear_out[k] = (s_raw_acc[k] / N_AVG) / norm;
    return s_raw_linear_out;
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
    if (n <= 0) return {};

    // 0. noise_lin first — used for both SNR and SFM ε floor
    const float noise_lin = std::pow(10.0f, noise_db / 10.0f);

    // 1. Convert channel bins to linear power
    std::vector<float> lin(n);
    float total = 0.0f;
    for (int i = 0; i < n; i++) {
        lin[i] = std::pow(10.0f, raw_db[lo + i] / 10.0f);
        total += lin[i];
    }

    // 2. OBW — ITU-R SM.443-4 β/2 method (FM_OBW_BETA power containment)
    // Integrate from each edge until (FM_OBW_BETA/2) of total power is consumed.
    // At β=0.10 (90%): each edge must accumulate 5% of total power before cutting,
    // making it much harder for a single adjacent-channel leakage bin to inflate OBW.
    const float beta_half = (FM_OBW_BETA / 2.0f) * total;
    float cum = 0.0f;
    int lo_cut = 0;
    for (int i = 0; i < n; i++) {
        cum += lin[i];
        if (cum >= beta_half) { lo_cut = i; break; }
    }
    cum = 0.0f;
    int hi_cut = n - 1;
    for (int i = n - 1; i >= 0; i--) {
        cum += lin[i];
        if (cum >= beta_half) { hi_cut = i; break; }
    }
    const int   obw_bins     = std::max(0, hi_cut - lo_cut + 1);
    const float obw_fraction = (float)obw_bins / n;

    // 3. SFM (Wiener entropy) + crest factor within OBW only
    // ε = noise_lin/100 (20 dB below noise floor): noise-consistent floor prevents
    // log(0) collapse without materially affecting real broadcast signals.
    // max_lin_obw tracked free alongside sum_lin_obw for crest factor.
    float sum_lin_obw = 0.0f, sum_loglin_obw = 0.0f, max_lin_obw = 0.0f;
    const float eps = noise_lin * 0.01f;
    for (int i = lo_cut; i <= hi_cut; i++) {
        if (lin[i] > max_lin_obw) max_lin_obw = lin[i];
        sum_lin_obw    += lin[i];
        sum_loglin_obw += std::log(std::max(lin[i], eps));
    }
    float sfm = 0.0f;
    float crest_factor_db = 0.0f;
    if (obw_bins >= 3) {
        const float geo   = std::exp(sum_loglin_obw / obw_bins);
        const float arith = sum_lin_obw / obw_bins;
        sfm = (arith > 0.0f) ? geo / arith : 0.0f;
        crest_factor_db = (arith > 0.0f) ? 10.0f * std::log10(max_lin_obw / arith) : 0.0f;
    }

    // 4. SNR — mean power within OBW vs step-wide noise floor
    // Using OBW bins only avoids diluting signal power with silent channel edges.
    const float arith_obw = (obw_bins > 0) ? sum_lin_obw / obw_bins : 0.0f;
    const float snr_db    = (arith_obw > noise_lin && noise_lin > 0.0f)
                          ? 10.0f * std::log10(arith_obw / noise_lin)
                          : 0.0f;

    // 5. Centre power fraction — fraction of OBW power in the middle third.
    // Splits OBW into three equal thirds; sums the inner third vs total OBW power.
    // Real FM: ~0.33–0.50 (uniform or carrier-heavy centre).
    // Trough (null between two adjacent stations): ~0.02–0.10 → rejected by gate.
    // Slope (one-sided rolloff from adjacent station): ~0.10–0.25 → rejected by gate.
    float centre_power_frac = 0.0f;
    if (obw_bins >= 3) {
        const int third    = obw_bins / 3;
        const int inner_lo = lo_cut + third;
        const int inner_hi = hi_cut - third;
        float centre_power = 0.0f;
        for (int i = inner_lo; i <= inner_hi; i++)
            centre_power += lin[i];
        centre_power_frac = (sum_lin_obw > 0.0f) ? centre_power / sum_lin_obw : 0.0f;
    }

    // 6. OBW asymmetry — centroid offset from channel centre, normalised to [0,1] (diagnostic).
    const float obw_centre    = (obw_bins > 0) ? (lo_cut + hi_cut) / 2.0f : (n / 2.0f);
    const float obw_asymmetry = (n > 0)
                               ? std::fabs(obw_centre - (n / 2.0f)) / (n / 2.0f)
                               : 0.0f;

    return {snr_db, obw_fraction, sfm, centre_power_frac, crest_factor_db, obw_asymmetry, 0.0f};
}

float fm_score(const FmChannelMetrics& m)
{
    // Base gates — SNR, OBW, CPF applied to all algorithms
    // SFM retained in algos that explicitly use it (SNR_OBW_SFM)
    if (m.snr_db            < FM_SNR_GATE_DB       ||
        m.obw_fraction      < FM_MOB_GATE_FRAC     ||
        m.centre_power_frac < FM_CENTRE_POWER_GATE)
        return 0.0f;

    const float snr_norm = std::max(0.0f, std::min(1.0f,
        (m.snr_db - FM_SNR_NORM_MIN) / (FM_SNR_NORM_MAX - FM_SNR_NORM_MIN)));
#if FM_SCORE_ALGO == FM_SCORE_ALGO_GATE
    return snr_norm;
#elif FM_SCORE_ALGO == FM_SCORE_ALGO_SNR_OBW
    return snr_norm * m.obw_fraction;
#elif FM_SCORE_ALGO == FM_SCORE_ALGO_SNR_OBW_SFM
    return snr_norm * m.obw_fraction * m.sfm;
#elif FM_SCORE_ALGO == FM_SCORE_ALGO_SNR_OBW_CPF
    if (m.centre_power_frac < FM_CENTRE_POWER_GATE) return 0.0f;
    return snr_norm * m.obw_fraction * m.centre_power_frac;
#elif FM_SCORE_ALGO == FM_SCORE_ALGO_GATE_CPF
    if (m.centre_power_frac < FM_CENTRE_POWER_GATE) return 0.0f;
    return snr_norm;
#else
    return 0.0f;
#endif
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
