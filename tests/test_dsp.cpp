#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_approx.hpp>

#include "dsp.h"
#include "config.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <numeric>
#include <vector>

// ── IQ sample generators ──────────────────────────────────────────────────────

// Gaussian noise via Box-Muller, fixed seed for reproducibility
static std::vector<std::complex<float>> make_noise_iq(float noise_std, unsigned seed = 42)
{
    const int N = N_FFT * N_AVG;
    std::vector<std::complex<float>> buf(N);
    srand(seed);
    for (int n = 0; n < N; n++) {
        float u1 = (rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
        float u2 = (float)rand() / (float)RAND_MAX;
        float r  = sqrtf(-2.0f * logf(u1)) * noise_std;
        buf[n] = { r * cosf(2.0f * M_PI * u2), r * sinf(2.0f * M_PI * u2) };
    }
    return buf;
}

// Single complex tone at offset_hz from fc, plus optional noise
static std::vector<std::complex<float>> make_tone_iq(
    float offset_hz, float amplitude,
    float noise_std = 0.0f, unsigned seed = 42)
{
    const int   N  = N_FFT * N_AVG;
    const float fs = (float)SAMPLE_RATE_HZ;
    auto buf = make_noise_iq(noise_std, seed);
    float phase     = 0.0f;
    float phase_inc = 2.0f * M_PI * offset_hz / fs;
    for (int n = 0; n < N; n++) {
        buf[n] = { buf[n].real() + amplitude * cosf(phase),
                   buf[n].imag() + amplitude * sinf(phase) };
        phase += phase_inc;
        if (phase >  M_PI) phase -= 2.0f * M_PI;
        if (phase < -M_PI) phase += 2.0f * M_PI;
    }
    return buf;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static int peak_bin(const std::array<float, N_DISPLAY>& out)
{
    return (int)(std::max_element(out.begin(), out.end()) - out.begin());
}

// Bin in freq_axis(fc) whose centre is closest to target_mhz
static int nearest_bin(float fc, float target_mhz)
{
    auto freqs = freq_axis(fc);
    int best = 0;
    for (int d = 1; d < N_DISPLAY; d++)
        if (fabsf(freqs[d] - target_mhz) < fabsf(freqs[best] - target_mhz))
            best = d;
    return best;
}

// Theoretical peak dBFS for Blackman-windowed CW tone at exact FFT bin
// Blackman coherent gain (DC component of window) ≈ 0.42
static float theory_dbfs(float amplitude)
{
    return 20.0f * log10f(amplitude * 0.42f / 2048.0f);
}

// ── 1. freq_axis ──────────────────────────────────────────────────────────────

TEST_CASE("freq_axis: bin centres span fc ± 4 MHz with 0.125 MHz spacing", "[dsp][freq_axis]")
{
    const float fc = 100.0f;
    auto freqs = freq_axis(fc);

    REQUIRE((int)freqs.size() == N_DISPLAY);

    // Leftmost and rightmost centres should be ~4 MHz from fc
    CHECK(freqs[0]  == Catch::Approx(fc - 4.0f + 0.0625f).margin(0.01f));
    CHECK(freqs[63] == Catch::Approx(fc + 4.0f - 0.0625f).margin(0.01f));

    // Uniform spacing
    for (int d = 1; d < N_DISPLAY; d++)
        CHECK(freqs[d] - freqs[d-1] == Catch::Approx(8.0f / N_DISPLAY).margin(1e-4f));
}

// ── 2. Stitching ──────────────────────────────────────────────────────────────

TEST_CASE("stitching: consecutive 3 MHz steps within each band are gapless", "[dsp][stitching]")
{
    struct Band { int start; int stop; int step; };
    const Band BANDS[] = { {88,108,3}, {174,240,3}, {468,693,3} };
    constexpr float HALF = 1.5f;  // TRIM_HALF_MHZ

    for (auto& b : BANDS) {
        float prev_stop = -1.0f;
        for (int fc = b.start; fc <= b.stop; fc += b.step) {
            float step_start = (float)fc - HALF;
            float step_stop  = (float)fc + HALF;
            if (prev_stop >= 0.0f)
                CHECK(step_start == Catch::Approx(prev_stop).margin(1e-4f));
            prev_stop = step_stop;
        }
    }
}

// ── 3. Noise floor ────────────────────────────────────────────────────────────

TEST_CASE("noise floor: Gaussian noise (std=30 ADC counts) stays below -50 dBFS", "[dsp][noise]")
{
    // This validates /2048 normalisation — wrong divisor (/32768) would push
    // all bins up ~24 dB, breaking this test immediately.
    auto buf = make_noise_iq(30.0f, 42);
    auto out = process_step(100.0f, buf);

    for (int d = 0; d < N_DISPLAY; d++)
        INFO("bin " << d << " = " << out[d] << " dBFS");

    float max_noise = *std::max_element(out.begin(), out.end());
    CHECK(max_noise < -50.0f);
}

// ── 4. Tone bin placement — parametric sweep ──────────────────────────────────

TEST_CASE("tone bin placement: peak lands in expected display bin (±1) at various offsets", "[dsp][tone][placement]")
{
    const float fc         = 100.0f;
    const float amplitude  = 1500.0f;
    const float offset_mhz = GENERATE(-3.0f, -2.0f, -1.0f, -0.5f, 0.5f, 1.0f, 2.0f, 3.0f);

    INFO("fc=" << fc << " MHz, tone offset=" << offset_mhz << " MHz");

    auto buf = make_tone_iq(offset_mhz * 1e6f, amplitude, 0.0f);
    auto out = process_step(fc, buf);

    int expected = nearest_bin(fc, fc + offset_mhz);
    int actual   = peak_bin(out);

    CHECK(abs(actual - expected) <= 1);
}

// ── 5. Tone amplitude — within 6 dB of Blackman coherent gain theory ─────────

TEST_CASE("tone amplitude: measured dBFS within 6 dB of Blackman theory", "[dsp][tone][amplitude]")
{
    // +0.125 MHz = exactly 1 display-bin width = 128 FFT bins from DC.
    // This lands on an exact FFT bin, minimising scalloping loss.
    const float fc        = 100.0f;
    const float offset_hz = 125000.0f;
    const float amplitude = 1500.0f;

    auto buf = make_tone_iq(offset_hz, amplitude, 0.0f);
    auto out = process_step(fc, buf);

    float measured  = out[peak_bin(out)];
    float expected  = theory_dbfs(amplitude);

    INFO("measured=" << measured << " dBFS, theory=" << expected << " dBFS");
    CHECK(measured > expected - 6.0f);
    CHECK(measured < expected + 6.0f);
}

// ── 6. SNR — tone clearly above noise floor ───────────────────────────────────

TEST_CASE("SNR: A=1500 tone sits >30 dB above noise floor (std=30)", "[dsp][snr]")
{
    auto buf = make_tone_iq(1.0e6f, 1500.0f, 30.0f, 42);
    auto out = process_step(100.0f, buf);

    float peak = *std::max_element(out.begin(), out.end());

    // Bottom-quartile average as noise floor estimate
    auto sorted = std::vector<float>(out.begin(), out.end());
    std::sort(sorted.begin(), sorted.end());
    float floor_avg = std::accumulate(sorted.begin(), sorted.begin() + N_DISPLAY/4, 0.0f)
                    / (N_DISPLAY / 4);

    INFO("peak=" << peak << " dBFS, floor=" << floor_avg << " dBFS, SNR=" << (peak - floor_avg) << " dB");

    CHECK(peak       > -25.0f);
    CHECK(floor_avg  < -55.0f);
    CHECK(peak - floor_avg > 30.0f);
}

// ── 7. Peak-max decimation — not averaging ────────────────────────────────────

TEST_CASE("peak-max decimation: strong tone in bin group not buried by averaging", "[dsp][decimation]")
{
    // Place a strong tone at +0.125 MHz (display bin ~32/33).
    // If the DSP were averaging 128 FFT bins instead of peak-detecting,
    // the output level would be ~21 dB lower (10*log10(128)).
    const float amplitude = 1500.0f;
    auto buf = make_tone_iq(125000.0f, amplitude, 0.0f);
    auto out = process_step(100.0f, buf);

    float measured = out[peak_bin(out)];
    float theory   = theory_dbfs(amplitude);

    // With averaging, result would be theory - 21 dB. With peak-max it's near theory.
    // Assert we're within 10 dB of theory (not 21 dB below it).
    INFO("measured=" << measured << " dBFS, theory=" << theory << " dBFS");
    CHECK(measured > theory - 10.0f);
}

// ── 8. End-to-end: specific frequency in → correct frequency + level out ──────

TEST_CASE("end-to-end: tone at known frequency reads back at correct freq and dBFS", "[dsp][e2e]")
{
    // Simulate realistic stations: fc chosen so tone is within ±1.5 MHz trim window.
    // For each: verify (a) peak bin is within 1 bin of the target frequency,
    //           (b) level is within 6 dB of Blackman theory for that amplitude.
    struct Case { float fc; float tone_mhz; float amplitude; const char *label; };
    auto c = GENERATE(
        Case{ 99.0f,   98.8f,  800.0f, "FM 98.8 MHz"    },
        Case{ 99.0f,   98.0f, 1500.0f, "FM 98.0 MHz"    },
        Case{199.0f,  198.31f, 500.0f, "VHF ch11 198.31 MHz"},
        Case{530.0f,  530.0f, 1000.0f, "DVB-T 530 MHz"  },
        Case{610.0f,  609.5f,  600.0f, "DVB-T 609.5 MHz"}
    );

    INFO("Case: " << c.label << "  fc=" << c.fc << " MHz  A=" << c.amplitude);

    auto buf = make_tone_iq((c.tone_mhz - c.fc) * 1e6f, c.amplitude, 0.0f);
    auto out = process_step(c.fc, buf);
    auto freqs = freq_axis(c.fc);

    // ── Frequency check ──────────────────────────────────────────────────────
    int pb = peak_bin(out);
    float measured_freq = freqs[pb];
    INFO("peak bin=" << pb << "  measured freq=" << measured_freq << " MHz  target=" << c.tone_mhz << " MHz");
    CHECK(fabsf(measured_freq - c.tone_mhz) <= 8.0f / N_DISPLAY);  // within 1 display bin (0.125 MHz)

    // ── Level check ──────────────────────────────────────────────────────────
    float measured_dbfs = out[pb];
    float expected_dbfs = theory_dbfs(c.amplitude);
    INFO("measured=" << measured_dbfs << " dBFS  theory=" << expected_dbfs << " dBFS");
    CHECK(measured_dbfs > expected_dbfs - 6.0f);
    CHECK(measured_dbfs < expected_dbfs + 6.0f);
}

// ── 10. estimate_step_noise_floor ────────────────────────────────────────────

TEST_CASE("noise floor estimate: 25th percentile lands on noise bins, not signal bins", "[dsp][fm][noise_floor]")
{
    // 90% of bins at -87 dBFS (noise), top 10% at -30 dBFS (signal).
    // 25th percentile index = N_FFT/4 = 2048, well within the 90% noise region.
    const float noise_db  = -87.0f;
    const float signal_db = -30.0f;
    std::vector<float> raw(N_FFT, noise_db);
    for (int k = N_FFT * 9 / 10; k < N_FFT; k++)
        raw[k] = signal_db;

    float result = estimate_step_noise_floor(raw.data(), N_FFT);
    INFO("result=" << result << " dBFS, expected≈" << noise_db);
    CHECK(result == Catch::Approx(noise_db).margin(1.0f));
}

TEST_CASE("noise floor estimate: uniform array returns that level", "[dsp][fm][noise_floor]")
{
    const float level = -80.0f;
    std::vector<float> raw(N_FFT, level);
    float result = estimate_step_noise_floor(raw.data(), N_FFT);
    INFO("result=" << result << " dBFS, expected=" << level);
    CHECK(result == Catch::Approx(level).margin(0.5f));
}

// ── 11. compute_fm_metrics ────────────────────────────────────────────────────

// Helper: returns the number of channel bins for a 200 kHz window at fc=98.7 MHz
static int channel_bin_count()
{
    const float bin_mhz = (float)SAMPLE_RATE_HZ / 1e6f / N_FFT;
    const float fc      = 98.7f;
    // replicate freq_to_idx logic used in compute_fm_metrics
    int lo = (int)std::round((fc - 0.1f - (fc - (float)SAMPLE_RATE_HZ / 2e6f)) / bin_mhz);
    int hi = (int)std::round((fc + 0.1f - (fc - (float)SAMPLE_RATE_HZ / 2e6f)) / bin_mhz);
    lo = std::max(0, std::min(N_FFT - 1, lo));
    hi = std::max(0, std::min(N_FFT - 1, hi));
    return hi - lo + 1;
}

TEST_CASE("FM metrics: uniform channel → obw≈1.0, sfm≈1.0, snr>0, cpf≈0.33", "[dsp][fm][metrics]")
{
    // Flat wideband signal: all channel bins equal, well above noise.
    const float noise_db   = -87.0f;
    const float channel_db = -60.0f;
    const float fc         = 98.7f;
    const float bin_mhz    = (float)SAMPLE_RATE_HZ / 1e6f / N_FFT;
    std::vector<float> raw(N_FFT, noise_db);

    int lo = N_FFT / 2 + (int)std::round(-0.1f / bin_mhz);
    int hi = N_FFT / 2 + (int)std::round( 0.1f / bin_mhz);
    for (int k = lo; k <= hi; k++) raw[k] = channel_db;

    FmChannelMetrics m = compute_fm_metrics(raw.data(), N_FFT, fc, fc - 0.1f, fc + 0.1f, noise_db);
    INFO("obw_fraction=" << m.obw_fraction << " sfm=" << m.sfm << " snr_db=" << m.snr_db
         << " crest_factor_db=" << m.crest_factor_db << " centre_power_frac=" << m.centre_power_frac);
    CHECK(m.obw_fraction        >= 0.85f);  // β=0.10 trims ~5% each edge: ~90% of channel
    CHECK(m.sfm                 >= 0.95f);
    CHECK(m.snr_db              >  0.0f);
    CHECK(m.crest_factor_db     <  3.0f);
    CHECK(m.centre_power_frac   == Catch::Approx(1.0f/3.0f).margin(0.05f));  // uniform → middle third ≈ 1/3
}

TEST_CASE("FM metrics: CW spike → obw<0.02, sfm=0, snr>10", "[dsp][fm][metrics]")
{
    // Single dominant tone at channel centre.
    const float noise_db = -87.0f;
    const float fc       = 98.7f;
    std::vector<float> raw(N_FFT, noise_db);
    raw[N_FFT / 2] = -30.0f;  // 57 dB above noise

    FmChannelMetrics m = compute_fm_metrics(raw.data(), N_FFT, fc, fc - 0.1f, fc + 0.1f, noise_db);
    INFO("obw_fraction=" << m.obw_fraction << " sfm=" << m.sfm << " snr_db=" << m.snr_db
         << " crest_factor_db=" << m.crest_factor_db);
    CHECK(m.obw_fraction <  0.02f);
    CHECK(m.sfm          == Catch::Approx(0.0f).margin(0.05f));
    CHECK(m.snr_db       >  10.0f);
    CHECK(m.crest_factor_db < 3.0f);  // obw_bins < 3 → CF guard fires, returns 0.0f
}

TEST_CASE("FM metrics: half-occupied → obw≈0.49, sfm≈1.0", "[dsp][fm][metrics]")
{
    // Bins 0..n/2-1 of channel at signal level, upper half at noise.
    // OBW should cut at the signal/noise boundary (~0.49).
    // SFM within OBW (uniform signal only) should be ≈1.0.
    const float noise_db   = -87.0f;
    const float channel_db = -60.0f;
    const float fc         = 98.7f;
    const float bin_mhz    = (float)SAMPLE_RATE_HZ / 1e6f / N_FFT;
    std::vector<float> raw(N_FFT, noise_db);

    int lo = N_FFT / 2 + (int)std::round(-0.1f / bin_mhz);
    int hi = N_FFT / 2 + (int)std::round( 0.1f / bin_mhz);
    int n  = hi - lo + 1;
    // Fill lower half of channel with signal
    for (int k = lo; k < lo + n / 2; k++) raw[k] = channel_db;

    FmChannelMetrics m = compute_fm_metrics(raw.data(), N_FFT, fc, fc - 0.1f, fc + 0.1f, noise_db);
    INFO("obw_fraction=" << m.obw_fraction << " sfm=" << m.sfm
         << " crest_factor_db=" << m.crest_factor_db);
    // β=0.10 trims ~5 bins from each edge of the occupied half → fraction slightly < 0.49
    CHECK(m.obw_fraction == Catch::Approx(0.46f).margin(0.05f));
    CHECK(m.sfm          == Catch::Approx(1.0f).margin(0.02f));
    CHECK(m.crest_factor_db < 3.0f);  // uniform signal in OBW: max/mean ≈ 1 → CF ≈ 0 dB
}

TEST_CASE("FM metrics: empty/noise-only → obw≈0.99, sfm≈1.0, snr≈0", "[dsp][fm][metrics]")
{
    // All bins at noise level. OBW is ~1.0 (noise fills full bandwidth).
    // SNR ≈ 0 — that's the noise gate, not OBW.
    const float noise_db = -87.0f;
    const float fc       = 98.7f;
    std::vector<float> raw(N_FFT, noise_db);

    FmChannelMetrics m = compute_fm_metrics(raw.data(), N_FFT, fc, fc - 0.1f, fc + 0.1f, noise_db);
    INFO("obw_fraction=" << m.obw_fraction << " sfm=" << m.sfm << " snr_db=" << m.snr_db
         << " crest_factor_db=" << m.crest_factor_db);
    CHECK(m.obw_fraction >= 0.85f);  // β=0.10 trims ~5% each edge from uniform noise
    CHECK(m.sfm          >= 0.90f);
    CHECK(m.snr_db       == Catch::Approx(0.0f).margin(2.0f));
    CHECK(m.crest_factor_db < 6.0f);  // uniform noise: max/mean close to 1 → CF ≈ 0 dB
}

TEST_CASE("FM metrics: two-tone → obw≈1.0, sfm<0.05", "[dsp][fm][metrics]")
{
    // Spikes at both channel edges: full OBW but all power in 2 bins → sfm≈0.
    const float noise_db = -87.0f;
    const float fc       = 98.7f;
    const float bin_mhz  = (float)SAMPLE_RATE_HZ / 1e6f / N_FFT;
    std::vector<float> raw(N_FFT, noise_db);

    int lo = N_FFT / 2 + (int)std::round(-0.1f / bin_mhz);
    int hi = N_FFT / 2 + (int)std::round( 0.1f / bin_mhz);
    raw[lo]     = -40.0f;  // first bin in channel window
    raw[hi - 1] = -40.0f;  // last bin in channel window (n = hi-lo, bins lo..hi-1)

    FmChannelMetrics m = compute_fm_metrics(raw.data(), N_FFT, fc, fc - 0.1f, fc + 0.1f, noise_db);
    INFO("obw_fraction=" << m.obw_fraction << " sfm=" << m.sfm
         << " crest_factor_db=" << m.crest_factor_db);
    CHECK(m.obw_fraction >= 0.95f);  // edge spikes → OBW = full channel (β consumed by spikes)
    CHECK(m.sfm          <  0.05f);
    CHECK(m.crest_factor_db > 15.0f);  // two spikes dominate: max >> mean → CF > 15 dB
}

TEST_CASE("FM metrics: eps floor — single zero bin in OBW does not collapse sfm", "[dsp][fm][metrics]")
{
    // All channel bins at -60 dBFS except centre bin at -120 dBFS (effectively zero).
    // Without ε clamp, geo_mean collapses to 0; with ε=noise_lin*0.01 sfm stays near 1.0.
    const float noise_db   = -87.0f;
    const float channel_db = -60.0f;
    const float fc         = 98.7f;
    const float bin_mhz    = (float)SAMPLE_RATE_HZ / 1e6f / N_FFT;
    std::vector<float> raw(N_FFT, noise_db);

    int lo = N_FFT / 2 + (int)std::round(-0.1f / bin_mhz);
    int hi = N_FFT / 2 + (int)std::round( 0.1f / bin_mhz);
    for (int k = lo; k <= hi; k++) raw[k] = channel_db;
    raw[(lo + hi) / 2] = -120.0f;  // one effectively-zero bin inside OBW

    FmChannelMetrics m = compute_fm_metrics(raw.data(), N_FFT, fc, fc - 0.1f, fc + 0.1f, noise_db);
    INFO("obw_fraction=" << m.obw_fraction << " sfm=" << m.sfm
         << " crest_factor_db=" << m.crest_factor_db);
    CHECK(m.obw_fraction >= 0.85f);  // β=0.10 trims ~5% each edge
    CHECK(m.sfm          >  0.5f);   // should remain well above 0 — not collapsed
    CHECK(m.crest_factor_db < 10.0f); // mostly uniform: max/mean near 1 → CF ≈ 0 dB
}

// ── 14. SFM + centre_power_frac gates — trough and slope rejection ───────────

TEST_CASE("fm_score: trough (bimodal edges) → score = 0 via SFM gate", "[dsp][fm][sfm]")
{
    // Two edge spikes simulate trough between adjacent stations.
    // Bimodal power → SFM near 0 → fails SFM gate.
    FmChannelMetrics m{};
    m.snr_db            = FM_SNR_GATE_DB + 5.0f;
    m.obw_fraction      = 0.95f;
    m.sfm               = 0.02f;   // bimodal → near zero
    m.centre_power_frac = 0.05f;   // null in centre → very low
    CHECK(fm_score(m) == 0.0f);
}

TEST_CASE("fm_score: slope (edge-skewed) → score = 0 via SFM gate", "[dsp][fm][sfm]")
{
    // Power concentrated at one edge (rolloff from adjacent station).
    FmChannelMetrics m{};
    m.snr_db            = FM_SNR_GATE_DB + 5.0f;
    m.obw_fraction      = 0.80f;
    m.sfm               = 0.07f;   // skewed → below FM_SFM_GATE
    m.centre_power_frac = 0.15f;   // edge-heavy → low centre fraction
    CHECK(fm_score(m) == 0.0f);
}

TEST_CASE("fm_score: real broadcast → passes SFM + CPF gates", "[dsp][fm][sfm]")
{
    // Genuine FM station: flat spectrum, power distributed through centre.
    FmChannelMetrics m{};
    m.snr_db            = FM_SNR_GATE_DB + 10.0f;
    m.obw_fraction      = 0.85f;
    m.sfm               = 0.35f;   // typical real broadcast
    m.centre_power_frac = 0.40f;   // power in centre → passes
    CHECK(fm_score(m) > 0.0f);
}

#if FM_SCORE_ALGO == FM_SCORE_ALGO_SNR_OBW_CPF || FM_SCORE_ALGO == FM_SCORE_ALGO_GATE_CPF
TEST_CASE("fm_score: trough passes SFM but fails CPF gate", "[dsp][fm][cpf]")
{
    // Realistic trough: two equal lobes → SFM is high (both halves uniformly filled)
    // but centre is the null → centre_power_frac is very low.
    FmChannelMetrics m{};
    m.snr_db            = FM_SNR_GATE_DB + 5.0f;
    m.obw_fraction      = 0.95f;
    m.sfm               = 0.85f;   // two equal lobes → SFM passes!
    m.centre_power_frac = 0.05f;   // null in centre → fails CPF gate
    CHECK(fm_score(m) == 0.0f);
}

TEST_CASE("fm_score: slope passes SFM but fails CPF gate", "[dsp][fm][cpf]")
{
    // Gentle slope: smooth gradient → SFM moderate, but power concentrated at one edge.
    FmChannelMetrics m{};
    m.snr_db            = FM_SNR_GATE_DB + 5.0f;
    m.obw_fraction      = 0.70f;
    m.sfm               = 0.30f;   // smooth gradient → SFM passes
    m.centre_power_frac = 0.18f;   // edge-heavy → fails CPF gate
    CHECK(fm_score(m) == 0.0f);
}
#endif

// ── 12. fm_score ─────────────────────────────────────────────────────────────

TEST_CASE("fm_score: noise-only channel → score = 0 (all algorithms)", "[dsp][fm][score]")
{
    // snr_db ≈ 0 → fails SNR gate before any other gate is checked.
    FmChannelMetrics m{};
    m.snr_db = 0.0f; m.obw_fraction = 0.99f; m.sfm = 1.0f; m.centre_power_frac = 0.40f;
    CHECK(fm_score(m) == Catch::Approx(0.0f).margin(0.001f));
}

TEST_CASE("fm_score: score is in [0, 1] for strong wideband signal", "[dsp][fm][score]")
{
    FmChannelMetrics m{};
    m.snr_db = 30.0f; m.obw_fraction = 0.85f; m.sfm = 0.80f; m.centre_power_frac = 0.40f;
    float s = fm_score(m);
    CHECK(s >= 0.0f);
    CHECK(s <= 1.0f);
}

#if FM_SCORE_ALGO == FM_SCORE_ALGO_GATE
TEST_CASE("fm_score GATE: below SNR threshold → 0", "[dsp][fm][score]")
{
    FmChannelMetrics m{};
    m.snr_db = FM_SNR_GATE_DB - 1.0f; m.obw_fraction = 0.9f; m.sfm = 0.5f; m.centre_power_frac = 0.40f;
    CHECK(fm_score(m) == 0.0f);
}

TEST_CASE("fm_score GATE: below OBW threshold → 0", "[dsp][fm][score]")
{
    FmChannelMetrics m{};
    m.snr_db = FM_SNR_GATE_DB + 5.0f; m.obw_fraction = FM_MOB_GATE_FRAC - 0.01f; m.sfm = 0.5f; m.centre_power_frac = 0.40f;
    CHECK(fm_score(m) == 0.0f);
}

TEST_CASE("fm_score GATE: higher SNR → higher score within passing set", "[dsp][fm][score]")
{
    FmChannelMetrics lo_snr{}, hi_snr{};
    lo_snr.snr_db = FM_SNR_GATE_DB + 5.0f;  lo_snr.obw_fraction = 0.80f; lo_snr.sfm = 0.5f; lo_snr.centre_power_frac = 0.40f;
    hi_snr.snr_db = FM_SNR_GATE_DB + 20.0f; hi_snr.obw_fraction = 0.80f; hi_snr.sfm = 0.5f; hi_snr.centre_power_frac = 0.40f;
    CHECK(fm_score(hi_snr) > fm_score(lo_snr));
}
#endif

// ── 13. get_raw_linear consistency ───────────────────────────────────────────

TEST_CASE("get_raw_linear: consistent with get_raw_db for all non-silent bins", "[dsp][linear]")
{
    // Process a step with a strong tone so most bins are well above -120 dB floor.
    auto buf = make_tone_iq(1.0e6f, 1500.0f, 30.0f, 42);
    process_step(100.0f, buf);

    const float* lin_out = get_raw_linear();
    const float* db_out  = get_raw_db();

    int checked = 0;
    for (int k = 0; k < N_FFT; k++) {
        if (db_out[k] <= -119.0f) continue;  // skip floor bins
        const float expected = std::pow(10.0f, db_out[k] / 10.0f);
        INFO("bin " << k << ": get_raw_linear=" << lin_out[k]
             << "  pow(10, get_raw_db/10)=" << expected);
        CHECK(lin_out[k] == Catch::Approx(expected).epsilon(1e-4f));
        ++checked;
    }
    // Sanity: at least some bins were above floor
    CHECK(checked > 10);
}

// ── 9. Sidelobe suppression — Blackman vs Hanning ────────────────────────────

TEST_CASE("sidelobe suppression: bins ≥4 away from a strong tone are below -50 dBFS", "[dsp][window][sidelobes]")
{
    // Strong tone at fc (bin ~31). With Blackman (-74 dB sidelobes),
    // bins far from the tone should remain near the noise floor.
    // With Hanning (-32 dB sidelobes), bins 4 away would read ~-32 dBFS above
    // the tone — so -32 + tone_level which is well above -50 dBFS.
    const float amplitude = 1500.0f;
    auto buf = make_tone_iq(0.0f, amplitude, 0.0f);
    auto out = process_step(100.0f, buf);

    int tone_bin = peak_bin(out);

    for (int d = 0; d < N_DISPLAY; d++) {
        if (abs(d - tone_bin) >= 4) {
            INFO("bin " << d << " (dist=" << abs(d - tone_bin) << ") = " << out[d] << " dBFS");
            CHECK(out[d] < -50.0f);
        }
    }
}
