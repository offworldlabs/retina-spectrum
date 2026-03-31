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
