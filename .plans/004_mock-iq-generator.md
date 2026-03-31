# Plan: Replace dB-domain Mock with IQ-sample Mock

## Context
`mock_step()` in `main.cpp` bypasses `process_step()` entirely — it fabricates
output in dB space, so the Blackman window, /2048 normalisation, and peak-max
decimation (all four Plan 003 fixes) are never exercised in mock mode.

Replace it with `mock_iq()`: generate complex IQ samples at ADC scale, then call
`process_step()` as the hardware path does. Mock mode becomes a real integration
test of the full DSP chain.

## File Modified
- `retina-spectrum/src/main.cpp`

---

## Change 1 — Replace `mock_step()` with `mock_iq()`

**Where:** lines 170–194 (the `// ── Mock spectrum generator` block)

**Remove** the entire `mock_step()` function and replace with `mock_iq()`:

```cpp
// ── Mock IQ generator ─────────────────────────────────────────────────────────
// Generates N_FFT*N_AVG complex samples at ADC scale (±2048 full-scale).
// Feeds through process_step() so all DSP (Blackman window, /2048, peak-max)
// is exercised identically to hardware. Signals: Gaussian noise + complex tones.

struct MockStation { float freq_mhz; float amplitude; };

static const MockStation MOCK_STATIONS[] = {
    {  89.1f, 300.0f },   // FM weak
    {  95.8f, 150.0f },   // FM medium
    {  98.8f, 800.0f },   // FM strong (primary test peak)
    { 103.5f, 100.0f },   // FM weak
    { 202.9f, 500.0f },   // DAB multiplex
    { 218.6f, 400.0f },   // DAB multiplex
    { 530.0f, 600.0f },   // DVB-T UHF ch28
    { 610.0f, 450.0f },   // DVB-T UHF ch38
};

static std::vector<std::complex<float>> mock_iq(float fc_mhz)
{
    constexpr int   N  = N_FFT * N_AVG;
    constexpr float fs = (float)SAMPLE_RATE_HZ;
    std::vector<std::complex<float>> buf(N);

    // Gaussian noise via Box-Muller (no extra deps)
    // noise_std 30 ADC counts → noise floor ~-87 dBFS after /2048 normalisation
    constexpr float noise_std = 30.0f;
    for (int n = 0; n < N; n++) {
        float u1 = (rand() + 1.0f) / ((float)RAND_MAX + 2.0f);  // avoid log(0)
        float u2 = (float)rand() / (float)RAND_MAX;
        float r  = sqrtf(-2.0f * logf(u1)) * noise_std;
        buf[n] = { r * cosf(2.0f * (float)M_PI * u2),
                   r * sinf(2.0f * (float)M_PI * u2) };
    }

    // Complex tones for each station within ±4 MHz of fc
    for (const auto& s : MOCK_STATIONS) {
        float offset_hz = (s.freq_mhz - fc_mhz) * 1.0e6f;
        if (fabsf(offset_hz) > 4.0e6f) continue;
        float phase     = 0.0f;
        float phase_inc = 2.0f * (float)M_PI * offset_hz / fs;
        for (int n = 0; n < N; n++) {
            buf[n] = { buf[n].real() + s.amplitude * cosf(phase),
                       buf[n].imag() + s.amplitude * sinf(phase) };
            phase += phase_inc;
            if (phase >  (float)M_PI) phase -= 2.0f * (float)M_PI;
            if (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
        }
    }

    return buf;
}
```

### Signal amplitudes rationale

| Amplitude (ADC counts) | After /2048 | Approx dBFS |
|---|---|---|
| 800 (strong FM)        | 0.39        | ~−8 dBFS    |
| 500 (DAB)              | 0.24        | ~−12 dBFS   |
| 300 (FM weak)          | 0.15        | ~−17 dBFS   |
| noise_std 30           | 0.015 rms   | ~−87 dBFS   |

Strong FM at −8 dBFS sits ~79 dB above the noise floor — clearly visible.
These values are deliberately generous; hardware gain will vary.

---

## Change 2 — Update sweep loop to call `mock_iq()`

**Where:** lines 226–229 (inside `if (g_mock)` block)

```cpp
// Before:
if (g_mock)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    power = mock_step(fc_mhz);
}

// After:
if (g_mock)
{
    power = process_step(fc_mhz, mock_iq(fc_mhz));
}
```

The `sleep_for(30ms)` is removed — `process_step()` over N_FFT*N_AVG = 65536
samples takes a few ms already, so pacing is natural.

---

## Verify

1. **Build:** `cmake --build build` — zero warnings
2. **Mock run:** `./retina-spectrum --mock`
   - Web GUI at `:3020` shows FM band (88–108 MHz) with sharp spikes at 89.1,
     95.8, 98.8, 103.5 MHz rising ~60–80 dB above the noise floor
   - DAB band (174–240 MHz) shows peaks at 202.9 and 218.6 MHz
   - UHF band shows peaks at 530 and 610 MHz
   - No signals outside those frequencies (confirms tone gating by ±4 MHz check)
3. **Stderr:** `process_step()` is no longer bypassed — same log cadence as hardware

---

## Notes

- `mock_iq()` uses `rand()` which is seeded once (or not at all) — noise pattern
  repeats across runs. Add `srand(time(nullptr))` in `main()` if varied noise is
  wanted, but not required for correctness testing.
- `MOCK_STATIONS` is a file-scope struct array: no heap allocation, no deps.
- Box-Muller only needs `<cmath>` which is already included.
