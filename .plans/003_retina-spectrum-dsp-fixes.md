# Plan: retina-spectrum DSP Fixes

## Context
retina-spectrum sweeps FM/DAB/UHF bands to locate local broadcast towers using an RSPduo.
Four DSP issues reduce measurement accuracy. Each phase is a self-contained commit that
can be built, run in --mock mode, and verified before the next phase begins.

## Files Modified
- `src/dsp.cpp` — window formula, normalisation, decimation
- `src/main.cpp` — band step size, TRIM constants, freq_start/freq_stop stitching

---

## Phase 1 — Blackman Window
**File:** `src/dsp.cpp`

**Why:** Hanning sidelobe suppression is −32 dB. A strong FM transmitter leaks energy
into adjacent bins, masking weaker nearby stations. Blackman gives −74 dB suppression
and is the RSP Spectrum Analyser's default window.

**Change:** `dsp_init()` lines 40–42. Replace Hanning formula:
```cpp
// Before:
s_window[n] = 0.5f * (1.0f - std::cos(2.0f * M_PI * n / (N_FFT - 1)));

// After:
s_window[n] = 0.42f
            - 0.5f  * std::cos(2.0f * M_PI * n / (N_FFT - 1))
            + 0.08f * std::cos(4.0f * M_PI * n / (N_FFT - 1));
```
Update comment line 40: `// Hanning window` → `// Blackman window`

**Verify:**
- Build cleanly
- `--mock` run: noise floor shape subtly broader per bin, no other change
- No functional difference visible in output — this is purely laying the groundwork

---

## Phase 2 — Step Size + Trim + Stitching Fix
**File:** `src/main.cpp`

**Why:** RSP manual p.21: Zero-IF 8MS/s usable flat region = ±1.5 MHz (3 MHz step).
Current 4 MHz steps leave 1 MHz gaps at every seam where both adjacent steps are in
filter roll-off. Additionally, `freq_start`/`freq_stop` are hardcoded and not derived
from TRIM — changing TRIM without fixing these breaks client-side frequency labelling.
All three sub-changes must land together.

### 2a — BANDS array (lines 33–37): step_mhz 4 → 3
```cpp
static const Band BANDS[] = {
    {"fm",   88,  108,  3},
    {"dab", 174,  240,  3},
    {"uhf", 468,  693,  3},   // ⚠ upper UHF bound TBC — may need extending to ~790 MHz
};
```

### 2b — TRIM constants (lines 117–119): ±2 MHz → ±1.5 MHz = 24 bins
8 MHz / 64 bins = 125 kHz/bin. ±1.5 MHz = 12 bins each side.
```cpp
static constexpr int TRIM_LO = 20;   // was 16
static constexpr int TRIM_HI = 44;   // was 48 (exclusive)
static constexpr int TRIM_N  = 24;   // was 32
```

### 2c — Derived TRIM_HALF_MHZ constant (add after TRIM block)
Removes the coupling between TRIM values and hardcoded ±2.0f/±4.0f elsewhere:
```cpp
// Half-width of trimmed display window — derived, not hardcoded
static constexpr float TRIM_HALF_MHZ = (TRIM_N / 2) * (8.0f / N_DISPLAY);  // = 1.5
```

### 2d — Fix hardcoded freq_start/freq_stop in slice_to_sse (lines 137–138)
```cpp
// Before:
<< ",\"freq_start\":"   << (sl.fc_mhz - 2.0f)
<< ",\"freq_stop\":"    << (sl.fc_mhz + 2.0f)

// After:
<< ",\"freq_start\":"   << (sl.fc_mhz - TRIM_HALF_MHZ)
<< ",\"freq_stop\":"    << (sl.fc_mhz + TRIM_HALF_MHZ)
```
Update comment line 114–116: `// centre ±2 MHz only` → `// centre ±TRIM_HALF_MHZ`

### 2e — Fix hardcoded freq_start/freq_stop in Slice construction (lines 262–263)
```cpp
// Before:
sl.freq_start_mhz = fc_mhz - 4.0f;
sl.freq_stop_mhz  = fc_mhz + 4.0f;

// After:
sl.freq_start_mhz = fc_mhz - TRIM_HALF_MHZ;
sl.freq_stop_mhz  = fc_mhz + TRIM_HALF_MHZ;
```

**Verify:**
- Build cleanly
- `--mock` run: each step JSON has 24 bins (not 32)
- Consecutive `freq_stop` / `freq_start` values differ by ≤ floating-point rounding
- Step count increases from 78 to ~107

---

## Phase 3 — Normalisation Fix
**File:** `src/dsp.cpp`

**Why:** RSP manual p.21: RSPduo at Zero-IF 8MS/s = **12-bit** ADC resolution. API
delivers right-justified samples in 16-bit shorts → full-scale peak = 2^11 = 2048.
Current /32768 under-reads by factor of 16 = **24 dB systematic error** on every reading.
Done after Phase 1 so the Blackman window coherent gain (0.42 vs Hanning 0.5) is already
in place before we calibrate the scale.

**Change:** `process_step()` lines 66–67:
```cpp
// Before:
s_fft_buf[n][0] = (samples[offset + n].real() / 32768.0f) * s_window[n];
s_fft_buf[n][1] = (samples[offset + n].imag() / 32768.0f) * s_window[n];

// After — 12-bit ADC (Zero-IF 8MS/s, RSP manual p.21), right-justified in 16-bit short:
s_fft_buf[n][0] = (samples[offset + n].real() / 2048.0f) * s_window[n];
s_fft_buf[n][1] = (samples[offset + n].imag() / 2048.0f) * s_window[n];
```

**Verify:**
- Build cleanly
- `--mock` run: noise floor shifts up ~24 dB relative to Phase 2 baseline
- On hardware: strong local FM station should read credibly (−30 to −10 dBFS range),
  not buried at −110 dBFS

---

## Phase 4 — Decimation: Average → Peak-Max
**File:** `src/dsp.cpp`

**Why:** Current code averages 128 FFT bins into each display bin. A narrowband FM
carrier occupies 1–2 bins; averaging with 127 noise bins suppresses it by ~21 dB.
RSP manual: "positive peak detection — signals always reported at correct level
irrespective of display size." Done last so we can validate against correct dBFS scale
from Phase 3.

**Change:** `process_step()` lines 82–89:
```cpp
// Before — averaging (signal buried by noise):
for (int d = 0; d < N_DISPLAY; d++)
{
    float sum = 0.0f;
    for (int g = 0; g < GROUP; g++)
        sum += linear[d * GROUP + g];
    acc[d] += sum / GROUP;
}

// After — peak-max (strongest bin wins per display bucket):
for (int d = 0; d < N_DISPLAY; d++)
{
    float peak = 0.0f;
    for (int g = 0; g < GROUP; g++)
        if (linear[d * GROUP + g] > peak)
            peak = linear[d * GROUP + g];
    acc[d] += peak;
}
```
Note: `acc[d]` still accumulates across N_AVG=8 FFTs and is divided by N_AVG in step 5.
This is correct: peak-detect within each frame, then average peaks across frames.

Update comment line 82: `// Decimate: sum linear-power bins` →
`// Decimate: peak-detect within each display bin group (positive peak detection)`

**Verify:**
- Build cleanly
- `--mock` run: FM peak at 98.8 MHz should be a sharp spike clearly above the noise floor
- Noise floor rises slightly (~3–5 dB) vs Phase 3 — expected, now showing peak noise
- On hardware: broadcast FM stations visible as distinct peaks, not merged into noise

---

## End-to-End Verification (after all phases)

1. `cmake --build build` — zero warnings
2. `./retina-spectrum --mock`
   - 24 bins per step in JSON
   - No freq gaps between consecutive `freq_stop` / `freq_start`
   - FM spike at 98.8 MHz clearly visible above noise floor
3. Hardware run against a known local FM station
   - Station reads as a clear peak at credible dBFS level
   - Adjacent stations not masked by spectral leakage from strong neighbours
