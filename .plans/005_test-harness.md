# Plan: DSP Test Harness (Catch2)

## Context
All four DSP fixes are implemented and verified visually in --mock mode.
This harness locks in correctness with deterministic, repeatable assertions.
Tests only exercise dsp.cpp — no SDR hardware or mock IQ generator needed.

## Framework
Catch2 v3 via CMake FetchContent. Only built under MOCK_ONLY.
Runs via `ctest` or directly as `./build/test_dsp`.

## Files
- `tests/test_dsp.cpp` — all test cases
- `CMakeLists.txt` — Catch2 FetchContent + test_dsp target inside MOCK_ONLY block

---

## Test Cases

### 1. freq_axis — bin centre positions
Deterministic arithmetic. Verify centres span fc ± ~4 MHz with 0.125 MHz spacing.

### 2. Stitching — no frequency gaps within each band
For each band with 3 MHz steps + ±1.5 MHz trim:
`freq_stop[step i] == freq_start[step i+1]` for all consecutive steps.

### 3. Noise floor — pure Gaussian noise stays below threshold
`make_noise_iq(std=30, seed=42)` → `process_step()` → all 64 bins < −50 dBFS.
Validates the /2048 normalisation is correct (wrong divisor would shift this ~24 dB).

### 4. Tone bin placement — parametric sweep across offsets
GENERATE over offsets: −3.0, −2.0, −1.0, −0.5, 0.5, 1.0, 2.0, 3.0 MHz
For each: pure tone (no noise, A=1500), `process_step()`, find expected bin
from `freq_axis()`, assert `|peak_bin − expected_bin| ≤ 1`.
This is the "iterate through frequencies and verify" test.

### 5. Tone amplitude — level within 6 dB of theory
Tone at +0.125 MHz (exact display bin boundary, minimises scalloping loss).
Expected dBFS = 20*log10(A * 0.42 / 2048) for Blackman coherent gain 0.42.
Assert measured level within ±6 dB of theory.

### 6. SNR — signal clearly above noise
Tone A=1500 + noise std=30. Assert:
- peak > −25 dBFS
- noise floor (median of bottom-quartile bins) < −55 dBFS
- SNR > 30 dB

### 7. Peak-max decimation — not averaging
Two tones in the same display bin group, separated by < GROUP FFT bins.
Verify the output bin reflects the stronger tone's level, not the average.
Validates peak-max decimation is in place (if averaging, result would be ~21 dB lower).

### 8. Sidelobe suppression — Blackman not Hanning
Strong tone + adjacent bin measurement. With Blackman (−74 dB sidelobes),
bins ≥ 3 display bins away from the tone should be < −50 dBFS.
If Hanning were used (−32 dB sidelobes), nearby bins would read higher.

---

## Build & Run
```
PKG_CONFIG_PATH="/opt/homebrew/opt/fftw/lib/pkgconfig" cmake -B build -DMOCK_ONLY=ON
cmake --build build
ctest --test-dir build --output-on-failure
# or directly:
./build/test_dsp
```
