# Plan: FM Channel Detection — retina-spectrum feat/radio-sweep

## Context

FM has no narrowband pilot tone. The current code detects FM stations by passing `ch->fc_mhz`
as the "pilot" frequency to `find_channel_peaks()` — a hack that works conceptually, but uses
a fixed threshold `PEAK_MIN_DBF = -75 dBFS`. That threshold doesn't adapt to the receiver's
actual noise floor.

**Goal (this plan)**: FM detection equivalent of `is_pilot` — binary "station present / absent"
using a noise-adaptive threshold. Signal quality metrics (SNR, occupancy, guard bands) are a
separate plan.

**TV is unchanged**: `find_channel_peaks()` keeps using `PEAK_MIN_DBF` for ATSC pilot detection.
The pilot is a strong CW tone; a fixed absolute threshold works fine there.

---

## Approach

For each step, the EMA-smoothed N_FFT-bin array (`cur_raw_ema`) is already available.
Noise floor estimate = 25th percentile of all N_FFT bins. In a dense FM market at most
~15–20% of bins are occupied, so the 25th percentile reliably lands on noise-only bins.

FM channel detected = local max within ±`tol_mhz` (±100 kHz) of channel centre AND
above `noise_floor + FM_DETECT_DB`. Identical logic to the TV pilot check, just adaptive.

---

## Changes

### `src/config.h`

```cpp
#define FM_DETECT_DB    10.0f   // FM channel detected if peak > noise_floor + FM_DETECT_DB
```

### `src/dsp.h` / `src/dsp.cpp`

New function:
```cpp
// 25th percentile of raw_db[0..n_fft-1]. O(n) via nth_element.
float estimate_step_noise_floor(const float* raw_db, int n_fft);
```

Add `threshold_db` parameter to `find_channel_peaks()` (default = `PEAK_MIN_DBF`, TV path unchanged):
```cpp
std::vector<ChannelPeak> find_channel_peaks(
    const float* raw_db, int n_fft,
    float step_fc_mhz, float ch_lo_mhz, float ch_hi_mhz,
    float pilot_mhz, float tol_mhz, int num_peaks,
    float threshold_db = PEAK_MIN_DBF);   // ← new
```

Inside `find_channel_peaks()`: replace the hardcoded `PEAK_MIN_DBF` in the `is_pilot`
check with `threshold_db`.

### `src/main.cpp`

Before the per-channel loop each step:
```cpp
const float step_noise_db = estimate_step_noise_floor(cur_raw_ema.data(), N_FFT);
```

In the channel loop, select threshold by band:
```cpp
float threshold = (ch->pilot_mhz == 0.0f)      // FM: no pilot → adaptive
                ? step_noise_db + FM_DETECT_DB
                : PEAK_MIN_DBF;                  // TV: fixed as before
float pilot = (ch->pilot_mhz > 0.0f) ? ch->pilot_mhz : ch->fc_mhz;
auto peaks = find_channel_peaks(cur_raw_ema.data(), N_FFT, fc_mhz,
                                lo_mhz, hi_mhz,
                                pilot, ch->tol_mhz, NUM_CHANNEL_PEAKS,
                                threshold);
```

No changes to structs, SSE format, or frontend. `is_pilot` continues to mean
"detected at expected position" for both FM and TV.

---

## Files changed

| File | Change |
|------|--------|
| `src/config.h` | Add `FM_DETECT_DB` |
| `src/dsp.h` | Declare `estimate_step_noise_floor()`; add `threshold_db` to `find_channel_peaks()` |
| `src/dsp.cpp` | Implement `estimate_step_noise_floor()`; thread `threshold_db` through |
| `src/main.cpp` | Compute noise floor per step; pass adaptive threshold for FM |

---

## Verification

1. Mock mode: `./retina-spectrum --mock --web-dir web` — FM channels at 89.1, 95.8, 98.8,
   103.5 MHz show `is_pilot:true` in SSE `channels[]`
2. Set `FM_DETECT_DB = 50.0f` → all FM stations disappear; restore to 10.0f → they return
3. Real hardware: detected channels match known local FM stations
