# Plan: feat/radio-sweep — Channel-Aware Spectrum Analyzer

## Context

Pivoting retina-spectrum from a raw power sweep toward a **tower-identifying** analyzer. We identify every standard US broadcast channel, find the top N local-maximum peaks within each channel, and check whether any match the known pilot frequency:

- **FM (88.1–107.9 MHz, 200 kHz channels):** top N peaks in the 200 kHz window; FM has no fixed offset pilot — just flag the dominant peak as the carrier
- **VHF/UHF TV (ch1–51, 6 MHz channels):** top N peaks in the 6 MHz window; check if any matches the ATSC pilot at **lower edge + 0.31 MHz = center − 2.69 MHz**

Each detected peak is drawn as a circle on the spectrum (green = pilot match, grey = other peak). First iteration keeps the existing spectrum chart shape for easy debugging, adding channel boundaries, labels, and peak circle annotations.

---

## US Channel Lookup Tables

All TV channels are 6 MHz wide. **ATSC pilot = lower_edge + 0.31 MHz**.

Pilot offset from center in raw FFT bins:
```
PILOT_BIN = round(-2.69e6 * N_FFT / SAMPLE_RATE_HZ)
           = round(-2.69e6 * 8192 / 8e6) = -2755
```

### VHF Low Band (ch1–6)
| Ch | Lower (MHz) | ATSC Pilot (MHz) | Center (MHz) | Upper (MHz) | Notes |
|----|-------------|------------------|--------------|-------------|-------|
| 1  | 44          | 44.31            | 47           | 50          | Rarely used |
| 2  | 54          | 54.31            | 57           | 60          | |
| 3  | 60          | 60.31            | 63           | 66          | |
| 4  | 66          | 66.31            | 69           | 72          | |
| 5  | 76          | 76.31            | 79           | 82          | Gap 72–76 MHz (aeronautical nav) |
| 6  | 82          | 82.31            | 85           | 88          | |

### VHF High Band (ch7–13)
| Ch | Lower (MHz) | ATSC Pilot (MHz) | Center (MHz) | Upper (MHz) |
|----|-------------|------------------|--------------|-------------|
| 7  | 174         | 174.31           | 177          | 180         |
| 8  | 180         | 180.31           | 183          | 186         |
| 9  | 186         | 186.31           | 189          | 192         |
| 10 | 192         | 192.31           | 195          | 198         |
| 11 | 198         | 198.31           | 201          | 204         |
| 12 | 204         | 204.31           | 207          | 210         |
| 13 | 210         | 210.31           | 213          | 216         |

### UHF (ch14–51)
Ch14: lower=470, pilot=470.31, center=473. Each +6 MHz. Ch51: center=695 MHz.
**Post-2020 repack active: ch14–36.** Ch37 = radio astronomy (protected). Ch38–51 reallocated (mobile broadband) but may still carry ATSC.

| Ch | Center | | Ch | Center | | Ch | Center |
|----|--------|--|-----|--------|--|-----|--------|
| 14 | 473    | | 22  | 521    | | 30  | 569    |
| 15 | 479    | | 23  | 527    | | 31  | 575    |
| 16 | 485    | | 24  | 533    | | 32  | 581    |
| 17 | 491    | | 25  | 539    | | 33  | 587    |
| 18 | 497    | | 26  | 545    | | 34  | 593    |
| 19 | 503    | | 27  | 551    | | 35  | 599    |
| 20 | 509    | | 28  | 557    | | 36  | 605    |
| 21 | 515    | | 29  | 563    | | 37–51 | 611–695 |

### FM
88.1, 88.3 … 107.9 MHz — 100 channels, 200 kHz spacing. No fixed pilot offset.

---

## Sweep Strategy — Keep Existing DSP

The FFT pipeline (8192-point, Blackman window, N_AVG=8, FFTW_MEASURE) is **unchanged**. Peak detection runs on **raw FFT bins before decimation/trim**, so the ±1.5 MHz display trim is irrelevant for analysis.

**Resolution check — why 977 Hz/bin is sufficient:**

| Target | Required precision | Bins needed | Verdict |
|--------|-------------------|-------------|---------|
| FM peak (200 kHz channel) | find max in 200 kHz window | 205 bins available | ✓ very comfortable |
| ATSC pilot (CW tone) | ±50 kHz tolerance | ±51 bins tolerance, ~3 bin Blackman lobe | ✓ fine |
| ATSC pilot bin offset | −2.69 MHz from centre | bin −2755, within ±4096 | ✓ in range |
| Focus display (TV ±3.0 MHz) | show pilot at −2755 | within ±3072 display bins | ✓ visible (317 bins from edge) |

A larger FFT (e.g. 32768-point) would improve frequency resolution but give zero practical benefit — our coarsest tolerance (±50 kHz) is already 51 bins wide at 977 Hz/bin. It would also significantly slow each step. **No DSP changes needed.**

### FM — batch processing (no extra tunes)
Keep the 7 × 3 MHz steps. After each FFT, scan all FM channel centres within ±1.5 MHz of the step centre. Run peak-finding on the raw bins within that 200 kHz channel window.

```
ch_bin_centre = round(Δf_Hz * N_FFT / SAMPLE_RATE_HZ)   // Δf = ch_fc - step_fc
ch_half_bins  = round(100e3 * N_FFT / SAMPLE_RATE_HZ)    // ≈ 102 bins = ±100 kHz
```

### VHF/UHF TV — per-channel steps
Replace 3 MHz VHF/UHF steps with one step per channel centre. 8 MHz capture fully contains the 6 MHz channel. Peak-find across ±3 MHz (full channel), then check for pilot at bin −2755.

New step count: 6 VHF-lo + 7 VHF-hi + 38 UHF + 7 FM = **58 steps** (down from 91, covers far more channels).

---

## Peak Detection Algorithm

### Channel identification
Channels are identified and labelled throughout by their **centre broadcast frequency** (`fc_mhz`):
- FM: "88.1", "88.3" … "107.9" MHz
- TV: "473.0", "479.0" … MHz (with optional channel number as secondary label, e.g. "Ch14 / 473.0")

The `Channel.fc_mhz` field is the primary key in the lookup table, in SSE data, and in UI annotations.

### New constants (`src/config.h`)
```cpp
static constexpr int   NUM_CHANNEL_PEAKS = 3;    // top peaks to find per channel
static constexpr float PILOT_TOL_MHZ     = 0.05f; // ±50 kHz pilot match tolerance
```

### New struct + function (`src/dsp.h` / `dsp.cpp`)

```cpp
struct ChannelPeak {
    float freq_mhz;   // absolute frequency of this peak
    float power_db;
    bool  is_pilot;   // within PILOT_TOL_MHZ of known pilot frequency
};

// Find the top num_peaks local-maximum peaks within raw FFT bins [lo_bin, hi_bin].
// Bins are offsets from FFT centre (may be negative).
// pilot_mhz: absolute frequency of expected pilot (0.0 = FM, skip check).
// step_fc_mhz: centre frequency of the current SDR tune step.
// Returns up to num_peaks peaks sorted by power descending.
std::vector<ChannelPeak> find_channel_peaks(
    const float* fft_db,  // raw FFT magnitude in dBFS, length N_FFT
    int n_fft,
    int lo_bin, int hi_bin,
    float step_fc_mhz,
    float pilot_mhz,
    int num_peaks);
```

**Algorithm:**
1. Scan bins `[lo_bin, hi_bin]` (wrapped to [0, N_FFT) using `(bin + n_fft) % n_fft`).
2. Collect **local maxima**: `fft_db[i] > fft_db[i-1] && fft_db[i] > fft_db[i+1]`.
3. Sort local maxima by power descending, take top `num_peaks`.
4. For each: `freq_mhz = step_fc_mhz + bin_offset * (SAMPLE_RATE_HZ / N_FFT) / 1e6`.
5. `is_pilot = (pilot_mhz > 0) && (fabsf(freq_mhz - pilot_mhz) <= PILOT_TOL_MHZ)`.

Called after `fftwf_execute()` + magnitude/log conversion, **before** `decimate_to_display()`.

---

## SSE Data Format Change

`slice_to_sse()` (main.cpp lines 139–155) adds a `"channels"` array:

```json
{
  "type": "step",
  "step": 12,
  "fc_mhz": 473.0,
  "freq_start": 469.5,
  "freq_stop": 476.5,
  "progress_pct": 42,
  "power_db": [...],
  "smooth_db": [...],
  "channels": [
    {
      "band": "uhf",
      "number": 14,
      "fc_mhz": 473.0,
      "pilot_mhz": 470.31,
      "peaks": [
        {"freq_mhz": 470.31, "power_db": -48.7, "is_pilot": true},
        {"freq_mhz": 471.90, "power_db": -55.2, "is_pilot": false},
        {"freq_mhz": 474.12, "power_db": -60.1, "is_pilot": false}
      ]
    }
  ]
}
```

---

## Files to Modify

| File | Lines | Change |
|------|-------|--------|
| `src/channels.h` | **new** | `Channel` struct + `FM_CHANNELS[100]`, `VHF_LO_CHANNELS[6]`, `VHF_HI_CHANNELS[7]`, `UHF_CHANNELS[38]` |
| `src/config.h` | 4–8 | Add `NUM_CHANNEL_PEAKS`, `PILOT_TOL_MHZ` |
| `src/dsp.h` | ~23 | Add `ChannelPeak` struct + `find_channel_peaks()` declaration |
| `src/dsp.cpp` | 94–111 | Add `find_channel_peaks()` implementation |
| `src/main.cpp` | 34–38 | Replace `BANDS[]` with channel-table-based step list |
| `src/main.cpp` | 120–123 | TV steps: no trim (use full ±4 MHz window for analysis) |
| `src/main.cpp` | 139–155 | `slice_to_sse()` — add `channels[].peaks[]` |
| `src/main.cpp` | 252–256 | Step list builder: FM batch + TV per-channel |
| `src/main.cpp` | 354–388 | After DSP: call `find_channel_peaks()`, populate Slice |
| `src/main.cpp` | 366–367 | Focus `freq_start`/`freq_stop`: FM ±0.2 MHz, TV ±3.5 MHz |
| `src/main.cpp` | 547–551 | `/api/focus`: snap fc to nearest channel centre |
| `web/index.html` | 81–128 | Add Chart.js annotation plugin (CDN); hardcode JS channel table |
| `web/index.html` | 100–104 | Click → snap to nearest channel centre |
| `web/index.html` | 172–176 | `setAxisFocus()`: use `freq_start`/`freq_stop` from SSE |
| `web/index.html` | 197–226 | `applyStep()`: parse `channels[].peaks[]`, update peak annotation map |
| `web/index.html` | new | Draw peak circles: Chart.js annotation `point` type at `{x: freq_mhz, y: power_db}` — green border if `is_pilot`, grey otherwise; updated each sweep step |
| `web/index.html` | new | Channel boundary vertical lines + band labels below x-axis |
| `web/index.html` | new | **Phase 1:** peak power text label (e.g. "−62 dB") centred inside each channel segment, updated each sweep. **Phase 2 (deferred):** swap for "pilot found" text when `is_pilot` true |

---

## Focus Mode

Show exactly the channel boundaries — no more, no less:

- **FM channel:** `freq_start = fc − 0.1`, `freq_stop = fc + 0.1` (exactly 200 kHz = full FM channel)
- **TV channel:** `freq_start = lower_edge`, `freq_stop = upper_edge` = `fc − 3.0` to `fc + 3.0` (exactly 6 MHz = full TV channel; ATSC pilot at −2.69 MHz from centre is visible within this window)

Backend sets `freq_start`/`freq_stop` in the SSE `"start"` event using the channel's known edges from the lookup table. Frontend uses these directly — no hardcoded axis values.

---

## Work Plan

### 1. Branch setup
- [ ] `git checkout -b feat/radio-sweep` inside `retina-spectrum/`
- [ ] Delete stale plan copies outside this repo

### 2. Channel lookup table (`src/channels.h`)
- [ ] Define `Channel` struct with `band`, `number`, `fc_mhz`, `bw_mhz`, `pilot_mhz`
- [ ] Populate `FM_CHANNELS[100]` — 88.1 … 107.9 MHz, 200 kHz spacing, `pilot_mhz = 0`
- [ ] Populate `VHF_LO_CHANNELS[6]` — ch1–6, centres 47/57/63/69/79/85, `pilot_mhz = lower + 0.31`
- [ ] Populate `VHF_HI_CHANNELS[7]` — ch7–13, centres 177–213, `pilot_mhz = lower + 0.31`
- [ ] Populate `UHF_CHANNELS[38]` — ch14–51, centres 473–695, `pilot_mhz = lower + 0.31`

### 3. DSP — peak detection (`src/dsp.h` / `src/dsp.cpp`)
- [ ] Define `ChannelPeak` struct (`freq_mhz`, `power_db`, `is_pilot`)
- [ ] Implement `find_channel_peaks()` — local-maximum search, top-N sort, pilot flag
- [ ] Add unit test: synthetic spike at known bin → correct `freq_mhz`, `is_pilot = true`
- [ ] Run `./build-mac/test_dsp` — all existing + new tests pass

### 4. Config constants (`src/config.h`)
- [ ] Add `NUM_CHANNEL_PEAKS = 3`
- [ ] Add `PILOT_TOL_MHZ = 0.05f`

### 5. Sweep redesign (`src/main.cpp`)
- [ ] Replace `BANDS[]` with step list built from channel arrays (FM batch + TV per-channel)
- [ ] FM steps: identify channels within ±1.5 MHz window, call `find_channel_peaks()` on raw FFT
- [ ] TV steps: call `find_channel_peaks()` across full ±3 MHz window, pilot bin = −2755
- [ ] Store `vector<ChannelPeak>` per channel in `Slice`
- [ ] Update `slice_to_sse()` — add `channels[]` with `peaks[]` array
- [ ] Focus mode: set `freq_start`/`freq_stop` from channel table edges (FM ±0.1 MHz, TV ±3.0 MHz)
- [ ] `/api/focus`: snap incoming `fc` to nearest channel `fc_mhz`

### 6. Mock build check
- [ ] `cmake -B build-mac -DMOCK_ONLY=ON && cmake --build build-mac` — clean compile

### 7. Frontend (`web/index.html`)
- [ ] Hardcode JS channel table (mirrors `channels.h`), keyed by `fc_mhz`
- [ ] Add Chart.js annotation plugin via CDN
- [ ] Draw vertical channel boundary lines at each channel edge
- [ ] Label each channel by centre frequency (FM: "88.1"; TV: "Ch14 / 473.0")
- [ ] `applyStep()`: parse `channels[].peaks[]`, maintain per-channel peak map
- [ ] Draw peak circles (annotation `point`) at `{x: freq_mhz, y: power_db}` — green = pilot, grey = other
- [ ] Peak power text label inside each channel segment ("−62 dB")
- [ ] Click handler: snap to nearest channel `fc_mhz` before calling `/api/focus`
- [ ] Focus x-axis: derive from `freq_start`/`freq_stop` in SSE `"start"` event (no hardcoding)

### 8. End-to-end test (mock)
- [ ] Channel boundaries and labels render correctly across FM / VHF / UHF
- [ ] Peak circles appear; grey in mock noise (no green — no pilot in synthetic data)
- [ ] Peak power labels update each sweep
- [ ] Click a TV channel → focus opens at exactly 6 MHz span; ATSC pilot offset visible
- [ ] Click an FM channel → focus opens at exactly 200 kHz span

### 9. Hardware test (RPi)
- [ ] Active FM stations show peak circles at carrier frequency
- [ ] ATSC pilot detected as green circle at `lower_edge + 0.31 MHz` for live TV transmitters

---

## Out of Scope (this branch)

- Bar-graph-per-channel final UI (future branch)
- Geolocation / tower matching
- UHF ch38–51 (in table, low test priority)

---

## Verification

1. **Mock build:** `cmake -B build-mac -DMOCK_ONLY=ON && cmake --build build-mac` — clean compile.
2. **DSP unit tests:** `./build-mac/test_dsp` — existing tests pass; add one test for `find_channel_peaks()` with a synthetic spike at a known bin, verifying `is_pilot` flag and frequency accuracy.
3. **Browser (mock):** Channel boundaries and labels visible; grey circles at top-N noise peaks; no green circles (no pilot in mock noise); TV focus shows 7 MHz span with pilot offset marked; FM focus shows 400 kHz span.
4. **Hardware (RPi):** Active FM stations show peak circles at carrier; ATSC pilots show green circles at `lower_edge + 0.31 MHz` for transmitting TV stations.
