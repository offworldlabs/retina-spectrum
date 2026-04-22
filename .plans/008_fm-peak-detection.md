# Plan: FM Channel Metrics — retina-spectrum feat/radio-sweep

## Context

FM has no narrowband pilot tone. The old approach passed `ch->fc_mhz` as a fake
"pilot" to `find_channel_peaks()` with a fixed threshold — this gives a binary
`is_pilot` result but no quality information.

**Goal (this plan)**: Replace FM detection with two continuous metrics computed
per channel slot per sweep pass:

- **`snr_db`** — how strong the channel is above the per-step noise floor
- **`flatness`** — how evenly power is spread across the channel (0 = CW spike, 1 = flat wideband)

No binary threshold. Backend emits numbers; frontend displays them in a channel list.
Empty slots (below `FM_MIN_REPORT_SNR`) are suppressed to avoid 100 empty rows per step.

**TV is unchanged**: `find_channel_peaks()` + ATSC pilot detection untouched.

---

## The two FM metrics

### 1. Channel SNR (dB)

```
snr_db = 10 * log10( mean_linear_power(channel_window) / noise_floor_per_bin )
```

- `channel_window`: all N_FFT bins within [fc − 100 kHz, fc + 100 kHz] (~205 bins at 977 Hz/bin)
- `noise_floor_per_bin`: 25th percentile of **all N_FFT bins in the current sweep step**,
  computed in linear domain. In a dense FM market <20% of bins are occupied so the
  25th percentile always lands on noise-only bins.
- Both computed in linear domain; result in dB.

Empty channel → `snr_db ≈ 0 dB`. Strong station → `snr_db = 15–30 dB`.

### 2. Spectral flatness (0–1)

Measures the **shape** of the spectrum within the channel, independent of level.

```
flatness = geometric_mean(linear_power_bins) / arithmetic_mean(linear_power_bins)
```

Over the same ~205-bin channel window.

| Signal type     | snr_db    | flatness    | Quality for passive radar |
|-----------------|-----------|-------------|---------------------------|
| Empty slot      | ≈ 0 dB    | meaningless | —                         |
| CW / dead air   | high      | → 0         | Strong but narrow — bad   |
| Music / speech  | high      | 0.7–0.9     | Strong and wide — good    |

Why flatness instead of counting bins above noise: Blackman window sidelobes spread
a CW spike into several adjacent bins, making bin-counting give a spuriously wide result.
Flatness measures shape correctly regardless of signal level or leakage.

---

## Backend changes

### `src/config.h`

```cpp
#define FM_MIN_REPORT_SNR   3.0f   // suppress channel slots below this SNR in SSE output
```

Not a detection threshold — just avoids emitting ~100 empty-slot entries per step.

### `src/dsp.h`

New struct and functions:

```cpp
struct FmChannelMetrics {
    float snr_db;    // channel mean power relative to step noise floor
    float flatness;  // geometric_mean / arithmetic_mean of channel bins (0=spike, 1=flat)
};

// 25th percentile of raw_db[0..n_fft-1] converted to linear, nth_element O(n), back to dB.
float estimate_step_noise_floor(const float* raw_db, int n_fft);

// Compute SNR + spectral flatness for one FM channel slot.
FmChannelMetrics compute_fm_metrics(
    const float* raw_db,
    int   n_fft,
    float step_fc_mhz,
    float ch_lo_mhz,   // fc - 0.1 MHz
    float ch_hi_mhz,   // fc + 0.1 MHz
    float noise_db);   // from estimate_step_noise_floor()
```

`find_channel_peaks()` signature and behaviour are unchanged.

### `src/dsp.cpp`

`estimate_step_noise_floor()`:
1. Copy `raw_db[0..n_fft-1]` to a temporary float vector, convert dB → linear
2. `std::nth_element` to extract 25th percentile in O(n)
3. Return `10 * log10(percentile_value)`

`compute_fm_metrics()`:
1. Bin-map `ch_lo_mhz..ch_hi_mhz` to index range using `step_fc_mhz`
2. Convert those bins dB → linear
3. `snr_db = 10*log10(arithmetic_mean(bins) / linear(noise_db))`
4. `flatness = geometric_mean(bins) / arithmetic_mean(bins)`
   — geometric mean via `exp(mean(log(bins)))` with a small floor to avoid log(0)

### `src/main.cpp`

**`ChannelResult` struct** — add `fm` field:

```cpp
struct ChannelResult {
    const Channel*           ch;
    std::vector<ChannelPeak> peaks;  // TV only; empty for FM
    FmChannelMetrics         fm;     // FM only; {0,0} for TV
};
```

**Once per step, before the channel loop**:
```cpp
const float step_noise_db = estimate_step_noise_floor(cur_raw_ema.data(), N_FFT);
```

**In the channel loop — branch FM vs TV**:
```cpp
if (ch->pilot_mhz == 0.0f) {
    // FM: compute metrics
    float lo = ch->fc_mhz - ch->bw_mhz * 0.5f;
    float hi = ch->fc_mhz + ch->bw_mhz * 0.5f;
    FmChannelMetrics fm = compute_fm_metrics(
        cur_raw_ema.data(), N_FFT, fc_mhz, lo, hi, step_noise_db);
    if (fm.snr_db >= FM_MIN_REPORT_SNR)
        ch_results.push_back({ch, {}, fm});
} else {
    // TV: pilot peak detection unchanged
    auto peaks = find_channel_peaks(cur_raw_ema.data(), N_FFT, fc_mhz,
                                    lo_mhz, hi_mhz,
                                    ch->pilot_mhz, ch->tol_mhz, NUM_CHANNEL_PEAKS);
    ch_results.push_back({ch, std::move(peaks), {}});
}
```

**SSE serialisation** — FM channels emit `snr_db` + `flatness`; TV emits `peaks[]`:

```json
{"band":"fm",     "fc_mhz":98.7,  "snr_db":22.4, "flatness":0.82}
{"band":"vhf_hi", "fc_mhz":189.0, "peaks":[{"freq_mhz":186.31,"power_db":-52.1,"is_pilot":true}]}
```

---

## FM focus mode resolution fix

**Problem**: Focus mode calls `process_step_focus()` which decimates to
`N_DISPLAY_FOCUS=1024` bins across 8 MHz → 7.8 kHz/bin → only **25 bins** per
200 kHz FM channel. TV channels (6 MHz) get ~769 bins. FM is nearly unresolvable.

**Fix**: `cur_raw_ema` (the full N_FFT=8192-bin EMA, 977 Hz/bin) is already computed
every focus step. For FM channels in focus mode, skip `process_step_focus()` and send
`cur_raw_ema` directly → **205 bins** across the 200 kHz channel. No DSP change.

```cpp
const Channel* focused_ch = find_channel(fc_mhz);
bool fm_focus = is_focus && focused_ch && (focused_ch->pilot_mhz == 0.0f);

if (fm_focus) {
    // FM focus: full-resolution EMA, 977 Hz/bin — 205 bins across 200 kHz
    power.assign(cur_raw_ema.begin(), cur_raw_ema.end());
    // freq_start/stop: fc ± 4 MHz (full window); frontend zooms to ±0.1 MHz via setAxisFocusChannel
} else if (is_focus) {
    // TV focus: existing decimated path unchanged
    auto arr = process_step_focus(fc_mhz, ...);
    power.assign(arr.begin(), arr.end());
} else {
    // Sweep
    auto arr = process_step(fc_mhz, ...);
    power.assign(arr.begin(), arr.end());
}
```

`full_bins=true` is already passed to `slice_to_sse()` in focus mode, so the full array
is serialised. The frontend `applyStep()` handles any `power_db` array length —
no frontend change needed for this fix.

---

## Frontend: channel list panel

A compact scrollable panel below the chart, updated on every SSE `step` event.
No ordering — channels are displayed in the order they are received from the backend
(sweep order, lowest frequency first within each band).

### Layout

```
┌─────────────────────────────────────────────────────────────────┐
│  header (tabs, status, legend)                                  │
├─────────────────────────────────────────────────────────────────┤
│  chart  (flex: 1, fills remaining height)                       │
├─────────────────────────────────────────────────────────────────┤
│  #channel-list  (fixed ~130px, overflow-y: auto)                │
│  FM tab:      98.7     SNR  22.4 dB    flatness  0.82           │
│  FM tab:      95.9     SNR  18.1 dB    flatness  0.76           │
│  VHF/UHF tab: ch 9     pilot ✓         -52.1 dBFS               │
│  VHF/UHF tab: ch 14    pilot ✗         —                        │
└─────────────────────────────────────────────────────────────────┘
```

### HTML / CSS

```html
<div id="channel-list"></div>
```

```css
#channel-list {
  height: 130px;
  overflow-y: auto;
  border-top: 1px solid #1f2937;
  padding: 4px 20px;
  font-size: 0.75rem;
  color: #94a3b8;
}
.ch-row {
  display: flex;
  gap: 24px;
  padding: 2px 0;
  border-bottom: 1px solid #1f293740;
}
.ch-label { width: 80px; color: #e2e8f0; }
.ch-metric { width: 120px; }
.ch-metric.good { color: #81c784; }
.ch-metric.warn { color: #ffb74d; }
```

### Data state

```js
const channelState = {};   // fc_mhz (string key) → latest channel object from SSE
```

On each SSE `step` event that contains `channels[]`, merge into `channelState`:
```js
for (const ch of step.channels) {
    channelState[ch.fc_mhz.toFixed(1)] = ch;
}
```

Then call `renderChannelList()`.

### `renderChannelList()`

The panel is present under both tabs but only FM channels are populated in this plan.
VHF/UHF tab shows the panel empty (or with a placeholder) — TV metrics are a separate plan.

```js
function renderChannelList() {
    const el = document.getElementById('channel-list');

    if (activeTab !== 'fm') {
        el.innerHTML = '<span style="color:#475569">channel metrics — FM band only</span>';
        return;
    }

    const rows = [];
    for (const ch of Object.values(channelState)) {
        if (ch.band !== 'fm') continue;
        const snrClass  = ch.snr_db  > 15 ? 'good' : ch.snr_db > 8 ? 'warn' : '';
        const flatClass = ch.flatness > 0.6 ? 'good' : ch.flatness > 0.3 ? 'warn' : '';
        rows.push(`<div class="ch-row">
            <span class="ch-label">${ch.fc_mhz.toFixed(1)} MHz</span>
            <span class="ch-metric ${snrClass}">SNR ${ch.snr_db.toFixed(1)} dB</span>
            <span class="ch-metric ${flatClass}">flatness ${ch.flatness.toFixed(2)}</span>
        </div>`);
    }

    el.innerHTML = rows.length ? rows.join('') : '<span style="color:#475569">no FM channels above threshold</span>';
}
```

Call `renderChannelList()` also from `switchTab()` so the list re-filters immediately
on tab click without waiting for the next SSE event.

No `number` field needed in SSE for FM — `fc_mhz` is the natural label for FM stations.
TV `number` field can be added when TV metrics are implemented.

---

## Files changed

| File | Change |
|------|--------|
| `src/config.h` | Add `FM_MIN_REPORT_SNR` |
| `src/dsp.h` | Add `FmChannelMetrics`; declare `estimate_step_noise_floor()`, `compute_fm_metrics()` |
| `src/dsp.cpp` | Implement both functions |
| `src/main.cpp` | Add `fm` to `ChannelResult`; FM/TV branch; noise floor per step; FM focus resolution; update `slice_to_sse()` FM serialisation |
| `web/index.html` | Add `#channel-list` panel + CSS; `channelState` accumulator; `renderChannelList()` (FM only); call from `switchTab()` |

---

## Tests (`tests/test_dsp.cpp`)

New test cases added alongside the existing 9. All operate on synthetic `raw_db` arrays
(no FFT pipeline needed — functions only depend on the dBFS array + frequency mapping).

### `estimate_step_noise_floor`

**T10 — 25th percentile lands on noise, not signal bins**
- Array of N_FFT bins all at −87 dBFS (noise), top 10% of positions set to −30 dBFS
- 25th percentile index (N_FFT/4) is within the 90% noise region
- Assert: result within ±1 dB of −87 dBFS

**T11 — all bins equal returns that level**
- N_FFT bins all = −80 dBFS
- Assert: result within ±0.5 dB of −80 dBFS

### `compute_fm_metrics`

**T12 — uniform channel bins → flatness ≈ 1**
- All bins at noise level (−87 dBFS), channel window bins set uniformly to −60 dBFS
- geometric_mean == arithmetic_mean when all bins equal
- Assert: `flatness` within 0.01 of 1.0, `snr_db > 0`

**T13 — single CW spike → low flatness, high SNR**
- All bins at noise level, one bin at channel centre = −30 dBFS
- One dominant bin → geometric mean << arithmetic mean
- Assert: `flatness < 0.3`, `snr_db > 10 dB`

**T14 — all bins at noise floor → snr_db ≈ 0**
- All bins at noise level (−87 dBFS), `noise_db` also = −87 dBFS
- arithmetic mean of channel ≈ noise floor → ratio ≈ 1 → SNR ≈ 0 dB
- Assert: `snr_db` within ±2 dB of 0

---

## Phased work plan

### Phase 1 — DSP layer
- [ ] Add `FM_MIN_REPORT_SNR` to `src/config.h`
- [ ] Add `FmChannelMetrics` struct to `src/dsp.h`
- [ ] Declare `estimate_step_noise_floor()` in `src/dsp.h`
- [ ] Declare `compute_fm_metrics()` in `src/dsp.h`
- [ ] Implement `estimate_step_noise_floor()` in `src/dsp.cpp`
- [ ] Implement `compute_fm_metrics()` in `src/dsp.cpp`
- [ ] Add T10–T14 to `tests/test_dsp.cpp`
- [ ] Build passes cleanly, all tests pass

### Phase 2 — main.cpp wiring
- [ ] Add `fm` field to `ChannelResult` struct
- [ ] Compute `step_noise_db` once per step before channel loop
- [ ] Branch channel loop: FM → `compute_fm_metrics`, TV → `find_channel_peaks` (unchanged)
- [ ] Update `slice_to_sse()` to emit `snr_db` + `flatness` for FM; `peaks[]` for TV
- [ ] Build + verify: `curl` mock SSE shows FM channels with `snr_db` and `flatness` fields

### Phase 3 — FM focus resolution
- [ ] Branch focus capture path in main.cpp: FM channel → assign `cur_raw_ema` directly, TV → existing `process_step_focus()`
- [ ] Build + verify: clicking an FM channel in focus mode shows visibly sharper 205-bin trace

### Phase 4 — Frontend channel list
- [ ] Add `#channel-list` div to `web/index.html` below `#chart-wrap`
- [ ] Add CSS for `#channel-list`, `.ch-row`, `.ch-label`, `.ch-metric`
- [ ] Add `channelState` accumulator, merge on each SSE `step` event
- [ ] Implement `renderChannelList()` — FM rows only; placeholder on VHF/UHF tab
- [ ] Call `renderChannelList()` from `switchTab()`
- [ ] Verify: FM tab shows channel list updating in real time; VHF/UHF tab shows placeholder

---

## Verification

1. Mock mode: 89.1, 95.9, 98.7, 103.5 MHz appear in SSE with `snr_db > 0`;
   other FM slots absent (below `FM_MIN_REPORT_SNR`)
2. 98.7 MHz (mock amplitude 800) has highest `snr_db` of the four
3. Mock stations are CW sinusoids → `flatness` near 0 (expected — spike not wideband)
4. FM tab channel list shows those four stations, each with SNR and flatness values
5. VHF/UHF tab channel list shows TV channels with pilot status and peak power
6. Tab switch immediately re-renders list (no wait for next SSE event)
7. Real hardware: active FM stations show `flatness` 0.7–0.9, `snr_db` 15–30 dB
8. FM focus mode: clicking an FM channel shows 205-bin trace; TV focus unchanged
9. TV `peaks[]` + pilot detection unchanged throughout
