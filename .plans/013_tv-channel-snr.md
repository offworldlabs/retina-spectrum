# Plan 013 — VHF/UHF Channel SNR + Spectrum Power Line

## Goal

Add illuminator quality scoring to VHF/UHF (ATSC) channels, consistent with FM:

1. **SNR score** — channel mean power vs noise floor, gated on pilot detection,
   normalised identically to FM so all bands rank on one scale for the towers API
2. **Horizontal power line** — drawn across the full channel width on the spectrum chart
   at the measured mean dBFS level, labelled with SNR in dB

---

## Background

ATSC 8-VSB occupies 5.38 MHz of a 6 MHz allocation. At 8 MHz sample rate there are
~1.31 MHz guard bands on each side (~33% noise bins) — the existing
`estimate_step_noise_floor()` 25th-percentile estimator anchors cleanly in these margins,
no new noise estimator needed.

ATSC is always full-bandwidth when on-air (no OBW variable like FM). The pilot tone
(lower_edge + 0.31 MHz) confirms the transmitter is active. **Score = 0 if no pilot.**

SNR is hardware-agnostic (normalises antenna gain, cable loss, receiver differences)
which is correct for cross-node ranking via the towers API.

---

## Metrics

```
channel_power_db  = 10·log10( mean_linear_power over 6 MHz channel bins )  [dBFS]
snr_db            = channel_power_db - step_noise_db                        [dB]
score             = clamp((snr_db - FM_SNR_NORM_MIN) / (FM_SNR_NORM_MAX - FM_SNR_NORM_MIN), 0, 1)
```

`channel_power_db` → used to draw the horizontal line at the correct y-position
`snr_db` / `score`  → used for ranking / towers API

---

## Changes

### `src/dsp.h`

Add struct and function:

```cpp
struct TvChannelMetrics {
    float channel_power_db;  // mean dBFS across full channel window
    float snr_db;            // channel_power_db - noise_db; 0 if no pilot
    float score;             // normalised [0,1]; 0 if no pilot
};

TvChannelMetrics compute_tv_metrics(
    const float* raw_db, int n_fft,
    float step_fc_mhz, float ch_lo_mhz, float ch_hi_mhz,
    float noise_db, bool pilot_found);
```

### `src/dsp.cpp`

Implement `compute_tv_metrics()`:
- Reuse `freq_to_idx` lambda pattern from `compute_fm_metrics()`
- Mean channel power: convert bins in [ch_lo, ch_hi] to linear, average, back to dB
- If `pilot_found`: compute `snr_db` and normalised `score`; else return zeros

### `src/main.cpp`

Add `TvChannelMetrics tv` to `ChannelResult`:

```cpp
struct ChannelResult {
    const Channel*           ch;
    std::vector<ChannelPeak> peaks;
    FmChannelMetrics         fm;
    TvChannelMetrics         tv;   // ← new
};
```

In the TV branch of the result loop:
1. `find_channel_peaks()` as now
2. Check `any_of(peaks, is_pilot)`
3. Call `compute_tv_metrics()` with `pilot_found`
4. Emit alongside existing peaks:
   ```cpp
   ss << ",\"channel_power_db\":" << cr.tv.channel_power_db
      << ",\"snr_db\":"           << cr.tv.snr_db
      << ",\"score\":"            << cr.tv.score;
   ```

### `web/index.html`

**SSE parse**: store `channel_power_db`, `snr_db`, `score` in `peakDisplay` for TV channels.

**`channelOverlayPlugin.afterDraw()`**: after existing pilot power text block, draw
horizontal line for each confirmed TV channel:

```js
for (const [key, pd] of Object.entries(peakDisplay)) {
  if (!pd.hasPilot || pd.channelPowerDb == null) continue;
  const ch = ALL_CHANNELS.find(c => c.fc.toFixed(1) === key);
  if (!ch) continue;
  const xLoPx = xScale.getPixelForValue(ch.fc - ch.bw / 2);
  const xHiPx = xScale.getPixelForValue(ch.fc + ch.bw / 2);
  const yPx   = yScale.getPixelForValue(pd.channelPowerDb);
  if (yPx < yScale.top || yPx > yScale.bottom) continue;
  ctx.save();
  ctx.strokeStyle = '#4ade80';
  ctx.lineWidth   = 1.5;
  ctx.beginPath(); ctx.moveTo(xLoPx, yPx); ctx.lineTo(xHiPx, yPx); ctx.stroke();
  ctx.font = '9px Courier New'; ctx.fillStyle = '#4ade80'; ctx.textAlign = 'center';
  ctx.fillText(`${pd.snrDb.toFixed(0)} dB`, (xLoPx + xHiPx) / 2, yPx - 4);
  ctx.restore();
}
```

---

## Files

| File | Change |
|------|--------|
| `src/dsp.h` | `TvChannelMetrics` struct + `compute_tv_metrics()` declaration |
| `src/dsp.cpp` | `compute_tv_metrics()` implementation |
| `src/main.cpp` | `tv` field in `ChannelResult`; call + emit TV metrics |
| `web/index.html` | Store TV metrics in `peakDisplay`; draw SNR line in `afterDraw` |

---

## Verification

1. Mock: VHF/UHF pilot channels emit `score > 0`, `snr_db > 0`, `channel_power_db` present
2. Spectrum chart shows green horizontal lines at correct dBFS for mock pilot channels
3. SNR label visible above each line
4. Off-air channels (no pilot): no line, `score = 0`
5. Tests pass — no new test cases needed (TV metrics are thin wrapper over existing DSP)
