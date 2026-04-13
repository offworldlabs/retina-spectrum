# Plan 010 — FM Illuminator Quality Metrics

## Background & Literature Basis

For passive FM radar, illuminator quality is determined by three things
(Griffiths NATO SET-119 2010; Baker & Griffiths IEE 2005):

1. **Received SNR** — minimum signal level for usable reference channel
2. **Instantaneous bandwidth** — determines range resolution (~750 m per 100 kHz)
3. **Ambiguity function shape** — noise-like signal → thumbtack AF → no false targets

The literature does not use SFM or OBW directly for illuminator ranking. However:
- **OBW (ITU-R SM.443-4 β/2 method)** is the correct standard measure of instantaneous bandwidth
- **SFM (Wiener entropy)** is a valid mathematical proxy for ambiguity function noise-likeness, even though it comes from audio (MPEG-7). SFM→1 implies white-noise-like PSD → ideal thumbtack AF.

Both are used here as engineering proxies for the two literature criteria, not
as direct literature citations.

---

## Previous approach (Plan 008/009) and its problems

Plan 008 introduced `occupancy` = fraction of channel bins > noise_floor + 10 dB.

Problems found on hardware:
1. Strong narrowband peaks inflate bin count via Blackman sidelobe leakage
2. Hardcoded 10 dB threshold is fragile
3. Bin-counting ignores how much power is in each bin

---

## New metrics

### 1. SNR — unchanged

```
snr_db = 10*log10( mean_linear_power(channel) / noise_floor_linear )
```

Noise floor = 25th percentile across all N_FFT bins (existing `estimate_step_noise_floor`).

### 2. OBW fraction — ITU-R SM.443-4 β/2 method

**Not** sort-by-power (non-contiguous, wrong). The standard algorithm:

```
total_power = sum of all channel bins (linear)
// integrate from low edge upward until 0.5% of total power consumed
// integrate from high edge downward until 0.5% of total power consumed
// OBW = remaining bandwidth between the two cut points
obw_fraction = OBW_bins / total_channel_bins  ∈ [0, 1]
```

- Wideband music  → obw_fraction ≈ 0.85–0.95
- Dead air/carrier → obw_fraction ≈ 0.01–0.05
- Leakage-robust: edge leakage contributes <1% of total power so β/2 cuts past it

### 3. SFM — Wiener entropy within OBW only

```
SFM = geometric_mean(power_bins_in_OBW) / arithmetic_mean(power_bins_in_OBW)
```

Computed **only over bins inside the OBW** (between the two β/2 cut points).
This avoids the original problem (noise bins outside OBW collapsing geo_mean).

- White-noise-like FM music → SFM ≈ 0.7–0.9
- Sparse subcarrier structure → SFM ≈ 0.1–0.3
- Single tone → SFM → 0
- Return SFM=0 if fewer than 3 bins in OBW

### 4. Score (frontend only)

Raw metrics are displayed unnormalised (`snr_db`, `obw_fraction`, `sfm`).
Score uses a normalised SNR to avoid mixing dB with dimensionless fractions:

```
snr_norm     = clamp((snr_db - SNR_NORM_MIN) / (SNR_NORM_MAX - SNR_NORM_MIN), 0, 1)
score        = snr_norm × obw_fraction × sfm   ∈ [0, 1]
```

Constants (frontend JS, easy to tune):
```js
const SNR_NORM_MIN = 5;   // dB — below this: useless, score → 0
const SNR_NORM_MAX = 35;  // dB — above this: already excellent, diminishing returns
```

Rationale:
- Below 5 dB: can't extract a usable reference signal
- Above 35 dB: already strong enough; doubling SNR at 35 dB doesn't meaningfully improve
  cross-correlation performance compared to improvements in OBW or SFM
- Linear ramp in between: interpretable, no extra tunable shape parameter
- Score ∈ [0,1]: directly interpretable, 0.7+ = excellent illuminator

Example scores with these constants:
| Station | SNR | OBW | SFM | snr_norm | score |
|---------|-----|-----|-----|----------|-------|
| Strong music | 30 dB | 0.88 | 0.80 | 0.83 | 0.58 |
| Moderate music | 18 dB | 0.82 | 0.75 | 0.43 | 0.27 |
| Dead air | 25 dB | 0.03 | 0.00 | 0.67 | 0.00 |
| CW spike | 35 dB | 0.01 | 0.00 | 1.00 | 0.00 |

All three metrics are still emitted raw in SSE — gating left to towers API.

---

## Struct change

```cpp
struct FmChannelMetrics {
    float snr_db;      // mean channel power vs noise floor (dB)
    float obw_fraction;// ITU-R OBW as fraction of channel bandwidth [0,1]
    float sfm;         // Wiener entropy within OBW [0,1] — AF noise-likeness proxy
};
```

---

## Implementation

### `src/dsp.cpp` — `compute_fm_metrics()`

Function signature (unchanged from current):
```cpp
FmChannelMetrics compute_fm_metrics(
    const float* raw_db,    // N_FFT-length dBFS array from get_raw_db()
    int          n_fft,     // = N_FFT
    float        step_fc_mhz,
    float        ch_lo_mhz, // = ch->fc_mhz - ch->bw_mhz * 0.5f
    float        ch_hi_mhz, // = ch->fc_mhz + ch->bw_mhz * 0.5f
    float        noise_db); // from estimate_step_noise_floor()
// lo, hi, n derived internally via freq_to_idx (same as current)
```

Body:
```cpp
// 0. noise_lin declared first — used for both SNR and SFM ε floor
float noise_lin = std::pow(10.0f, noise_db / 10.0f);

// 1. Convert all channel bins to linear
std::vector<float> lin(n);
float total = 0;
for (int i = 0; i < n; i++) {
    lin[i] = std::pow(10.0f, raw_db[lo + i] / 10.0f);
    total += lin[i];
}

// 2. OBW — β/2 method (ITU-R SM.443-4)
const float beta_half = 0.005f * total;  // 0.5% each side → 99% containment
float cum = 0;
int lo_cut = 0;
for (int i = 0; i < n; i++) {
    cum += lin[i];
    if (cum >= beta_half) { lo_cut = i; break; }
}
cum = 0;
int hi_cut = n - 1;
for (int i = n - 1; i >= 0; i--) {
    cum += lin[i];
    if (cum >= beta_half) { hi_cut = i; break; }
}
int obw_bins = std::max(0, hi_cut - lo_cut + 1);
float obw_fraction = (float)obw_bins / n;

// 3. SFM within OBW
// ε floor tied to noise estimate: prevents log(0) collapse without materially
// affecting real broadcast signals. Using noise_lin/100 (20 dB below noise floor)
// is consistent with the hardware noise level rather than a magic constant.
float sum_lin_obw = 0, sum_loglin_obw = 0;
const float eps = noise_lin * 0.01f;  // 20 dB below noise floor
for (int i = lo_cut; i <= hi_cut; i++) {
    sum_lin_obw    += lin[i];
    sum_loglin_obw += std::log(std::max(lin[i], eps));
}
float sfm = 0.0f;
if (obw_bins >= 3) {
    float geo   = std::exp(sum_loglin_obw / obw_bins);
    float arith = sum_lin_obw / obw_bins;
    sfm = (arith > 0.0f) ? geo / arith : 0.0f;
}

// 4. SNR
float arith_all = total / n;
float snr_db    = (arith_all > noise_lin && noise_lin > 0.0f)
                ? 10.0f * std::log10(arith_all / noise_lin) : 0.0f;

return {snr_db, obw_fraction, sfm};
```

### `src/dsp.h` — update struct + comments
### `src/main.cpp` — emit `obw_fraction` + `sfm` in SSE JSON
### `web/index.html` — display obw + sfm; score = snr_db × obw_fraction × sfm

---

## Tests (`tests/test_dsp.cpp`)

| Test | Setup | Expected | What it proves |
|------|-------|----------|----------------|
| Uniform channel | All 205 channel bins at -60 dBFS, noise at -95 dBFS | obw≈1.0, sfm≈1.0, snr>0 | Baseline: flat wideband signal scores perfectly |
| CW spike | Single bin at -30 dBFS, rest at -95 dBFS | obw<0.02, sfm=0, snr>10 | β_half ∝ spike power so noise edge can't reach it |
| Half-occupied | Bins 0–99 at -60 dBFS, bins 100–204 at -95 dBFS | obw≈0.49, sfm≈1.0, snr>0 | OBW cuts at signal/noise boundary; sfm ≈ 1.0 to tight margin (±0.02) confirms noise bins excluded from SFM window — off-by-one would depress sfm noticeably |
| Empty/noise-only | All bins at -95 dBFS | obw≈0.99, sfm≈1.0, snr≈0 | OBW is not a noise gate; SNR is. Score≈0 via snr_norm→0 |
| Two-tone | Bin 0 and bin 204 at -40 dBFS, rest at -95 dBFS | obw≈1.0, sfm<0.05 | OBW and SFM are orthogonal: full bandwidth but all power in 2 spikes |
| ε floor | Bins 0–204 all at -60 dBFS except bin 102 (centre) at -120 dBFS (effectively zero) | obw≈1.0, sfm degrades gracefully (small drop from 1.0, not collapse to 0) | Tests ε clamp within OBW: without ε, one zero bin collapses geo_mean to 0 regardless of 204 strong bins; with ε = noise_lin×0.01 the zero bin is clamped and sfm stays near 1.0 |

Two-tone is the critical orthogonality test. Half-occupied with tight sfm margin
is the OBW boundary correctness test — an off-by-one including even one noise bin
in the SFM window would depress sfm from 1.0 by a measurable amount.

Note: empty/noise-only channels are suppressed by SNR≈0 giving score≈0,
not by OBW (which is ~1.0 for noise). OBW is not a noise gate — SNR is.

---

## Known limitations

- **No temporal stability term** — metrics are computed per-step, not averaged over time. A station playing a jingle between silence may score well on one sweep and poorly on the next. EMA smoothing on `raw_db` reduces noise variance but does not mitigate content-driven SFM/OBW swings (music → speech → silence) which are the real concern for static site selection. Full temporal stability scoring (variance of SFM/OBW over N sweeps) is deferred.

---

## Files changed

| File | Change |
|------|--------|
| `src/dsp.h` | Replace `occupancy` with `obw_fraction` + add `sfm` |
| `src/dsp.cpp` | Rewrite `compute_fm_metrics()` |
| `src/main.cpp` | Emit `obw_fraction` + `sfm`; update comments |
| `web/index.html` | Display raw snr_db, obw_fraction, sfm; score = snr_norm × obw_fraction × sfm where snr_norm = clamp((snr_db - 5) / 30, 0, 1) |
| `tests/test_dsp.cpp` | 5 tests covering all metric combinations |
