# 012 — Replace All EMA with Linear-Domain Ring Buffer

## Goal

Strip out **all EMA** (`sweep_ema`, `focus_ema`, `sweep_raw_ema`, `focus_raw_ema`,
`EMA_ALPHA`, `FOCUS_EMA_ALPHA`) and replace with a single `SpectrumRing` struct.

One constant. One code path. Both modes. Display and metrics unified.

---

## Why Ring > EMA

| Property | EMA (current) | Ring Buffer |
|----------|--------------|-------------|
| Averaging domain | dB (geometric mean) ✗ | Linear (arithmetic mean) ✓ |
| Convergence indicator | None — always "valid" ✗ | `count` — exact sample count ✓ |
| Old data eviction | Never fully gone ✗ | Clean drop after N steps ✓ |
| Cold-start bias | 85% weight on seed for many steps ✗ | Uniform average from step 1 ✓ |
| Code paths | Two alphas, four buffers, six flags ✗ | One struct, one constant ✓ |
| API export gating | Not possible ✗ | `count >= METRICS_MIN_ENTRIES` ✓ |
| Memory | ~2 MB total (4 × N_FFT floats) | ~3.2 MB total — trivial on Pi 5 |

---

## Startup Time

```
Sweep step interval ≈ 35s per pass (from EMA comment: 7 sweeps ≈ 4 min)
Focus step interval = 500ms (FOCUS_PAUSE_MS)

METRICS_MIN_ENTRIES = 5  → first reliable export
METRICS_AVG_STEPS   = 20 → full ring (best quality)

Sweep:  5 entries ≈  3 min cold-start    20 entries ≈ 12 min full ring
Focus:  5 entries ≈  2.5s               20 entries ≈ 10s
```

---

## Architecture (before → after)

**Before** — four separate EMA buffers, two alphas, six validity flags:
```
s_raw_acc → s_raw_db (dB) → get_raw_db()
  sweep_raw_ema = 0.15 * raw_db + 0.85 * prev   ← dB EMA, sweep
  focus_raw_ema = 0.30 * raw_db + 0.70 * prev   ← dB EMA, focus
  sweep_ema     = 0.15 * power  + 0.85 * prev   ← display EMA, sweep
  focus_ema     = 0.30 * power  + 0.70 * prev   ← display EMA, focus
  compute_fm_metrics(cur_raw_ema)                ← re-linearises for OBW
  sl.smooth_db = cur_ema                         ← display
```

**After** — one ring, linear domain, one pass:
```
s_raw_acc → get_raw_linear()
  ring.push(linear)          ← one ring per step position, sweep + focus
  ring.get_db(averaged_db)   ← arithmetic mean in linear → dB once
  decimate → sl.smooth_db    ← display from same averaged spectrum
  compute_fm_metrics(averaged_db)   ← no interface change
```

---

## Phased Work Plan

### Phase 1 — DSP layer: expose linear spectrum
**Files:** `src/dsp.h`, `src/dsp.cpp`

`s_raw_acc` already holds N_AVG-averaged linear power. Just expose it:

```cpp
// dsp.h
const float* get_raw_linear();   // N_FFT linear power bins (N_AVG-averaged)

// dsp.cpp
static std::vector<float> s_raw_linear_buf;
const float* get_raw_linear() {
    s_raw_linear_buf.resize(N_FFT);
    const float norm = (float)N_FFT * (float)N_FFT;
    for (int k = 0; k < N_FFT; k++)
        s_raw_linear_buf[k] = (s_raw_acc[k] / N_AVG) / norm;
    return s_raw_linear_buf.data();
}
```

Verify: `get_raw_linear()[k] == pow(10, get_raw_db()[k] / 10)` for non-zero bins.

---

### Phase 2 — Config: strip EMA constants, add ring constants
**File:** `src/config.h`

```cpp
// Remove:
#define EMA_ALPHA       0.15f
#define FOCUS_EMA_ALPHA 0.3f

// Add:
#define METRICS_AVG_STEPS    20   // ring depth — both modes, display + metrics
#define METRICS_MIN_ENTRIES   5   // min count before scores/export are valid
                                  // sweep: 5 × 35s ≈ 3 min  |  focus: 5 × 500ms = 2.5s

// Raise:
#define FM_SNR_GATE_DB      15.0f   // was 10 dB — filters flat low-SNR randoms
```

---

### Phase 3 — main.cpp: SpectrumRing struct + state replacement
**File:** `src/main.cpp`

**3a. Add SpectrumRing** (top of file or small header):

```cpp
struct SpectrumRing {
    std::vector<std::vector<float>> buf;  // [METRICS_AVG_STEPS][N_FFT] linear
    std::vector<float> sum;               // running linear sum
    int head  = 0;
    int count = 0;

    void push(const float* lin, int n) {
        if (buf.empty()) {
            buf.assign(METRICS_AVG_STEPS, std::vector<float>(n, 0.0f));
            sum.assign(n, 0.0f);
        }
        if (count == METRICS_AVG_STEPS)
            for (int k = 0; k < n; k++) sum[k] -= buf[head][k];
        buf[head].assign(lin, lin + n);
        for (int k = 0; k < n; k++) sum[k] += lin[k];
        head  = (head + 1) % METRICS_AVG_STEPS;
        count = std::min(count + 1, METRICS_AVG_STEPS);
    }

    void get_db(std::vector<float>& out) const {
        out.resize(sum.size());
        const float n = (float)std::max(count, 1);
        for (int k = 0; k < (int)sum.size(); k++) {
            const float p = sum[k] / n;
            out[k] = (p > 0.0f) ? 10.0f * std::log10(p) : -120.0f;
        }
    }

    bool ready() const { return count >= METRICS_MIN_ENTRIES; }
};
```

**3b. Replace EMA state declarations** (lines 372–384):

```cpp
// Remove all of:
//   sweep_ema, sweep_ema_valid, sweep_ema_total
//   focus_ema, focus_ema_valid, focus_ema_fc
//   sweep_raw_ema, sweep_raw_ema_valid
//   focus_raw_ema

// Add:
std::vector<SpectrumRing> sweep_rings;
SpectrumRing              focus_ring;
float                     focus_ring_fc = -1.0f;
std::vector<float>        averaged_db(N_FFT, -120.0f);
```

**3c. Replace reset logic** (lines 401–417):

```cpp
// Focus: reset ring on channel change
if (is_focus && focus != focus_ring_fc) {
    focus_ring    = SpectrumRing{};
    focus_ring_fc = focus;
}

// Sweep: resize if step count changes
if (!is_focus && (int)sweep_rings.size() != total)
    sweep_rings.resize(total);
```

---

### Phase 4 — main.cpp: replace update + display + metrics block
**File:** `src/main.cpp` (lines 517–561)

```cpp
// Push new linear spectrum into ring
auto& ring = is_focus ? focus_ring : sweep_rings[i];
ring.push(get_raw_linear(), N_FFT);
ring.get_db(averaged_db);

// Display: decimate averaged_db → N_DISPLAY bins (max within GROUP)
constexpr int GROUP = N_FFT / N_DISPLAY;
sl.smooth_db.resize(is_focus ? N_DISPLAY_FOCUS : N_DISPLAY);
{
    const int disp = is_focus ? N_DISPLAY_FOCUS : N_DISPLAY;
    const int grp  = N_FFT / disp;
    for (int d = 0; d < disp; d++) {
        float peak = -120.0f;
        for (int g = 0; g < grp; g++)
            peak = std::max(peak, averaged_db[d * grp + g]);
        sl.smooth_db[d] = peak;
    }
}
sl.power_db = sl.smooth_db;

// Metrics: gate on ring readiness
if (ring.ready()) {
    const float step_noise_db = estimate_step_noise_floor(averaged_db.data(), N_FFT);
    for (const Channel* ch : step_channels) {
        // ... existing per-channel loop unchanged
        // pass averaged_db.data() instead of cur_raw_ema.data()
    }
}
```

**Remove flags-set block** (lines 627–629) — `sweep_ema_valid`, `sweep_raw_ema_valid`,
`focus_ema_valid` all deleted.

---

### Phase 5 — Tests (`tests/test_dsp.cpp` + new `tests/test_ring.cpp`)

**SpectrumRing unit tests** (`tests/test_ring.cpp`):

```
TEST: push single entry → get_db() matches 10*log10(input) exactly
TEST: push N identical entries → get_db() == 10*log10(input) (average of N identical = same)
TEST: push N+1 entries → oldest evicted, average reflects only last N
TEST: ready() false before METRICS_MIN_ENTRIES, true after
TEST: reset (assign SpectrumRing{}) → count=0, get_db() returns -120 dB floor
TEST: two entries A and B → get_db() == 10*log10((A+B)/2) in linear (arithmetic, not geometric)
TEST: linear vs geometric mean differ for non-uniform input — confirm ring uses linear
```

**`get_raw_linear()` smoke test** (`tests/test_dsp.cpp`):
```
TEST: get_raw_linear()[k] ≈ pow(10, get_raw_db()[k] / 10) for all non-silent bins
      (confirms the new function is consistent with the existing dB output)
```

---

### Phase 6 — Build, test, deploy
1. `cmake -B build-mock -DMOCK=ON && cmake --build build-mock` — confirm compile
2. Run `tests/test_ring` — all ring tests pass
3. Run existing `tests/test_dsp` — no regressions
4. Run mock: OBW stable after `METRICS_MIN_ENTRIES` captures
5. Deploy to node, confirm:
   - OBW no longer jumps 50→37%
   - Low-SNR flat stations absent from ranked list (SNR gate 15 dB)
   - Focus channel change resets ring cleanly

---

## Memory

```
Per ring: 20 × 8192 × 4 bytes = 640 KB
sweep_rings (~4 positions): 2.56 MB
focus_ring: 640 KB
Total: ~3.2 MB                   ← trivial
```

## Files touched

| File | Phase | Change |
|------|-------|--------|
| `src/dsp.h` | 1 | Declare `get_raw_linear()` |
| `src/dsp.cpp` | 1 | Implement `get_raw_linear()` |
| `src/config.h` | 2 | Remove `EMA_ALPHA`/`FOCUS_EMA_ALPHA`; add `METRICS_AVG_STEPS`/`METRICS_MIN_ENTRIES`; raise `FM_SNR_GATE_DB` |
| `src/main.cpp` | 3–4 | `SpectrumRing`; replace all EMA state + update loop |
| `tests/test_ring.cpp` | 5 | New: SpectrumRing unit tests |
| `tests/test_dsp.cpp` | 5 | Add `get_raw_linear()` consistency test |
| `CMakeLists.txt` | 5 | Wire `test_ring.cpp` into test build |
