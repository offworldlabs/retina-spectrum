# Plan: Single-Frequency Focus Mode

## Context
Sweep mode steps through 106 frequencies. For development/debugging (e.g. diagnosing
PPM offset against a known station) you want to lock onto one fc and see the full
8 MHz window continuously at full 64-bin resolution.

Triggered via URL param: `http://node:3020/?fc=98.8`
Clear focus via: `http://node:3020/` (no param = normal sweep)

---

## Backend — main.cpp

### 1. Global focus state
```cpp
static std::atomic<float> g_focus_mhz{0.0f};  // 0 = sweep mode
```

### 2. New `/api/focus` endpoint
```
GET /api/focus?fc=98.8  → set focus, returns {"focus_mhz":98.8}
GET /api/focus?fc=0     → clear focus (back to sweep)
GET /api/focus          → returns {"focus_mhz":0}
```
Reads `fc` query param, validates it's a plausible RF frequency (50–1000 MHz),
sets `g_focus_mhz`. The running sweep thread picks it up on its next outer loop.

### 3. append_bins helper (replaces hardcoded append_centre)
```cpp
static void append_bins(std::ostringstream& ss,
                         const std::array<float, N_DISPLAY>& arr,
                         int lo, int hi)   // hi exclusive
```
Sweep mode calls with `TRIM_LO, TRIM_HI` (24 bins).
Focus mode calls with `0, N_DISPLAY` (all 64 bins).

### 4. slice_to_sse — add bool full_bins param
- `freq_start/stop`: ±TRIM_HALF_MHZ (sweep) or ±4.0f (focus)
- bin output: TRIM range (sweep) or full 64 (focus)

### 5. sweep_fn restructure
At top of outer `while(true)`:
- Read `g_focus_mhz`
- If > 0: `steps = {{focus_mhz, "focus"}}` (single step)
- Else: `steps = build_steps()` (normal 106 steps)
- Reset EMA if step count OR focus frequency changed

`start` SSE event gains a `mode` field:
```json
{"type":"start","mode":"focus","fc":98.8}
{"type":"start","mode":"sweep"}
```
Frontend uses this to clear chart and reset stepSlots when mode changes.

---

## Frontend — index.html

### On page load
```js
const params = new URLSearchParams(window.location.search);
const focusFc = params.get('fc') ? parseFloat(params.get('fc')) : null;
if (focusFc) fetch(`/api/focus?fc=${focusFc}`);
else         fetch('/api/focus?fc=0');  // ensure sweep mode on plain load
```

### On `start` event
If mode or fc changed vs last known: clear all datasets + stepSlots, update X axis.
- Sweep mode: X axis min=84, max=698 (full bands)
- Focus mode: X axis min=fc-4.5, max=fc+4.5 (8 MHz window + margin)

### Status line
- Sweep: `scanning… 42%`
- Focus: `FOCUS 98.8 MHz — scanning…`

---

## Verify
1. `http://node:3020/` — normal sweep, 106 steps, 24 bins each, unchanged
2. `http://node:3020/?fc=98.8` — single step, 64 bins, ±4 MHz around 98.8 MHz,
   loops continuously, X axis narrows to 94–103.3 MHz
3. Navigate between the two — chart clears and redraws correctly each time
4. Mock mode: `./retina-spectrum --mock --web-dir web` — both modes work
