# .plans/002-phase2-dsp.md — Phase 2: Real C++ Sweep Binary

## Goal
Replace the nginx placeholder with a real C++ sweep binary. Sweep FM/DAB/UHF DVB-T,
serve the spectrum as JSON + SSE, display as a Chart.js line plot. No peak detection
yet — just prove the pipeline works end to end.

---

## Workplan

### 0. Parameters header
- [ ] Write `src/config.h` — all tuneable parameters as `#define`
  ```c
  // DSP
  #define N_FFT           8192    // samples captured and FFT'd per step (~977 Hz/bin at 8 MHz fs)
  #define N_DISPLAY       64      // display bins per step — 125 kHz/bin
  // NOTE: No averaging — broadcast towers are 20-40 dB above noise floor,
  //       single FFT is sufficient. Add averaging in Phase 3 if needed.

  // SDR
  #define SAMPLE_RATE_HZ      8000000
  #define AGC_SETPOINT        -30     // dBfs AGC target
  #define RESET_TIMEOUT_MS    200     // max ms to wait for reset flag after retune

  // Server
  #define HTTP_PORT   3020
  ```

### 1. SDR layer
- [ ] Copy `blah2-arm/src/capture/rspduo/RspDuo.h` → `src/sdr.h`
  - Strip `Source` inheritance and `IqData` includes
  - `#include "config.h"` — use `N_FFT`, `AGC_SETPOINT`, `SAMPLE_RATE_HZ`
  - Declare globals: `g_tuner`, `g_capture_buf`, `g_capture_done`, `g_waiting_reset`
  - Keep static C callback wrapper pattern verbatim
- [ ] Copy `blah2-arm/src/capture/rspduo/RspDuo.cpp` → `src/sdr.cpp`
  - Remove class wrapper, keep all SDR globals verbatim
  - `get_device()`: change `Tuner_Both` + `Dual_Tuner` → `g_tuner` + `Single_Tuner`
  - `set_device_parameters()`: BW_8_000, IF_Zero, decimation off, notches both 0, AGC 50Hz
  - `stream_a_callback()`: check `reset` flag first, then accumulate — see below
  - Add `retune()`: call `sdrplay_api_Update`, set `g_waiting_reset = true` (no fixed sleep)
  - Keep `open_api()`, `event_callback()`, `initialise_device()`, `uninitialise_device()` verbatim

  **`stream_a_callback()` logic — reset-aware, no fixed sleep:**
  ```cpp
  // globals
  std::atomic<bool> g_waiting_reset{false};  // set true by retune(), cleared on reset flag
  std::atomic<bool> g_capture_done{false};
  std::vector<std::complex<float>> g_capture_buf;

  void stream_a_callback(short *xi, short *xq, ..., unsigned int numSamples,
                         unsigned int reset, ...) {
      if (g_waiting_reset) {
          if (!reset) return;              // still old frequency — discard
          g_capture_buf.clear();
          g_waiting_reset = false;         // reset seen — now at new frequency
      }
      if (g_capture_done) return;
      for (unsigned int i = 0; i < numSamples; i++)
          g_capture_buf.push_back({(float)xi[i], (float)xq[i]});
      if ((int)g_capture_buf.size() >= N_FFT)
          g_capture_done = true;
  }

  void retune(double fc_hz) {
      chParams->tunerParams.rfFreq.rfHz = fc_hz;
      g_capture_done   = false;
      g_waiting_reset  = true;             // discard samples until reset fires
      sdrplay_api_Update(chosenDevice->dev, g_tuner,
          sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
      // no sleep — sweep thread spins on g_capture_done
  }
  ```

  **Sweep thread timeout fallback** (in `main.cpp`): if `g_capture_done` hasn't fired
  within `RESET_TIMEOUT_MS`, log a warning and skip the step — avoids infinite hang
  if reset never arrives.

### 2. DSP layer
- [ ] Write `src/dsp.cpp` (adapted from `blah2-arm/src/process/spectrum/SpectrumAnalyser.cpp`)
  - Switch `double`/`fftw_` → `float`/`fftwf_`
  - Add Hanning window before FFT
  - Copy fftshift logic verbatim: `(k + nfft/2 + 1) % nfft`
  - No averaging — single FFT per step
  - **Correct DSP order: stay in linear power domain until the very last step**
    ```
    1. Hanning window → FFT → fftshift               (complex)
    2. Magnitude squared per bin: re² + im²           (linear power)
    3. Decimate: average 128 linear-power bins → 1    (still linear power)
    4. dBFS conversion LAST: 10*log10f(avg / N_FFT²)  (one conversion for all bins)
    ```
    Averaging dB values is wrong — must decimate in linear domain first.
  - Frequency axis from actual `fc_mhz`, not hardcoded 204640000 Hz offset

### 3. Sweep + HTTP server
- [ ] Write `src/main.cpp`
  - Parse `--mock`, `--tuner A|B` flags
  - Init SDR or skip for mock
  - Sweep thread: step list → retune → wait capture → process_step → ring buffer → SSE
  - Mock generator: noise floor + Gaussian peaks at known broadcast freqs
  - httplib server: `/`, `/api/status`, `/api/sweep`, `/api/start`, `/api/events` (SSE)
  - Per-client SSE queue with condition variable (so sweep thread pushes, HTTP thread blocks)

### 4. Build system
- [ ] Write `CMakeLists.txt`
  - Sources: `src/main.cpp`, `src/sdr.cpp`, `src/dsp.cpp`
  - Deps: `fftw3f`, `sdrplay_api`, `pthread`, `httplib` (FetchContent)
- [ ] Update `Dockerfile` → multi-stage: cmake build → debian-slim runtime
- [ ] Update `docker-compose.yml` → add `privileged: true`, `/dev/bus/usb`, libsdrplay mount

### 5. Web UI
- [ ] Update `web/index.html`
  - Chart.js line chart: frequency MHz (x) vs power dBfs (y), no point markers
  - Shaded band regions: FM / DAB / UHF
  - On load: POST `/api/start`, open `EventSource /api/events`
  - Append each SSE step to chart live (scanning effect)
  - On `complete` event: close SSE, show "Scan again" button

---
> **First test point** — after steps 0–5 are complete, run `./retina-spectrum --mock`
> on Mac. No SDR hardware needed. Verifies the full HTTP/SSE/Chart.js pipeline.
> Step 6 is the first real hardware test on the node.
---

### 6. Test
- [ ] Mock mode on Mac: `./retina-spectrum --mock` → `localhost:3020` shows synthetic peaks
- [ ] Push to node, `docker compose up`, real sweep via `spectrum2a.retnode.com`
- [ ] `curl localhost:3020/api/sweep` — verify JSON arrays equal length

---

---

## Repo structure (Phase 2)

```
retina-spectrum/
├── src/
│   ├── sdr.h                 stripped RspDuo.h — no Source/IqData deps, adds g_tuner
│   ├── sdr.cpp               copied RspDuo.cpp — minimal changes only (see below)
│   ├── dsp.cpp               adapted SpectrumAnalyser.cpp — float FFT, real fc, avg+decimate
│   └── main.cpp              sweep loop + httplib HTTP/SSE server
├── web/
│   └── index.html            Chart.js UI, SSE client
├── CMakeLists.txt
├── Dockerfile                multi-stage: build → runtime
└── docker-compose.yml        port 3020, /dev/bus/usb + libsdrplay mount
```

---

## Sweep bands

At 8 MHz per step, coverage is efficient:

| Band | Range | Steps | Notes |
|------|-------|-------|-------|
| FM | 88–108 MHz | 3 | 92, 100, 108 MHz |
| DAB | 174–240 MHz | 9 | 178, 186, …, 242 MHz |
| UHF DVB-T | 470–694 MHz | 28 | 474, 482, …, 690 MHz |
| **Total** | | **40 steps** | ~4 seconds @ 100ms/step |

```cpp
struct Band {
    std::string name;
    float start_mhz;
    float stop_mhz;
    float step_mhz;
};

const std::vector<Band> BANDS = {
    {"fm",      88.0f,  108.0f, 8.0f},
    {"dab",    174.0f,  240.0f, 8.0f},
    {"uhf",    470.0f,  694.0f, 8.0f},
};
```

---

## DSP pipeline

```
RSPDuo Tuner A (8 MHz BW, zero-IF, AGC 50Hz @ -30 dBfs)
  │
  │  retune(fc_hz) → sdrplay_api_Update → g_waiting_reset = true
  │
  │  stream_a_callback:
  │    if g_waiting_reset && reset==0 → discard (still old frequency)
  │    if g_waiting_reset && reset!=0 → clear buf, g_waiting_reset=false (PLL locked)
  │    push_back {xi[i], xq[i]} until N_FFT samples → g_capture_done = true
  ▼
Capture buffer  std::vector<complex<float>>, N_FFT = 8192 samples (~1ms at 8 MHz)
  │
  ▼  dsp.cpp: process_step(fc_mhz, capture_buf)
Hanning window  x[n] *= 0.5 * (1 - cos(2π·n / (N_FFT - 1)))
  │
  ▼
FFTW3 (float)   fftwf_plan_dft_1d, N_FFT = 8192, in-place
  │
  ▼
fftshift        (k + N_FFT/2 + 1) % N_FFT  — verbatim from SpectrumAnalyser.cpp
  │
  ▼
Magnitude²      power[k] = re² + im²        ← stay LINEAR, do not convert to dB yet
  │
  ▼
Decimate        average 128 linear-power bins → 1 display bin  (N_FFT/N_DISPLAY = 128)
  │
  ▼
dBFS LAST       display[d] = 10 * log10f(avg_power / N_FFT²)   ← single conversion
  │
  ▼
Store slice     write Slice{fc_mhz, freq_start, freq_stop, power_db[64]} to ring buffer
                broadcast SSE event immediately
```

### Key DSP numbers

| Parameter | Value | Reason |
|-----------|-------|--------|
| Sample rate fs | 8 MHz | Covers 1 DVB-T channel per step |
| Bandwidth | `sdrplay_api_BW_8_000` | Matches fs |
| IF mode | `sdrplay_api_IF_Zero` | Zero-IF, no decimation needed |
| FFT size N | 8192 | ~977 Hz/bin, 1.024ms capture per step |
| Averaging | None | Broadcast towers 20–40 dB above noise — not needed |
| Display bins | 64 | 125 kHz/bin displayed |
| Settle | Wait for `reset` flag | SDK signals PLL locked — no fixed sleep |
| Timeout | 200ms | Skip step and warn if reset never fires |
| Total time/step | ~1ms + lock time | Dominated by PLL settle, not capture |

---

## Shared state (ring buffer)

```cpp
const int N_STEPS   = 64;   // generous upper bound across all bands
const int N_FFT     = 8192;
const int N_DISPLAY = 64;
// N_AVG removed — single FFT per step is sufficient for broadcast towers

struct Slice {
    float    fc_mhz;
    float    freq_start_mhz;   // fc - 4.0
    float    freq_stop_mhz;    // fc + 4.0
    std::array<float, N_DISPLAY> power_db;
    bool     valid = false;
};

struct SweepState {
    std::mutex          mtx;
    std::string         state = "idle";   // idle | sweeping | complete
    int                 progress_pct = 0;
    std::array<Slice, N_STEPS> buffer;
    std::vector<std::function<void(const std::string&)>> sse_clients;
};

SweepState g_sweep;
```

The ring buffer is indexed by step position (0..N_STEPS-1). Each slot is written by
the sweep thread and read by the HTTP thread. The `/api/sweep` endpoint stitches all
valid slices into a flat JSON array.

---

## SDR setup — copied from blah2-arm

### src/sdr.cpp — what is copied vs changed

`sdr.cpp` is `blah2-arm/src/capture/rspduo/RspDuo.cpp` with the class shell removed
(no `Source` inheritance, no `IqData`). All function bodies are copied verbatim except
where noted below.

**Verbatim copy (zero changes):**
- `open_api()` — sdrplay_api_Open + version check
- `event_callback()` — gain change + overload ack + device removed
- `initialise_device()` — sdrplay_api_Init
- `uninitialise_device()` — sdrplay_api_Uninit + ReleaseDevice + Close

**`get_device()` — one line changed:**
```cpp
// blah2 (dual tuner for radar):
chosenDevice->tuner      = sdrplay_api_Tuner_Both;
chosenDevice->rspDuoMode = sdrplay_api_RspDuoMode_Dual_Tuner;

// retina-spectrum (single tuner, configurable):
chosenDevice->tuner      = g_tuner;   // sdrplay_api_Tuner_A or _B, default A
chosenDevice->rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
```

**`set_device_parameters()` — four things changed:**
```cpp
// 1. 8 MHz bandwidth + zero-IF (blah2 used IF_1_620 + narrow BW + decimation)
chParams->tunerParams.bwType                    = sdrplay_api_BW_8_000;
chParams->tunerParams.ifType                    = sdrplay_api_IF_Zero;
chParams->ctrlParams.decimation.enable          = 0;
chParams->ctrlParams.decimation.decimationFactor = 1;

// 2. AGC 50 Hz at -30 dBfs (same as blah2's AGC_50HZ path, just hardcoded)
chParams->ctrlParams.agc.enable        = sdrplay_api_AGC_50HZ;
chParams->ctrlParams.agc.setPoint_dBfs = -30;

// 3. Notches OFF — blah2 enables both to block FM/DAB for radar; we want to see them
chParams->rspDuoTunerParams.rfNotchEnable    = 0;
chParams->rspDuoTunerParams.rfDabNotchEnable = 0;

// 4. Only StreamA callback needed (single tuner); StreamB set to no-op stub
cbFns.StreamACbFn = _stream_a_callback;
cbFns.StreamBCbFn = _stream_b_callback_noop;
cbFns.EventCbFn   = _event_callback;
```

**`stream_a_callback()` — gutted (blah2 interleaves dual-channel into buffer_16_ar; we just accumulate):**
```cpp
// blah2: malloc buffer_16_ar, interleave xi/xq at slots 0,1 leaving 2,3 for tuner B
// retina-spectrum: just push into capture vector
void stream_a_callback(short *xi, short *xq, ..., unsigned int numSamples, ...) {
    if (g_capture_done) return;
    for (unsigned int i = 0; i < numSamples; i++)
        g_capture_buf.push_back({(float)xi[i], (float)xq[i]});
    if ((int)g_capture_buf.size() >= N_CAPTURE)
        g_capture_done = true;
}
```

**`retune()` — new, not in blah2 (blah2 never retunes mid-stream):**
```cpp
void retune(double fc_hz) {
    chParams->tunerParams.rfFreq.rfHz = fc_hz;
    sdrplay_api_Update(chosenDevice->dev, g_tuner,
        sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_MS));
}
```

### src/sdr.h — stripped RspDuo.h

Same static C callback wrapper pattern from `RspDuo.h`:
```cpp
static void _stream_a_callback(short *xi, short *xq,
    sdrplay_api_StreamCbParamsT *params, unsigned int numSamples,
    unsigned int reset, void *cbContext)
{
    stream_a_callback(xi, xq, params, numSamples, reset, cbContext);
}
```
No class, no `Source`/`IqData` includes. Just declarations for the five functions above
plus the SDR globals (`chosenDevice`, `deviceParams`, `chParams`, `cbFns`, `err`)
and the tuner global:
```cpp
extern sdrplay_api_TunerSelectT g_tuner;  // default sdrplay_api_Tuner_A
```

### CLI flag — --tuner A|B

Parsed in `main()`, sets `g_tuner` before `open_api()` is called:
```
./retina-spectrum              → Tuner A (SMA1, reference antenna) [default]
./retina-spectrum --tuner B    → Tuner B (SMA2, surveillance antenna)
./retina-spectrum --mock       → mock mode, no SDR
```

### src/dsp.cpp — adapted from SpectrumAnalyser.cpp

`SpectrumAnalyser.cpp` uses `double` + `fftw_` and has a blah2-specific 204640000 Hz
offset. We adapt to:
- `float` + `fftwf_` (faster on ARM, matches capture buffer type)
- Hanning window applied before FFT (SpectrumAnalyser.cpp has no window)
- N_AVG=4 FFTs accumulated then averaged before dBFS conversion
- `fftshift` logic copied verbatim: `(k + nfft/2 + 1) % nfft`
- Decimate N_FFT→N_DISPLAY by **averaging** 128-bin groups (not striding as blah2 does)
- Frequency axis computed from actual `fc_mhz` (not hardcoded 204640000 Hz offset)

---

## Sweep loop (thread)

```cpp
void sweep_thread_fn() {
    std::vector<std::pair<float, std::string>> steps;
    for (auto& band : BANDS)
        for (float fc = band.start_mhz; fc <= band.stop_mhz; fc += band.step_mhz)
            steps.push_back({fc, band.name});

    { std::lock_guard lk(g_sweep.mtx); g_sweep.state = "sweeping"; }

    for (int i = 0; i < (int)steps.size(); i++) {
        auto [fc_mhz, band] = steps[i];

        retune(fc_mhz * 1e6f);

        g_capture_buf.clear();
        g_capture_done = false;
        while (!g_capture_done) std::this_thread::sleep_for(1ms);

        auto power = process_step(fc_mhz);   // Hanning + FFT + avg + decimate

        Slice slice;
        slice.fc_mhz         = fc_mhz;
        slice.freq_start_mhz = fc_mhz - 4.0f;
        slice.freq_stop_mhz  = fc_mhz + 4.0f;
        slice.power_db       = power;
        slice.valid          = true;

        int pct = (i + 1) * 100 / (int)steps.size();
        {
            std::lock_guard lk(g_sweep.mtx);
            g_sweep.buffer[i]    = slice;
            g_sweep.progress_pct = pct;
        }
        sse_broadcast(i, slice, pct);
    }

    std::lock_guard lk(g_sweep.mtx);
    g_sweep.state = "complete";
}
```

---

## HTTP API (httplib, port 3020)

```
GET  /              → web/index.html
GET  /api/status    → { "state": "sweeping", "progress_pct": 67 }
GET  /api/sweep     → full stitched ring buffer as flat JSON (see below)
GET  /api/events    → SSE stream — one event per step + final complete
POST /api/start     → trigger new sweep (no-op if already sweeping)
```

### /api/sweep response

Stitches all valid ring buffer slots into flat parallel arrays:

```json
{
  "state": "complete",
  "data": {
    "frequency_mhz": [88.0, 88.125, 88.25, ...],
    "power_db":      [-62.3, -61.1, -63.0, ...]
  }
}
```

Each slice contributes 64 frequency/power pairs. 40 slices → 2560 points total.

### /api/events (SSE)

```
data: {"type":"step","step":12,"fc_mhz":530,"freq_start":526,"freq_stop":534,"power_db":[...],"progress_pct":67}

data: {"type":"complete"}
```

---

## Mock mode (--mock flag)

Skips SDR + FFTW. Generates synthetic spectrum:
- White noise floor at -60 dBfs
- Gaussian peaks at 98.8 MHz (FM), 202 MHz (DAB), 530 + 610 MHz (UHF DVB-T)
- Same ring buffer writes, same SSE events, same HTTP API

For developing/testing the web UI on Mac with no SDR attached.

---

## Web UI (web/index.html, Chart.js)

- Line chart: x = frequency (MHz), y = power (dBfs), `pointRadius: 0`
- Band regions shaded as background annotations (FM / DAB / UHF)
- SSE updates chart slice-by-slice as sweep progresses (scanning effect)
- On load: POST /api/start, open EventSource /api/events
- On complete event: close EventSource, final render

---

## Dockerfile (multi-stage)

```dockerfile
FROM debian:bookworm AS build
RUN apt-get update && apt-get install -y cmake g++ libfftw3-dev
COPY src/ /app/src/
COPY CMakeLists.txt /app/
COPY web/ /app/web/
WORKDIR /app/build
RUN cmake .. && make -j4

FROM debian:bookworm-slim AS runtime
COPY --from=build /app/build/retina-spectrum /usr/local/bin/
COPY --from=build /app/web/ /web/
# libsdrplay_api.so.3.15 bind-mounted from host at runtime
CMD ["retina-spectrum"]
```

docker-compose additions:

```yaml
volumes:
  - /dev/bus/usb:/dev/bus/usb
  - /usr/local/lib/libsdrplay_api.so.3.15:/usr/local/lib/libsdrplay_api.so.3.15
privileged: true
```

---

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(retina-spectrum)
set(CMAKE_CXX_STANDARD 17)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F REQUIRED fftw3f)

# httplib vendored single-header (cpp-httplib)
include(FetchContent)
FetchContent_Declare(httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG        v0.15.3)
FetchContent_MakeAvailable(httplib)

add_executable(retina-spectrum
    src/main.cpp
    src/sdr.cpp
    src/dsp.cpp)
target_link_libraries(retina-spectrum
    ${FFTW3F_LIBRARIES} sdrplay_api pthread httplib::httplib)
target_include_directories(retina-spectrum PRIVATE
    ${FFTW3F_INCLUDE_DIRS}
    /usr/local/include)   # sdrplay_api.h location on host/container
```

---

## Thread architecture

```
main()
  ├── parse args (--mock, --tuner A|B)
  ├── init SDR (or skip if mock): open_api → get_device → set_device_parameters → initialise_device
  ├── auto-trigger sweep_thread on launch
  └── httplib::Server::listen() — blocks, handles HTTP + SSE on port 3020

sweep_thread
  └── for each step:
        retune(fc_hz)          ← sdrplay_api_Update + SETTLE_MS sleep
        wait g_capture_done    ← spin on atomic bool
        process_step()         ← dsp.cpp: Hanning + fftwf + fftshift + avg + decimate
        write ring buffer[i]
        sse_broadcast()        ← push SSE message to all connected clients

SDR callback thread (SDK-managed)
  └── stream_a_callback → push_back to g_capture_buf → set g_capture_done when N_CAPTURE reached

HTTP threads (httplib thread pool)
  └── GET /api/events → per-client SseClient queue, blocks until sweep complete or disconnect
```

---

## Verification

1. **Mock on Mac**: `./retina-spectrum --mock` → `localhost:3020` shows Chart.js
   spectrum with synthetic FM/DAB/UHF peaks, SSE updates chart slice by slice
2. **Real sweep on node**: `docker compose up` on Pi with RSPDuo →
   `spectrum2a.retnode.com` shows live spectrum, DVB-T towers visible as elevated
   power blocks in UHF band
3. **API**: `curl localhost:3020/api/sweep` returns valid JSON with
   `frequency_mhz` and `power_db` arrays of equal length

---

## Phase 3 (future)
- Peak detection: noise floor = median(all bins), threshold = floor + 10 dB,
  local maxima, merge within 1 MHz, classify by band, snap UHF to DVB-T channels
- `/api/peaks` endpoint for retina-gui onboarding integration
- "Use these frequencies" button in UI → passes peaks to Tower-Finder
