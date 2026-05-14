# retina-spectrum

Passive radar spectrum analyser for the SDRplay RSPduo. Continuously sweeps FM (88–108 MHz), VHF (174–216 MHz), and UHF (468–608 MHz), computes per-channel signal quality metrics (SNR, OBW, FM score), and streams live results to a browser via SSE.

## What it is and does

The system captures IQ samples from an RSPduo, runs them through an FFT-based DSP pipeline, and broadcasts per-step spectrum data to connected browsers in real time. It identifies FM illuminators and TV transmitters by quality — SNR, occupied bandwidth, and a gate-based score — for use as passive radar reference signals.

It is a **standalone service**, separate from the main radar stack (retina-node / blah2). Both claim the RSPduo exclusively and cannot run simultaneously.

## Architecture

- **Sweep loop** (`src/main.cpp`) — 69 steps across FM/VHF/UHF at 3 MHz spacing. Per step: retune SDR, wait for `rfChanged` callback, capture samples, run DSP, compute metrics, broadcast SSE event.
- **DSP pipeline** (`src/dsp.cpp`) — Blackman window → FFT (N_FFT=8192) → fftshift → magnitude² → peak-max decimation → dBFS. Noise floor from 25th percentile; OBW via ITU-R SM.443-4 at β=0.10; SNR = OBW mean vs noise floor.
- **Ring buffer** (`src/ring.h`) — Rolling 20-step linear-domain average for stable metrics. Metrics gated until 5 entries accumulated.
- **FM scoring** — Gate: SNR ≥ 15 dB AND OBW fraction ≥ 0.42. Passing signals scored 0–1 by normalised SNR (5–50 dB range).
- **HTTP + SSE** (`src/main.cpp`, cpp-httplib) — `GET /api/events` streams `start` / `step` / `complete` events. Focus mode (`/api/focus?fc=X`) switches to single-channel full-resolution (1024 bins, 7.8 kHz/bin) loop.

## Two UIs

| Route | File | Purpose |
|-------|------|---------|
| `/` | `web/wizard.html` | Production — setup wizard, SVG spectrum chart, signal summary, "Send RF profile" |
| `/debug` | `web/index.html` | Debug — Chart.js interactive chart, FM/VHF/UHF tabs, focus mode, rank mode |

## Key constants (`src/config.h`)

- `N_FFT = 8192`, `N_AVG = 8`, `N_DISPLAY = 64` (sweep), `N_DISPLAY_FOCUS = 1024`
- `FM_SNR_GATE_DB = 15.0`, `FM_MOB_GATE_FRAC = 0.42`, `FM_OBW_BETA = 0.10`
- `HTTP_PORT = 3020`

## Deployment

Runs as a Docker container on Raspberry Pi. Requires `network_mode: host`, `pid: host`, and `privileged: true` for SDRplay shared memory and USB access. Startup command kills any stale `sdrplay_apiService` before claiming the RSP.

Exposed publicly via Cloudflare Tunnel at `spectrumx.retnode.com`.

## Stopping/starting alongside blah2

The retina-node stack includes blah2 and must be stopped before running the spectrum analyser:

```bash
cd /data/mender-docker-compose/current/manifests && docker compose -p retina-node down
cd /data/dev/retina-spectrum && docker compose up --build
```

Restore radar stack: `docker compose down` in retina-spectrum, then `docker compose -p retina-node up -d` in manifests.

## Mock mode (Mac, no hardware)

```bash
brew install fftw
cmake -B build-mac -DCMAKE_BUILD_TYPE=Release -DMOCK_ONLY=ON && cmake --build build-mac --parallel
./build-mac/retina-spectrum --mock --web-dir web
```
