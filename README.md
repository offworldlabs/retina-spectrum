# retina-spectrum

Passive radar spectrum analyser for the SDRplay RSPduo. Used to identify RF illuminators — FM broadcast stations and TV transmitters — and measure their signal quality as candidate reference signals for passive radar.

The system sweeps FM (88–108 MHz), VHF (174–216 MHz), and UHF (468–608 MHz) and computes per-transmitter metrics: signal strength (dBFS), SNR, and occupied bandwidth. These metrics are intended to be consumed by an external API to rank and select illuminators — the analyser surfaces the data, it does not make the ranking decision itself. The "Send RF profile" button in the wizard UI exports the full signal profile for handoff.

## Interfaces

The server runs on port 3020 and serves two UIs:

| Route | Purpose |
|-------|---------|
| `http://<ip>:3020/` | **Setup wizard** — scan, signal summary, ranked illuminator list. This is the production view. |
| `http://<ip>:3020/debug` | **Debug spectrum** — full interactive Chart.js chart, FM/VHF/UHF tabs, focus mode, rank overlay. |

Use `/` for a site survey. Use `/debug` for development and signal investigation.

## Running on hardware (Raspberry Pi)

**Hardware required:**
- SDRplay RSPduo with the SDRplay API service running on the host
- Reference antenna connected to **SMA1 (Tuner A)** — this is the default input

The container uses Tuner A (SMA1) by default. To use Tuner B (SMA2), pass `--tuner B` in the compose command.

```bash
docker compose up --build
```

Open `http://<pi-ip>:3020`.

## Running on Mac (mock mode)

No SDR hardware needed. Generates synthetic IQ data with mock FM and TV transmitters.

**Prerequisites (one-time):**
```bash
brew install fftw
```

**Build:**
```bash
cmake -B build-mac -DCMAKE_BUILD_TYPE=Release -DMOCK_ONLY=ON
cmake --build build-mac --parallel
```

**Run:**
```bash
./build-mac/retina-spectrum --mock --web-dir web
```

Open `http://localhost:3020` for the wizard or `http://localhost:3020/debug` for the full spectrum view.

## Running tests

Tests require a `MOCK_ONLY` build (same as above).

```bash
cmake -B build-mac -DCMAKE_BUILD_TYPE=Release -DMOCK_ONLY=ON
cmake --build build-mac --parallel
./build-mac/test_dsp
```

The test suite (`tests/test_dsp.cpp`) covers:
- Frequency axis spacing and span
- Band stitching continuity
- Noise floor level (validates `/2048` ADC normalisation)
- Tone bin placement at multiple frequency offsets
- Measured dBFS vs Blackman window theory
- SNR (tone vs noise floor)
- Peak-max decimation (not bin averaging)
- End-to-end: frequency in → correct frequency + level out
- Sidelobe suppression

## Running a public demo via Cloudflare Tunnel

This exposes the spectrum analyser at `spectrumx.retnode.com` through a Cloudflare Tunnel, without opening any firewall ports.

### Prerequisites

The node must already have `cloudflared` installed and the tunnel token configured (see owl-os README — Cloudflare Tunnel section). The tunnel token is stored at `/data/cloudflared/tunnel-token` and the service is managed by systemd.

In the Cloudflare dashboard, add a public hostname route for the tunnel:

- **Subdomain:** `spectrumx`
- **Domain:** `retnode.com`
- **Service:** `http://<pi-ip>:3020`

### Bring down radar services

The RSPduo can only be claimed by one process at a time. Stop the main radar stack first — this brings down blah2 and all associated services (blah2-web, blah2-api, blah2-host):

```bash
cd /data/mender-docker-compose/current/manifests
docker compose -p retina-node down
```

### Start the spectrum analyser

Clone or pull the repo to `/data/dev/retina-spectrum` (recommended path), then bring it up:

```bash
cd /data/dev/retina-spectrum
docker compose up --build
```

The container automatically kills any stale `sdrplay_apiService` on the host before starting.

The analyser is now accessible at `https://spectrumx.retnode.com`.

### Teardown

```bash
# In /data/dev/retina-spectrum
docker compose down

# Restore the main radar stack
cd /data/mender-docker-compose/current/manifests
docker compose -p retina-node up -d
```

### Troubleshooting: web UI not loading / no data

The SDRplay API runs a background daemon (`sdrplay_apiService`) that persists on the host even after Docker containers are stopped. If the web UI is unreachable or the spectrum shows no data, a stale daemon is likely holding the hardware.

Kill it before starting either stack:

```bash
sudo pkill -9 -f sdrplay_apiService
```

Note: `-f` is required — the process name exceeds Linux's 15-character comm limit so `-x` silently matches nothing.

## Architecture

- `src/main.cpp` — sweep thread, SSE broadcast, HTTP API (cpp-httplib)
- `src/dsp.cpp` — FFT pipeline (FFTW3 float, Blackman window, peak-max decimation)
- `src/config.h` — DSP and server constants
- `web/wizard.html` — setup wizard, SVG spectrum chart, illuminator ranking
- `web/index.html` — Chart.js debug frontend, SSE client, focus/sweep navigation
