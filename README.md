# retina-spectrum

Passive radar spectrum analyser for SDRplay RSPduo. Continuously sweeps FM (88–108 MHz), VHF (174–216 MHz), and UHF (468–693 MHz) and streams results to a browser via SSE.

## Running on hardware (Raspberry Pi)

Requires the SDRplay API service running on the host and the `.so` bind-mounted into the container (see `docker-compose.yml`).

```bash
docker compose up --build
```

Open `http://<pi-ip>:3020`.

## Running on Mac (mock mode)

No SDR hardware needed. Builds without the SDRplay API dependency and generates synthetic IQ data.

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

Open `http://localhost:3020`.

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

## Architecture

- `src/main.cpp` — sweep thread, SSE broadcast, HTTP API (cpp-httplib)
- `src/dsp.cpp` — FFT pipeline (FFTW3 float, Blackman window, peak-max decimation)
- `src/config.h` — DSP and server constants
- `web/index.html` — Chart.js frontend, SSE client, focus/sweep navigation
