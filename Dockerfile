# ── Stage 1: build ────────────────────────────────────────────────────────────
FROM debian:bookworm AS build

RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    make \
    g++ \
    git \
    libfftw3-dev \
    pkg-config \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# sdrplay_api.h needed at compile time — bind-mounted .so at runtime
# The header ships with the SDRplay API installer; copy from host build context
COPY lib/sdrplay_api.h /usr/local/include/sdrplay_api.h

WORKDIR /app
COPY CMakeLists.txt .
COPY src/ src/
COPY web/ web/

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel 4

# ── Stage 2: runtime ──────────────────────────────────────────────────────────
FROM debian:bookworm-slim AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    libfftw3-single3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /app/build/retina-spectrum /usr/local/bin/retina-spectrum
COPY --from=build /app/web/                  /web/

# libsdrplay_api.so.3.15 is bind-mounted from the host at runtime
# (same pattern as blah2 — the .so is installed by SDRplay API package on host)

EXPOSE 3020
CMD ["retina-spectrum"]
