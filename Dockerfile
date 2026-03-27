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

# SDRplay API headers needed at compile time — .so bind-mounted from host at runtime
COPY lib/ /usr/local/include/

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
