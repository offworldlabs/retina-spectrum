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

# SDRplay API — same install pattern as blah2-arm Dockerfile
# Headers → /usr/local/include, arm64 .so → /usr/local/lib (matches blah2)
COPY lib/*.h /usr/local/include/
COPY lib/arm64/libsdrplay_api.so.3.15 /usr/local/lib/libsdrplay_api.so.3.15
RUN chmod 644 /usr/local/lib/libsdrplay_api.so.3.15 && \
    ln -s libsdrplay_api.so.3.15 /usr/local/lib/libsdrplay_api.so.3 && \
    ln -s libsdrplay_api.so.3.15 /usr/local/lib/libsdrplay_api.so && \
    ldconfig

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
