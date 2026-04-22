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
# Binaries sourced from blah2-arm/lib/sdrplay-3.15.2/arm64/ (same version, same repo)
COPY lib/*.h /usr/local/include/
COPY lib/arm64/libsdrplay_api.so.3.15 /usr/local/lib/libsdrplay_api.so.3.15
COPY lib/arm64/sdrplay_apiService /usr/local/bin/sdrplay_apiService
RUN chmod 644 /usr/local/lib/libsdrplay_api.so.3.15 && \
    ln -s libsdrplay_api.so.3.15 /usr/local/lib/libsdrplay_api.so.3 && \
    ln -s libsdrplay_api.so.3.15 /usr/local/lib/libsdrplay_api.so && \
    chmod +x /usr/local/bin/sdrplay_apiService && \
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

COPY --from=build /app/build/retina-spectrum        /usr/local/bin/retina-spectrum
COPY --from=build /app/web/                         /web/
COPY --from=build /usr/local/lib/libsdrplay_api.so.3.15  /usr/local/lib/libsdrplay_api.so.3.15
COPY --from=build /usr/local/lib/libsdrplay_api.so.3     /usr/local/lib/libsdrplay_api.so.3
COPY --from=build /usr/local/bin/sdrplay_apiService      /usr/local/bin/sdrplay_apiService
RUN ldconfig

EXPOSE 3020
CMD ["retina-spectrum"]
