#pragma once

// DSP
#define N_FFT           8192    // samples per FFT (~977 Hz/bin at 8 MHz fs)
#define N_DISPLAY       64      // display bins per step (125 kHz/bin)
#define N_AVG           4       // FFT averages per step — reduces noise variance ~6 dB
#define EMA_ALPHA       0.3f    // sweep-to-sweep EMA weight for new data (0=frozen, 1=no smoothing)

// SDR
#define SAMPLE_RATE_HZ      8000000
#define AGC_SETPOINT        -30         // dBfs AGC target
#define RESET_TIMEOUT_MS    200         // max ms to wait for reset flag after retune

// Server
#define HTTP_PORT   3020
