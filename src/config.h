#pragma once

// DSP
#define N_FFT           8192    // samples captured and FFT'd per step (~977 Hz/bin at 8 MHz fs)
#define N_DISPLAY       64      // display bins per step (125 kHz/bin)
// No averaging — broadcast towers are 20-40 dB above noise floor, single FFT is sufficient

// SDR
#define SAMPLE_RATE_HZ      8000000
#define AGC_SETPOINT        -30         // dBfs AGC target
#define RESET_TIMEOUT_MS    200         // max ms to wait for reset flag after retune

// Server
#define HTTP_PORT   3020
