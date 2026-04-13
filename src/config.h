#pragma once

// DSP
#define N_FFT           8192    // samples per FFT (~977 Hz/bin at 8 MHz fs)
#define N_DISPLAY       64      // sweep mode display bins (125 kHz/bin, GROUP=128)
#define N_DISPLAY_FOCUS 1024    // focus mode display bins (7.8 kHz/bin, GROUP=8)
#define N_AVG           8       // FFT averages per step — reduces noise variance ~9 dB
#define EMA_ALPHA       0.15f   // sweep-to-sweep EMA — ~7 sweeps to converge (~4 min at 35s/sweep)
#define FOCUS_EMA_ALPHA 0.3f    // focus-mode EMA — converges in ~3 captures (~1.5 s at 500 ms pause)

// SDR
#define SAMPLE_RATE_HZ      8000000
#define AGC_SETPOINT        -30         // dBfs AGC target
#define RESET_TIMEOUT_MS    500         // max ms to wait for reset flag after retune
#define FOCUS_PAUSE_MS      500         // ms to idle between focus scans (checked every 10 ms)

// Channel peak detection
#define NUM_CHANNEL_PEAKS   3       // top local-maximum peaks to find per channel
#define PILOT_TOL_MHZ       0.05f   // ±50 kHz tolerance for ATSC pilot frequency match
#define PEAK_MIN_DBF        -75.0f  // minimum dBFS to flag a peak is_pilot=true
                                    // mock noise floor ≈ -87 dBFS; real towers well above -75

// FM channel metrics
#define FM_MIN_REPORT_SNR   3.0f    // suppress FM slots below this SNR (dB) from SSE output
                                    // avoids emitting ~100 empty-slot entries per step

// Server
#define HTTP_PORT   3020
