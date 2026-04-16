#pragma once

// DSP
#define N_FFT           8192    // samples per FFT (~977 Hz/bin at 8 MHz fs)
#define N_DISPLAY       64      // sweep mode display bins (125 kHz/bin, GROUP=128)
#define N_DISPLAY_FOCUS 1024    // focus mode display bins (7.8 kHz/bin, GROUP=8)
#define N_AVG           8       // FFT averages per step — reduces noise variance ~9 dB
#define METRICS_AVG_STEPS    20 // ring depth for spectrum averaging — both scan and focus modes
                                // sweep: 20 passes × 35s ≈ 12 min full ring (5 entries ≈ 3 min)
                                // focus: 20 × 500ms = 10s full ring  (5 entries = 2.5s)
#define METRICS_MIN_ENTRIES   5 // min pushes before ring.ready() — gates API export

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

// ── FM OBW measurement ────────────────────────────────────────────────────────
// β=0.10 → 90% power containment (5% trimmed from each edge).
// More robust than ITU-R 99% (β=0.01): edge bins must accumulate 5% of total
// power before lo_cut/hi_cut moves inward — a single leakage bin (0.5%) cannot
// inflate OBW as it can at β=0.01.
#define FM_OBW_BETA         0.10f   // 0.01 = 99% ITU-R, 0.10 = 90% (default)

// ── Gate thresholds (coupled to FM_OBW_BETA) ─────────────────────────────────
// FM_MOB_GATE_FRAC is calibrated against FM_OBW_BETA.
// At β=0.10: frac=0.42 ≈ 100 kHz actual BW ≈ 1500 m range resolution
// At β=0.01: frac=0.50 ≈ 100 kHz actual BW ≈ 1500 m range resolution
// If FM_OBW_BETA changes, re-calibrate FM_MOB_GATE_FRAC on hardware.
#define FM_MOB_GATE_FRAC    0.42f  // minimum OBW fraction to pass bandwidth gate
#define FM_SNR_GATE_DB      15.0f  // minimum SNR (dB) to pass signal strength gate
#define FM_SFM_GATE          0.2f // minimum spectral flatness (Wiener entropy) to pass
                                   // rejects CW spikes (SFM≈0) and extreme bimodal cases
                                   // real FM broadcast: 0.2–0.8 depending on programme content
#define FM_CENTRE_POWER_GATE 0.25f // minimum fraction of OBW power in middle third of OBW
                                   // real FM: ~0.33 (uniform) to ~0.50+ (carrier-heavy)
                                   // trough (null in centre): 0.02–0.10 → rejected
                                   // slope (edge-heavy): 0.10–0.25 → rejected

// ── SNR normalisation (rank ordering within passing set) ─────────────────────
#define FM_SNR_NORM_MIN      5.0f   // SNR below this → rank score = 0
#define FM_SNR_NORM_MAX     50.0f   // SNR above this → rank score = 1 (capped)

// ── Scoring algorithm ────────────────────────────────────────────────────────
// All algorithms share gates: SNR >= FM_SNR_GATE_DB, OBW >= FM_MOB_GATE_FRAC,
// SFM >= FM_SFM_GATE. Change FM_SCORE_ALGO + rebuild (~10s) to switch scoring.
#define FM_SCORE_ALGO_GATE        0  // gates: SNR+OBW+SFM          score = snr_norm
#define FM_SCORE_ALGO_SNR_OBW     1  // gates: SNR+OBW+SFM          score = snr_norm × obw_fraction
#define FM_SCORE_ALGO_SNR_OBW_SFM 2  // gates: SNR+OBW+SFM          score = snr_norm × obw_fraction × sfm
#define FM_SCORE_ALGO_SNR_OBW_CPF 3  // gates: SNR+OBW+SFM+CPF      score = snr_norm × obw_fraction × centre_power_frac
#define FM_SCORE_ALGO_GATE_CPF    4  // gates: SNR+OBW+SFM+CPF      score = snr_norm
#define FM_SCORE_ALGO  FM_SCORE_ALGO_GATE

// Server
#define HTTP_PORT   3020
