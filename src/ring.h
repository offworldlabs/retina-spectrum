#pragma once

#include "config.h"

#include <algorithm>
#include <cmath>
#include <vector>

// Linear-domain ring buffer for spectrum averaging.
//
// Stores up to METRICS_AVG_STEPS linear power spectra.  Running sum maintained
// in O(1) per push via subtract-on-eviction.  get_db() converts the arithmetic
// mean to dB once — correct domain for power-based OBW and SNR measurements.
//
// Usage:
//   ring.push(get_raw_linear(), N_FFT);   // each DSP step
//   ring.get_db(averaged_db);             // → pass to metrics + display
//   if (ring.ready()) { /* export */ }    // gate on min sample count

struct SpectrumRing {
    std::vector<std::vector<float>> buf;  // [METRICS_AVG_STEPS][n] — circular
    std::vector<float>              sum;  // running linear sum across all entries
    int head  = 0;
    int count = 0;

    void push(const float* lin, int n) {
        if (buf.empty()) {
            buf.assign(METRICS_AVG_STEPS, std::vector<float>(n, 0.0f));
            sum.assign(n, 0.0f);
        }
        // Evict oldest slot from running sum before overwriting
        if (count == METRICS_AVG_STEPS)
            for (int k = 0; k < n; k++) sum[k] -= buf[head][k];

        buf[head].assign(lin, lin + n);
        for (int k = 0; k < n; k++) sum[k] += lin[k];

        head  = (head + 1) % METRICS_AVG_STEPS;
        count = std::min(count + 1, METRICS_AVG_STEPS);
    }

    // Convert running arithmetic mean (linear) to dB.
    // Returns empty vector if push() has never been called.
    void get_db(std::vector<float>& out) const {
        out.resize(sum.size());
        const float n = (float)std::max(count, 1);
        for (int k = 0; k < (int)sum.size(); k++) {
            const float p = sum[k] / n;
            out[k] = (p > 0.0f) ? 10.0f * std::log10(p) : -120.0f;
        }
    }

    // True once METRICS_MIN_ENTRIES captures have been pushed —
    // before this, the average is based on fewer samples than intended.
    bool ready() const { return count >= METRICS_MIN_ENTRIES; }
};
