// retina-spectrum — FM/VHF/UHF ATSC sweep binary
// Sweep bands, serve spectrum as JSON + SSE, display via Chart.js

#include "channels.h"
#include "config.h"
#include "dsp.h"
#include "sdr.h"

#include <httplib.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// ── Channel-based sweep steps ─────────────────────────────────────────────────
//
// FM:  8 × 3 MHz steps, each covering ~15 FM channels within ±1.5 MHz trim window.
//      Pilot detection: find peak in each 200 kHz channel window (no fixed offset).
// TV:  one step per channel centre. ATSC pilot at lower_edge + 0.31 MHz (= fc − 2.69 MHz).
//      Total: 8 FM + 6 VHF-lo + 7 VHF-hi + 38 UHF = 59 steps.

struct Step {
    float fc_mhz;
    const char *band;
    std::vector<const Channel*> channels;  // channels to analyse in this step
};

// Band table — same 3 MHz integer grid as main branch.
// Each step serves the centre 24/64 bins (±1.5 MHz flat region of Zero-IF filter).
// Steps tile gaplessly; ATSC pilots (lower_edge+0.31 MHz) land 0.31–0.69 MHz
// from a step centre → display bin 32±2–6, well inside TRIM_LO=20..TRIM_HI=44.
struct Band { const char *name; int start_mhz; int stop_mhz; int step_mhz; };
static const Band BANDS[] = {
    {"fm",      88, 108, 3},  // FM 88.1–107.9
    {"vhf_hi", 174, 216, 3},  // VHF ch7–13
    {"uhf",    468, 698, 3},  // UHF ch14–51
};

static std::vector<Step> build_steps()
{
    std::vector<Step> steps;
    for (auto& b : BANDS) {
        for (int fc_i = b.start_mhz; fc_i <= b.stop_mhz; fc_i += b.step_mhz) {
            float fc = (float)fc_i;
            Step s; s.fc_mhz = fc; s.band = b.name;

            // FM: associate channels whose centre falls within ±1.5 MHz
            for (int j = 0; j < N_FM_CHANNELS; j++)
                if (fabsf(FM_CHANNELS[j].fc_mhz - fc) < 1.5f)
                    s.channels.push_back(&FM_CHANNELS[j]);
            // TV: associate channels whose pilot frequency falls within ±1.5 MHz
            for (int j = 0; j < N_VHF_HI_CHANNELS; j++)
                if (fabsf(VHF_HI_CHANNELS[j].pilot_mhz - fc) < 1.5f)
                    s.channels.push_back(&VHF_HI_CHANNELS[j]);
            for (int j = 0; j < N_UHF_CHANNELS; j++)
                if (fabsf(UHF_CHANNELS[j].pilot_mhz - fc) < 1.5f)
                    s.channels.push_back(&UHF_CHANNELS[j]);

            steps.push_back(std::move(s));
        }
    }
    std::cerr << "[sweep] " << steps.size() << " steps total" << std::endl;
    return steps;
}

// ── Channel lookup helpers ────────────────────────────────────────────────────

// Snap a frequency to the nearest channel centre across all band tables.
static float snap_to_channel(float fc_mhz)
{
    float best_dist = 1e9f, best_fc = fc_mhz;
    auto check = [&](const Channel* tbl, int n) {
        for (int i = 0; i < n; i++) {
            float d = fabsf(tbl[i].fc_mhz - fc_mhz);
            if (d < best_dist) { best_dist = d; best_fc = tbl[i].fc_mhz; }
        }
    };
    check(FM_CHANNELS,     N_FM_CHANNELS);
    check(VHF_LO_CHANNELS, N_VHF_LO_CHANNELS);
    check(VHF_HI_CHANNELS, N_VHF_HI_CHANNELS);
    check(UHF_CHANNELS,    N_UHF_CHANNELS);
    return best_fc;
}

// Find the Channel entry whose centre is within 0.15 MHz of fc_mhz.
static const Channel* find_channel(float fc_mhz)
{
    const Channel* best = nullptr;
    float best_dist = 0.15f;
    auto check = [&](const Channel* tbl, int n) {
        for (int i = 0; i < n; i++) {
            float d = fabsf(tbl[i].fc_mhz - fc_mhz);
            if (d < best_dist) { best_dist = d; best = &tbl[i]; }
        }
    };
    check(FM_CHANNELS,     N_FM_CHANNELS);
    check(VHF_LO_CHANNELS, N_VHF_LO_CHANNELS);
    check(VHF_HI_CHANNELS, N_VHF_HI_CHANNELS);
    check(UHF_CHANNELS,    N_UHF_CHANNELS);
    return best;
}

// ── Ring buffer ───────────────────────────────────────────────────────────────

struct ChannelResult {
    const Channel*           ch;
    std::vector<ChannelPeak> peaks;
};

struct Slice {
    float fc_mhz         = 0;
    float freq_start_mhz = 0;
    float freq_stop_mhz  = 0;
    std::vector<float>         power_db;        // raw (current sweep)
    std::vector<float>         smooth_db;       // EMA-smoothed
    std::vector<ChannelResult> channel_results; // per-channel peak detection
    bool  valid          = false;
};

// ── Per-client SSE queue ──────────────────────────────────────────────────────
// Sweep thread pushes messages; HTTP thread blocks waiting on pop().
// Using shared_ptr + weak_ptr so dead connections are cleaned up automatically.

struct SseClient {
    std::mutex              mtx;
    std::condition_variable cv;
    std::queue<std::string> messages;
    bool                    done = false;

    void push(const std::string& msg)
    {
        { std::lock_guard<std::mutex> lk(mtx); messages.push(msg); }
        cv.notify_one();
    }

    void close()
    {
        { std::lock_guard<std::mutex> lk(mtx); done = true; }
        cv.notify_one();
    }

    // Returns next message, or "" on timeout (keep-alive heartbeat) or done
    std::string pop(int timeout_ms)
    {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
            [this]{ return !messages.empty() || done; });
        if (messages.empty()) return "";
        auto msg = std::move(messages.front());
        messages.pop();
        return msg;
    }
};

// ── Sweep state ───────────────────────────────────────────────────────────────

struct SweepState {
    std::mutex  mtx;
    std::string state        = "idle";  // idle | sweeping | complete
    int         progress_pct = 0;
    std::vector<Slice> buffer;
    std::vector<std::weak_ptr<SseClient>> sse_clients;
};

static SweepState        g_state;
static std::thread       g_sweep_thread;
static std::atomic<bool>  g_mock{false};
static std::atomic<float> g_focus_mhz{0.0f};  // 0 = sweep mode; >0 = focus on one fc
static std::string        g_web_dir = "/web";

// ── SSE helpers ───────────────────────────────────────────────────────────────

// Only output the centre 24 bins (d=20..43) of the 64-bin FFT.
// These bins fall within ±1.5 MHz of the step centre — the genuinely flat region
// of the Zero-IF 8MS/s filter (RSP manual p.21 freq step = 3 MHz).
// With 3 MHz steps the ranges tile gaplessly without overlap.
static constexpr int   TRIM_LO       = 20;
static constexpr int   TRIM_HI       = 44;    // exclusive
static constexpr int   TRIM_N        = TRIM_HI - TRIM_LO;           // = 24
static constexpr float TRIM_HALF_MHZ = (TRIM_N / 2) * (8.0f / N_DISPLAY);  // = 1.5

static void append_bins(std::ostringstream& ss,
                         const std::vector<float>& arr,
                         int lo, int hi)
{
    ss << '[';
    for (int d = lo; d < hi; d++) {
        if (d > lo) ss << ',';
        ss << arr[d];
    }
    ss << ']';
}

// full_bins=true → serve all display bins (focus mode, ±4 MHz)
// full_bins=false → serve centre 24 bins ±1.5 MHz (sweep mode)
static std::string slice_to_sse(int step, const Slice& sl, int pct, bool full_bins = false)
{
    const int lo = full_bins ? 0    : TRIM_LO;
    const int hi = full_bins ? (int)sl.power_db.size() : TRIM_HI;
    std::ostringstream ss;
    ss << "data: {\"type\":\"step\""
       << ",\"step\":"         << step
       << ",\"fc_mhz\":"       << sl.fc_mhz
       << ",\"freq_start\":"   << sl.freq_start_mhz
       << ",\"freq_stop\":"    << sl.freq_stop_mhz
       << ",\"progress_pct\":" << pct
       << ",\"power_db\":";   append_bins(ss, sl.power_db,  lo, hi);
    ss << ",\"smooth_db\":";  append_bins(ss, sl.smooth_db, lo, hi);

    // Per-channel pilot detection results
    ss << ",\"channels\":[";
    for (int ci = 0; ci < (int)sl.channel_results.size(); ci++) {
        const auto& cr = sl.channel_results[ci];
        if (ci > 0) ss << ',';
        ss << "{\"band\":\"" << cr.ch->band << "\""
           << ",\"number\":"     << cr.ch->number
           << ",\"fc_mhz\":"     << cr.ch->fc_mhz
           << ",\"pilot_mhz\":"  << cr.ch->pilot_mhz
           << ",\"peaks\":[";
        for (int pi = 0; pi < (int)cr.peaks.size(); pi++) {
            const auto& pk = cr.peaks[pi];
            if (pi > 0) ss << ',';
            ss << "{\"freq_mhz\":" << pk.freq_mhz
               << ",\"power_db\":" << pk.power_db
               << ",\"is_pilot\":"  << (pk.is_pilot ? "true" : "false")
               << '}';
        }
        ss << "]}";
    }
    ss << "]}\n\n";
    return ss.str();
}

static void sse_broadcast(const std::string& msg)
{
    std::lock_guard<std::mutex> lk(g_state.mtx);
    // remove expired clients while we iterate
    auto& clients = g_state.sse_clients;
    clients.erase(
        std::remove_if(clients.begin(), clients.end(),
            [](const std::weak_ptr<SseClient>& wp){ return wp.expired(); }),
        clients.end());
    for (auto& wp : clients)
        if (auto sp = wp.lock()) sp->push(msg);
}

static void sse_close_all()
{
    std::lock_guard<std::mutex> lk(g_state.mtx);
    for (auto& wp : g_state.sse_clients)
        if (auto sp = wp.lock()) sp->close();
    g_state.sse_clients.clear();
}

// ── Mock IQ generator ─────────────────────────────────────────────────────────
// Generates N_FFT*N_AVG complex samples at ADC scale (±2048 full-scale).
// Feeds through process_step() so all DSP (Blackman window, /2048, peak-max)
// is exercised identically to hardware.

struct MockStation { float freq_mhz; float amplitude; };

static const MockStation MOCK_STATIONS[] = {
    {  89.1f,   300.0f },  // FM weak         (ch 89.1 MHz)
    {  95.9f,   150.0f },  // FM medium        (ch 95.9 MHz)
    {  98.7f,   800.0f },  // FM strong        (ch 98.7 MHz, primary test peak)
    { 103.5f,   100.0f },  // FM weak          (ch 103.5 MHz)
    { 198.31f,  500.0f },  // VHF ch11 ATSC pilot (lower 198 + 0.31)
    { 210.31f,  400.0f },  // VHF ch13 ATSC pilot (lower 210 + 0.31)
    { 530.31f,  600.0f },  // UHF ch24 ATSC pilot (lower 530 + 0.31, center 533)
    { 614.31f,  450.0f },  // UHF ch38 ATSC pilot (lower 614 + 0.31, center 617)
};

static std::vector<std::complex<float>> mock_iq(float fc_mhz)
{
    constexpr int   N  = N_FFT * N_AVG;
    constexpr float fs = (float)SAMPLE_RATE_HZ;
    std::vector<std::complex<float>> buf(N);

    // Gaussian noise via Box-Muller — noise_std 30 ADC counts → ~-87 dBFS floor
    constexpr float noise_std = 30.0f;
    for (int n = 0; n < N; n++) {
        float u1 = (rand() + 1.0f) / ((float)RAND_MAX + 2.0f);  // avoid log(0)
        float u2 = (float)rand() / (float)RAND_MAX;
        float r  = sqrtf(-2.0f * logf(u1)) * noise_std;
        buf[n] = { r * cosf(2.0f * (float)M_PI * u2),
                   r * sinf(2.0f * (float)M_PI * u2) };
    }

    // Complex tone for each station within ±4 MHz of fc
    for (const auto& s : MOCK_STATIONS) {
        float offset_hz = (s.freq_mhz - fc_mhz) * 1.0e6f;
        if (fabsf(offset_hz) > 4.0e6f) continue;
        float phase     = 0.0f;
        float phase_inc = 2.0f * (float)M_PI * offset_hz / fs;
        for (int n = 0; n < N; n++) {
            buf[n] = { buf[n].real() + s.amplitude * cosf(phase),
                       buf[n].imag() + s.amplitude * sinf(phase) };
            phase += phase_inc;
            if (phase >  (float)M_PI) phase -= 2.0f * (float)M_PI;
            if (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
        }
    }

    return buf;
}

// ── Sweep thread ──────────────────────────────────────────────────────────────

static void sweep_fn()
{
    // Sweep EMA — persists across focus excursions so the averaged line doesn't
    // have to rebuild from scratch every time the user returns from focus mode.
    std::vector<std::vector<float>> sweep_ema;
    bool  sweep_ema_valid = false;
    int   sweep_ema_total = 0;

    // Focus EMA — separate buffer, reset when the focus fc changes.
    std::vector<std::vector<float>> focus_ema;
    bool  focus_ema_valid = false;
    float focus_ema_fc    = -1.0f;

    // Full-resolution (N_FFT) EMA for peak detection — one buffer per sweep step.
    std::vector<std::vector<float>> sweep_raw_ema;
    bool  sweep_raw_ema_valid = false;
    std::vector<float> focus_raw_ema(N_FFT, 0.0f);

    float last_tuned_mhz = 0.0f;  // track last hardware tune — avoid same-freq rfChanged issue

    while (true)  // continuous sweep loop
    {
        float focus    = g_focus_mhz.load();
        bool  is_focus = (focus > 0.0f);

        std::vector<Step> steps;
        if (is_focus)
            steps = {{focus, "focus"}};
        else
            steps = build_steps();

        int total = (int)steps.size();

        // Reset focus EMA when the focus frequency changes
        if (is_focus && focus != focus_ema_fc) {
            focus_ema.assign(1, std::vector<float>(N_DISPLAY_FOCUS, 0.0f));
            std::fill(focus_raw_ema.begin(), focus_raw_ema.end(), 0.0f);
            focus_ema_valid = false;
            focus_ema_fc    = focus;
        }
        // Reset sweep EMA if step count changed (e.g. band config change)
        if (!is_focus && (int)sweep_ema.size() != total) {
            sweep_ema.assign(total, std::vector<float>(N_DISPLAY, 0.0f));
            sweep_ema_valid = false;
            sweep_ema_total = total;
        }
        if (!is_focus && (int)sweep_raw_ema.size() != total) {
            sweep_raw_ema.assign(total, std::vector<float>(N_FFT, 0.0f));
            sweep_raw_ema_valid = false;
        }

        {
            std::lock_guard<std::mutex> lk(g_state.mtx);
            g_state.state        = "sweeping";
            g_state.progress_pct = 0;
            g_state.buffer.assign(total, Slice{});
        }

        // start event carries mode so the frontend can clear + re-axis on switch
        {
            std::ostringstream start;
            start << "data: {\"type\":\"start\",\"mode\":\""
                  << (is_focus ? "focus" : "sweep") << "\"";
            if (is_focus) start << ",\"fc\":" << focus;
            start << "}\n\n";
            sse_broadcast(start.str());
        }

        for (int i = 0; i < total; i++)
        {
            // Restart outer loop immediately if focus mode changed mid-sweep
            if (g_focus_mhz.load() != focus) break;

            float       fc_mhz = steps[i].fc_mhz;
            const char *band   = steps[i].band;
            std::cerr << "[sweep] step " << (i+1) << "/" << total
                      << "  fc=" << fc_mhz << " MHz  band=" << band << std::endl;

            std::vector<float> power;

            if (g_mock)
            {
                if (is_focus) {
                    auto arr = process_step_focus(fc_mhz, mock_iq(fc_mhz));
                    power.assign(arr.begin(), arr.end());
                } else {
                    auto arr = process_step(fc_mhz, mock_iq(fc_mhz));
                    power.assign(arr.begin(), arr.end());
                }
            }
            else
            {
                if (fc_mhz != last_tuned_mhz) {
                    // New frequency — use rfChanged mechanism to flush LO settling
                    retune((double)fc_mhz * 1e6);
                    last_tuned_mhz = fc_mhz;
                } else {
                    // Same frequency (focus mode looping) — rfChanged won't fire
                    // reliably on a no-op tune. Just restart capture directly.
                    // Order matters: clear while callbacks suppressed, release last.
                    g_capture_buf.clear();
                    g_capture_done      = false;
                    g_waiting_rf_change = false;  // release callback suppression
                }

                auto deadline = std::chrono::steady_clock::now()
                              + std::chrono::milliseconds(RESET_TIMEOUT_MS);
                while (!g_capture_done)
                {
                    if (std::chrono::steady_clock::now() > deadline)
                    {
                        std::cerr << "[sweep] WARN: timeout at " << fc_mhz << " MHz — skipping" << std::endl;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                if (!g_capture_done)
                    continue;

                std::cerr << "[sweep] captured " << g_capture_buf.size()
                          << " samples at " << fc_mhz << " MHz" << std::endl;

                if (is_focus) {
                    auto arr = process_step_focus(fc_mhz, g_capture_buf);
                    power.assign(arr.begin(), arr.end());
                } else {
                    auto arr = process_step(fc_mhz, g_capture_buf);
                    power.assign(arr.begin(), arr.end());
                }
            }

            // EMA blend — smooth_db tracks weighted history, power_db is raw
            auto& cur_ema       = is_focus ? focus_ema[0]  : sweep_ema[i];
            bool& cur_ema_valid = is_focus ? focus_ema_valid : sweep_ema_valid;
            if (cur_ema_valid) {
                const float alpha = is_focus ? FOCUS_EMA_ALPHA : EMA_ALPHA;
                for (int d = 0; d < (int)power.size(); d++)
                    cur_ema[d] = alpha * power[d] + (1.0f - alpha) * cur_ema[d];
            } else {
                cur_ema = power;  // first pass: seed with raw
            }

            // ── Per-channel peak detection on EMA-smoothed FFT bins ──────
            const float* raw_db = get_raw_db();
            auto& cur_raw_ema = is_focus ? focus_raw_ema : sweep_raw_ema[i];
            bool  raw_ema_seeded = is_focus ? focus_ema_valid : sweep_raw_ema_valid;
            if (raw_ema_seeded) {
                const float alpha = is_focus ? FOCUS_EMA_ALPHA : EMA_ALPHA;
                for (int k = 0; k < N_FFT; k++)
                    cur_raw_ema[k] = alpha * raw_db[k] + (1.0f - alpha) * cur_raw_ema[k];
            } else {
                std::copy(raw_db, raw_db + N_FFT, cur_raw_ema.begin());
            }
            std::vector<ChannelResult> ch_results;
            // In focus mode, analyse the single focused channel; in sweep mode,
            // analyse all channels associated with this step.
            std::vector<const Channel*> step_channels;
            if (is_focus) {
                const Channel* ch = find_channel(fc_mhz);
                if (ch) step_channels.push_back(ch);
            } else {
                step_channels = steps[i].channels;
            }
            for (const Channel* ch : step_channels) {
                float lo_mhz = ch->fc_mhz - ch->bw_mhz * 0.5f;
                float hi_mhz = ch->fc_mhz + ch->bw_mhz * 0.5f;
                // FM: pilot_mhz=0 in table — use channel centre as detection target
                float pilot = (ch->pilot_mhz > 0.0f) ? ch->pilot_mhz : ch->fc_mhz;
                auto peaks = find_channel_peaks(cur_raw_ema.data(), N_FFT, fc_mhz,
                                                lo_mhz, hi_mhz,
                                                pilot, ch->tol_mhz, NUM_CHANNEL_PEAKS);
                ch_results.push_back({ch, std::move(peaks)});
            }

            // ── Frequency window for this slice ───────────────────────────
            float freq_start, freq_stop;
            if (is_focus) {
                // Always use the actual ±4 MHz capture window so the frontend
                // maps bins to frequencies correctly (binWidth = 8/N_DISPLAY_FOCUS).
                // The x-axis is set separately by setAxisFocusChannel() on the client.
                freq_start = fc_mhz - 4.0f;
                freq_stop  = fc_mhz + 4.0f;
            } else {
                freq_start = fc_mhz - TRIM_HALF_MHZ;
                freq_stop  = fc_mhz + TRIM_HALF_MHZ;
            }

            Slice sl;
            sl.fc_mhz         = fc_mhz;
            sl.freq_start_mhz = freq_start;
            sl.freq_stop_mhz  = freq_stop;
            sl.power_db       = power;
            sl.smooth_db      = cur_ema;
            sl.channel_results = std::move(ch_results);
            sl.valid          = true;

            int pct = (i + 1) * 100 / total;
            {
                std::lock_guard<std::mutex> lk(g_state.mtx);
                g_state.buffer[i]    = sl;
                g_state.progress_pct = pct;
            }

            sse_broadcast(slice_to_sse(i, sl, pct, is_focus));

            // Pause between focus scans — makes the display readable.
            // Check every 10 ms so a mode change exits promptly.
            if (is_focus) {
                for (int ms = 0; ms < FOCUS_PAUSE_MS; ms += 10) {
                    if (g_focus_mhz.load() != focus) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        }

        // Mark EMA as having history after the first complete pass
        sweep_ema_valid     = true;
        sweep_raw_ema_valid = true;
        focus_ema_valid     = true;

        {
            std::lock_guard<std::mutex> lk(g_state.mtx);
            g_state.state = "complete";
        }
        std::cerr << "[sweep] complete" << std::endl;
        sse_broadcast("data: {\"type\":\"complete\"}\n\n");
        // SSE clients stay connected — next iteration sends "start" then new steps
    }
}

static bool start_sweep()
{
    {
        std::lock_guard<std::mutex> lk(g_state.mtx);
        if (g_state.state == "sweeping") {
            std::cerr << "[sweep] already sweeping — ignoring start request" << std::endl;
            return false;
        }
    }
    if (g_sweep_thread.joinable()) g_sweep_thread.join();
    g_sweep_thread = std::thread(sweep_fn);
    return true;
}

// ── JSON helpers ──────────────────────────────────────────────────────────────

static std::string build_sweep_json()
{
    std::lock_guard<std::mutex> lk(g_state.mtx);
    bool is_focus = (g_focus_mhz.load() > 0.0f);
    int  lo = is_focus ? 0 : TRIM_LO;
    std::ostringstream freq_arr, power_arr, smooth_arr;
    bool first = true;
    for (auto& sl : g_state.buffer) {
        if (!sl.valid) continue;
        int   hi       = (int)sl.power_db.size();
        float bin_width = 8.0f / hi;  // MHz per display bin
        float f_start   = sl.fc_mhz - 4.0f;
        for (int d = lo; d < hi; d++) {
            if (!first) { freq_arr << ','; power_arr << ','; smooth_arr << ','; }
            freq_arr   << (f_start + (d + 0.5f) * bin_width);
            power_arr  << sl.power_db[d];
            smooth_arr << sl.smooth_db[d];
            first = false;
        }
    }
    std::ostringstream ss;
    ss << "{\"state\":\"" << g_state.state << "\""
       << ",\"data\":{\"frequency_mhz\":[" << freq_arr.str()  << "]"
       << ",\"power_db\":["                 << power_arr.str() << "]"
       << ",\"smooth_db\":["                << smooth_arr.str() << "]}}";
    return ss.str();
}

static std::string read_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f) return "";
    return std::string(std::istreambuf_iterator<char>(f), {});
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--mock") {
            g_mock = true;
        } else if (arg == "--tuner" && i + 1 < argc) {
            std::string t = argv[++i];
            if (t == "B" || t == "b") {
                g_tuner = sdrplay_api_Tuner_B;
                std::cerr << "[sdr] tuner override: B (SMA2)" << std::endl;
            } else if (t == "A" || t == "a") {
                g_tuner = sdrplay_api_Tuner_A;
                std::cerr << "[sdr] tuner override: A (SMA1, reference)" << std::endl;
            } else {
                std::cerr << "Error: --tuner must be A or B" << std::endl;
                return 1;
            }
        } else if (arg == "--web-dir" && i + 1 < argc) {
            g_web_dir = argv[++i];
        } else {
            std::cerr << "Usage: retina-spectrum [--mock] [--tuner A|B] [--web-dir path]" << std::endl;
            return 1;
        }
    }

    std::cerr << "[retina-spectrum] mock="   << (g_mock ? "yes" : "no")
              << "  tuner="  << (g_tuner == sdrplay_api_Tuner_A ? "A" : "B")
              << "  web-dir=" << g_web_dir
              << "  port="   << HTTP_PORT << std::endl;

    if (!g_mock) {
        open_api();
        get_device();
        set_device_parameters(88e6);  // initial frequency — first FM step
        initialise_device();
    }

    // Auto-start sweep on launch
    start_sweep();

    // ── HTTP server ───────────────────────────────────────────────────────────
    httplib::Server svr;

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        auto html = read_file(g_web_dir + "/index.html");
        if (html.empty()) {
            res.status = 404;
            res.set_content("index.html not found", "text/plain");
            std::cerr << "[http] WARN: index.html not found at " << g_web_dir << std::endl;
            return;
        }
        res.set_content(html, "text/html");
    });

    svr.Get("/api/status", [](const httplib::Request&, httplib::Response& res) {
        std::string state; int pct;
        {
            std::lock_guard<std::mutex> lk(g_state.mtx);
            state = g_state.state;
            pct   = g_state.progress_pct;
        }
        std::ostringstream ss;
        ss << "{\"state\":\"" << state << "\",\"progress_pct\":" << pct << "}";
        res.set_content(ss.str(), "application/json");
    });

    svr.Get("/api/sweep", [](const httplib::Request&, httplib::Response& res) {
        res.set_content(build_sweep_json(), "application/json");
    });

    svr.Post("/api/start", [](const httplib::Request&, httplib::Response& res) {
        if (start_sweep()) {
            res.set_content("{\"ok\":true}", "application/json");
        } else {
            res.status = 409;
            res.set_content("{\"error\":\"already sweeping\"}", "application/json");
        }
    });

    svr.Get("/api/focus", [](const httplib::Request& req, httplib::Response& res) {
        if (req.has_param("fc")) {
            float fc = 0.0f;
            try { fc = std::stof(req.get_param_value("fc")); }
            catch (...) {
                res.status = 400;
                res.set_content("{\"error\":\"invalid fc\"}", "application/json");
                return;
            }
            if (fc != 0.0f && (fc < 40.0f || fc > 2000.0f)) {
                res.status = 400;
                res.set_content("{\"error\":\"fc out of range\"}", "application/json");
                return;
            }
            if (fc != 0.0f) fc = snap_to_channel(fc);
            g_focus_mhz = fc;
            std::cerr << "[focus] " << (fc > 0 ? std::to_string(fc) + " MHz" : "cleared") << std::endl;
        }
        std::ostringstream ss;
        ss << "{\"focus_mhz\":" << g_focus_mhz.load() << "}";
        res.set_content(ss.str(), "application/json");
    });

    // SSE — each client gets its own queue; sweep thread pushes, this thread blocks
    svr.Get("/api/events", [](const httplib::Request&, httplib::Response& res) {
        auto client = std::make_shared<SseClient>();
        {
            std::lock_guard<std::mutex> lk(g_state.mtx);
            // If sweep already complete, send a single complete event immediately
            if (g_state.state == "complete") {
                res.set_header("Content-Type", "text/event-stream");
                res.set_header("Cache-Control", "no-cache");
                res.set_content("data: {\"type\":\"complete\"}\n\n", "text/event-stream");
                std::cerr << "[sse] client connected post-completion — sent complete immediately" << std::endl;
                return;
            }
            g_state.sse_clients.push_back(client);
        }
        std::cerr << "[sse] client connected" << std::endl;

        res.set_header("Cache-Control", "no-cache");
        res.set_chunked_content_provider("text/event-stream",
            [client](size_t, httplib::DataSink& sink) -> bool {
                auto msg = client->pop(500);  // 500ms timeout = heartbeat interval
                if (!msg.empty())
                    return sink.write(msg.c_str(), msg.size());
                if (client->done) {
                    std::cerr << "[sse] client done — closing" << std::endl;
                    return false;
                }
                // Send SSE comment as keepalive so browser doesn't time out
                std::string heartbeat = ": keepalive\n\n";
                return sink.write(heartbeat.c_str(), heartbeat.size());
            });
    });

    std::cerr << "[retina-spectrum] listening on :" << HTTP_PORT << std::endl;
    svr.listen("0.0.0.0", HTTP_PORT);

    // Cleanup
    if (g_sweep_thread.joinable()) g_sweep_thread.join();
    if (!g_mock) uninitialise_device();
    return 0;
}
