// retina-spectrum — FM/DAB/UHF DVB-T sweep binary
// Sweep bands, serve spectrum as JSON + SSE, display via Chart.js

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

// ── Sweep bands ───────────────────────────────────────────────────────────────

struct Band { const char *name; int start_mhz; int stop_mhz; int step_mhz; };

// Integer MHz to avoid float comparison issues in loop bounds
// Step = 3 MHz: matches Zero-IF 8MS/s usable flat region (RSP manual p.21)
static const Band BANDS[] = {
    {"fm",   88,  108,  3},   //  7 steps: 88..108  (covers 86.5–109.5 MHz ±1.5)
    {"dab", 174,  240,  3},   // 23 steps: 174..240 (covers 172.5–241.5 MHz ±1.5)
    {"uhf", 468,  693,  3},   // 76 steps: 468..693 (covers 466.5–694.5 MHz ±1.5)
};                            // 106 steps — centre 24/64 bins served (±1.5 MHz flat region)

struct Step { float fc_mhz; const char *band; };

static std::vector<Step> build_steps()
{
    std::vector<Step> steps;
    for (auto& b : BANDS)
        for (int fc = b.start_mhz; fc <= b.stop_mhz; fc += b.step_mhz)
            steps.push_back({(float)fc, b.name});
    std::cerr << "[sweep] " << steps.size() << " steps total" << std::endl;
    return steps;
}

// ── Ring buffer ───────────────────────────────────────────────────────────────

struct Slice {
    float fc_mhz         = 0;
    float freq_start_mhz = 0;
    float freq_stop_mhz  = 0;
    std::array<float, N_DISPLAY> power_db{};   // raw (current sweep)
    std::array<float, N_DISPLAY> smooth_db{};  // EMA-smoothed
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
                         const std::array<float, N_DISPLAY>& arr,
                         int lo, int hi)
{
    ss << '[';
    for (int d = lo; d < hi; d++) {
        if (d > lo) ss << ',';
        ss << arr[d];
    }
    ss << ']';
}

// full_bins=true → serve all 64 bins ±4 MHz (focus mode)
// full_bins=false → serve centre 24 bins ±1.5 MHz (sweep mode)
static std::string slice_to_sse(int step, const Slice& sl, int pct, bool full_bins = false)
{
    const float half = full_bins ? 4.0f : TRIM_HALF_MHZ;
    const int   lo   = full_bins ? 0    : TRIM_LO;
    const int   hi   = full_bins ? N_DISPLAY : TRIM_HI;
    std::ostringstream ss;
    ss << "data: {\"type\":\"step\""
       << ",\"step\":"         << step
       << ",\"fc_mhz\":"       << sl.fc_mhz
       << ",\"freq_start\":"   << (sl.fc_mhz - half)
       << ",\"freq_stop\":"    << (sl.fc_mhz + half)
       << ",\"progress_pct\":" << pct
       << ",\"power_db\":";   append_bins(ss, sl.power_db,  lo, hi);
    ss << ",\"smooth_db\":";  append_bins(ss, sl.smooth_db, lo, hi);
    ss << "}\n\n";
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
    {  89.1f, 300.0f },   // FM weak
    {  95.8f, 150.0f },   // FM medium
    {  98.8f, 800.0f },   // FM strong (primary test peak)
    { 103.5f, 100.0f },   // FM weak
    { 202.9f, 500.0f },   // DAB multiplex
    { 218.6f, 400.0f },   // DAB multiplex
    { 530.0f, 600.0f },   // DVB-T UHF ch28
    { 610.0f, 450.0f },   // DVB-T UHF ch38
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
    std::vector<std::array<float, N_DISPLAY>> ema;
    bool  ema_valid = false;
    int   ema_total = 0;
    float ema_focus = -1.0f;  // track fc EMA was built for (focus mode reset)
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

        // Reset EMA when mode or focus frequency changes
        if (total != ema_total || focus != ema_focus) {
            ema.assign(total, {});
            ema_valid = false;
            ema_total = total;
            ema_focus = focus;
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

            std::array<float, N_DISPLAY> power;

            if (g_mock)
            {
                power = process_step(fc_mhz, mock_iq(fc_mhz));
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
                    g_capture_buf.clear();
                    g_capture_done = false;
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

                power = process_step(fc_mhz, g_capture_buf);
            }

            // EMA blend — smooth_db tracks weighted history, power_db is raw
            if (ema_valid)
                for (int d = 0; d < N_DISPLAY; d++)
                    ema[i][d] = EMA_ALPHA * power[d] + (1.0f - EMA_ALPHA) * ema[i][d];
            else
                ema[i] = power;  // first sweep: seed with raw

            Slice sl;
            sl.fc_mhz         = fc_mhz;
            sl.freq_start_mhz = fc_mhz - (is_focus ? 4.0f : TRIM_HALF_MHZ);
            sl.freq_stop_mhz  = fc_mhz + (is_focus ? 4.0f : TRIM_HALF_MHZ);
            sl.power_db       = power;
            sl.smooth_db      = ema[i];
            sl.valid          = true;

            int pct = (i + 1) * 100 / total;
            {
                std::lock_guard<std::mutex> lk(g_state.mtx);
                g_state.buffer[i]    = sl;
                g_state.progress_pct = pct;
            }

            sse_broadcast(slice_to_sse(i, sl, pct, is_focus));
        }

        ema_valid = true;  // from second sweep onwards, EMA has history

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
    int  lo = is_focus ? 0      : TRIM_LO;
    int  hi = is_focus ? N_DISPLAY : TRIM_HI;
    std::ostringstream freq_arr, power_arr, smooth_arr;
    bool first = true;
    for (auto& sl : g_state.buffer) {
        if (!sl.valid) continue;
        auto freqs = freq_axis(sl.fc_mhz);
        for (int d = lo; d < hi; d++) {
            if (!first) { freq_arr << ','; power_arr << ','; smooth_arr << ','; }
            freq_arr   << freqs[d];
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
            if (fc != 0.0f && (fc < 50.0f || fc > 2000.0f)) {
                res.status = 400;
                res.set_content("{\"error\":\"fc out of range (50–2000 MHz)\"}", "application/json");
                return;
            }
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
