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
static const Band BANDS[] = {
    {"fm",   88,  104,  8},   //  3 steps: 88, 96, 104  (covers 84–108 MHz)
    {"dab", 174,  238,  8},   //  9 steps: 174..238     (covers 170–242 MHz)
    {"uhf", 470,  690,  8},   // 28 steps: 470..690     (covers 466–694 MHz)
};

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
static std::atomic<bool> g_mock{false};
static std::string       g_web_dir = "/web";

// ── SSE helpers ───────────────────────────────────────────────────────────────

static void append_array(std::ostringstream& ss, const std::array<float, N_DISPLAY>& arr)
{
    ss << '[';
    for (int d = 0; d < N_DISPLAY; d++) {
        if (d) ss << ',';
        ss << arr[d];
    }
    ss << ']';
}

static std::string slice_to_sse(int step, const Slice& sl, int pct)
{
    std::ostringstream ss;
    ss << "data: {\"type\":\"step\""
       << ",\"step\":"         << step
       << ",\"fc_mhz\":"       << sl.fc_mhz
       << ",\"freq_start\":"   << sl.freq_start_mhz
       << ",\"freq_stop\":"    << sl.freq_stop_mhz
       << ",\"progress_pct\":" << pct
       << ",\"power_db\":";   append_array(ss, sl.power_db);
    ss << ",\"smooth_db\":";  append_array(ss, sl.smooth_db);
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

// ── Mock spectrum generator ───────────────────────────────────────────────────

static std::array<float, N_DISPLAY> mock_step(float fc_mhz)
{
    auto freqs = freq_axis(fc_mhz);
    std::array<float, N_DISPLAY> out;

    for (int d = 0; d < N_DISPLAY; d++)
        out[d] = -65.0f + (float)(rand() % 40 - 20) * 0.1f;  // noise floor ±2 dB

    // Gaussian peak: add_peak(centre_mhz, amplitude_db, sigma_mhz)
    auto add_peak = [&](float centre, float amp, float sigma) {
        for (int d = 0; d < N_DISPLAY; d++) {
            float diff = freqs[d] - centre;
            out[d] += amp * std::exp(-0.5f * diff * diff / (sigma * sigma));
        }
    };

    add_peak( 98.8f,  25.0f, 0.15f);  // FM station
    add_peak(202.9f,  18.0f, 2.0f);   // DAB multiplex (wideband)
    add_peak(530.0f,  28.0f, 3.5f);   // DVB-T UHF ch28 (fills most of 8 MHz)
    add_peak(610.0f,  22.0f, 3.5f);   // DVB-T UHF ch38

    return out;
}

// ── Sweep thread ──────────────────────────────────────────────────────────────

static void sweep_fn()
{
    auto steps = build_steps();
    int  total = (int)steps.size();

    // EMA state — persists across sweeps, reset if step count changes
    std::vector<std::array<float, N_DISPLAY>> ema(total);
    bool ema_valid = false;

    while (true)  // continuous sweep loop
    {
        {
            std::lock_guard<std::mutex> lk(g_state.mtx);
            g_state.state        = "sweeping";
            g_state.progress_pct = 0;
            g_state.buffer.assign(total, Slice{});
        }
        sse_broadcast("data: {\"type\":\"start\"}\n\n");

        for (int i = 0; i < total; i++)
        {
            float       fc_mhz = steps[i].fc_mhz;
            const char *band   = steps[i].band;
            std::cerr << "[sweep] step " << (i+1) << "/" << total
                      << "  fc=" << fc_mhz << " MHz  band=" << band << std::endl;

            std::array<float, N_DISPLAY> power;

            if (g_mock)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                power = mock_step(fc_mhz);
            }
            else
            {
                retune((double)fc_mhz * 1e6);

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
            sl.freq_start_mhz = fc_mhz - 4.0f;
            sl.freq_stop_mhz  = fc_mhz + 4.0f;
            sl.power_db       = power;
            sl.smooth_db      = ema[i];
            sl.valid          = true;

            int pct = (i + 1) * 100 / total;
            {
                std::lock_guard<std::mutex> lk(g_state.mtx);
                g_state.buffer[i]    = sl;
                g_state.progress_pct = pct;
            }

            sse_broadcast(slice_to_sse(i, sl, pct));
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
    std::ostringstream freq_arr, power_arr, smooth_arr;
    bool first = true;
    for (auto& sl : g_state.buffer) {
        if (!sl.valid) continue;
        auto freqs = freq_axis(sl.fc_mhz);
        for (int d = 0; d < N_DISPLAY; d++) {
            if (!first) { freq_arr << ','; power_arr << ','; smooth_arr << ','; }
            freq_arr  << freqs[d];
            power_arr << sl.power_db[d];
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
