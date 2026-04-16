#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ring.h"
#include "config.h"

#include <cmath>
#include <vector>

// Small n for fast tests — ring is n-agnostic
static constexpr int N = 16;

// ── 1. Single push ────────────────────────────────────────────────────────────

TEST_CASE("SpectrumRing: single push → get_db matches 10*log10(input)", "[ring]")
{
    SpectrumRing ring;
    std::vector<float> lin(N, 0.01f);
    ring.push(lin.data(), N);

    std::vector<float> db;
    ring.get_db(db);

    REQUIRE((int)db.size() == N);
    const float expected = 10.0f * std::log10(0.01f);  // = -20 dB
    for (int k = 0; k < N; k++)
        CHECK(db[k] == Catch::Approx(expected).margin(0.01f));
}

// ── 2. N identical pushes ─────────────────────────────────────────────────────

TEST_CASE("SpectrumRing: N identical pushes → same get_db as single push", "[ring]")
{
    SpectrumRing ring;
    const float val = 0.05f;
    std::vector<float> lin(N, val);

    for (int i = 0; i < METRICS_AVG_STEPS; i++)
        ring.push(lin.data(), N);

    std::vector<float> db;
    ring.get_db(db);

    const float expected = 10.0f * std::log10(val);
    for (int k = 0; k < N; k++)
        CHECK(db[k] == Catch::Approx(expected).margin(0.01f));
}

// ── 3. Eviction: N+1 pushes drops oldest ─────────────────────────────────────

TEST_CASE("SpectrumRing: N+1 pushes evicts oldest entry", "[ring]")
{
    SpectrumRing ring;

    // Fill ring with A
    std::vector<float> la(N, 1.0f);
    for (int i = 0; i < METRICS_AVG_STEPS; i++)
        ring.push(la.data(), N);

    // One push of B — oldest A evicted, sum = (N-1)*A + B
    std::vector<float> lb(N, 0.01f);
    ring.push(lb.data(), N);

    std::vector<float> db;
    ring.get_db(db);

    const float expected_linear = ((METRICS_AVG_STEPS - 1) * 1.0f + 0.01f) / METRICS_AVG_STEPS;
    const float expected_db     = 10.0f * std::log10(expected_linear);
    for (int k = 0; k < N; k++)
        CHECK(db[k] == Catch::Approx(expected_db).margin(0.1f));
}

// ── 4. ready() gate ───────────────────────────────────────────────────────────

TEST_CASE("SpectrumRing: ready() false before METRICS_MIN_ENTRIES, true after", "[ring]")
{
    SpectrumRing ring;
    std::vector<float> lin(N, 0.1f);

    for (int i = 0; i < METRICS_MIN_ENTRIES - 1; i++) {
        ring.push(lin.data(), N);
        CHECK(!ring.ready());
    }

    ring.push(lin.data(), N);
    CHECK(ring.ready());
}

// ── 5. Reset ──────────────────────────────────────────────────────────────────

TEST_CASE("SpectrumRing: reset → count=0, ready()=false, get_db returns empty", "[ring]")
{
    SpectrumRing ring;
    std::vector<float> lin(N, 0.5f);
    for (int i = 0; i < 5; i++) ring.push(lin.data(), N);

    ring = SpectrumRing{};  // reset

    CHECK(!ring.ready());

    std::vector<float> db;
    ring.get_db(db);
    CHECK(db.empty());  // sum not yet initialised — nothing to return
}

// ── 6. Arithmetic mean in linear domain ───────────────────────────────────────

TEST_CASE("SpectrumRing: two entries A,B → arithmetic mean in linear", "[ring]")
{
    SpectrumRing ring;
    const float a = 1.0f, b = 0.01f;
    std::vector<float> la(N, a), lb(N, b);
    ring.push(la.data(), N);
    ring.push(lb.data(), N);

    std::vector<float> db;
    ring.get_db(db);

    // Arithmetic mean: (a+b)/2
    const float expected = 10.0f * std::log10((a + b) / 2.0f);
    for (int k = 0; k < N; k++)
        CHECK(db[k] == Catch::Approx(expected).margin(0.01f));
}

// ── 7. Linear ≠ geometric mean — confirms domain is correct ──────────────────

TEST_CASE("SpectrumRing: arithmetic mean differs from geometric mean for skewed input", "[ring]")
{
    // a=1.0, b=0.01 (40 dB apart)
    //   arithmetic: (1.0 + 0.01)/2 = 0.505  → ~-2.97 dB  ← ring should return this
    //   geometric:  sqrt(1.0*0.01) = 0.1    → -10.0  dB  ← EMA-in-dB would return this
    SpectrumRing ring;
    const float a = 1.0f, b = 0.01f;
    std::vector<float> la(N, a), lb(N, b);
    ring.push(la.data(), N);
    ring.push(lb.data(), N);

    std::vector<float> db;
    ring.get_db(db);

    const float arith_db = 10.0f * std::log10((a + b) / 2.0f);   // ≈ -2.97 dB
    const float geom_db  = 10.0f * std::log10(std::sqrt(a * b));  // = -10.0 dB

    CHECK(db[0] == Catch::Approx(arith_db).margin(0.01f));
    CHECK(std::fabsf(db[0] - geom_db) > 5.0f);  // meaningfully different from EMA-in-dB
}
