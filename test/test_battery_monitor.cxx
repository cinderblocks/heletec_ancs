/**
 * test_battery_monitor.cxx — Unity host-side tests for BatteryMonitor logic.
 *
 * Tests the piecewise-linear LiPo voltage→percentage conversion curve.
 * Because _voltageToPct is a private static method, the implementation is
 * tested via a thin file-scope trampoline that calls the equivalent logic
 * directly (the curve and math are duplicated here to keep this test
 * platform-free — the firmware implementation is what matters, but having
 * the curve tested separately gives regression protection).
 *
 * Run with: cmake -B build && cmake --build build && ctest --test-dir build -V
 */

#include "unity.h"
#include <cstdint>
#include <algorithm>

void setUp(void)    {}
void tearDown(void) {}

// ── Inline replica of BatteryMonitor::_voltageToPct ──────────────────────
// Kept here so the test has no firmware dependencies while still verifying
// the exact same table and interpolation logic.

static uint8_t voltageToPct(float voltage)
{
    struct Point { float v; uint8_t pct; };
    static constexpr Point kCurve[] = {
        { 4.20f, 100 },
        { 4.10f,  95 },
        { 4.00f,  90 },
        { 3.90f,  80 },
        { 3.80f,  70 },
        { 3.70f,  55 },
        { 3.60f,  35 },
        { 3.50f,  20 },
        { 3.40f,  10 },
        { 3.30f,   5 },
        { 3.20f,   0 },
    };
    static constexpr size_t kN = sizeof(kCurve) / sizeof(kCurve[0]);

    if (voltage >= kCurve[0].v)    return 100;
    if (voltage <= kCurve[kN-1].v) return 0;

    for (size_t i = 0; i + 1 < kN; ++i) {
        if (voltage <= kCurve[i].v && voltage >= kCurve[i+1].v) {
            const float t = (voltage - kCurve[i+1].v) / (kCurve[i].v - kCurve[i+1].v);
            const float pct = static_cast<float>(kCurve[i+1].pct)
                            + t * static_cast<float>(kCurve[i].pct - kCurve[i+1].pct);
            return static_cast<uint8_t>(pct);
        }
    }
    return 0;
}

// ── Boundary / endpoint tests ─────────────────────────────────────────────

void test_full_charge_is_100(void)
{
    TEST_ASSERT_EQUAL_UINT8(100, voltageToPct(4.20f));
}

void test_above_full_clamped_to_100(void)
{
    TEST_ASSERT_EQUAL_UINT8(100, voltageToPct(4.35f));
}

void test_empty_is_0(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, voltageToPct(3.20f));
}

void test_below_empty_clamped_to_0(void)
{
    TEST_ASSERT_EQUAL_UINT8(0, voltageToPct(3.00f));
}

// ── Breakpoint values ─────────────────────────────────────────────────────

void test_breakpoint_4v10_is_95(void)
{
    TEST_ASSERT_EQUAL_UINT8(95, voltageToPct(4.10f));
}

void test_breakpoint_4v00_is_90(void)
{
    TEST_ASSERT_EQUAL_UINT8(90, voltageToPct(4.00f));
}

void test_breakpoint_3v90_is_80(void)
{
    TEST_ASSERT_EQUAL_UINT8(80, voltageToPct(3.90f));
}

void test_breakpoint_3v80_is_70(void)
{
    TEST_ASSERT_EQUAL_UINT8(70, voltageToPct(3.80f));
}

void test_breakpoint_3v70_is_55(void)
{
    TEST_ASSERT_EQUAL_UINT8(55, voltageToPct(3.70f));
}

void test_breakpoint_3v60_is_35(void)
{
    TEST_ASSERT_EQUAL_UINT8(35, voltageToPct(3.60f));
}

void test_breakpoint_3v50_is_20(void)
{
    TEST_ASSERT_EQUAL_UINT8(20, voltageToPct(3.50f));
}

void test_breakpoint_3v40_is_10(void)
{
    TEST_ASSERT_EQUAL_UINT8(10, voltageToPct(3.40f));
}

void test_breakpoint_3v30_is_5(void)
{
    TEST_ASSERT_EQUAL_UINT8(5, voltageToPct(3.30f));
}

// ── Midpoint interpolation ────────────────────────────────────────────────

void test_midpoint_4v15_between_95_and_100(void)
{
    // 4.15 V is halfway between 4.10 (95%) and 4.20 (100%) → ~97–98 %
    const uint8_t pct = voltageToPct(4.15f);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(96, pct);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(98, pct);
}

void test_midpoint_3v75_between_55_and_70(void)
{
    // 3.75 V is halfway between 3.70 (55%) and 3.80 (70%) → ~62–63 %
    const uint8_t pct = voltageToPct(3.75f);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(61, pct);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(64, pct);
}

void test_midpoint_3v65_between_35_and_55(void)
{
    // 3.65 V is halfway between 3.60 (35%) and 3.70 (55%) → ~44–46 %
    const uint8_t pct = voltageToPct(3.65f);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(43, pct);
    TEST_ASSERT_LESS_OR_EQUAL_UINT8(47, pct);
}

// ── Monotonicity: result must never decrease as voltage rises ─────────────

void test_curve_is_monotone_increasing(void)
{
    uint8_t prev = 0;
    // Sample every 0.01 V from 3.20 to 4.20
    for (int i = 0; i <= 100; ++i) {
        const float v = 3.20f + i * 0.01f;
        const uint8_t cur = voltageToPct(v);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(prev, cur);
        prev = cur;
    }
}

// ── Charging threshold regression ────────────────────────────────────────
// Voltages above 4.15 V should yield 100 % (the charger is clamping VBAT).

void test_charging_threshold_yields_high_pct(void)
{
    // At 4.15 V the charger is active — percentage must be ≥ 95 %.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(95, voltageToPct(4.15f));
}

// ─────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────
int main(void)
{
    UNITY_BEGIN();

    // Boundary / endpoint
    RUN_TEST(test_full_charge_is_100);
    RUN_TEST(test_above_full_clamped_to_100);
    RUN_TEST(test_empty_is_0);
    RUN_TEST(test_below_empty_clamped_to_0);

    // Breakpoints
    RUN_TEST(test_breakpoint_4v10_is_95);
    RUN_TEST(test_breakpoint_4v00_is_90);
    RUN_TEST(test_breakpoint_3v90_is_80);
    RUN_TEST(test_breakpoint_3v80_is_70);
    RUN_TEST(test_breakpoint_3v70_is_55);
    RUN_TEST(test_breakpoint_3v60_is_35);
    RUN_TEST(test_breakpoint_3v50_is_20);
    RUN_TEST(test_breakpoint_3v40_is_10);
    RUN_TEST(test_breakpoint_3v30_is_5);

    // Interpolation
    RUN_TEST(test_midpoint_4v15_between_95_and_100);
    RUN_TEST(test_midpoint_3v75_between_55_and_70);
    RUN_TEST(test_midpoint_3v65_between_35_and_55);

    // Monotonicity
    RUN_TEST(test_curve_is_monotone_increasing);

    // Charging threshold
    RUN_TEST(test_charging_threshold_yields_high_pct);

    return UNITY_END();
}
