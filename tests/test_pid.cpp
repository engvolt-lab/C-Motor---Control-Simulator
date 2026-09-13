// test_pid.cpp — Unit tests for PIDController
// Part of the Motor Control Simulation project.

#include "test_framework.h"
#include "PIDController.h"

/// Runs all PID controller tests.  Returns the number of failures.
int runPIDTests() {
    std::cout << "\n── PID Controller Tests ─────────────────────────\n\n";

    constexpr double dt = 0.01;

    // ── 1. Zero error produces zero output ──────────────────────────────
    {
        PIDController pid;
        double out = pid.compute(1000.0, 1000.0, dt);
        ASSERT_NEAR(out, 0.0, 0.01, "Zero error -> zero output");
    }

    // ── 2. Positive error produces positive output ──────────────────────
    {
        PIDController pid;
        double out = pid.compute(1000.0, 0.0, dt);
        ASSERT_GT(out, 0.0, "Positive error -> positive output");
    }

    // ── 3. Proportional response scales with error ──────────────────────
    {
        PIDController::Gains gains{0.1, 0.0, 0.0};  // P-only
        PIDController pid(gains);

        double out1 = pid.compute(100.0, 0.0, dt);
        pid.reset();
        double out2 = pid.compute(200.0, 0.0, dt);

        ASSERT_NEAR(out2 / out1, 2.0, 0.01,
                    "P-only: doubling error doubles output");
    }

    // ── 4. Integral accumulation ────────────────────────────────────────
    {
        PIDController::Gains gains{0.0, 1.0, 0.0};  // I-only
        PIDController pid(gains);

        pid.compute(100.0, 0.0, dt);  // integral += 100 * 0.01 = 1.0
        double out = pid.compute(100.0, 0.0, dt);  // integral += 1.0 -> 2.0
        ASSERT_NEAR(out, 2.0, 0.01,
                    "I-only: integral accumulates error*dt");
    }

    // ── 5. Anti-windup clamping ─────────────────────────────────────────
    {
        PIDController::Gains  gains{0.0, 1.0, 0.0};
        PIDController::Limits limits{-1000.0, 1000.0, -5.0, 5.0};
        PIDController pid(gains, limits);

        // Pump a huge error many times to saturate the integrator.
        for (int i = 0; i < 10000; ++i) {
            pid.compute(10000.0, 0.0, dt);
        }

        double integral = pid.getIntegral();
        ASSERT_NEAR(integral, 5.0, 0.01,
                    "Anti-windup: integral clamped to max");
    }

    // ── 6. Output clamping ──────────────────────────────────────────────
    {
        PIDController::Gains  gains{1.0, 0.0, 0.0};
        PIDController::Limits limits{0.0, 10.0, -200.0, 200.0};
        PIDController pid(gains, limits);

        double out = pid.compute(100.0, 0.0, dt);  // P = 100, clamp -> 10
        ASSERT_NEAR(out, 10.0, 0.01,
                    "Output clamped to max");
    }

    // ── 7. Negative output clamped to min ───────────────────────────────
    {
        PIDController::Gains  gains{1.0, 0.0, 0.0};
        PIDController::Limits limits{0.0, 48.0, -200.0, 200.0};
        PIDController pid(gains, limits);

        double out = pid.compute(0.0, 100.0, dt);  // error = -100, P = -100
        ASSERT_NEAR(out, 0.0, 0.01,
                    "Negative output clamped to min (0)");
    }

    // ── 8. Reset clears all state ───────────────────────────────────────
    {
        PIDController pid;
        pid.compute(1000.0, 0.0, dt);
        pid.compute(1000.0, 0.0, dt);
        pid.reset();

        ASSERT_NEAR(pid.getError(),    0.0, 0.001, "Reset clears error");
        ASSERT_NEAR(pid.getIntegral(), 0.0, 0.001, "Reset clears integral");
        ASSERT_NEAR(pid.getOutput(),   0.0, 0.001, "Reset clears output");
    }

    // ── 9. Derivative responds to error change ──────────────────────────
    {
        PIDController::Gains gains{0.0, 0.0, 1.0};  // D-only
        PIDController::Limits limits{-1000.0, 1000.0, -200.0, 200.0};
        PIDController pid(gains, limits);

        pid.compute(0.0, 0.0, dt);       // First call — derivative skipped.
        double out = pid.compute(100.0, 0.0, dt);
        // d(error)/dt = (100 - 0) / 0.01 = 10000;  D = 1.0 * 10000 = 10000
        // clamped to 1000
        ASSERT_NEAR(out, 1000.0, 0.01,
                    "D-only: responds to error change (clamped)");
    }

    // ── 10. Zero dt returns last output ─────────────────────────────────
    {
        PIDController pid;
        pid.compute(500.0, 0.0, dt);
        double prev = pid.getOutput();
        double out  = pid.compute(999.0, 0.0, 0.0);  // dt = 0
        ASSERT_NEAR(out, prev, 0.001,
                    "dt=0 returns previous output (no update)");
    }

    return test::g_failed;
}
