// test_framework.h — Minimal assertion framework (no external dependencies)
// Part of the Motor Control Simulation project.

#pragma once

#include <cmath>
#include <iostream>
#include <string>

namespace test {

/// Global counters shared across all translation units.
inline int g_passed = 0;
inline int g_failed = 0;

/// Report a single assertion result.
inline void report(bool condition, const std::string& name,
                   const char* file, int line) {
    if (condition) {
        ++g_passed;
        std::cout << "  [PASS] " << name << "\n";
    } else {
        ++g_failed;
        std::cout << "  [FAIL] " << name
                  << "  (" << file << ":" << line << ")\n";
    }
}

/// Print a final summary and return a process exit code.
inline int summarise() {
    std::cout << "\n========================================\n";
    std::cout << "  Results: " << g_passed << " passed, "
              << g_failed << " failed, "
              << (g_passed + g_failed) << " total\n";
    std::cout << "========================================\n";
    return (g_failed == 0) ? 0 : 1;
}

} // namespace test

// ── Assertion macros ─────────────────────────────────────────────────────

#define ASSERT_TRUE(cond, msg) \
    test::report((cond), (msg), __FILE__, __LINE__)

#define ASSERT_FALSE(cond, msg) \
    test::report(!(cond), (msg), __FILE__, __LINE__)

#define ASSERT_NEAR(actual, expected, tolerance, msg) \
    test::report(std::abs((actual) - (expected)) < (tolerance), \
                 (msg), __FILE__, __LINE__)

#define ASSERT_GT(a, b, msg) \
    test::report((a) > (b), (msg), __FILE__, __LINE__)

#define ASSERT_LT(a, b, msg) \
    test::report((a) < (b), (msg), __FILE__, __LINE__)

#define ASSERT_EQ(a, b, msg) \
    test::report((a) == (b), (msg), __FILE__, __LINE__)
