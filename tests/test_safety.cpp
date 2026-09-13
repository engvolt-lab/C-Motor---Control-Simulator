// test_safety.cpp — Unit tests for SafetySystem
// Part of the Motor Control Simulation project.

#include "test_framework.h"
#include "SafetySystem.h"

/// Runs all safety system tests.  Returns the number of failures.
int runSafetyTests() {
    std::cout << "\n── Safety System Tests ──────────────────────────\n\n";

    // ── 1. No fault under normal conditions ─────────────────────────────
    {
        SafetySystem safety;
        safety.check(5.0, 40.0, 2000.0, true);
        ASSERT_FALSE(safety.isTripped(),
                     "Normal conditions -> no trip");
    }

    // ── 2. Overcurrent trip ─────────────────────────────────────────────
    {
        SafetySystem safety(SafetySystem::Thresholds{15.0, 80.0, 3500.0});
        safety.check(16.0, 40.0, 2000.0, true);
        ASSERT_TRUE(safety.isTripped(),
                    "Overcurrent (16 A > 15 A) -> trip");

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults.size(), static_cast<size_t>(1),
                  "Exactly one fault active");
        ASSERT_EQ(faults[0], FaultType::Overcurrent,
                  "Fault type is Overcurrent");
    }

    // ── 3. Overtemperature trip ──────────────────────────────────────────
    {
        SafetySystem safety;
        safety.check(5.0, 81.0, 2000.0, true);
        ASSERT_TRUE(safety.isTripped(),
                    "Overtemperature (81 C > 80 C) -> trip");

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults[0], FaultType::Overtemperature,
                  "Fault type is Overtemperature");
    }

    // ── 4. Overspeed trip ───────────────────────────────────────────────
    {
        SafetySystem safety;
        safety.check(5.0, 40.0, 3600.0, true);
        ASSERT_TRUE(safety.isTripped(),
                    "Overspeed (3600 RPM > 3500 RPM) -> trip");

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults[0], FaultType::Overspeed,
                  "Fault type is Overspeed");
    }

    // ── 5. Encoder failure trip ─────────────────────────────────────────
    {
        SafetySystem safety;
        safety.check(5.0, 40.0, 2000.0, false);  // encoderOk = false
        ASSERT_TRUE(safety.isTripped(),
                    "Encoder failure -> trip");

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults[0], FaultType::EncoderFailure,
                  "Fault type is EncoderFailure");
    }

    // ── 6. Emergency stop ───────────────────────────────────────────────
    {
        SafetySystem safety;
        safety.triggerEStop();
        ASSERT_TRUE(safety.isTripped(),
                    "Emergency stop -> trip");

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults[0], FaultType::EmergencyStop,
                  "Fault type is EmergencyStop");
    }

    // ── 7. Fault latching (stays tripped after condition clears) ────────
    {
        SafetySystem safety;
        safety.check(16.0, 40.0, 2000.0, true);  // Trip on overcurrent.
        ASSERT_TRUE(safety.isTripped(), "Tripped after overcurrent");

        safety.check(5.0, 40.0, 2000.0, true);   // Condition clears.
        ASSERT_TRUE(safety.isTripped(),
                    "Fault latches — still tripped after condition clears");
    }

    // ── 8. Reset clears all faults ──────────────────────────────────────
    {
        SafetySystem safety;
        safety.check(16.0, 81.0, 3600.0, false);  // All faults!
        safety.triggerEStop();

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults.size(), static_cast<size_t>(5),
                  "All five faults active");

        safety.reset();
        ASSERT_FALSE(safety.isTripped(),
                     "Reset clears trip state");
        ASSERT_EQ(safety.getActiveFaults().size(), static_cast<size_t>(0),
                  "Reset clears all faults");
        ASSERT_EQ(safety.getFaultString(), std::string("OK"),
                  "Fault string is OK after reset");
    }

    // ── 9. Multiple simultaneous faults ─────────────────────────────────
    {
        SafetySystem safety;
        safety.check(20.0, 90.0, 2000.0, true);  // Over-I and over-T

        auto faults = safety.getActiveFaults();
        ASSERT_EQ(faults.size(), static_cast<size_t>(2),
                  "Two simultaneous faults detected");
    }

    // ── 10. Fault string formatting ─────────────────────────────────────
    {
        SafetySystem safety;
        ASSERT_EQ(safety.getFaultString(), std::string("OK"),
                  "No-fault string is 'OK'");

        safety.check(20.0, 40.0, 2000.0, true);
        std::string fs = safety.getFaultString();
        ASSERT_TRUE(fs.find("OVERCURRENT") != std::string::npos,
                    "Fault string contains 'OVERCURRENT'");
    }

    // ── 11. Values at thresholds do not trip ────────────────────────────
    {
        SafetySystem safety(SafetySystem::Thresholds{15.0, 80.0, 3500.0});
        safety.check(15.0, 80.0, 3500.0, true);  // Exactly at limits.
        ASSERT_FALSE(safety.isTripped(),
                     "Values at (not above) thresholds -> no trip");
    }

    return test::g_failed;
}
