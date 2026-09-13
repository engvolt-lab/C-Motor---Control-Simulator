// SafetySystem.h — Fault monitoring with latching protection
// Part of the Motor Control Simulation project.

#pragma once

#include <string>
#include <vector>

/// Enumeration of all recognised fault types.
enum class FaultType {
    None,
    Overcurrent,
    Overtemperature,
    Overspeed,
    EncoderFailure,
    EmergencyStop
};

/// Convert a FaultType to a human-readable string.
std::string faultToString(FaultType fault);

/// @brief Monitors motor telemetry and latches safety faults.
///
/// Once a fault is detected the system enters a *tripped* state that
/// persists until explicitly cleared with reset().  This mirrors real
/// industrial safety relay behaviour.
class SafetySystem {
public:
    /// Configurable trip thresholds.
    struct Thresholds {
        double maxCurrent     = 15.0;   ///< Overcurrent limit [A]
        double maxTemperature = 80.0;   ///< Overtemperature limit [C]
        double maxRPM         = 3500.0; ///< Overspeed limit [RPM]
    };

    SafetySystem();
    explicit SafetySystem(const Thresholds& thresholds);

    /// Evaluate all fault conditions against the current telemetry.
    void check(double current, double temperature, double rpm, bool encoderOk);

    /// Trigger an external emergency-stop.
    void triggerEStop();

    /// @return true if any fault is currently latched.
    bool isTripped() const;

    /// @return a vector of all currently active fault types.
    std::vector<FaultType> getActiveFaults() const;

    /// @return a formatted string summarising the current fault state.
    std::string getFaultString() const;

    /// Clear all latched faults and return to normal operation.
    void reset();

private:
    Thresholds thresholds_;

    bool tripped_;
    bool overcurrent_;
    bool overtemperature_;
    bool overspeed_;
    bool encoderFailure_;
    bool emergencyStop_;
};
