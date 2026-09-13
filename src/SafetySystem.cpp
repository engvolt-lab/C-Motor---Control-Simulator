// SafetySystem.cpp — Fault monitoring implementation
// Part of the Motor Control Simulation project.

#include "SafetySystem.h"

// ─────────────────────────────────────────────────────────────────────────────
// Free function
// ─────────────────────────────────────────────────────────────────────────────

std::string faultToString(FaultType fault) {
    switch (fault) {
        case FaultType::Overcurrent:     return "OVERCURRENT";
        case FaultType::Overtemperature: return "OVERTEMPERATURE";
        case FaultType::Overspeed:       return "OVERSPEED";
        case FaultType::EncoderFailure:  return "ENCODER_FAILURE";
        case FaultType::EmergencyStop:   return "EMERGENCY_STOP";
        default:                         return "NONE";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction / reset
// ─────────────────────────────────────────────────────────────────────────────

SafetySystem::SafetySystem()
    : SafetySystem(Thresholds{})
{}

SafetySystem::SafetySystem(const Thresholds& thresholds)
    : thresholds_(thresholds)
    , tripped_(false)
    , overcurrent_(false)
    , overtemperature_(false)
    , overspeed_(false)
    , encoderFailure_(false)
    , emergencyStop_(false)
{}

void SafetySystem::reset() {
    tripped_         = false;
    overcurrent_     = false;
    overtemperature_ = false;
    overspeed_       = false;
    encoderFailure_  = false;
    emergencyStop_   = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Fault evaluation
// ─────────────────────────────────────────────────────────────────────────────

void SafetySystem::check(double current, double temperature,
                          double rpm, bool encoderOk) {
    if (current > thresholds_.maxCurrent) {
        overcurrent_ = true;
        tripped_     = true;
    }
    if (temperature > thresholds_.maxTemperature) {
        overtemperature_ = true;
        tripped_         = true;
    }
    if (rpm > thresholds_.maxRPM) {
        overspeed_ = true;
        tripped_   = true;
    }
    if (!encoderOk) {
        encoderFailure_ = true;
        tripped_        = true;
    }
}

void SafetySystem::triggerEStop() {
    emergencyStop_ = true;
    tripped_       = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Query
// ─────────────────────────────────────────────────────────────────────────────

bool SafetySystem::isTripped() const {
    return tripped_;
}

std::vector<FaultType> SafetySystem::getActiveFaults() const {
    std::vector<FaultType> faults;
    if (overcurrent_)     faults.push_back(FaultType::Overcurrent);
    if (overtemperature_) faults.push_back(FaultType::Overtemperature);
    if (overspeed_)       faults.push_back(FaultType::Overspeed);
    if (encoderFailure_)  faults.push_back(FaultType::EncoderFailure);
    if (emergencyStop_)   faults.push_back(FaultType::EmergencyStop);
    return faults;
}

std::string SafetySystem::getFaultString() const {
    if (!tripped_) return "OK";

    std::string result = "FAULT:";
    const auto faults = getActiveFaults();
    for (const auto& f : faults) {
        result += " " + faultToString(f);
    }
    return result;
}
