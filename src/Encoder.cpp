// Encoder.cpp — Simulated rotary encoder implementation
// Part of the Motor Control Simulation project.

#include "Encoder.h"

#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

Encoder::Encoder(int pulsesPerRevolution, double noiseAmplitude)
    : ppr_(pulsesPerRevolution)
    , noiseAmplitude_(noiseAmplitude)
    , failed_(false)
    , rng_(42)                  // Fixed seed for reproducible simulations.
    , noiseDist_(-1.0, 1.0)
{}

// ─────────────────────────────────────────────────────────────────────────────
// Reading
// ─────────────────────────────────────────────────────────────────────────────

double Encoder::read(double trueRPM) {
    // A failed encoder returns zero — the safety system detects this.
    if (failed_) {
        return 0.0;
    }

    // Quantise based on encoder resolution.
    // Minimum resolvable RPM step = 60 / PPR.
    const double resolution = 60.0 / static_cast<double>(ppr_);
    double quantised = std::round(trueRPM / resolution) * resolution;

    // Additive measurement noise (uniform ±noiseAmplitude_).
    const double noise = noiseAmplitude_ * noiseDist_(rng_);
    quantised += noise;

    // RPM is always non-negative.
    return std::max(quantised, 0.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Failure injection
// ─────────────────────────────────────────────────────────────────────────────

void Encoder::setFailure(bool failed) {
    failed_ = failed;
}

bool Encoder::hasFailed() const {
    return failed_;
}
