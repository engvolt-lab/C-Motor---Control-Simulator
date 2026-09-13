// Encoder.h — Simulated rotary encoder with noise and failure injection
// Part of the Motor Control Simulation project.

#pragma once

#include <random>

/// @brief Simulates an incremental rotary encoder.
///
/// Converts the motor's true angular velocity to a measured RPM value,
/// adding configurable Gaussian noise and supporting failure injection
/// (output drops to zero when failed).
class Encoder {
public:
    /// @param pulsesPerRevolution  Encoder PPR (determines quantisation).
    /// @param noiseAmplitude       Peak noise amplitude [RPM].
    explicit Encoder(int pulsesPerRevolution = 1024, double noiseAmplitude = 0.5);

    /// Read the encoder and return measured RPM.
    /// @param trueRPM  The motor's actual speed [RPM].
    double read(double trueRPM);

    /// Inject or clear an encoder failure.
    void setFailure(bool failed);

    /// @return true if the encoder is currently in a failed state.
    bool hasFailed() const;

private:
    int    ppr_;
    double noiseAmplitude_;
    bool   failed_;

    std::mt19937                          rng_;
    std::uniform_real_distribution<double> noiseDist_;
};
