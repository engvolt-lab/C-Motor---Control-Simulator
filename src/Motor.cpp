// Motor.cpp — First-order DC motor model implementation
// Part of the Motor Control Simulation project.

#include "Motor.h"

#include <algorithm>
#include <cmath>

namespace {
    constexpr double kPi            = 3.14159265358979323846;
    constexpr double kRadPerSecToRPM = 60.0 / (2.0 * kPi);
}

// ─────────────────────────────────────────────────────────────────────────────
// Construction / reset
// ─────────────────────────────────────────────────────────────────────────────

Motor::Motor()
    : Motor(Parameters{})
{}

Motor::Motor(const Parameters& params)
    : params_(params)
    , omega_(0.0)
    , current_(0.0)
    , temperature_(params.ambientTemperature)
{}

void Motor::reset() {
    omega_       = 0.0;
    current_     = 0.0;
    temperature_ = params_.ambientTemperature;
}

// ─────────────────────────────────────────────────────────────────────────────
// Simulation step
// ─────────────────────────────────────────────────────────────────────────────

void Motor::update(double voltage, double loadTorque, double dt) {
    // Clamp input voltage to non-negative (unidirectional drive).
    voltage = std::max(voltage, 0.0);

    // 1. Back-EMF: V_back = Ke * omega
    const double backEmf = params_.backEmfConstant * omega_;

    // 2. Armature current: I = (V_in - V_back) / R
    //    Clamped to >= 0 (no regenerative braking in this model).
    current_ = std::max((voltage - backEmf) / params_.resistance, 0.0);

    // 3. Motor torque: tau = Kt * I
    const double motorTorque = params_.torqueConstant * current_;

    // 4. Angular acceleration: alpha = (tau_motor - tau_load - B*omega) / J
    const double netTorque = motorTorque - loadTorque - params_.friction * omega_;
    const double alpha     = netTorque / params_.inertia;

    // 5. Integrate angular velocity (forward Euler).
    omega_ += alpha * dt;
    omega_  = std::max(omega_, 0.0);   // No reverse rotation.

    // 6. Thermal model: dT/dt = (I^2*R*Rth - (T - T_amb)) / tau_th
    const double powerDissipated = current_ * current_ * params_.resistance;
    const double dTemp =
        (powerDissipated * params_.thermalResistance
         - (temperature_ - params_.ambientTemperature))
        / params_.thermalTimeConstant;
    temperature_ += dTemp * dt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Getters
// ─────────────────────────────────────────────────────────────────────────────

double Motor::getRPM() const {
    return omega_ * kRadPerSecToRPM;
}

double Motor::getAngularVelocity() const {
    return omega_;
}

double Motor::getCurrent() const {
    return current_;
}

double Motor::getTemperature() const {
    return temperature_;
}

double Motor::getBackEmf() const {
    return params_.backEmfConstant * omega_;
}
