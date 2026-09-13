// PIDController.cpp — Discrete PID controller implementation
// Part of the Motor Control Simulation project.

#include "PIDController.h"

#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// Construction / reset
// ─────────────────────────────────────────────────────────────────────────────

PIDController::PIDController()
    : PIDController(Gains{}, Limits{})
{}

PIDController::PIDController(const Gains& gains)
    : PIDController(gains, Limits{})
{}

PIDController::PIDController(const Gains& gains, const Limits& limits)
    : gains_(gains)
    , limits_(limits)
    , prevError_(0.0)
    , integral_(0.0)
    , lastOutput_(0.0)
    , firstRun_(true)
{}

void PIDController::reset() {
    prevError_  = 0.0;
    integral_   = 0.0;
    lastOutput_ = 0.0;
    firstRun_   = true;
}

void PIDController::setGains(const Gains& gains) {
    gains_ = gains;
}

PIDController::Gains PIDController::getGains() const {
    return gains_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Core computation
// ─────────────────────────────────────────────────────────────────────────────

double PIDController::compute(double setpoint, double measurement, double dt) {
    // Guard against degenerate timesteps.
    if (dt <= 0.0) {
        return lastOutput_;
    }

    const double error = setpoint - measurement;

    // ── Proportional term ────────────────────────────────────────────────
    const double pTerm = gains_.kp * error;

    // ── Integral term with anti-windup clamp ─────────────────────────────
    integral_ += error * dt;
    integral_  = std::clamp(integral_, limits_.integralMin, limits_.integralMax);
    const double iTerm = gains_.ki * integral_;

    // ── Derivative term (skip first sample to avoid a derivative spike) ──
    double dTerm = 0.0;
    if (!firstRun_) {
        const double derivative = (error - prevError_) / dt;
        dTerm = gains_.kd * derivative;
    }
    firstRun_ = false;
    prevError_ = error;

    // ── Sum and clamp ────────────────────────────────────────────────────
    double output = pTerm + iTerm + dTerm;
    output = std::clamp(output, limits_.outputMin, limits_.outputMax);

    lastOutput_ = output;
    return output;
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostics
// ─────────────────────────────────────────────────────────────────────────────

double PIDController::getError()    const { return prevError_;  }
double PIDController::getIntegral() const { return integral_;   }
double PIDController::getOutput()   const { return lastOutput_; }
