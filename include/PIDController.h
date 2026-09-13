// PIDController.h — Discrete PID controller with anti-windup
// Part of the Motor Control Simulation project.

#pragma once

/// @brief Standard discrete PID controller.
///
/// Features:
///   - Proportional, integral, derivative terms
///   - Anti-windup via integral clamping
///   - Output clamping to [outputMin, outputMax]
///   - Derivative kick avoidance on first sample
class PIDController {
public:
    /// PID gain constants.
    struct Gains {
        double kp = 0.01;   ///< Proportional gain [V/RPM]
        double ki = 0.05;   ///< Integral gain     [V/(RPM*s)]
        double kd = 0.001;  ///< Derivative gain   [V*s/RPM]
    };

    /// Output and integral limits for anti-windup.
    struct Limits {
        double outputMin   =    0.0;  ///< Minimum controller output [V]
        double outputMax   =   48.0;  ///< Maximum controller output [V]
        double integralMin = -200.0;  ///< Anti-windup lower bound
        double integralMax =  200.0;  ///< Anti-windup upper bound
    };

    PIDController();
    explicit PIDController(const Gains& gains);
    PIDController(const Gains& gains, const Limits& limits);

    /// Compute the controller output for a single timestep.
    /// @param setpoint    Desired value [RPM]
    /// @param measurement Current measured value [RPM]
    /// @param dt          Timestep [s]
    /// @return            Control voltage [V]
    double compute(double setpoint, double measurement, double dt);

    /// Reset internal state (integral, previous error, first-run flag).
    void reset();

    /// Set PID gain constants.
    void setGains(const Gains& gains);

    /// Get current PID gain constants.
    Gains getGains() const;

    // ── Diagnostics ──────────────────────────────────────────────────────

    double getError()     const; ///< Most recent error
    double getIntegral()  const; ///< Current integral accumulator
    double getOutput()    const; ///< Most recent output

private:
    Gains  gains_;
    Limits limits_;

    double prevError_;
    double integral_;
    double lastOutput_;
    bool   firstRun_;
};
