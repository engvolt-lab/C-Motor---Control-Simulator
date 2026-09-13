// MotorController.h — Top-level simulation orchestrator
// Part of the Motor Control Simulation project.

#pragma once

#include "Motor.h"
#include "Encoder.h"
#include "PIDController.h"
#include "SafetySystem.h"

#include <string>

/// @brief Orchestrates the closed-loop motor control simulation.
///
/// Owns one instance each of Motor, Encoder, PIDController, and
/// SafetySystem.  Runs a fixed-timestep simulation loop, manages the
/// speed-setpoint profile, injects load disturbances, prints a
/// real-time–style telemetry table, and coordinates fault response
/// (cuts voltage on safety trip).
class MotorController {
public:
    MotorController();

    /// Simulation scenario modes.
    enum class Scenario {
        Nominal,
        OvercurrentTest,
        OvertemperatureTest,
        OverspeedTest,
        EncoderFailureTest,
        EmergencyStopTest
    };

    /// Run the automated 30s benchmark profile.
    void run(double duration = 30.0, double dt = 0.01, double printInterval = 0.5,
             Scenario scenario = Scenario::Nominal);

    // ── Interactive Controls (User Inputs) ────────────────────────────────

    /// Set the target speed [0 to 3000 RPM].
    void setTargetRPM(double rpm);
    double getTargetRPM() const;

    /// Set the external load torque [N*m].
    void setLoadTorque(double torque);
    double getLoadTorque() const;

    /// Set the acceleration ramp rate [RPM/s]. Set to 0 to disable ramping (instant step).
    void setRampRate(double rpmPerSec);
    double getRampRate() const;

    /// Update PID gains.
    void setPIDGains(double kp, double ki, double kd);
    PIDController::Gains getPIDGains() const;

    /// Trigger an emergency stop.
    void triggerEStop();

    /// Inject or clear encoder failure.
    void setEncoderFailure(bool failed);

    /// Reset safety latch and clear tripped status.
    void resetSafety();

    /// Reset motor, PID, encoder, and simulated time to t=0.
    void resetSimulation();

    /// Step simulation forward by `duration` seconds at current input setpoints,
    /// printing telemetry at each printInterval.
    void stepInteractive(double duration, double dt = 0.01, double printInterval = 0.5);

    /// Launch interactive wizard (prompts user for Target RPM, Load, Duration).
    void runInteractiveWizard();

    /// Launch interactive REPL command shell (type commands live).
    void runInteractiveREPL();

    // ── Telemetry Getters ────────────────────────────────────────────────
    double getCurrentTime() const;
    const Motor& getMotor() const;
    const Encoder& getEncoder() const;
    const PIDController& getPID() const;
    const SafetySystem& getSafety() const;

private:
    Motor          motor_;
    Encoder        encoder_;
    PIDController  pid_;
    SafetySystem   safety_;

    double currentTime_;
    double targetRPM_;
    double currentSetpoint_;
    double rampRate_;
    double loadTorque_;

    /// Piecewise-linear setpoint profile for automated 30s benchmark.
    double getBenchmarkSetpoint(double time)   const;

    /// Load-torque profile for automated 30s benchmark.
    double getBenchmarkLoadTorque(double time) const;

    // ── Table formatting helpers ─────────────────────────────────────────

    void printBanner()    const;
    void printHeader(bool isInteractive = false) const;
    void printSeparator() const;
    void printRow(double time, double setpoint, double actualRPM,
                  double error, double current, double temperature,
                  const std::string& status) const;
    void printSummary(double duration) const;
};
