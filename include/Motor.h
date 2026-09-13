// Motor.h — First-order DC motor model with thermal dynamics
// Part of the Motor Control Simulation project.

#pragma once

/// @brief Simulates a brushed DC motor using a first-order electrical model
///        and a lumped-parameter thermal model.
///
/// Physics:
///   V_back = Ke * omega
///   I      = (V_in - V_back) / R
///   tau    = Kt * I
///   d(omega)/dt = (tau - tau_load - B * omega) / J
///   dT/dt  = (I^2 * R * Rth - (T - T_amb)) / tau_th
class Motor {
public:
    /// Motor electrical and mechanical parameters.
    struct Parameters {
        double resistance          = 1.0;    ///< Armature resistance [Ohm]
        double backEmfConstant     = 0.01;   ///< Back-EMF constant Ke [V*s/rad]
        double torqueConstant      = 0.01;   ///< Torque constant Kt [N*m/A]
        double inertia             = 0.001;  ///< Rotor moment of inertia J [kg*m^2]
        double friction            = 0.0001; ///< Viscous friction coefficient B [N*m*s/rad]
        double thermalResistance   = 2.0;    ///< Thermal resistance Rth [C/W]
        double thermalTimeConstant = 20.0;   ///< Thermal time constant tau_th [s]
        double ambientTemperature  = 25.0;   ///< Ambient temperature T_amb [C]
    };

    /// Construct a motor with default parameters.
    Motor();

    /// Construct a motor with custom parameters.
    explicit Motor(const Parameters& params);

    /// Advance the motor state by one timestep.
    /// @param voltage    Applied armature voltage [V]
    /// @param loadTorque External load torque [N*m]
    /// @param dt         Timestep [s]
    void update(double voltage, double loadTorque, double dt);

    /// Reset all state to initial conditions.
    void reset();

    // ── Getters ──────────────────────────────────────────────────────────

    double getRPM()             const; ///< Current speed [RPM]
    double getAngularVelocity() const; ///< Current speed [rad/s]
    double getCurrent()         const; ///< Armature current [A]
    double getTemperature()     const; ///< Winding temperature [C]
    double getBackEmf()         const; ///< Back-EMF voltage [V]

private:
    Parameters params_;

    double omega_;       ///< Angular velocity [rad/s]
    double current_;     ///< Armature current [A]
    double temperature_; ///< Winding temperature [C]
};
