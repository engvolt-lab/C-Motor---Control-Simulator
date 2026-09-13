// MotorController.cpp — Simulation orchestrator implementation
// Part of the Motor Control Simulation project.

#include "MotorController.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

MotorController::MotorController()
    : motor_(Motor::Parameters{
          /* resistance          */ 1.0,
          /* backEmfConstant     */ 0.01,
          /* torqueConstant      */ 0.01,
          /* inertia             */ 0.001,
          /* friction            */ 0.0001,
          /* thermalResistance   */ 0.5,
          /* thermalTimeConstant */ 20.0,
          /* ambientTemperature  */ 25.0
      })
    , encoder_(1024, 0.5)
    , pid_(PIDController::Gains{0.01, 0.05, 0.001},
           PIDController::Limits{0.0, 48.0, -200.0, 200.0})
    , safety_(SafetySystem::Thresholds{15.0, 80.0, 3500.0})
    , currentTime_(0.0)
    , targetRPM_(0.0)
    , currentSetpoint_(0.0)
    , rampRate_(800.0)
    , loadTorque_(0.01)
{}

// ─────────────────────────────────────────────────────────────────────────────
// Interactive Controls & Getters
// ─────────────────────────────────────────────────────────────────────────────

void MotorController::setTargetRPM(double rpm) {
    targetRPM_ = std::clamp(rpm, 0.0, 4500.0);
}

double MotorController::getTargetRPM() const {
    return targetRPM_;
}

void MotorController::setRampRate(double rpmPerSec) {
    rampRate_ = std::max(rpmPerSec, 0.0);
}

double MotorController::getRampRate() const {
    return rampRate_;
}

void MotorController::setLoadTorque(double torque) {
    loadTorque_ = std::max(torque, 0.0);
}

double MotorController::getLoadTorque() const {
    return loadTorque_;
}

void MotorController::setPIDGains(double kp, double ki, double kd) {
    pid_.setGains(PIDController::Gains{kp, ki, kd});
}

PIDController::Gains MotorController::getPIDGains() const {
    return pid_.getGains();
}

void MotorController::triggerEStop() {
    safety_.triggerEStop();
}

void MotorController::setEncoderFailure(bool failed) {
    encoder_.setFailure(failed);
}

void MotorController::resetSafety() {
    safety_.reset();
}

void MotorController::resetSimulation() {
    motor_.reset();
    pid_.reset();
    safety_.reset();
    encoder_.setFailure(false);
    currentTime_     = 0.0;
    targetRPM_       = 0.0;
    currentSetpoint_ = 0.0;
    loadTorque_      = 0.01;
}

double MotorController::getCurrentTime() const {
    return currentTime_;
}

const Motor& MotorController::getMotor() const {
    return motor_;
}

const Encoder& MotorController::getEncoder() const {
    return encoder_;
}

const PIDController& MotorController::getPID() const {
    return pid_;
}

const SafetySystem& MotorController::getSafety() const {
    return safety_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Benchmark profiles (Automated 30s run)
// ─────────────────────────────────────────────────────────────────────────────

double MotorController::getBenchmarkSetpoint(double t) const {
    if (t <  5.0) return (t / 5.0)              * 1000.0;
    if (t < 10.0) return 1000.0;
    if (t < 12.0) return 1000.0 + ((t - 10.0) / 2.0) * 1000.0;
    if (t < 18.0) return 2000.0;
    if (t < 20.0) return 2000.0 + ((t - 18.0) / 2.0) * 1000.0;
    if (t < 25.0) return 3000.0;
    if (t < 30.0) return 3000.0 * (1.0 - (t - 25.0) / 5.0);
    return 0.0;
}

double MotorController::getBenchmarkLoadTorque(double t) const {
    constexpr double baseTorque        = 0.01;
    constexpr double disturbanceTorque = 0.05;

    if (t >= 15.0 && t < 20.0) {
        return baseTorque + disturbanceTorque;
    }
    return baseTorque;
}

// ─────────────────────────────────────────────────────────────────────────────
// Table formatting
// ─────────────────────────────────────────────────────────────────────────────

void MotorController::printSeparator() const {
    std::cout << "+--------+-----------+-----------+-----------+"
                 "---------+----------+--------------------+\n";
}

void MotorController::printBanner() const {
    std::cout << "\n"
        "  __  __       _               ____            _             _\n"
        " |  \\/  | ___ | |_ ___  _ __  / ___|___  _ __ | |_ _ __ ___ | |\n"
        " | |\\/| |/ _ \\| __/ _ \\| '__|| |   / _ \\| '_ \\| __| '__/ _ \\| |\n"
        " | |  | | (_) | || (_) | |   | |__| (_) | | | | |_| | | (_) | |\n"
        " |_|  |_|\\___/ \\__\\___/|_|    \\____\\___/|_| |_|\\__|_|  \\___/|_|\n"
        "               S i m u l a t i o n   v 1 . 0\n\n";
}

void MotorController::printHeader(bool isInteractive) const {
    printBanner();

    if (isInteractive) {
        std::cout << "  [INTERACTIVE CONTROL MODE]\n";
        std::cout << "  Target: " << targetRPM_ << " RPM  |  Load: " << loadTorque_
                  << " N*m  |  Ramp Rate: " << rampRate_ << " RPM/s  |  Time: "
                  << currentTime_ << " s\n\n";
    } else {
        std::cout << "  Duration: 30.0 s  |  Timestep: 10 ms  |  Print interval: 0.5 s\n";
        std::cout << "  Load disturbance: +0.05 N*m  @ t = 15-20 s\n\n";
    }

    printSeparator();
    std::cout << "|  Time  |  Target   |  Actual   |   Error   |"
                 " Current |   Temp   |      Status        |\n";
    std::cout << "|   (s)  |   (RPM)   |   (RPM)   |   (RPM)   |"
                 "   (A)   |   (C)    |                    |\n";
    printSeparator();
}

void MotorController::printRow(double time, double setpoint, double actualRPM,
                                double error, double current,
                                double temperature,
                                const std::string& status) const {
    std::cout << "| " << std::fixed
              << std::setprecision(1) << std::setw(6) << time    << " | "
              << std::setprecision(1) << std::setw(9) << setpoint << " | "
              << std::setprecision(1) << std::setw(9) << actualRPM << " | "
              << std::setprecision(1) << std::setw(9) << error    << " | "
              << std::setprecision(2) << std::setw(7) << current  << " | "
              << std::setprecision(1) << std::setw(8) << temperature << " | "
              << std::setw(18) << std::left << status << std::right << " |\n";
}

void MotorController::printSummary(double duration) const {
    printSeparator();
    std::cout << "\n=== Step Complete (Simulated +" << std::fixed
              << std::setprecision(1) << duration << " s -> Total: "
              << currentTime_ << " s) ===\n";
    std::cout << "  Actual Speed:      "
              << std::setprecision(1) << motor_.getRPM() << " RPM (Target: "
              << targetRPM_ << " RPM)\n";
    std::cout << "  Winding Temp:      "
              << std::setprecision(1) << motor_.getTemperature() << " C\n";
    std::cout << "  Armature Current:  "
              << std::setprecision(2) << motor_.getCurrent() << " A\n";
    std::cout << "  Applied Load:      "
              << std::setprecision(3) << loadTorque_ << " N*m\n";

    if (safety_.isTripped()) {
        std::cout << "  Safety Status:     " << safety_.getFaultString() << " (POWER CUT)\n";
    } else {
        std::cout << "  Safety Status:     All systems nominal (OK)\n";
    }
    std::cout << std::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Interactive Step Simulation
// ─────────────────────────────────────────────────────────────────────────────

void MotorController::stepInteractive(double duration, double dt, double printInterval) {
    printHeader(true);

    const double startTime = currentTime_;
    const double endTime   = startTime + duration;
    double nextPrint       = startTime;

    while (currentTime_ <= endTime + dt * 0.5) {
        // Controlled acceleration/deceleration ramp towards targetRPM_
        if (rampRate_ > 0.0) {
            if (currentSetpoint_ < targetRPM_) {
                currentSetpoint_ = std::min(currentSetpoint_ + rampRate_ * dt, targetRPM_);
            } else if (currentSetpoint_ > targetRPM_) {
                currentSetpoint_ = std::max(currentSetpoint_ - rampRate_ * dt, targetRPM_);
            }
        } else {
            currentSetpoint_ = targetRPM_;
        }

        // 1. Read encoder feedback.
        const double measuredRPM = encoder_.read(motor_.getRPM());

        // 2. Check safety.
        safety_.check(motor_.getCurrent(), motor_.getTemperature(),
                      measuredRPM, !encoder_.hasFailed());

        // 3. Compute control signal (or cut power if tripped).
        double voltage = 0.0;
        if (!safety_.isTripped()) {
            voltage = pid_.compute(currentSetpoint_, measuredRPM, dt);
        }

        // 4. Update motor physics with applied load torque.
        motor_.update(voltage, loadTorque_, dt);

        // 5. Print telemetry at each print interval.
        if (currentTime_ >= nextPrint - dt * 0.5) {
            const double error = targetRPM_ - measuredRPM;
            const std::string status =
                safety_.isTripped() ? safety_.getFaultString() : "OK";

            printRow(currentTime_, targetRPM_, measuredRPM, error,
                     motor_.getCurrent(), motor_.getTemperature(), status);
            nextPrint += printInterval;
        }

        currentTime_ += dt;
    }

    printSummary(duration);
}

// ─────────────────────────────────────────────────────────────────────────────
// Automated 30s Benchmark Run
// ─────────────────────────────────────────────────────────────────────────────

void MotorController::run(double duration, double dt, double printInterval,
                          Scenario scenario) {
    resetSimulation();
    printHeader(false);

    if (scenario != Scenario::Nominal) {
        std::cout << "  [SCENARIO TEST ACTIVE: ";
        switch (scenario) {
            case Scenario::OvercurrentTest:     std::cout << "OVERCURRENT FAULT INJECTION AT t=16s]\n\n"; break;
            case Scenario::OvertemperatureTest: std::cout << "OVERTEMPERATURE FAULT INJECTION AT t=16s]\n\n"; break;
            case Scenario::OverspeedTest:       std::cout << "OVERSPEED FAULT INJECTION AT t=16s]\n\n"; break;
            case Scenario::EncoderFailureTest:  std::cout << "ENCODER FAILURE INJECTION AT t=16s]\n\n"; break;
            case Scenario::EmergencyStopTest:   std::cout << "EMERGENCY STOP TRIGGERED AT t=16s]\n\n"; break;
            default: break;
        }
    }

    currentTime_     = 0.0;
    double nextPrint = 0.0;

    while (currentTime_ <= duration + dt * 0.5) {
        targetRPM_  = getBenchmarkSetpoint(currentTime_);
        loadTorque_ = getBenchmarkLoadTorque(currentTime_);

        if (currentTime_ >= 16.0) {
            switch (scenario) {
                case Scenario::OvercurrentTest:
                    loadTorque_ = 0.25;
                    break;
                case Scenario::OverspeedTest:
                    targetRPM_ = 4200.0;
                    break;
                case Scenario::EncoderFailureTest:
                    encoder_.setFailure(true);
                    break;
                case Scenario::EmergencyStopTest:
                    safety_.triggerEStop();
                    break;
                case Scenario::OvertemperatureTest:
                    loadTorque_ = 0.18;
                    break;
                default:
                    break;
            }
        }

        const double measuredRPM = encoder_.read(motor_.getRPM());

        safety_.check(motor_.getCurrent(), motor_.getTemperature(),
                      measuredRPM, !encoder_.hasFailed());

        double voltage = 0.0;
        if (!safety_.isTripped()) {
            voltage = pid_.compute(targetRPM_, measuredRPM, dt);
        }

        motor_.update(voltage, loadTorque_, dt);

        if (currentTime_ >= nextPrint - dt * 0.5) {
            const double error = targetRPM_ - measuredRPM;
            const std::string status =
                safety_.isTripped() ? safety_.getFaultString() : "OK";

            printRow(currentTime_, targetRPM_, measuredRPM, error,
                     motor_.getCurrent(), motor_.getTemperature(), status);
            nextPrint += printInterval;
        }

        currentTime_ += dt;
    }

    printSummary(duration);
}

// ─────────────────────────────────────────────────────────────────────────────
// Interactive Guided Wizard (Prompt-based)
// ─────────────────────────────────────────────────────────────────────────────

void MotorController::runInteractiveWizard() {
    std::cout << "\n=======================================================\n";
    std::cout << "       DC MOTOR CONTROL SIMULATOR — GUIDED MODE\n";
    std::cout << "=======================================================\n";
    std::cout << "Enter custom inputs at each step to see live motor output.\n\n";

    while (true) {
        std::cout << "Current State: Time=" << std::fixed << std::setprecision(1)
                  << currentTime_ << "s | Actual=" << motor_.getRPM()
                  << " RPM | Temp=" << motor_.getTemperature() << " C\n";

        std::cout << "\nEnter Target Speed (0 to 3000 RPM) [current: "
                  << targetRPM_ << "]: ";
        std::string line;
        if (!std::getline(std::cin, line)) break;
        if (!line.empty()) {
            try {
                setTargetRPM(std::stod(line));
            } catch (...) {
                std::cout << "Invalid input; keeping " << targetRPM_ << " RPM\n";
            }
        }

        std::cout << "Enter Load Torque in N*m (0.00 to 0.20) [current: "
                  << loadTorque_ << "]: ";
        if (!std::getline(std::cin, line)) break;
        if (!line.empty()) {
            try {
                setLoadTorque(std::stod(line));
            } catch (...) {
                std::cout << "Invalid input; keeping " << loadTorque_ << " N*m\n";
            }
        }

        double duration = 5.0;
        std::cout << "Enter Duration to simulate in seconds [default 5.0]: ";
        if (!std::getline(std::cin, line)) break;
        if (!line.empty()) {
            try {
                duration = std::max(0.1, std::stod(line));
            } catch (...) {
                duration = 5.0;
            }
        }

        std::cout << "Optional Action [0=None, 1=Reset Faults, 2=Trigger E-Stop, 3=Fail Encoder, 4=Reset Sim]: ";
        if (std::getline(std::cin, line) && !line.empty()) {
            if (line == "1") resetSafety();
            else if (line == "2") triggerEStop();
            else if (line == "3") setEncoderFailure(true);
            else if (line == "4") resetSimulation();
        }

        // Run simulation step with these inputs!
        stepInteractive(duration, 0.01, 0.5);

        std::cout << "\nPerform another step? [Y/n]: ";
        if (std::getline(std::cin, line)) {
            if (!line.empty() && (line[0] == 'n' || line[0] == 'N')) {
                break;
            }
        } else {
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Interactive REPL Shell (Command-based)
// ─────────────────────────────────────────────────────────────────────────────

void MotorController::runInteractiveREPL() {
    std::cout << "\n=======================================================\n";
    std::cout << "     DC MOTOR CONTROL SIMULATOR — INTERACTIVE SHELL\n";
    std::cout << "=======================================================\n";
    std::cout << "Type 'help' for command list or 'exit' to quit.\n\n";

    std::string line;
    while (true) {
        std::cout << "motor-sim (t=" << std::fixed << std::setprecision(1)
                  << currentTime_ << "s, " << std::setprecision(0)
                  << motor_.getRPM() << "/" << targetRPM_ << " RPM)> ";
        if (!std::getline(std::cin, line)) break;

        std::stringstream ss(line);
        std::string cmd;
        ss >> cmd;
        if (cmd.empty()) continue;

        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if (cmd == "exit" || cmd == "quit" || cmd == "q") {
            break;
        } else if (cmd == "help" || cmd == "?") {
            std::cout << "\nAvailable Commands:\n"
                      << "  set <rpm>          Set target speed [0 to 3000 RPM]\n"
                      << "  load <torque>      Set load torque in N*m [e.g. 0.01 - 0.20]\n"
                      << "  step <seconds>     Simulate forward for N seconds with telemetry\n"
                      << "  run <seconds>      Alias for step\n"
                      << "  estop              Trigger Emergency Stop\n"
                      << "  encoder <0|1>      Set encoder health (1=OK, 0=Failed)\n"
                      << "  reset              Reset latched safety faults\n"
                      << "  restart            Reset whole simulation to t=0\n"
                      << "  tune <kp> <ki> <kd> Update PID gains\n"
                      << "  status             Show detailed current telemetry\n"
                      << "  benchmark          Run automated 30s benchmark\n"
                      << "  exit, quit         Exit the simulator\n\n";
        } else if (cmd == "set") {
            double rpm = 0.0;
            if (ss >> rpm) {
                setTargetRPM(rpm);
                std::cout << "Target set to " << targetRPM_ << " RPM.\n";
            } else {
                std::cout << "Usage: set <rpm> (e.g. set 1800)\n";
            }
        } else if (cmd == "load") {
            double torque = 0.0;
            if (ss >> torque) {
                setLoadTorque(torque);
                std::cout << "Load torque set to " << loadTorque_ << " N*m.\n";
            } else {
                std::cout << "Usage: load <Nm> (e.g. load 0.04)\n";
            }
        } else if (cmd == "step" || cmd == "run") {
            double sec = 2.0;
            if (!(ss >> sec)) sec = 2.0;
            stepInteractive(sec, 0.01, 0.5);
        } else if (cmd == "estop") {
            triggerEStop();
            std::cout << "EMERGENCY STOP TRIGGERED! Power cut to motor.\n";
        } else if (cmd == "encoder") {
            int ok = 1;
            if (ss >> ok) {
                setEncoderFailure(ok == 0);
                std::cout << "Encoder " << (ok ? "restored to normal" : "FAILURE INJECTED") << ".\n";
            } else {
                std::cout << "Usage: encoder 0 (fail) or encoder 1 (ok)\n";
            }
        } else if (cmd == "reset") {
            resetSafety();
            std::cout << "Safety system reset. Faults cleared.\n";
        } else if (cmd == "restart") {
            resetSimulation();
            std::cout << "Simulation reset to initial state (t=0).\n";
        } else if (cmd == "tune") {
            double kp, ki, kd;
            if (ss >> kp >> ki >> kd) {
                setPIDGains(kp, ki, kd);
                std::cout << "PID gains updated: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << "\n";
            } else {
                std::cout << "Usage: tune <kp> <ki> <kd> (current: "
                          << pid_.getGains().kp << " " << pid_.getGains().ki << " "
                          << pid_.getGains().kd << ")\n";
            }
        } else if (cmd == "status") {
            std::cout << "\n── Current Telemetry ────────────────────────\n"
                      << "  Simulated Time:   " << currentTime_ << " s\n"
                      << "  Target Speed:     " << targetRPM_ << " RPM\n"
                      << "  Actual Speed:     " << motor_.getRPM() << " RPM\n"
                      << "  Speed Error:      " << (targetRPM_ - motor_.getRPM()) << " RPM\n"
                      << "  Armature Current: " << motor_.getCurrent() << " A\n"
                      << "  Winding Temp:     " << motor_.getTemperature() << " C\n"
                      << "  Load Torque:      " << loadTorque_ << " N*m\n"
                      << "  Safety Tripped:   " << (safety_.isTripped() ? "YES (" + safety_.getFaultString() + ")" : "NO (Nominal)") << "\n"
                      << "─────────────────────────────────────────────\n\n";
        } else if (cmd == "benchmark") {
            run(30.0, 0.01, 0.5, Scenario::Nominal);
        } else {
            std::cout << "Unknown command '" << cmd << "'. Type 'help' for available commands.\n";
        }
    }
}
