// main.cpp — Entry point with interactive input/output simulation
// Part of the Motor Control Simulation project.

#include "MotorController.h"

#include <iostream>
#include <string>

void printMainMenu() {
    std::cout << "\n"
        "===========================================================\n"
        "   CLOSED-LOOP DC MOTOR CONTROL SIMULATOR (C++17)\n"
        "===========================================================\n"
        "  Select an operating mode:\n\n"
        "  [1] Interactive Guided Mode (Input Target RPM, Load, Time)\n"
        "  [2] Interactive Command Shell (REPL for live controls)\n"
        "  [3] Automated 30-Second Benchmark (0 -> 3000 -> 0 RPM)\n"
        "  [4] Safety Fault Demonstrations (Overcurrent, E-Stop, etc.)\n"
        "  [5] Exit\n"
        "===========================================================\n"
        "Enter selection [1-5]: ";
}

void runFaultDemoMenu(MotorController& controller) {
    std::cout << "\nChoose a safety fault demonstration:\n"
              << "  [1] Overcurrent Fault (Current > 15 A under heavy load)\n"
              << "  [2] Overtemperature Fault (Temp > 80 C under sustained load)\n"
              << "  [3] Overspeed Fault (RPM > 3500)\n"
              << "  [4] Encoder Sensor Failure (Signal loss)\n"
              << "  [5] Emergency Stop (Immediate external trip)\n"
              << "Selection [1-5]: ";
    std::string choice;
    if (std::getline(std::cin, choice)) {
        if (choice == "1") controller.run(30.0, 0.01, 0.5, MotorController::Scenario::OvercurrentTest);
        else if (choice == "2") controller.run(30.0, 0.01, 0.5, MotorController::Scenario::OvertemperatureTest);
        else if (choice == "3") controller.run(30.0, 0.01, 0.5, MotorController::Scenario::OverspeedTest);
        else if (choice == "4") controller.run(30.0, 0.01, 0.5, MotorController::Scenario::EncoderFailureTest);
        else if (choice == "5") controller.run(30.0, 0.01, 0.5, MotorController::Scenario::EmergencyStopTest);
        else std::cout << "Invalid selection.\n";
    }
}

int main(int argc, char* argv[]) {
    MotorController controller;

    // Command-line flag dispatch
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--interactive" || arg == "-i") {
            controller.runInteractiveWizard();
            return 0;
        } else if (arg == "--shell" || arg == "-s") {
            controller.runInteractiveREPL();
            return 0;
        } else if (arg == "--auto" || arg == "--benchmark" || arg == "-a") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::Nominal);
            return 0;
        } else if (arg == "--overcurrent") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::OvercurrentTest);
            return 0;
        } else if (arg == "--overtemp") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::OvertemperatureTest);
            return 0;
        } else if (arg == "--overspeed") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::OverspeedTest);
            return 0;
        } else if (arg == "--encoder-fail") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::EncoderFailureTest);
            return 0;
        } else if (arg == "--estop") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::EmergencyStopTest);
            return 0;
        } else if (arg == "--target" && argc >= 3) {
            // Batch command input: e.g. motor_sim --target 2200 [--load 0.05] [--time 5]
            double target = std::stod(argv[2]);
            double load = 0.01;
            double duration = 5.0;

            for (int i = 3; i < argc; ++i) {
                std::string flag = argv[i];
                if (flag == "--load" && i + 1 < argc) {
                    load = std::stod(argv[++i]);
                } else if (flag == "--time" && i + 1 < argc) {
                    duration = std::stod(argv[++i]);
                }
            }
            controller.setTargetRPM(target);
            controller.setLoadTorque(load);
            controller.stepInteractive(duration, 0.01, 0.5);
            return 0;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "DC Motor Control Simulator (C++17)\n"
                      << "Usage:\n"
                      << "  motor_sim                Interactive menu (default)\n"
                      << "  motor_sim -i, --interactive Guided input mode\n"
                      << "  motor_sim -s, --shell       Interactive command shell\n"
                      << "  motor_sim -a, --auto        Run 30s automated benchmark\n"
                      << "  motor_sim --target <RPM> [--load <Nm>] [--time <s>]  Direct batch input\n"
                      << "  motor_sim --overcurrent     Run overcurrent fault demo\n"
                      << "  motor_sim --overtemp        Run overtemperature fault demo\n"
                      << "  motor_sim --overspeed       Run overspeed fault demo\n"
                      << "  motor_sim --encoder-fail    Run encoder failure demo\n"
                      << "  motor_sim --estop           Run emergency stop demo\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n"
                      << "Run 'motor_sim --help' for options.\n";
            return 1;
        }
    }

    // Default: Interactive Menu
    while (true) {
        printMainMenu();
        std::string choice;
        if (!std::getline(std::cin, choice)) break;

        if (choice == "1") {
            controller.runInteractiveWizard();
        } else if (choice == "2") {
            controller.runInteractiveREPL();
        } else if (choice == "3") {
            controller.run(30.0, 0.01, 0.5, MotorController::Scenario::Nominal);
        } else if (choice == "4") {
            runFaultDemoMenu(controller);
        } else if (choice == "5" || choice == "exit" || choice == "quit" || choice == "q") {
            std::cout << "Exiting simulator. Goodbye!\n";
            break;
        } else {
            std::cout << "Invalid option. Please choose 1 to 5.\n";
        }
    }

    return 0;
}
