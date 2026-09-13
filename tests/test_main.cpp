// test_main.cpp — Test runner
// Part of the Motor Control Simulation project.

#include "test_framework.h"

// Defined in test_pid.cpp and test_safety.cpp respectively.
int runPIDTests();
int runSafetyTests();

int main() {
    std::cout << "========================================\n";
    std::cout << "  Motor Control Simulation — Unit Tests\n";
    std::cout << "========================================\n";

    runPIDTests();
    runSafetyTests();

    return test::summarise();
}
