@echo off
setlocal enabledelayedexpansion

echo ===================================================
echo   Building Motor Control Simulation (C++17)
echo ===================================================

:: Check for g++ in G:\Apps\w64devkit\bin or PATH
if exist "G:\Apps\w64devkit\bin\g++.exe" (
    set "PATH=G:\Apps\w64devkit\bin;!PATH!"
)

where g++ >nul 2>nul
if %ERRORLEVEL% equ 0 (
    echo Using G++ compiler...
    if not exist bin mkdir bin

    echo Compiling motor_sim.exe...
    g++ -std=c++17 -O3 -Wall -Wextra -Iinclude ^
        src\Motor.cpp ^
        src\Encoder.cpp ^
        src\PIDController.cpp ^
        src\SafetySystem.cpp ^
        src\MotorController.cpp ^
        src\main.cpp ^
        -o bin\motor_sim.exe

    if !ERRORLEVEL! neq 0 (
        echo Build failed for motor_sim.exe
        exit /b !ERRORLEVEL!
    )

    echo Compiling motor_tests.exe...
    g++ -std=c++17 -O3 -Wall -Wextra -Iinclude -Itests ^
        src\Motor.cpp ^
        src\Encoder.cpp ^
        src\PIDController.cpp ^
        src\SafetySystem.cpp ^
        src\MotorController.cpp ^
        tests\test_main.cpp ^
        tests\test_pid.cpp ^
        tests\test_safety.cpp ^
        -o bin\motor_tests.exe

    if !ERRORLEVEL! neq 0 (
        echo Build failed for motor_tests.exe
        exit /b !ERRORLEVEL!
    )

    echo ===================================================
    echo   Build Successful!
    echo   Executables:
    echo     - bin\motor_sim.exe
    echo     - bin\motor_tests.exe
    echo ===================================================
    exit /b 0
)

echo Error: Neither g++ nor a configured MSVC compiler was found.
exit /b 1
