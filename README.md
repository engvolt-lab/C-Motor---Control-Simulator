# Motor Control Simulation

A professional C++17 simulation of closed-loop PID speed control for a brushed DC motor, featuring encoder feedback, thermal modelling, load disturbances, and a comprehensive safety/fault system.

> **Zero external dependencies.** Only the C++ Standard Library is used.

---

## Architecture

<p align="center">
  <img src="docs/architecture.svg" alt="DC Motor Closed-Loop Control System Architecture" width="100%">
</p>

### System Signal Flow

```
                 [ Operator Input: Target RPM (0-3000), Load, Faults ]
                                           │
                                           ▼
┌───────────────────────────────────────────────────────────────────────────────────────────┐
│  MotorController (Orchestrator)                                                           │
│                                                                                           │
│   r(t)        r_lim(t)      e(k)        V_cmd         V_in          ω, I, T               │
│  ──────▶ [ Slew Rate ] ──▶ (Σ) ──▶ [ PID ] ──▶ [ Gate ] ──▶ [ Motor ] ─────────┐          │
│            Limiter          ▲                                  │                │         │
│          (±800 RPM/s)       │ -                                │ ω              │ I, T    │
│                             │                                  ▼                │         │
│                             └────────── [ Encoder ] ◀──────────┘                │         │
│                                           y(k)         (1024 PPR + Noise)       │         │
│                                             │                                   ▼         │
│                                             └───────────────────────────▶ [ SafetySystem ]│
│                                                Tripped? Cut Voltage ◀───── (5 Monitors)   │
└───────────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
                [ Real-Time Telemetry Stream: Table / REPL / HTML Dashboard ]
```

### Architectural Subsystems & Mathematical Formulations

1. **Ramp Generator & Setpoint Shaper**:
   - Bounds acceleration to $\pm 800\text{ RPM/s}$ by default to prevent large step errors from commanding full $48\text{ V}$ rail voltage from stall, preventing false overcurrent trips while maintaining rapid dynamic response.

2. **Discrete PID Controller (`PIDController`)**:
   - **Proportional Term**: $P(k) = K_p \cdot e(k)$ provides immediate restorative torque.
   - **Integral Term with Anti-Windup**: $I(k) = \text{clamp}\left(I(k-1) + K_i \cdot e(k) \cdot \Delta t,\; -200,\; 200\right)$ eliminates steady-state load droop without integrator runaway.
   - **Derivative Term with Kick Avoidance**: $D(k) = K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}$, suppressed during the first sample after reset.
   - **Actuator Clamping**: Output command is clamped to $[0\text{ V},\; 48\text{ V}]$ (unidirectional H-bridge equivalent).

3. **Physical & Thermal Plant (`Motor`)**:
   - **Electrical Circuit**: Armature current $I = \max\left(0,\; \frac{V_{in} - K_e \cdot \omega}{R}\right)$, where $K_e = 0.01\text{ V}\cdot\text{s/rad}$ and $R = 1.0\ \Omega$.
   - **Rotor Dynamics**: Electromagnetic torque $\tau = K_t \cdot I$. Rotational acceleration $\dot{\omega} = \frac{\tau - \tau_{load} - B \cdot \omega}{J}$, with inertia $J = 0.001\text{ kg}\cdot\text{m}^2$ and viscous damping $B = 0.0001\text{ N}\cdot\text{m}\cdot\text{s/rad}$.
   - **Lumped Thermal Model**: Internal heating $P_{diss} = I^2 R$. Temperature evolution $\dot{T} = \frac{P_{diss} \cdot R_{th} - (T - T_{amb})}{\tau_{th}}$ with thermal resistance $R_{th} = 0.5\ ^\circ\text{C/W}$ and thermal time constant $\tau_{th} = 20\text{ s}$.

4. **Sensor Feedback Loop (`Encoder`)**:
   - Converts continuous mechanical speed $\omega$ to discrete pulses ($1024\text{ PPR}$).
   - Injects realistic zero-mean sensor noise ($\pm 0.5\text{ RPM}$) to test derivative noise sensitivity.
   - Supports software fault injection (`setFailure(true)`) modeling cable severance or sensor failure.

5. **Supervisory Safety Monitor (`SafetySystem`)**:
   - Polled on every control loop iteration ($10\text{ ms}$) across 5 independent protective monitors:
     - **Overcurrent Protection**: $I > 15.0\text{ A}$ (latches immediately on rotor stall or massive mechanical shock).
     - **Thermal Overload**: $T > 80.0\ ^\circ\text{C}$ (protects stator windings from insulation breakdown).
     - **Overspeed Protection**: $\omega > 3500\text{ RPM}$ (protects bearings from mechanical stress).
     - **Sensor Integrity**: Detects encoder signal failure.
     - **Hardware/Software Emergency Stop**: Instantaneous manual override.
   - **Latching Action**: When tripped, the safety system opens the power gate ($V_{in} = 0\text{ V}$), forcing the motor to coast safely down to a stop until an explicit `reset()` command is issued.

### Class Responsibilities

| Class | File | Responsibility |
|---|---|---|
| **`Motor`** | `Motor.h/cpp` | First-order DC motor physics: voltage → current → torque → angular velocity. Lumped thermal model tracks winding temperature. |
| **`Encoder`** | `Encoder.h/cpp` | Converts true angular velocity to a quantised, noisy RPM reading. Supports failure injection (output drops to zero). |
| **`PIDController`** | `PIDController.h/cpp` | Standard discrete PID with anti-windup (integral clamping), output clamping, and derivative-kick avoidance on the first sample. |
| **`SafetySystem`** | `SafetySystem.h/cpp` | Monitors current, temperature, RPM, and encoder health. Latches faults (like an industrial safety relay) until explicitly reset. |
| **`MotorController`** | `MotorController.h/cpp` | Top-level orchestrator. Owns all subsystems, runs the fixed-timestep loop, manages the setpoint/load profiles, and prints the telemetry table. |

---

## Control Algorithm

### PID Controller

The controller implements the textbook discrete PID:

```
e(k) = setpoint − measurement

P = Kp · e(k)
I = Ki · Σ e(k) · dt      (clamped to [integral_min, integral_max])
D = Kd · (e(k) − e(k−1)) / dt

output = clamp(P + I + D, output_min, output_max)
```

**Anti-windup:** The integral accumulator is clamped to prevent saturation during large setpoint changes or fault conditions. This avoids long recovery transients after the error sign reverses.

**Derivative kick avoidance:** On the first sample after construction or `reset()`, the derivative term is zeroed to avoid a spike from an uninitialised previous-error.

### Default Tuning

| Parameter | Value | Rationale |
|---|---|---|
| Kp | 0.01 V/RPM | Conservative proportional — prevents overcurrent during ramps |
| Ki | 0.05 V/(RPM·s) | Integral eliminates steady-state error within ~2 s |
| Kd | 0.001 V·s/RPM | Light derivative damps oscillation |
| Anti-windup | ±200 | Prevents integrator saturation |
| Output clamp | [0, 48] V | Matches supply voltage, unidirectional drive |

---

## Motor Model

### Electrical Dynamics (First-Order)

| Equation | Description |
|---|---|
| `V_back = Ke · ω` | Back-EMF proportional to speed |
| `I = (V_in − V_back) / R` | Armature current (clamped ≥ 0) |
| `τ_motor = Kt · I` | Electromagnetic torque |
| `dω/dt = (τ_motor − τ_load − B·ω) / J` | Rotational Newton's 2nd law |

### Thermal Model (Lumped)

```
dT/dt = (I²·R·Rth − (T − T_amb)) / τ_th
```

The winding temperature rises with I²R losses and decays toward ambient through a single thermal resistance.

### Default Parameters

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Armature resistance | R | 1.0 | Ω |
| Back-EMF constant | Ke | 0.01 | V·s/rad |
| Torque constant | Kt | 0.01 | N·m/A |
| Rotor inertia | J | 0.001 | kg·m² |
| Viscous friction | B | 0.0001 | N·m·s/rad |
| Thermal resistance | Rth | 2.0 | °C/W |
| Thermal time constant | τ_th | 20.0 | s |
| Ambient temperature | T_amb | 25.0 | °C |

---

## Safety System

| Fault | Condition | Threshold |
|---|---|---|
| Overcurrent | I > limit | 15 A |
| Overtemperature | T > limit | 80 °C |
| Overspeed | RPM > limit | 3500 RPM |
| Encoder failure | Encoder `hasFailed()` | Boolean |
| Emergency stop | External trigger | Boolean |

Faults are **latched** — once tripped, the system cuts voltage to zero and requires an explicit `reset()` call. This mirrors real industrial safety relay behaviour.

---

## Simulation Scenario (30 seconds)

| Time (s) | Setpoint | Event |
|---|---|---|
| 0 – 5 | Ramp 0 → 1000 RPM | Motor start-up |
| 5 – 10 | Hold 1000 RPM | Steady state |
| 10 – 12 | Ramp 1000 → 2000 RPM | Speed change |
| 12 – 18 | Hold 2000 RPM | **Load +0.05 N·m at t = 15 s** |
| 18 – 20 | Ramp 2000 → 3000 RPM | Full speed |
| 20 – 25 | Hold 3000 RPM | Max operating speed |
| 25 – 30 | Ramp 3000 → 0 RPM | Controlled deceleration |

---

## Building and Running

### Prerequisites

- A C++17 compiler (GCC, Clang, or MSVC)
- *(Optional)* CMake ≥ 3.16 or Make

### Quick Build (Windows)

Use the included build batch script which automatically detects GCC/G++ or MSVC:

```cmd
cd motor-control-sim
build.bat
```

### Build with Make (Linux / macOS / MinGW)

```bash
make
```

### Build with CMake

```bash
cmake -B build -S .
cmake --build build
```

---

## Running the Simulator (Interactive Inputs & Outputs)

The simulator provides multiple ways to give inputs (target RPM, load torque, duration, fault actions, PID gains) and inspect live outputs:

### 1. Interactive Menu Mode (Default)

Running without arguments opens the main interactive menu:

```cmd
.\bin\motor_sim.exe
```

```
===========================================================
   CLOSED-LOOP DC MOTOR CONTROL SIMULATOR (C++17)
===========================================================
  Select an operating mode:

  [1] Interactive Guided Mode (Input Target RPM, Load, Time)
  [2] Interactive Command Shell (REPL for live controls)
  [3] Automated 30-Second Benchmark (0 -> 3000 -> 0 RPM)
  [4] Safety Fault Demonstrations (Overcurrent, E-Stop, etc.)
  [5] Exit
===========================================================
Enter selection [1-5]:
```

---

### 2. Guided Input Mode (`-i` or `--interactive`)

Directly prompts you for Target RPM, Load Torque, Duration, and Fault Actions, runs the simulation, prints the telemetry table, and lets you continue or modify inputs:

```cmd
.\bin\motor_sim.exe -i
```

**Prompts:**
- `Enter Target Speed (0 to 3000 RPM):` e.g. `2400`
- `Enter Load Torque in N*m (0.00 to 0.20):` e.g. `0.04`
- `Enter Duration to simulate in seconds:` e.g. `6.0`
- `Optional Action [0=None, 1=Reset Faults, 2=Trigger E-Stop, 3=Fail Encoder, 4=Reset Sim]:` e.g. `0`

Outputs the live ASCII table and step summary.

---

### 3. Interactive Command Shell REPL (`-s` or `--shell`)

A persistent live shell where you can type commands to control the motor step by step:

```cmd
.\bin\motor_sim.exe -s
```

**Commands:**
| Command | Description | Example |
|---|---|---|
| `set <rpm>` | Set target speed (0 - 3000 RPM) | `set 2200` |
| `load <torque>` | Set external load torque in N·m | `load 0.05` |
| `step <seconds>` | Simulate forward for N seconds with telemetry table | `step 4` |
| `run <seconds>` | Alias for `step` | `run 5` |
| `estop` | Trigger Emergency Stop | `estop` |
| `encoder <0\|1>` | Inject/clear encoder failure (0 = fail, 1 = OK) | `encoder 0` |
| `reset` | Clear latched safety faults | `reset` |
| `restart` | Reset motor & simulation state to t=0 | `restart` |
| `tune <kp> <ki> <kd>` | Modify PID controller gains | `tune 0.02 0.08 0.002` |
| `status` | Print current telemetry summary | `status` |
| `benchmark` | Run 30s benchmark scenario | `benchmark` |
| `exit` | Quit shell | `exit` |

---

### 4. Direct Command-Line Input Mode

Provide parameters directly as CLI arguments:

```cmd
.\bin\motor_sim.exe --target 2400 --load 0.04 --time 6
```

---

### 5. Visual Web Dashboard (`simulator.html`)

Open [`simulator.html`](simulator.html) directly in any web browser (no server needed):
- Interactive sliders for **Target Speed** (0–3500 RPM), **Load Torque** (0–0.25 N·m), and **Acceleration Ramp Rate**.
- Interactive buttons for **Emergency Stop**, **Encoder Failure**, **Overload**, and **Reset**.
- Live **gauges** and **dual-trace real-time graphs** showing Target vs Actual RPM and Current.

---

## Running Unit Tests

Run the zero-dependency test runner containing **34 assertions** across the PID controller and safety subsystem:

```cmd
# Direct binary
.\bin\motor_tests.exe

# Via Make
make test

# Via CTest (if built with CMake)
cd build && ctest --verbose
```

---

## Project Structure

```
motor-control-sim/
├── CMakeLists.txt              # Build system
├── README.md                   # This file
├── include/
│   ├── Motor.h                 # DC motor model
│   ├── Encoder.h               # Rotary encoder
│   ├── PIDController.h         # PID controller
│   ├── SafetySystem.h          # Fault monitoring
│   └── MotorController.h       # Simulation orchestrator
├── src/
│   ├── Motor.cpp
│   ├── Encoder.cpp
│   ├── PIDController.cpp
│   ├── SafetySystem.cpp
│   ├── MotorController.cpp
│   └── main.cpp                # Entry point
└── tests/
    ├── test_framework.h        # Minimal assertion macros
    ├── test_pid.cpp            # PID controller tests (10 cases)
    ├── test_safety.cpp         # Safety system tests (11 cases)
    └── test_main.cpp           # Test runner
```

---

## Assumptions

1. **First-order electrical model** — Armature inductance is neglected (valid when the control-loop timestep ≫ L/R time constant).
2. **Unidirectional drive** — Current and speed are clamped to ≥ 0 (no regenerative braking or reverse rotation).
3. **Lumped thermal model** — A single thermal mass represents the entire winding; no spatial temperature gradients.
4. **Ideal power supply** — Unlimited current sourcing capability; voltage is the only controlled quantity.
5. **No PWM modelling** — The control signal is a continuous voltage, not a duty-cycle-modulated waveform.
6. **Forward Euler integration** — Acceptable for the 10 ms timestep and the system's time constants.

## Limitations

- No field-weakening region above base speed.
- No cogging torque or torque ripple.
- No bearing or windage losses (only viscous friction).
- No sensor delay or communication latency.
- Encoder noise model is uniform, not Gaussian.
- No closed-loop current control (only speed loop).
- No state-space or observer-based control (classical PID only).

---

## License

This project is provided for educational and demonstration purposes.
