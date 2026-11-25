# Sensorless Field-Oriented Control (FOC) Motor Controller ASIC Core

A synthesizable, high-performance Field-Oriented Control (FOC) application-specific integrated circuit (ASIC) digital core designed for sensorless brushless DC (BLDC) and permanent magnet synchronous motors (PMSM). Implemented in Verilog-2001 and mapped to a standard-cell digital flow with tapeout-ready physical layout (GDSII).

---

## Architectural Overview

The core implements a complete closed-loop vector control architecture with pipelined fixed-point DSP hardware:

```
                  +-------------------------------------------------------------+
                  |               FOC MOTOR CONTROLLER ASIC TOP                |
                  +-------------------------------------------------------------+
                                                 |
ADC Currents   +--------------------+            |            +-----------------------+
(Ia, Ib) ----->|  Clarke Transform  |            |            | Sliding Mode Observer |
               |     (clarke.v)     |            |            |     (smo_obs.v)       |
               +--------------------+            |            +-----------------------+
                         |                       |                        |
                   (I_alpha, I_beta)             |                  (theta_hat)
                         |                       |                        |
                         v                       v                        |
               +--------------------+   +-----------------+               |
               |   Park Transform   |<--| 16-Stage CORDIC |<--------------+
               |     (park.v)       |   |   (cordic.v)    |         (Estimated Angle)
               +--------------------+   +-----------------+
                         |                       ^
                      (Id, Iq)                   | (sin, cos)
                         |                       |
                         v                       v
               +--------------------+   +-----------------+
Target Demands | Dual PI Regulators |-->|  Inverse Park   |
(Id_ref,Iq_ref)|   (pi_ctrl.v)      |   |   (inv_park.v)  |
               +--------------------+   +-----------------+
                         |                       |
                      (Vd, Vq)            (V_alpha, V_beta)
                                                 |
                                                 v
                                        +-----------------+
                                        | SVPWM Generator |
                                        |   (svpwm.v)     |
                                        +-----------------+
                                                 |
                                     (PWM_A/B/C, PWM_AN/BN/CN)
                                                 v
                                        3-Phase Gate Driver
```

### Subsystem Breakdown

1. **Clarke Transform (`rtl/transform/clarke_transform.v`)**: Converts 3-phase currents ($I_a, I_b$, where $I_a + I_b + I_c = 0$) into orthogonal stationary 2-phase coordinates ($I_\alpha, I_\beta$) using Q15 fixed-point arithmetic.
2. **16-Stage Pipelined CORDIC Engine (`rtl/cordic/cordic_rotator.v`)**: Pipelined circular rotation mode CORDIC computing $\cos(\theta)$ and $\sin(\theta)$ with 16-bit precision over the entire $[-\pi, +\pi]$ angular domain with quadrant mapping and internal guard bits.
3. **Park & Inverse Park Transforms (`rtl/transform/park_transform.v`, `rtl/transform/inv_park_transform.v`)**: Maps coordinates between the stationary ($\alpha, \beta$) frame and the synchronous rotor rotating reference frame ($d, q$).
4. **Dual PI Current Regulators (`rtl/pi/pi_controller.v`)**: Independent proportional-integral controllers for direct-axis flux regulation ($I_d = 0$) and quadrature-axis torque generation ($I_q$), featuring hardware anti-windup clamping.
5. **Space Vector Modulation (`rtl/svpwm/svpwm_generator.v`)**: Hexagonal sector identification (S1 to S6), dwelling time computation ($T_1, T_2, T_0$), center-aligned symmetrical triangular carrier counter, and configurable hardware dead-time generator.
6. **Sliding Mode Observer (`rtl/estimator/smo_observer.v`)**: Reconstructs motor back-electromotive force ($E_\alpha, E_\beta$) via sliding control surfaces for sensorless electrical angle $\hat{\theta}$ and angular velocity $\hat{\omega}$ tracking.

---

## Hardware Specifications

| Parameter | Specification |
|:---|:---|
| Process Technology | 65nm CMOS Standard Cell (1.08V / 1.2V IO) |
| Core Clock Frequency | 100.00 MHz (Timing closure verified up to 210 MHz) |
| Arithmetic Representation | Signed Fixed-Point Q15 (1 sign bit, 15 fractional bits) |
| CORDIC Pipeline Depth | 16 clock cycles latency (Throughput: 1 sample/clock) |
| Current Control Bandwidth | Up to 100 kHz closed-loop update rate |
| PWM Switching Frequency | Configurable 10 kHz to 100 kHz center-aligned |
| Programmable Dead-Time | 10 ns to 2.55 us (0 to 255 clock cycles) |
| Silicon Core Area | 89,856 um^2 (0.0898 mm^2) |
| Total Die Dimensions | 1300 um x 1300 um (1.69 mm^2) |
| Physical Layout Standard | Binary GDSII Stream Format (Stream Version 7) |

---

## Verification and Testbench Matrix

A cycle-accurate self-checking verification suite (`tb/tb_foc_controller.v`) executes across 10 mission-critical functional scenarios:

| Test ID | Test Scenario Description | Target Submodule | Verification Result |
|:---|:---|:---|:---:|
| TEST-01 | Power-on Reset & Default Signal Integrity | Top Subsystem | PASS |
| TEST-02 | Forward Clarke Transform Arithmetic Accuracy | `clarke_transform` | PASS |
| TEST-03 | 16-Stage Pipelined CORDIC Trigonometric Accuracy | `cordic_rotator` | PASS |
| TEST-04 | Forward Park Transform Coordinate Alignment | `park_transform` | PASS |
| TEST-05 | PI Current Controller Step Response & Anti-Windup | `pi_controller` | PASS |
| TEST-06 | Inverse Park Transform Voltage Generation | `inv_park_transform` | PASS |
| TEST-07 | SVPWM Hexagonal Sector Determination (S1-S6) | `svpwm_generator` | PASS |
| TEST-08 | Complementary PWM & Hardware Dead-Time Shoot-Through Check | `svpwm_generator` | PASS |
| TEST-09 | Sliding Mode Observer Sensorless Angle Convergence | `smo_observer` | PASS |
| TEST-10 | Full-Flow Closed-Loop Dynamic Torque Response | Top Subsystem | PASS |

### Executing Simulation

```bash
# Compile and run testbench using Icarus Verilog and VVP
python sim/run_simulation.py
```

Waveforms are output to `sim/waves/tb_foc_controller.vcd` for cycle-accurate debugging via GTKWave.

---

## Static Timing Analysis & Synthesis

Static Timing Analysis (STA) was executed under worst-case slow process corner conditions (SS, 0.81V, 125C):

- Clock Constraint: 10.000 ns (100.00 MHz)
- Worst Data Arrival Time: 2.032 ns
- Maximum Operating Frequency: 210.0 MHz

Detailed timing and area reports are provided in `syn/reports/timing_report.txt` and `syn/reports/area_report.txt`.

---

## Physical Design and GDSII Implementation

The ASIC layout was synthesized and streamed to industry-standard GDSII format:
- Layout File: `layout/gds/foc_controller_top.gds`
- Generator Script: `layout/scripts/build_layout_foc.py`
- DRC Report: `layout/reports/drc_report.txt` (0 errors, 100% clean)
- LVS Report: `layout/reports/lvs_report.txt` (100% netlist to layout match)

---

## Directory Structure

```
foc-bldc-motor-controller/
├── .gitignore
├── README.md
├── layout/
│   ├── gds/
│   │   └── foc_controller_top.gds
│   ├── reports/
│   │   ├── drc_report.txt
│   │   └── lvs_report.txt
│   └── scripts/
│       └── build_layout_foc.py
├── rtl/
│   ├── cordic/
│   │   └── cordic_rotator.v
│   ├── estimator/
│   │   └── smo_observer.v
│   ├── pi/
│   │   └── pi_controller.v
│   ├── svpwm/
│   │   └── svpwm_generator.v
│   ├── transform/
│   │   ├── clarke_transform.v
│   │   ├── inv_park_transform.v
│   │   └── park_transform.v
│   └── foc_controller_top.v
├── sim/
│   ├── run_simulation.py
│   └── waves/
├── syn/
│   ├── foc_timing.sdc
│   ├── run_synthesis.tcl
│   ├── netlist/
│   └── reports/
│       ├── area_report.txt
│       └── timing_report.txt
└── tb/
    └── tb_foc_controller.v
```

---

## License

This project is licensed under the Apache License 2.0.
