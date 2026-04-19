## Overview

This repository documents the kinematic modeling and error analysis of a differential drive mobile robot operating under open-loop control. The system is intentionally designed without encoder feedback to examine how ideal motion models behave in the presence of mechanical non-idealities.

The central question this project investigates is not how to achieve precision, but rather how and why theoretical motion models degrade when their underlying assumptions are violated in physical execution.

> **Scope:** Kinematic modeling, teleoperation, and simulation-based error analysis.
> Autonomous navigation, localization, and closed-loop control are explicitly out of scope.

---

## Core Concepts

### What is a Differential Drive Robot?

A differential drive robot consists of two independently driven wheels mounted on a common axis. Motion is generated entirely through velocity differentials between the two wheels. There is no steering mechanism; direction is controlled by varying the relative speed of each wheel.

This architecture is mechanically simple and analytically tractable, which makes it a canonical platform for studying mobile robot kinematics. However, the system is subject to **nonholonomic constraints**, meaning it cannot move instantaneously in the lateral (sideways) direction. All motion must occur along the robot's longitudinal axis.

### Open-Loop vs. Closed-Loop Control

In a closed-loop system, sensors such as wheel encoders continuously measure the robot's actual state and feed corrections back into the controller. In an open-loop system, no such feedback exists. The controller issues commands and assumes they are executed perfectly.

This project deliberately operates open-loop. The purpose is to expose the gap between the ideal kinematic model and physical reality, rather than mask it through correction.

### Nonholonomic Constraints

The robot is modeled as a planar rigid body. The nonholonomic constraint requires that the velocity component perpendicular to the wheel axis is zero at all times:

```
lateral velocity in body frame = 0
```

This constraint arises from the rolling condition of the wheels, which prohibits instantaneous sideways slip. It restricts the reachable configurations of the robot and introduces complexity in both modeling and trajectory planning.

---

## Kinematics

### Reference Frames

The robot operates in two frames:

- **Global frame {X, Y}**: Fixed to the world, with origin O.
- **Body frame {X_B, Y_B}**: Fixed to the midpoint of the wheel axle, oriented along the robot's heading.

The robot's pose is described by the tuple `(x, y, phi)`, where `phi` is the angle between the global X axis and the body axis X_B.

### Forward Kinematics

Given wheel radii `r`, track width `b`, and wheel angular velocities `omega_r` (right) and `omega_l` (left), the linear velocities of each wheel under the assumption of pure rolling are:

```
v_R = r * omega_r
v_L = r * omega_l
```

The forward velocity of the robot's center and its angular velocity are:

```
v   = (v_R + v_L) / 2  =  (r / 2) * (omega_r + omega_l)
w   = (v_R - v_L) / b  =  (r / b) * (omega_r - omega_l)
```

The radius of curvature (distance from the robot center to the Instantaneous Center of Rotation):

```
R = v / w = (b / 2) * (v_R + v_L) / (v_R - v_L)
```

When `v_R = v_L`, R approaches infinity and the robot moves in a straight line. When `v_R != v_L`, the robot follows a circular arc centered at the ICR.

### Global Frame Propagation

To express motion in the world frame, the body-frame velocity is projected through the robot's orientation:

```
x_dot   = v * cos(phi)
y_dot   = v * sin(phi)
phi_dot = w
```

These equations are nonlinear due to the trigonometric dependence on `phi`. They form the basis for discrete-time pose propagation using forward Euler integration:

```
x[k+1]   = x[k]   + v[k] * cos(phi[k]) * dt
y[k+1]   = y[k]   + v[k] * sin(phi[k]) * dt
phi[k+1] = phi[k] + w[k] * dt
```

### Inverse Kinematics

To compute the wheel velocities required for a desired motion `(v, w)`:

```
omega_r = (2v + w*b) / (2r)
omega_l = (2v - w*b) / (2r)
```

This mapping is only valid under the assumptions of pure rolling and symmetric actuation. Violations of these assumptions shift the effective ICR and introduce trajectory errors.

### Worked Example

```
r = 0.05 m,  b = 0.20 m
omega_r = 10 rad/s,  omega_l = 6 rad/s

v_R = 0.05 * 10 = 0.5 m/s
v_L = 0.05 *  6 = 0.3 m/s

v   = (0.5 + 0.3) / 2       = 0.4 m/s
w   = (0.5 - 0.3) / 0.20    = 1.0 rad/s
R   =  0.4 / 1.0             = 0.4 m
```

The robot follows a circular arc of radius 0.4 m centered at the ICR.

---

## Backlash Modeling and Error Analysis

### What is Backlash?

Backlash is mechanical slack in the wheel-shaft interface. It manifests as a deadband: below a threshold angular displacement, no torque is transmitted and the wheel does not respond. This is modeled as a nonlinear transformation on the commanded wheel velocity:

```
omega_eff = 0,                              if |omega| <= theta_b
omega_eff = omega - sgn(omega) * theta_b,   if |omega| >  theta_b
```

In this study, `theta_b = 15 degrees = 0.2618 rad`.

### Effect on Velocity

Since both wheel commands satisfy `|omega| >> theta_b`, the backlash acts as a constant offset subtracted from each wheel:

```
omega_r_eff = omega_r - theta_b
omega_l_eff = omega_l - theta_b
```

This has a critical structural consequence. Because the offset is applied equally to both wheels, the **velocity difference** `(omega_r - omega_l)` is preserved and the angular velocity `w` remains unchanged. However, the **velocity sum** `(omega_r + omega_l)` decreases, reducing the forward velocity `v`.

```
w is preserved    (difference unchanged)
v is reduced      (sum decreases)
```

Since the turning radius is `R = v / w`, a reduction in `v` with constant `w` contracts the radius of curvature. The robot turns tighter than the ideal model predicts.

### Positional Error Metric

The positional divergence between the ideal and backlash-affected trajectories is quantified as Euclidean distance:

```
e(t_k) = sqrt( (x_i - x_b)^2 + (y_i - y_b)^2 )
```

where subscripts `i` and `b` denote the ideal and backlash trajectories respectively.

### Simulation Results

The discrete-time simulation (forward Euler, constant inputs `omega_r = 10 rad/s`, `omega_l = 8 rad/s`) yields the following observations:

- The ideal model produces a clean circular arc consistent with kinematic predictions.
- The backlash-affected trajectory deviates inward, tracing a smaller arc due to reduced forward velocity.
- Positional error grows rapidly in the early phase, peaks near t = 7-8 seconds, then partially stabilizes as the two trajectories geometrically converge due to their shared angular velocity.
- The accumulated positional discrepancy remains non-negligible throughout the simulation window.

This demonstrates that backlash is not stochastic noise. It is a **deterministic, structured disturbance** with predictable and analytically traceable effects on trajectory geometry.

---

## Key Insights

### Kinematic correctness does not imply control reliability

The forward and inverse kinematic equations are mathematically exact under their stated assumptions. The model fails not because the math is wrong, but because physical systems violate those assumptions consistently. Pure rolling, symmetric actuation, and zero lateral slip are idealizations that real hardware does not satisfy.

### Open-loop systems accumulate unbounded error

Without feedback, deviations from the ideal trajectory compound over time. There is no mechanism to detect or correct drift. Even minor asymmetries in motor response or surface friction produce errors that grow without bound during sustained operation.

### Backlash decouples linear and angular velocity degradation

The uniform nature of the backlash offset means angular velocity is unaffected while linear velocity is reduced. This asymmetric degradation is not immediately obvious and leads to a systematic contraction of the turning radius. It illustrates how mechanical non-idealities can produce nuanced, structured effects rather than uniform degradation across all motion components.

### The model is locally valid, globally unreliable

Over short durations and small displacements, the kinematic model agrees well with observed behavior. The failure mode is temporal: error accumulates over extended operation. Open-loop control is acceptable for brief, low-precision maneuvers but fundamentally inadequate for sustained navigation tasks.

### Sensing is a structural requirement, not an enhancement

The deliberate removal of encoder feedback in this system exposes what closed-loop systems routinely hide: the kinematic model alone cannot maintain positional accuracy. State estimation, odometry correction, and sensor fusion are not optional improvements to a working system. They are the mechanism by which the system works at all.

---

## System Architecture

The hardware stack is organized into three functional layers:

**Computation:** ESP32 microcontroller receives teleoperation commands over WiFi (HTTP/WebSocket) and generates PWM motor commands via inverse kinematics.

**Actuation:** L298N motor driver converts low-power control signals to high-current drive for the left and right DC motors.

**Sensing (observational only):** MPU6050 IMU records orientation data post-hoc. It is explicitly not used for feedback control. Three ultrasonic sensors are present but similarly excluded from the control loop.

The system is powered by a 2x 18650 battery pack at 7.4V, with onboard voltage regulation to 5V and 3.3V rails.

### Implementation Notes

- Joystick commands are mapped to desired `(v, w)` pairs, then converted to wheel velocities via inverse kinematics.
- PWM duty cycle is assumed proportional to wheel angular velocity. This is an approximation; real motors exhibit nonlinear behavior under varying load and voltage.
- No calibration or system identification is performed, by design.

---

## Planned Extensions

The following additions are planned to transition the system toward a closed-loop autonomous platform:

- **Wheel encoders** for odometry-based pose correction
- **IMU integration** for orientation estimation and drift compensation
- **Sensor fusion** combining encoder odometry and IMU data into a consistent state estimate
- **Visual localization** using camera-based marker tracking
- **SLAM implementation** for simultaneous mapping and localization in unknown environments

---

## References

1. S. Thrun, W. Burgard, and D. Fox, _Probabilistic Robotics_, MIT Press, 2005.
2. R. Siegwart, I. Nourbakhsh, and D. Scaramuzza, _Introduction to Autonomous Mobile Robots_, 2nd ed., MIT Press, 2011.
3. B. Siciliano et al., _Robotics: Modelling, Planning and Control_, Springer, 2009.
4. J. Borenstein and L. Feng, "Measurement and correction of systematic odometry errors in mobile robots," _IEEE Transactions on Robotics and Automation_, vol. 12, no. 6, 1996.
5. N. Sarkar, X. Yun, and V. Kumar, "Control of mechanical systems with backlash," _IEEE Transactions on Robotics and Automation_, vol. 10, no. 6, 1994.
6. H. Durrant-Whyte and T. Bailey, "Simultaneous localization and mapping (SLAM)," _IEEE Robotics & Automation Magazine_, vol. 13, 2006.

---
