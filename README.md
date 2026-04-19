# open-loop-robotics-lab

> Most beginner robotics projects teach you how to make a robot move.  
> This one shows you why it **doesn’t**.

---

## What is this?

This is a deliberately constrained differential drive robot built as a learning system.

No encoders.  
No feedback.  
No corrections hiding in the background.

Just kinematics, pushed into the real world until it starts breaking.

---

## Why this exists

Differential drive is often sold as simple.

Two wheels.  
A couple equations.  
Done.

But those equations assume:

- perfect rolling
- no slip
- identical motors
- linear response

Remove feedback and those assumptions collapse.

This project isolates that collapse.

---

## What you will learn

- Why correct equations still produce wrong motion
- How error accumulates in open-loop systems
- Why feedback is not optional in robotics
- Where theory diverges from physical systems

---

## Core Idea

> Kinematic correctness does not imply control reliability.

---

## System Overview

Minimal by design:

- ESP32 for control
- L298N for motor driving
- Dual DC motors
- Li-ion power
- IMU for observation only

Nothing here is trying to fix the system.

---

## What actually happens

At first, everything looks right.

Then:

- straight lines drift
- turns don’t close
- trajectories deform

The robot does exactly what you told it to do.  
Reality just interprets it differently.

---

## Repository Structure (80/20)

```
.
├── README.md
├── firmware/        # ESP32 control code
├── hardware/        # wiring + components
├── experiments/     # tests, results, observations
├── docs.md          # concepts + kinematics + insights
├── simulations/     # optional modeling
├── report.pdf       # full academic report
```

No clutter. Everything has a purpose.

---

## How to use this

Two modes:

### Read

Go through `docs.md` and understand what is happening.

### Build

Replicate it and test:

- Run straight for 10 seconds → measure drift
- Apply slight PWM mismatch → observe curvature
- Compare expected vs actual motion

If your robot behaves perfectly, you missed something.

---

## Experiments

Focused and repeatable:

- Straight line drift
- Turning radius deviation
- Long run error accumulation

Each experiment answers:

> What should happen vs what actually happens

---

## Simulations

Used to contrast:

- ideal motion
- real-world deviation

Not to prove the model is correct, but to show where it fails.

---

## Where this goes next

- Add encoders → odometry
- Use IMU → orientation estimation
- Introduce feedback → control
- Move toward SLAM

This repo stops right before things start working properly.

---

## Final Note

This is not a clean demo.

It is a controlled failure.

And if you understand it, you understand mobile robotics far better than someone who just made a robot move.
