---
name: power-skill
description: Analyze and implement chassis power control for RoboMaster-style mobile bases. Use when Codex needs to design, explain, tune, or debug power limiting, power buffer handling, motor current scaling, supercapacitor coordination, referee-system power constraints, or code paths that prevent chassis over-power during acceleration, collision, or continuous motion.
---

# Power Skill

Use this skill to reason about chassis power limiting as a closed-loop control problem instead of a single hard clamp.

## Workflow

1. Confirm the power constraint source first.
   Common sources are referee system chassis power limit, power buffer, capacitor state, battery sag, or a lab-specific software cap.
2. Identify the control insertion point.
   Prefer limiting the total chassis torque or wheel current command after kinematic decomposition and before final motor command output.
3. Distinguish steady-state limiting from transient protection.
   Use power prediction or buffer-aware scaling for transient motion. Avoid binary cutoffs that create oscillation.
4. Verify the feedback signals.
   Check chassis power, buffer energy, bus voltage, motor speed, estimated torque/current, and update rate.
5. Tune the limiter from outer to inner behavior.
   Set hard safety bounds first, then tune smooth scaling gain, recovery rate, and anti-windup handling.

## Apply A Practical Control Strategy

Prefer this sequence:

1. Compute target chassis motion from operator or planner command.
2. Convert chassis motion to wheel speed or current demand.
3. Estimate expected electrical or mechanical power.
4. Compare the estimate and measured power against the current limit and buffer state.
5. Scale all wheel torque-related outputs proportionally when exceeding the safe region.
6. Re-run saturation checks so no single motor exceeds its local limit.

Use proportional scaling across all drive motors when the goal is to preserve the commanded motion direction. Avoid clipping only one wheel unless handling a motor-specific fault.

## Check Common Failure Modes

Inspect these issues when the chassis feels weak or unstable:

- Limit applied before kinematic decomposition, causing wheel imbalance after rotation coupling.
- Limit based only on measured power, causing lag and repeated overshoot.
- No buffer-aware recovery logic, causing sudden drop after aggressive starts.
- PID integrator not reset or back-calculated after current saturation.
- Mixed units between current, torque constant, speed rpm, and rad/s.
- Power control executed at a slower rate than motor control.
- Supercapacitor state ignored even though the hardware path changes the available power envelope.

## Debug In This Order

1. Log commanded chassis velocity, wheel targets, wheel feedback, current command, measured chassis power, and power buffer.
2. Mark the exact sample where over-power starts.
3. Check whether the spike is prediction error, measurement delay, or actuator saturation.
4. Confirm whether the limiter reduces all drive outputs consistently.
5. Re-test with step input, ramp input, and diagonal plus spin combined motion.

## Use References

- Read [references/power-control.md](references/power-control.md) when you need a concise checklist for power-limiting logic, tuning, and code review.

