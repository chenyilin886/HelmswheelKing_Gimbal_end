---
name: chassis-skill
description: Design, analyze, tune, and debug chassis control for RoboMaster-style mobile bases. Use when Codex needs to work on chassis kinematics, wheel speed decomposition, coordinate transforms, follow-yaw behavior, velocity loops, motion stabilization, driver feel tuning, or debugging code that controls omni-wheel, mecanum, or other multi-wheel chassis motion.
---

# Chassis Skill

Use this skill to work on chassis motion control from command generation to wheel execution.

## Workflow

1. Confirm the chassis type first.
   Distinguish mecanum, omni, differential, or custom wheel layout before touching equations.
2. Confirm the control frame.
   Separate body-frame commands from field-centric commands and verify yaw source.
3. Define the command path.
   Trace input from remote or planner to chassis velocity target, wheel targets, and motor loop output.
4. Check loop structure.
   Keep position, velocity, and current loops clearly separated. Avoid mixing frame transforms inside the inner loop.
5. Tune from low-risk motion to high-coupling motion.
   Test pure x, pure y, pure wz, then combined motion.

## Solve Chassis Motion

When handling a normal four-wheel omnidirectional chassis:

1. Convert operator intent into `vx`, `vy`, and `wz`.
2. Apply field-to-body transform first if using field-centric control.
3. Decompose body velocity into wheel targets with the correct geometry signs.
4. Normalize wheel commands if any wheel exceeds the actuator limit.
5. Feed wheel targets into the motor speed or current controller.

Keep sign conventions explicit. Most persistent bugs come from a wrong wheel order, wrong yaw sign, or swapped `vx` and `vy`.

## Tune Controller Behavior

Use this order:

1. Verify each motor direction and encoder polarity.
2. Tune each wheel speed loop in isolation.
3. Validate kinematic decomposition with low-speed commands.
4. Add yaw-follow or chassis-follow behavior.
5. Tune command filtering, acceleration limiting, and deadband.

Prefer rate limiting and smooth ramps over abrupt clipping when you need stable driver feel.

## Check Common Failure Modes

- Frame transform uses degrees in one place and radians in another.
- IMU yaw direction does not match chassis positive rotation definition.
- Wheel geometry constants do not match the physical base.
- Normalization destroys motion ratio because it is applied inconsistently.
- Speed loop is stable alone but unstable under coupled chassis motion due to saturation.
- Follow-yaw logic fights operator rotation command.
- Power limit and chassis control are tuned independently and interfere during acceleration.

## Debug In This Order

1. Log `vx`, `vy`, `wz`, yaw, wheel targets, wheel feedback, and output current.
2. Command single-axis motion and verify wheel symmetry.
3. Command pure rotation and verify all wheel signs.
4. Test combined translation and rotation.
5. Add power limiting only after baseline chassis control is correct.

## Use References

- Read [references/chassis-control.md](references/chassis-control.md) when you need a compact checklist for kinematics, frame handling, and tuning.

