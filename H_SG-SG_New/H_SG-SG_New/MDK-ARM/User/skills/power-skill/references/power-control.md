# Power Control Reference

## Checklist

- Confirm referee power limit, buffer limit, and any software fallback limit.
- Confirm whether the code limits current, torque, duty, or speed command.
- Confirm the limiter runs after chassis decomposition and before motor output.
- Confirm PID anti-windup exists when current is scaled down.
- Confirm the estimate uses consistent units.

## Recommended Signals

- Chassis measured power
- Chassis power buffer
- DC bus voltage
- Motor speed
- Motor current command
- Supercapacitor voltage or state flag

## Tuning Hints

- Start with conservative hard caps.
- Add smooth proportional scaling before the hard cap region.
- Use recovery hysteresis or recovery ramp to avoid chatter.
- Test forward rush, lateral rush, spin, and mixed translation plus rotation.

