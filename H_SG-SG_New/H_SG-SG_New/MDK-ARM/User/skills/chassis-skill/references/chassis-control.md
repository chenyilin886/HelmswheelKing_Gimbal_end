# Chassis Control Reference

## Checklist

- Confirm chassis type and wheel order.
- Confirm body frame and world frame definitions.
- Confirm IMU yaw sign and zero reference.
- Confirm `vx`, `vy`, and `wz` units are consistent.
- Confirm normalization is applied after wheel target calculation.

## Minimum Debug Signals

- Commanded `vx`, `vy`, `wz`
- Measured yaw
- Wheel target speeds
- Wheel measured speeds
- Motor current commands

## Tuning Hints

- Tune one wheel loop first, then replicate.
- Validate pure translation and pure rotation separately.
- Add command ramping if the chassis feels abrupt.
- Re-check power limiting after chassis loop tuning because the two layers interact.
