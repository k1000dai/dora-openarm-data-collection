# dora-openarm-gripper-adapter

A [Dora](https://dora-rs.ai/) node that rescales the gripper element of an
OpenArm IK position from the **v1 MuJoCo model's units (prismatic meters)** to
the **gripper-motor's units (radians)** used by the real followers.

The OpenArm v1 gripper is a parallel-jaw slide (0–0.044 m) driven by a rotary
DM4310 motor. The v1 IK (`dora-openarm-ik`) emits the 8th joint value as a
prismatic finger position in meters, but the `dora-openarm` follower forwards
that value to the gripper motor as radians. This node bridges the two by
rescaling the gripper element into the v2 convention (`closed 0`, `open ∓0.785
rad`), so the real gripper opens/closes and the follower's `--align-trigger
gripper` interlock behaves exactly like v2. Arm joints 1–7 pass through
unchanged.

## Usage

```yaml
nodes:
  # ...
  - id: gripper-adapter
    build: pip install -e nodes/dora-openarm-gripper-adapter
    path: dora-openarm-gripper-adapter
    args: "--in-open 0.044 --out-open-right -0.785 --out-open-left 0.785"
    inputs:
      position_right: ik/position_right
      position_left: ik/position_left
    outputs:
      - position_right
      - position_left
```

### Node arguments

| Argument | Description |
| --- | --- |
| `--in-open` | v1 model finger open position in meters. Default: `0.044`. |
| `--out-open-right` | Right gripper motor angle at full open, in radians. Default: `-0.785`. |
| `--out-open-left` | Left gripper motor angle at full open, in radians. Default: `0.785`. |

The full-open radian is a calibration knob: tune `--out-open-{right,left}` to the
real jaw's fully-open motor angle, and confirm the sign per side (a released
trigger should open the jaw, not close it).

### Inputs

| Input | Description |
| --- | --- |
| `position_right` | Right arm IK position, float32[8], gripper in meters. |
| `position_left` | Left arm IK position, float32[8], gripper in meters. |

### Outputs

| Output | Description |
| --- | --- |
| `position_right` | Right arm position, float32[8], gripper rescaled to radians. |
| `position_left` | Left arm position, float32[8], gripper rescaled to radians. |

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.
