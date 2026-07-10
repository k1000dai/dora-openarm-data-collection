# Data collection configurations for OpenArm with dora-rs

This repository provides data collection configurations for [OpenArm](https://openarm.dev/) with [dora-rs](https://dora-rs.ai/).

## Configurations

[`metadata.yaml`](metadata.yaml) is metadata used by all configurations.

### Real configuration

TODO

### Dummy configuration

[`dataflow_dummy.yaml`](dataflow_dummy.yaml) is a configuration that doesn't use real OpenArm. We can use this for testing a dataflow without real OpenArm.

### VR teleoperation (OpenArm v1, MuJoCo)

[`dataflow-vr-v1-mujoco.yaml`](dataflow-vr-v1-mujoco.yaml) is a teleoperation-only
configuration (no recorder or cameras) that drives the **OpenArm v1** model in
MuJoCo from Meta Quest controllers: VR → IK → MuJoCo viewer.

The v1 MuJoCo assets are vendored under [`models/openarm_mujoco_v1/`](models/openarm_mujoco_v1)
(patched with `*_ee_control_point` sites and a `home` keyframe). The binary mesh
tree is not committed — fetch it once before running:

```bash
models/openarm_mujoco_v1/fetch_v1_model.sh
dora build dataflow-vr-v1-mujoco.yaml
dora run dataflow-vr-v1-mujoco.yaml
```

### VR teleoperation (OpenArm v1, real robot)

[`dataflow-vr-v1.yaml`](dataflow-vr-v1.yaml) drives the **real OpenArm v1** arms
from Meta Quest controllers, teleoperation-only (no recorder, cameras, or
dataset), with a MuJoCo viewer kept alongside as a live digital twin:
VR → IK → `dora-openarm-gripper-adapter` → real followers (+ MuJoCo twin).

The v1 gripper is a parallel-jaw slide (meters) driven by the rotary gripper
motor (radians); the
[`dora-openarm-gripper-adapter`](nodes/dora-openarm-gripper-adapter) node
rescales the IK gripper element into the gripper-motor convention so the gripper
and the follower's `--align-trigger gripper` interlock behave exactly like v2
(**squeeze a trigger to arm the alignment ramp**).

The follower hardware config lives in [`configs/openarm_v1.yaml`](configs/openarm_v1.yaml).
Its `joint_offsets` are `0`: the OpenArm hardware uses an identity motor↔joint
mapping and the v1 motor 0-pose is aligned to the v1 model 0-pose, so no offset
is needed (assuming the motors are zeroed per the standard OpenArm setup). The
step-limited alignment ramp keeps first bring-up safe.

```bash
models/openarm_mujoco_v1/fetch_v1_model.sh
# Bring up CAN (can0/can1) for the two arms (standard OpenArm motor zeroing).
dora build dataflow-vr-v1.yaml
dora run dataflow-vr-v1.yaml
```

> **Safety:** the arms auto-start on boot but the alignment ramp is gated — the
> arms only move once you squeeze a trigger. Keep the workspace clear and an
> e-stop reachable, and on the first run confirm each joint moves the right
> direction (a model/hardware sign flip looks fine in sim but reversed on
> hardware; see the config header).

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
