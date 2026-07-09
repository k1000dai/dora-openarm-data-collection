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

## License

Licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

Copyright 2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
