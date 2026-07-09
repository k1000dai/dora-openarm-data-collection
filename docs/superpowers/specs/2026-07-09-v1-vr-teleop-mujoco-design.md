# v1 VR Teleoperation (MuJoCo sim) — Design

Date: 2026-07-09
Status: Approved for planning

## Goal

Provide a minimal, teleoperation-only dataflow that runs the **OpenArm v1**
model in MuJoCo: VR controllers → IK → MuJoCo viewer. No recorder, no cameras,
no data-collection UI. This mirrors `nodes/dora-openarm-vr/config/dataflow-mujoco.yaml`
(the existing teleop-only sim example) but targets the v1 robot instead of v2.

"v1" here means the OpenArm v1 hardware generation (per
https://docs.openarm.dev/1.0/), whose MuJoCo assets live in
`enactic/openarm_mujoco` under `v1/`.

## Background / Key Findings

The IK node (`dora-openarm-ik`) and the viewer node (`dora-openarm-mujoco`) are
**fully `--xml`-driven**. The shared `openarm_control` package builds its
`JointResolver` from the loaded `MjModel` **at runtime**, resolving joints and
actuators by naming convention (`openarm_left_jointN`, `openarm_right_jointN`,
`{side}_jointN_ctrl`, `{side}_finger1_ctrl`). It "works with any MJCF that
follows the openarm_left_* / openarm_right_* naming convention."

The v1 `openarm_bimanual.xml` uses the **identical** naming convention as v2:

| Feature | v2 | v1 |
|---|---|---|
| Joint names (`openarm_{side}_jointN`, `..._finger_jointN`) | ✓ | ✓ same |
| Actuator names (`{side}_jointN_ctrl`, `{side}_finger1_ctrl`) | ✓ | ✓ same |
| EE frame | `{side}_ee_control_point` **site** | **missing** — has `openarm_{side}_hand_tcp` **body** |
| `home` keyframe | ✓ | **missing** |
| Distribution | pip wheel `openarm-mujoco==2.0.0` ships `v2/` | assets **only on GitHub**, not in any wheel |

Consequences:

- **No node code changes are required.** The default XML in `openarm_control`
  imports `openarm_mujoco.v2` only to compute a *default* path; we override it
  with `--xml`. The v2 `JointResolver` works on the v1 model because the naming
  matches.
- We must **vendor** the v1 assets locally (they are not pip-installable), and
  we must **patch** the v1 MJCF to add the two missing pieces (EE sites + home
  keyframe) so v1 presents the same interface the nodes expect.

## Scope

**In scope**

- Vendored, patched v1 MuJoCo model under `models/openarm_mujoco_v1/`.
- A new top-level teleop-only dataflow: `dataflow-vr-v1-mujoco.yaml`.

**Out of scope**

- Real v1 hardware (followers via `dora-openarm`).
- Recorder / dataset collection, cameras, cell lifter, data-collection UI.
- Publishing v1 as a pip package or modifying upstream nodes.

## Components

### 1. Vendored v1 model — `models/openarm_mujoco_v1/`

Contents fetched from `enactic/openarm_mujoco@master/v1/`:

- `openarm_bimanual.xml` — the robot (patched, see below).
- `scene.xml` — includes `openarm_bimanual.xml` and adds floor + lights +
  skybox. This is the file passed as `--xml` to both nodes.
- `meshes/` — the full `collision/` and `visual/` mesh tree, preserved with the
  same relative layout the XML's `meshdir="meshes"` expects.
- `fetch_v1_model.sh` — a small script that downloads the above from the pinned
  upstream ref, so the vendoring is reproducible and auditable.

**Patches to `openarm_bimanual.xml`** (v2 parity):

1. **EE sites.** Add `<site name="left_ee_control_point" .../>` and
   `<site name="right_ee_control_point" .../>` inside the
   `openarm_left_hand_tcp` / `openarm_right_hand_tcp` bodies (at local origin).
   These are the IK's default `--frame-right` / `--frame-left` targets.
2. **home keyframe.** Add `<keyframe><key name="home" qpos="..."/></keyframe>`
   with a neutral bent-elbow posture. The qpos pattern is adapted from v2's
   `home` keyframe (same 7-DOF-per-arm layout), verified to load without the
   "keyframe not found" warning.

Patches are applied to the vendored copy only; upstream is untouched.

### 2. New dataflow — `dataflow-vr-v1-mujoco.yaml`

Top-level file, following the structure and node build conventions of the
existing `dataflow-vr-mujoco.yaml` (`build: pip install -e nodes/...`). Nodes:

- `quittable-tick-leader` — `dora-openarm-quitter`, `tick: dora/timer/millis/2`.
- `udp-receiver` — `dora-openarm-vr` (`dora-openarm-quest-receiver`),
  `args: "--host 0.0.0.0 --port 5006"`, ticked by the leader.
- `ik` — `dora-openarm-ik`,
  `args: "--xml models/openarm_mujoco_v1/scene.xml --keyframe home --mode bimanual --max-iters 10 --dt 0.1 --damping 0.1 --posture-cost 0.01 --lm-damping 0.01"`.
  Inputs: tick, `target_right`/`target_left` (poses), `trigger_right`/`trigger_left`.
- `mujoco-viewer` — `dora-openarm-mujoco`,
  `args: "--xml models/openarm_mujoco_v1/scene.xml --keyframe home --viewer --debug-frames"`.
  Inputs: `position_left`/`position_right` from IK, `pose_right`/`pose_left` and
  `joystick_y` from the receiver (for debug frames), mirroring the v2 sim example.

No `ui`, `recorder`, camera, follower, or lifter nodes.

## Data Flow

```
Quest controllers ──UDP──> udp-receiver ──pose_right/left, trigger_right/left──> ik
                                                                                  │
                                              position_right/left (joint angles)  ▼
udp-receiver ──pose_right/left, joystick_y──────────────────────────────> mujoco-viewer (v1 scene)
```

Nothing is persisted.

## Error / Edge Handling

- **Missing meshes** → MuJoCo raises on `from_xml_path`. Mitigated by vendoring
  the complete `meshes/` tree and verifying model load in testing.
- **Missing `home` keyframe** → `openarm_control` prints a warning and falls
  back to defaults. The patch adds the keyframe so this does not occur.
- **Wrong EE frame name** → `ArmSetup` raises `Site '...' not found`. The patch
  adds the expected sites; the defaults (`{side}_ee_control_point`) then resolve.
- **No Quest connected** → the pipeline builds and the model loads, but no pose
  events flow, so the arms stay at `home`. Build + model-load is still a valid
  smoke test.

## Testing

1. `dora build dataflow-vr-v1-mujoco.yaml` — installs the (already-present)
   nodes; must succeed.
2. Model-load smoke check: load `models/openarm_mujoco_v1/scene.xml` with MuJoCo
   and confirm `home` keyframe + `{side}_ee_control_point` sites resolve and the
   `JointResolver` builds without error.
3. `dora run dataflow-vr-v1-mujoco.yaml` — MuJoCo viewer opens showing the v1
   bimanual arms at `home`.
4. With a Quest connected: pulling triggers moves the arms; controller motion
   tracks in sim. (Hardware-dependent; documented as manual verification.)

## Rollout

Single PR: vendored model directory + fetch script + new dataflow yaml + a short
README note pointing at the new teleop-only v1 sim config.
