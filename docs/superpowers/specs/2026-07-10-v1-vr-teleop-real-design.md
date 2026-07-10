# v1 VR Teleoperation (real robot) — Design

Date: 2026-07-10
Status: Approved for planning

## Goal

Drive the **real OpenArm v1** bimanual arms from Meta Quest controllers,
teleoperation-only (no recorder, cameras, or dataset), with a MuJoCo viewer kept
alongside as a live digital twin for monitoring. This is the hardware analog of
[`dataflow-vr-v1-mujoco.yaml`](../../../dataflow-vr-v1-mujoco.yaml): the same
VR → IK front end (v1 model), with the MuJoCo *sink* replaced by two real
`dora-openarm` followers — i.e. the v1 counterpart of the v2
[`dataflow-vr.yaml`](../../../dataflow-vr.yaml), trimmed to teleop only.

"v1" is the OpenArm v1 hardware generation (per https://docs.openarm.dev/1.0/),
whose MuJoCo assets are vendored, patched, under `models/openarm_mujoco_v1/`
(see the [v1 sim design](2026-07-09-v1-vr-teleop-mujoco-design.md)).

## Decisions (from brainstorming)

- **Scope:** teleop only — no recorder, cameras, dataset, or cell lifter.
- **Follower config:** vendor a v1-specific `configs/openarm_v1.yaml` in this repo.
- **Viewer:** keep a MuJoCo viewer as a live digital twin alongside the real arms.
- **Start command:** reuse `dora-openarm-data-collection-ui` as a lightweight arm
  control panel (it auto-emits `arm_command: start` on boot and maps VR buttons);
  no cameras/recorder wired to it.
- **Gripper:** arm-first. The gripper is left inert this iteration (VR triggers
  are **not** wired into IK); the meters→radians gripper mapping is a tracked
  follow-up (see Findings §1 and Follow-up).
- **Joint offsets:** `configs/openarm_v1.yaml` seeds `joint_offsets` at **0** with
  a documented, safety-gated on-hardware calibration procedure; `joint_limits`
  are taken from the v1 MJCF.

## Key Findings — v1 and v2 do NOT share a joint convention

The `dora-openarm` follower moves in joint space and delegates to
`openarm_driver`. Its command mapping (from the driver's `send_position`) is:

```
motor_angle[i] = model_angle[i] + joint_offsets[i]     # additive only
gripper: set_position(model_angle[-1] + joint_offsets[-1])   # radians, rotary motor
```

Offsets **only shift the zero** — they cannot flip a joint's sign or rescale
units. Comparing the vendored v1 `openarm_bimanual.xml` to the pip v2 model
(`openarm-mujoco==2.0.0`, `v2/openarm_bimanual.xml`) shows the conventions differ:

**1. Gripper — concrete units mismatch (blocking for gripper control).**
The kinematics fork (`k1000dai/dora-openarm-kinematics`, `ik.py`) reads the
finger's "open" limit from the model per robot — comment: *"v2's revolute
fingers open at ±0.785 rad, v1's prismatic fingers at 0.044 m."* So for v1, IK
emits the 8th value as a **prismatic finger position in meters (0–0.044)**. The
driver forwards that 8th value to the **rotary gripper motor as radians**. 0.044
"rad" ≈ 2.5° — the v1 gripper will not open/close. The fix (a meters→radians
gripper mapping) **cannot be expressed in the driver config** (offsets are
additive), so the gripper is deferred (arm-first decision).

**2. Arm joints 1–7 — offsets/signs must be calibrated on hardware.**
The v1 vs v2 joint `axis` vectors differ on most joints (e.g. right `joint3`
`0 0 1`→`0 0 -1`, `joint4` `0 1 0`→`0 -1 0`, `joint6`/`joint7` reoriented). Some
is benign body-frame reorientation, but any v1 joint whose *positive direction*
is opposite the motor's cannot be corrected by an additive offset — it would
drive the wrong way. Therefore `joint_offsets` (and, if a sign is inverted, the
v1 MJCF joint axis itself) must be **derived against the real v1 motors**, not
copied from `openarm_cell.yaml` (which is calibrated to the v2 model).

**3. `joint_limits`** are read directly from the v1 MJCF (model units), so those
are set correctly for v1 up front.

**Consequence:** `configs/openarm_v1.yaml` is a **calibration template**, not a
"copy v2 and tune" artifact. The step-limited alignment ramp in the follower
(`AlignState.step_limit`) makes first bring-up with zero offsets physically safe
(the arm creeps, it does not jump).

## Scope

**In scope**

- New teleop-only real-robot dataflow: `dataflow-vr-v1.yaml`.
- Vendored v1 driver config: `configs/openarm_v1.yaml` (calibration template).
- Minimal `metadata_v1.yaml` for the UI panel (display-only).
- README section + a documented on-hardware calibration procedure.

**Out of scope**

- Recorder/dataset, cameras, cell lifter.
- Gripper control on v1 (deferred; see Follow-up).
- Changes to upstream node code (`openarm_driver`, `dora-openarm`, `dora-openarm-mujoco`).
- Bilateral force-feedback teleop (the `openarm_teleop` reference paradigm).

## Components

### 1. `dataflow-vr-v1.yaml` (7 nodes)

Built on `dataflow-vr-v1-mujoco.yaml`, with real followers + UI added and the
gripper trigger wiring removed.

| Node | Package | Notes |
|---|---|---|
| `ui` | `dora-openarm-data-collection-ui` | `env: METADATA_FILE: metadata_v1.yaml`. Inputs: `tick` (secs/1), `button_a`/`button_b` (receiver), `arm_status_{right,left}` (followers). Outputs `command`, `arm_command`. Auto-starts arms on boot. No camera/recorder inputs. |
| `quittable-tick-leader` | `dora-openarm-quitter` | `tick: millis/2`, `command: ui/command`. |
| `udp-receiver` | `dora-openarm-vr` | `--host 0.0.0.0 --port 5006 --ee-correction-deg 180 0 0 --frame-offset 0.0 0 0.6` (v1 VR calibration, carried from the v1 sim flow). |
| `ik` | `dora-openarm-ik` | `--xml models/openarm_mujoco_v1/scene.xml --keyframe home --mode bimanual --max-iters 10 --dt 0.1 --damping 0.1 --posture-cost 0.01 --lm-damping 0.01`. Inputs: `tick`, `target_{right,left}` (poses). **`trigger_{right,left}` are intentionally NOT wired** (gripper deferred). |
| `follower-right` | `dora-openarm` | `--side right --config configs/openarm_v1.yaml` (no `--align-trigger`; see below). Inputs: `move_position: ik/position_right`, `command: ui/arm_command`. Output `status` → ui. |
| `follower-left` | `dora-openarm` | `--side left --config configs/openarm_v1.yaml` (no `--align-trigger`; see below). Inputs: `move_position: ik/position_left`, `command: ui/arm_command`. Output `status` → ui. |
| `mujoco-viewer` | `dora-openarm-mujoco` | `--xml models/openarm_mujoco_v1/scene.xml --keyframe home --viewer --debug-frames`. Fed by `ik/position_{l,r}` + receiver poses/joystick. Shows the *commanded* pose (digital twin). |

Differences from `dataflow-vr.yaml` (v2): v1 `--xml`/VR calibration; `--config
configs/openarm_v1.yaml`; no recorder/cameras/lifter; `trigger_*` not wired
(gripper deferred); no `--align-trigger` (see below); the `request_state`
follower input is dropped (nothing consumes follower `state`, and it would force
a CAN refresh every tick — the UI uses `follower/status`, and the viewer uses IK
`position_*`).

**Alignment / `--align-trigger`.** The follower's soft-start ramp (`_align`) is
used and is what makes zero-offset bring-up safe: it seeds `align_target` from the
*actual* motor position, then steps toward the command by `clip(diff, ±0.001 rad)`
per event, flipping to `ALIGNED` only once every joint is within `align_threshold`
(0.1 rad), after which it tracks directly with a >0.1 rad divergence re-ramp. At
`millis/2` that is ~0.5 rad/s — the arm creeps, never jumps.

`--align-trigger gripper` is **dropped for v1**. Its gate compares the 8th
(gripper) value against `±np.deg2rad(5)` (≈ ±0.0873), thresholds authored for
v2's *revolute* gripper (`-0.785..0` rad). v1's gripper is *prismatic meters*
(`[0, 0.044]`), so every value satisfies both `> -0.0873` (right) and `< 0.0873`
(left): `is_gripping` is always True → the interlock never blocks and never
functions (a permanently-armed no-op). It cannot work with a meters-valued
gripper, so v1 runs plain alignment (`trigger=None`, gate skipped). Re-evaluate
the interlock if/when the gripper follow-up gives v1 a radian-scaled gripper.

### 2. `configs/openarm_v1.yaml` (calibration template)

A driver config resolved by `openarm_driver.Config` as a repo-relative path from
the dataflow working directory. Version-independent parts copied from the v2
`openarm_cell.yaml`; v1-specific parts set from the v1 MJCF / left as calibration
targets:

- `motor_config` — **identical** to v2 (standard 7-DOF + gripper build:
  `DM8009×2, DM4340×2, DM4310×4`; `send_ids 0x01–0x08`, `recv_ids 0x11–0x18`).
- `can_interface` — `right_arm: can0`, `left_arm: can1` (cabling-dependent knob).
- `joint_limits` — from the v1 MJCF (model units), both arms:
  - right `joint1..7`: `[-1.396263, 3.490659]`, `[-0.174533, 3.316125]`,
    `[-1.570796, 1.570796]`, `[0.0, 2.443461]`, `[-1.570796, 1.570796]`,
    `[-0.785398, 0.785398]`, `[-1.570796, 1.570796]`; gripper `[0.0, 0.044]` (m).
  - left `joint1..7`: `[-3.490659, 1.396263]`, `[-3.316125, 0.174533]`, then the
    same j3–j7 as right; gripper `[0.0, 0.044]` (m).
- `joint_offsets` — **all zeros** (8 per arm), with a prominent header comment:
  *these must be calibrated on the real v1 arm before absolute positioning is
  trusted.*
- `control_gains` (`kps`/`kds`), `joint_delta_position_limits`, `gripper_posforce`
  / `gripper_posforce_limits` — copied from `openarm_cell.yaml` as reasonable
  starting points (same DM motor family); flagged "verify on v1".
- `start` / `stop` `initial` position — the v1 `home` posture
  `[0.0, 0.0, 0.0, 1.570796, 0.0, 0.0, 0.0, 0.0]` for both arms (elbow bent 90°,
  gripper 0), replacing v2's.

The file header documents which fields are v1-authoritative vs. calibration
targets vs. copied-from-v2.

### 3. `metadata_v1.yaml`

Minimal panel metadata (arms `1.0`, no lifter, no perceptions) so the UI does not
advertise v2 hardware absent from this flow. Display-only; the UI reads it for the
panel and is otherwise agnostic.

### 4. README

A "VR teleoperation (OpenArm v1, real robot)" section: fetch the v1 model → bring
up CAN (`can0`/`can1`) → **calibrate `configs/openarm_v1.yaml`** (link the
procedure) → `dora build dataflow-vr-v1.yaml` → `dora run`, with a safety note and
an explicit "gripper is inert on v1 for now" caveat.

## Data Flow

```
Quest ──UDP──▶ udp-receiver ──pose_{r,l}──▶ ik (v1 scene) ──position_{r,l}──┬──▶ follower-right (real, configs/openarm_v1.yaml)
                    │  button_a/b                                            ├──▶ follower-left  (real, configs/openarm_v1.yaml)
                    ▼                                                        └──▶ mujoco-viewer (v1 digital twin)
                   ui ──arm_command:start──▶ followers    followers ──status──▶ ui (start/stop + status panel)
                   ui ──command:quit──────▶ quittable-tick-leader
```

Triggers are not wired; the gripper holds a constant (inert). Nothing is persisted.

## On-hardware Calibration Procedure (documented in the config header / README)

Safety-gated, one joint at a time, with `joint_offsets` starting at 0:

1. CAN up (`can0`/`can1`), arms powered, workspace clear, e-stop reachable.
2. `dora run dataflow-vr-v1.yaml`; arms auto-start and ramp (step-limited) toward
   the commanded neutral. Watch for any joint moving the **wrong direction** — that
   signals an inverted axis (config offsets cannot fix it; the v1 MJCF joint axis
   must be flipped instead). Stop if so.
3. Hold the Quest neutral; for each joint, read the follower `state.qpos` (model
   frame) vs. the intended model angle and set `joint_offsets[i]` to align motor
   zero to model zero. Re-run; iterate until neutral matches.
4. Verify soft `joint_limits` are not tripped in the intended workspace.
5. Only after arm calibration is trusted, address the gripper follow-up.

## Error / Edge Handling

- **Wrong-direction joint** (inverted axis) → caught visually in step 2; fix the
  v1 MJCF axis, not the config. Documented as the first calibration check.
- **Uncalibrated offsets** → arm settles to a wrong-but-bounded neutral; the
  step-limited ramp prevents jumps. Bounded by `joint_limits` from the MJCF.
- **CAN device mismatch** (`can0`/`can1` naming) → driver raises on init; documented
  as a config knob.
- **No Quest connected** → pipeline builds, arms auto-start and hold neutral; no
  pose events flow. Valid smoke test.
- **UI reachable** → the web panel provides a visible start/stop and live status,
  the intended manual safety control for hardware.

## Testing

1. `dora build dataflow-vr-v1.yaml` succeeds (followers pull `openarm-driver`;
   full run needs CAN libs + hardware on the robot host).
2. `pre-commit run --all-files` clean (YAML/format/lint).
3. Config-load smoke (host, no hardware):
   `openarm_driver.Config("configs/openarm_v1.yaml")` parses and every getter
   (`get_joint_limits`, `get_joint_offsets`, `get_motor_types`, …) resolves for
   both arms; lengths are 8.
4. Hardware (manual, documented): run the calibration procedure above; confirm
   arms track Quest motion and the MuJoCo twin mirrors the command; gripper inert.

## Follow-up (out of this iteration)

- **Gripper (v1):** add a meters→radians mapping so VR triggers drive the v1
  gripper. Cannot live in the driver config; candidates: a hardware-gripper output
  (radians) in the kinematics fork, or a small meters→rad adapter node between IK
  and the followers. Then wire `trigger_{right,left}` back into IK.
- Optional: promote calibrated `joint_offsets` and any MJCF axis fixes upstream.

## Rollout

Single PR: `dataflow-vr-v1.yaml` + `configs/openarm_v1.yaml` + `metadata_v1.yaml`
+ README section. No upstream node changes.
