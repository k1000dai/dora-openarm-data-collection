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
- **Gripper:** made to work, **identically to v2**. A small `gripper-adapter`
  node converts the IK gripper element from v1 model units (prismatic meters) to
  gripper-motor radians (v2 convention) for the followers, so both the gripper
  *and* the `--align-trigger gripper` interlock behave like v2 ("squeeze the
  trigger to align"). VR triggers are wired into IK; `--align-trigger gripper` is
  kept on the followers. (See Findings §1 and Components §2.)
- **Joint offsets:** `configs/openarm_v1.yaml` sets `joint_offsets` to **0**. The
  OpenArm hardware uses an identity motor↔joint mapping (openarm_teleop's
  `JointMapper`: `joint[i] = motor[i]`) and the v1 motor 0-pose is aligned to the
  v1 model 0-pose, so no offset is needed. `joint_limits` are taken from the v1
  MJCF. A first-run per-joint **sign** check is the only residual (a flip looks
  fine in sim but reverses on hardware; not fixable by an offset).

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

**1. Gripper — units mismatch, fixed by the `gripper-adapter` node.**
The kinematics fork (`k1000dai/dora-openarm-kinematics`, `ik.py`) reads the
finger's "open" limit from the model per robot — comment: *"v2's revolute
fingers open at ±0.785 rad, v1's prismatic fingers at 0.044 m."* So for v1, IK
emits the 8th value as a **prismatic finger position in meters (0–0.044)**, while
the driver forwards that 8th value to the **rotary gripper motor as radians** —
0.044 "rad" ≈ 2.5°, so the gripper would not open/close, and the interlock
thresholds (±5°, radians) never fire against meters values.

The v1 gripper hardware is a **parallel-jaw slide (0–44 mm) driven by the rotary
DM4310 through a crank-slider** (per docs.openarm.dev/hardware); v2's is
revolute (`±0.785 rad`). The fix cannot live in the driver config (offsets are
additive, no scale) and we do not modify upstream nodes, so a small
`gripper-adapter` node rescales the gripper element meters→radians into the v2
convention (`closed 0`, `open ∓0.785 rad`, sign per side) between IK and the
followers. A linear map suffices for teleop open/close and the interlock; the
crank-slider's mild nonlinearity is an optional later refinement. The full-open
radian magnitude is a **calibration knob** (default `∓0.785`, tuned on hardware),
like the arm offsets. The MuJoCo viewer still reads IK directly (meters), so the
prismatic twin stays faithful.

**2. Arm joints 1–7 — `joint_offsets = 0`; only a first-run sign check remains.**
The OpenArm hardware uses an identity motor↔joint mapping (openarm_teleop's
`JointMapper` copies `joint[i] = motor[i]`, no sign flip, no offset), and the v1
motor 0-pose is aligned to the v1 model 0-pose (confirmed by the working
openarm_teleop bring-up). So IK's model angles already equal motor angles →
`joint_offsets = 0` is correct, and no per-joint offset measurement is needed.
(The v2 `openarm_cell.yaml` has non-zero offsets because the v2 *model* uses a
different zero — not a hardware requirement that transfers to v1.) The one
residual: the driver is additive, so a joint whose v1 *model* sign is opposite
the motor would drive the wrong way and cannot be fixed by an offset — it would
need the v1 MJCF axis flipped. Sim teleop can't catch this (no physical motor),
so it gets a gentle first-motion check; if the v1 model was authored from the
hardware joint definitions it is already correct.

**3. `joint_limits`** are read directly from the v1 MJCF (model units), so those
are set correctly for v1 up front.

**Consequence:** `configs/openarm_v1.yaml` ships ready-to-run (`joint_offsets = 0`,
limits from the MJCF), not as a measure-everything template. The step-limited
alignment ramp in the follower (`AlignState.step_limit`) still makes first
bring-up physically safe (the arm creeps, it does not jump), and the first-run
sign check is the only manual step.

## Scope

**In scope**

- New teleop-only real-robot dataflow: `dataflow-vr-v1.yaml`.
- New local node `nodes/dora-openarm-gripper-adapter` (meters→radians gripper).
- Vendored v1 driver config: `configs/openarm_v1.yaml` (calibration template).
- Minimal `metadata_v1.yaml` for the UI panel (display-only).
- README section + documented first-run checks (sign + gripper).

**Out of scope**

- Recorder/dataset, cameras, cell lifter.
- Changes to upstream node code (`openarm_driver`, `dora-openarm`, `dora-openarm-mujoco`)
  and to the forks (`dora-openarm-vr`, `dora-openarm-kinematics`) — the gripper
  units fix lives in the new adapter node, not in these.
- Crank-slider-exact gripper mapping (linear approximation is used; see Follow-up).
- Bilateral force-feedback teleop (the `openarm_teleop` reference paradigm).

## Components

### 1. `dataflow-vr-v1.yaml` (8 nodes)

Built on `dataflow-vr-v1-mujoco.yaml`, with real followers + UI + the
`gripper-adapter` added.

| Node | Package | Notes |
|---|---|---|
| `ui` | `dora-openarm-data-collection-ui` | `env: METADATA_FILE: metadata_v1.yaml`. Inputs: `tick` (secs/1), `button_a`/`button_b` (receiver), `arm_status_{right,left}` (followers). Outputs `command`, `arm_command`. Auto-starts arms on boot. No camera/recorder inputs. |
| `quittable-tick-leader` | `dora-openarm-quitter` | `tick: millis/2`, `command: ui/command`. |
| `udp-receiver` | `dora-openarm-vr` | `--host 0.0.0.0 --port 5006 --ee-correction-deg 180 0 0 --frame-offset 0.0 0 0.6` (v1 VR calibration, carried from the v1 sim flow). |
| `ik` | `dora-openarm-ik` | `--xml models/openarm_mujoco_v1/scene.xml --keyframe home --mode bimanual --max-iters 10 --dt 0.1 --damping 0.1 --posture-cost 0.01 --lm-damping 0.01`. Inputs: `tick`, `target_{right,left}` (poses), `trigger_{right,left}` (VR triggers → gripper, in v1 model meters). |
| `gripper-adapter` | `dora-openarm-gripper-adapter` (new, local) | Inputs: `position_{right,left}` from IK. Outputs: `position_{right,left}` with joints 1–7 passed through and the 8th (gripper) rescaled meters→radians (v2 convention). Args expose the per-side full-open radian (calibration knob). See §2. |
| `follower-right` | `dora-openarm` | `--side right --align-trigger gripper --config configs/openarm_v1.yaml`. Inputs: `move_position: gripper-adapter/position_right`, `command: ui/arm_command`. Output `status` → ui. |
| `follower-left` | `dora-openarm` | `--side left --align-trigger gripper --config configs/openarm_v1.yaml`. Inputs: `move_position: gripper-adapter/position_left`, `command: ui/arm_command`. Output `status` → ui. |
| `mujoco-viewer` | `dora-openarm-mujoco` | `--xml models/openarm_mujoco_v1/scene.xml --keyframe home --viewer --debug-frames`. Fed by `ik/position_{l,r}` (meters — the faithful prismatic twin) + receiver poses/joystick. Shows the *commanded* pose. |

Note the split: **followers** read `gripper-adapter/position_*` (radians);
the **viewer** reads `ik/position_*` (meters). The 7 arm joints are identical on
both wires — only the gripper element differs by units.

Differences from `dataflow-vr.yaml` (v2): v1 `--xml`/VR calibration; `--config
configs/openarm_v1.yaml`; no recorder/cameras/lifter; the `gripper-adapter` sits
between IK and the followers; the `request_state` follower input is dropped
(nothing consumes follower `state`, and it would force a CAN refresh every tick —
the UI uses `follower/status`, and the viewer uses IK `position_*`).

**Alignment / `--align-trigger gripper`.** The follower's soft-start ramp
(`_align`) makes zero-offset bring-up safe: it seeds `align_target` from the
*actual* motor position, then steps toward the command by `clip(diff, ±0.001 rad)`
per event, flipping to `ALIGNED` only once every joint is within `align_threshold`
(0.1 rad), after which it tracks directly with a >0.1 rad divergence re-ramp. At
`millis/2` that is ~0.5 rad/s — the arm creeps, never jumps.

`--align-trigger gripper` is **kept**, and now works exactly like v2 because the
adapter feeds the gripper in radians. The gate marks the arm "gripping" when the
8th value is near closed (`> -0.0873` right / `< 0.0873` left); with the adapter's
v2 convention (`closed 0`, `open ∓0.785`), squeezing the trigger drives the
gripper toward 0 → gate passes → the ramp proceeds, and releasing opens the
gripper → gate blocks. "Squeeze to align," identical to v2. (At boot, before VR
connects, IK holds the `home` gripper value `0` = closed, exactly as v2 does, so
startup gating matches v2 too.)

### 2. `nodes/dora-openarm-gripper-adapter` (new local node)

A small, focused dora node that converts the IK gripper element from v1 model
units (prismatic **meters**, `[0, 0.044]`) to gripper-motor **radians** in the v2
convention, so the real followers and the `--align-trigger gripper` interlock
behave exactly like v2.

- **Inputs:** `position_right`, `position_left` — float32[8] from IK.
- **Outputs:** `position_right`, `position_left` — float32[8], joints 1–7 passed
  through unchanged, the 8th (gripper) rescaled.
- **Transform (per side):** `rad = out_open_side * (meters / in_open)`, with
  `in_open = 0.044` (v1 model finger open), `out_open_right = -0.785`,
  `out_open_left = +0.785` (v2 convention: `closed 0`, `open ∓0.785`). Linear,
  monotonic; `meters = 0` (closed) maps to `0` on both sides.
- **Args (calibration knobs):** `--in-open` (default `0.044`),
  `--out-open-right` (default `-0.785`), `--out-open-left` (default `0.785`).
  The full-open radian is tuned on hardware to the real jaw's fully-open motor
  angle; the sign per side matches how the real gripper motors mount.
- **Event-driven:** emits a converted `position_{side}` on each corresponding IK
  input; no tick needed.

Layout mirrors the other small local nodes (`pyproject.toml`, `src/…/main.py`,
`README.md`). Kept deliberately tiny and pure so it is unit-testable without
hardware. This is the *only* place the v1 gripper units fix lives — no upstream
or fork code changes.

### 3. `configs/openarm_v1.yaml`

A driver config resolved by `openarm_driver.Config` as a repo-relative path from
the dataflow working directory. Version-independent parts copied from the v2
`openarm_cell.yaml`; v1-specific parts set from the v1 MJCF. Ships ready-to-run:

- `motor_config` — **identical** to v2 (standard 7-DOF + gripper build:
  `DM8009×2, DM4340×2, DM4310×4`; `send_ids 0x01–0x08`, `recv_ids 0x11–0x18`).
- `can_interface` — `right_arm: can0`, `left_arm: can1` (cabling-dependent knob).
- `joint_limits` — arm joints 1–7 from the v1 MJCF (model units); the gripper
  (8th) is in **radians**, since the adapter feeds the follower radians (the
  safety checker runs on the post-adapter command). Both arms:
  - right `joint1..7`: `[-1.396263, 3.490659]`, `[-0.174533, 3.316125]`,
    `[-1.570796, 1.570796]`, `[0.0, 2.443461]`, `[-1.570796, 1.570796]`,
    `[-0.785398, 0.785398]`, `[-1.570796, 1.570796]`; gripper `[-1.047198, 0.4]`
    (rad, from `openarm_cell.yaml` — encompasses the adapter's `[-0.785, 0]`).
  - left `joint1..7`: `[-3.490659, 1.396263]`, `[-3.316125, 0.174533]`, then the
    same j3–j7 as right; gripper `[-0.4, 1.047198]` (rad, encompasses `[0, 0.785]`).
- `joint_offsets` — **all zeros** (8 per arm). The hardware uses an identity
  motor↔joint mapping and the v1 0-pose is aligned, so `0` is correct; set a
  value only if the first-run check shows a joint's 0-pose is off (header
  documents this).
- `control_gains` (`kps`/`kds`), `joint_delta_position_limits`, `gripper_posforce`
  / `gripper_posforce_limits` — copied from `openarm_cell.yaml` as reasonable
  starting points (same DM motor family); flagged "verify on v1".
- `start` / `stop` `initial` position — the v1 `home` posture
  `[0.0, 0.0, 0.0, 1.570796, 0.0, 0.0, 0.0, 0.0]` for both arms (elbow bent 90°,
  gripper 0), replacing v2's.

The file header documents which fields are v1-authoritative vs. calibration
targets vs. copied-from-v2.

### 4. `metadata_v1.yaml`

Minimal panel metadata (arms `1.0`, no lifter, no perceptions) so the UI does not
advertise v2 hardware absent from this flow. Display-only; the UI reads it for the
panel and is otherwise agnostic.

### 5. README

A "VR teleoperation (OpenArm v1, real robot)" section: fetch the v1 model → bring
up CAN (`can0`/`can1`, standard OpenArm motor zeroing) → `dora build
dataflow-vr-v1.yaml` → `dora run`, with a safety note, the "squeeze the trigger
to align" gesture, and the first-run sign check called out (`joint_offsets = 0`
is expected correct; tune the `gripper-adapter` open-radian by eye).

## Data Flow

```
Quest ─UDP▶ udp-receiver ─pose_{r,l}─▶ ik (v1 scene) ─position_{r,l} (m)─┬─▶ gripper-adapter ─position_{r,l} (rad)─┬─▶ follower-right (real, openarm_v1.yaml)
              │  trigger_{r,l} ───────▶ (→ gripper, meters)              │                                        └─▶ follower-left  (real, openarm_v1.yaml)
              │  button_a/b                                              └─▶ mujoco-viewer (v1 twin, meters)
              ▼
             ui ─arm_command:start─▶ followers     followers ─status─▶ ui (start/stop + status panel)
             ui ─command:quit──────▶ quittable-tick-leader
```

Followers read radians (via the adapter); the viewer reads IK meters directly.
"Squeeze the trigger to align" (v2-identical interlock). Nothing is persisted.

## First-run Checks (documented in the config header / README)

No offset calibration is expected — `joint_offsets = 0` given the identity
motor↔joint mapping and the aligned v1 0-pose. The steps below are safety checks,
not a measure-everything procedure:

1. CAN up (`can0`/`can1`, standard OpenArm motor zeroing), arms powered,
   workspace clear, e-stop reachable.
2. `dora run dataflow-vr-v1.yaml`. **Squeeze a trigger** to arm the alignment
   ramp (the interlock blocks alignment until you do). Arms ramp (step-limited)
   toward the commanded neutral. Confirm each joint moves the **right
   direction** — a model/hardware sign flip looks fine in sim but reverses on
   hardware; fix it by flipping that joint's axis in the v1 MJCF (an offset
   cannot). Release the trigger / e-stop if a joint reverses.
3. Only if a joint's 0-pose turns out misaligned, set its `joint_offsets[i]`
   (compare follower `state.qpos` to the intended model angle). Not expected.
4. **Gripper:** tune the `gripper-adapter` `--out-open-{right,left}` so a released
   trigger fully opens the jaw and a squeezed trigger closes it, without straining
   at the mechanical limit. Confirm the sign per side (jaw opens, not closes, on
   release).

## Error / Edge Handling

- **Wrong-direction joint** (inverted model/hardware sign) → caught on the first
  motion (step 2); fix the v1 MJCF axis, not the config (an offset cannot flip a
  sign). The only expected manual check.
- **Misaligned 0-pose** (unexpected) → set that joint's `joint_offsets[i]`; the
  step-limited ramp prevents jumps meanwhile. Bounded by `joint_limits`.
- **CAN device mismatch** (`can0`/`can1` naming) → driver raises on init; documented
  as a config knob.
- **No Quest connected** → pipeline builds, arms auto-start and hold the `home`
  gripper (closed); the interlock leaves the ramp gated until a trigger squeeze
  (same as v2). Valid build/model smoke test.
- **Un-squeezed at start** → the interlock holds the arms at their power-on pose
  (no ramp) until a trigger is squeezed — the intended safety gesture.
- **Gripper sign wrong / over-travel** → adapter `--out-open-{side}` mis-set; the
  jaw closes on release or strains at the limit. Caught in calibration step 5;
  fix the sign/magnitude arg. `joint_limits` (rad) clamp the follower command.
- **CAN device mismatch** (`can0`/`can1` naming) → driver raises on init; documented
  as a config knob.
- **UI reachable** → the web panel provides a visible start/stop and live status,
  the intended manual safety control for hardware.

## Testing

1. `dora build dataflow-vr-v1.yaml` succeeds (followers pull `openarm-driver`;
   full run needs CAN libs + hardware on the robot host).
2. `pre-commit run --all-files` clean (YAML/format/lint).
3. `gripper-adapter` unit tests (no hardware): joints 1–7 pass through untouched;
   gripper `0.0 m → 0.0 rad`, `0.044 m → -0.785 rad` (right) / `+0.785 rad` (left);
   output length 8; args override the open radian.
4. Config-load smoke (host, no hardware):
   `openarm_driver.Config("configs/openarm_v1.yaml")` parses and every getter
   (`get_joint_limits`, `get_joint_offsets`, `get_motor_types`, …) resolves for
   both arms; lengths are 8; gripper limit is the radian range.
5. Hardware (manual, documented): run the first-run checks; confirm the
   squeeze-to-align interlock, each joint moving the right direction, arms
   tracking Quest motion, the gripper opening/closing with the trigger, and the
   MuJoCo twin mirroring the command.

## Follow-up (out of this iteration)

- **Crank-slider-exact gripper mapping:** replace the adapter's linear meters→rad
  approximation with the true crank-slider inverse if open/close precision needs it.
- Optional: promote the tuned `gripper-adapter` open-radian and any v1 MJCF axis
  fixes upstream.

## Rollout

Single PR: `dataflow-vr-v1.yaml` + `nodes/dora-openarm-gripper-adapter/` +
`configs/openarm_v1.yaml` + `metadata_v1.yaml` + README section. No upstream node
changes.
