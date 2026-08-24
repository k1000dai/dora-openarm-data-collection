# MoveIt 3D GUI — design

Date: 2026-08-24
Branch: `dev/moveit`

Replace the OpenArm MoveIt end-effector GUI's 2D top-down canvas with a
MuJoCo WASM 3D view, and fix the defects found in the existing MoveIt
integration.

## Goals

1. Drive the dora MoveIt pipeline from a 3D browser view that runs the same
   MuJoCo model the simulation runs, with the real robot state visible.
2. Separate planning from execution: see the planned path before the robot
   moves.
3. Fix the defects listed in "Defects fixed" below.

## Non-goals

- RViz interoperability. The RViz MotionPlanning display is the visual
  reference only; nothing in this design talks to ROS or RViz.
- Offline operation. `three` and `@mujoco/mujoco` load from the jsDelivr CDN
  via an import map, as upstream does.
- A Stop control. The upstream trajectory executor has no stop input.
- Scene-object editing and stored-state management.

## Background

`nodes/dora-openarm-moveit` wraps `dora-rs/dora-moveit2` for the OpenArm
bimanual robot. The current GUI (`static/eef_pose.html`) is a form of
XYZ/RPY/gripper fields plus a top-down 2D canvas.

`enactic/openarm_mujoco` master ships a complete browser MuJoCo stack in
`web/`:

- `ik.js` — `PoseController`, damped-least-squares differential IK on a
  kinematics-only `MjData`, targeting the `left_ee_control_point` /
  `right_ee_control_point` sites in the `arm_origin` frame.
- `model-vfs.js` — walks a scene XML's `<model file>` / `meshdir` references
  and loads everything into an `MjVFS`.
- `teleop.js` / `keymap.js` — ports of `dora-openarm-keyboard`, including its
  home pose.
- `main.js` — three.js renderer plus an `App` shell specific to the
  standalone page.

Measured facts that this design relies on:

- The repository tag `2.1.0` and the installed PyPI `openarm-mujoco` 2.1.0
  contain the identical `v2/openarm_bimanual.xml` blob
  (`0a4e8e96550a634a81d8ac8bc74f73c125006c15`).
- The wheel ships `share/openarm_mujoco/v2/` (XMLs and meshes) but not
  `web/`.
- `openarm_mujoco.v2.openarm_demo_xml()` resolves the same scene file the
  dora MuJoCo node loads.
- In the `demo` scene at the `home` keyframe, the `arm_origin` site is at
  world `[0.185, 0, 1.34]` with identity rotation; the arm base joints sit at
  `[0, ±0.0935, 0]` in that frame; the EE control points sit at
  `[0.216, ±0.1535, -0.22]`, matching `teleop.js`'s `DEFAULT_HOME` and the
  current GUI's `DEFAULTS`.
- `node` v24 is available, so `node --test` runs dependency-free ES modules
  with no `npm install`.

## Architecture

```
                        ┌────────── bridge/joint_positions (16) ──────────┐
                        │                                                  │
mujoco ──arm_l/r_obs──> bridge ──joint_commands──< trajectory_executor     │
   ▲                      │                              ▲                 │
   └── position_l/r ──────┘                              │ trajectory      │
                                                         │ (released on    │
                                                         │  execute)       │
eef-gui ──target_l/r──> ik ──position_l/r──────────> pose-goal <──trajectory── planner
   │                (kinematics node)                    │  ▲                  ▲
   ├── plan {auto_execute} ────────────────────────────> │  └── plan_request ──┘
   ├── execute ───────────────────────────────────────>  │
   │
   └<── joint_positions (bridge), trajectory (planner),
        plan_status (planner), execution_status (trajectory_executor)
```

`pose-goal` becomes the planning-request node: it holds the goal, issues a
`plan_request` only when a `plan` command arrives, holds the returned
trajectory, and forwards it to the executor only when an `execute` command
arrives. The browser subscribes to the planner's `trajectory` directly for
preview; that path does not reach the executor.

Removing the implicit tick-driven trigger is what eliminates defect 2 below.

### Browser

The page owns two robot states in one MuJoCo WASM instance plus one
three.js scene:

- **Goal state** — opaque. Driven by the browser's `PoseController` from the
  gizmo and keyboard teleop. Never touches dora until `plan` is pressed.
- **Current state** — translucent ghost. Driven by `joint_positions` polled
  from `/state`. This is the real simulation's state.
- **Planned path** — translucent ghost animated along the planned
  trajectory, with a time slider. Fetched once per plan from `/plan/<id>`.

Interaction, both available at once:

- A three.js `TransformControls` 6-DOF gizmo per arm, attached to the EE
  control point.
- Keyboard teleop with upstream's bindings via `teleop.js` / `keymap.js`.

Panel controls: scene selector (defaults to the scene the dataflow runs),
planning group, Home, Plan, Execute, Plan & Execute, planning time,
planned-path slider, status.

**Planning group** selects which arms the next plan moves: left, right, or
both. Every `plan_request` is still a 16-DOF dual request, because that is
what the upstream planner's `mode: "dual_arm"` expects; a deselected arm's
half of the goal is pinned to its current state instead of the gizmo's, so
it holds position.

**Home** returns the goal state to the teleop home pose through
`PoseController.startFromHome`, exactly as upstream's Backspace does. It is a
pose, so it travels the same `target_left` / `target_right` → IK route as
every other goal; there is no separate joint-space goal path. The config's
named joint poses (`home`, `safe`) stay where they are used today — the
scripted `demo.py` flow and the executor's boot fallback — and get no GUI
control.

## Components

### New submodule

`nodes/openarm-mujoco` → `https://github.com/enactic/openarm_mujoco.git`,
pinned at `5ba0271` on `master`. Only `web/*.js` is used from it.

The pin is a master commit rather than the tag `2.1.0` this design first
named: `web/` was added to the repository after that tag, so `2.1.0` has no
browser modules at all. Because the models are served from the wheel and not
from the submodule, the pin only decides which `web/*.js` is used, and model
drift between the pin and the installed wheel is impossible by construction.
The pinned tree's `v2/openarm_bimanual.xml` is in any case still the identical
blob.

Model assets are **not** served from the submodule. They are served from the
installed `openarm-mujoco` wheel, located via
`openarm_mujoco.v2.openarm_demo_xml()` and friends, so the browser and the
dora MuJoCo node load byte-identical files even if the submodule pin and the
installed wheel ever diverge.

### `nodes/dora-openarm-moveit`

| File | Change |
| --- | --- |
| `static/index.html` | New. Panel + canvas layout. Builds the import map from the submodule's `web/package-lock.json`, adding `TransformControls.js` to upstream's entries. |
| `static/app.js` | New. three.js + WASM shell adapted from upstream `main.js`; adds the ghost states, the gizmo, and the plan/execute controls. |
| `static/planning.js` | New. Dependency-free pure helpers: pose↔form conversion, trajectory sampling for the slider, state diffing. Tested by `node --test`. |
| `static/eef_pose.html` | Deleted. |
| `gui.py` | Serves the page, `static/`, `web/`, and `v2/`; adds `GET /state`, `GET /plan/<id>`, `POST /command`; resolves the scene through `openarm_mujoco.v2`. |
| `goal_planner.py` | Becomes the plan/execute gate. |
| `openarm_ik.py` | Deleted. |
| `config/openarm.py` | Adds `ARM_BASE_TRANSFORMS`; gives `SAFE_CONFIG` a real retracted pose. |
| `pyproject.toml` | Drops the `dora-openarm-moveit-openarm-ik` script; adds `static/*.js` to package data. |
| `README.md` | Rewritten for the 3D GUI and the planner's real limitations. |

`layout.py`, `pose.py`, `bridge.py`, `demo.py`, `planner.py`,
`ik_solver.py`, `planning_scene.py`, `trajectory_executor.py` are unchanged.

### Dataflows

`dataflow-openarm-moveit-gui-mujoco.yaml` and its headless variant get the
new wiring. The `ik` node switches to
`pip install -e nodes/dora-openarm-kinematics` / `path: dora-openarm-ik`,
which is what the repository's other dataflows already use.

`dataflow-openarm-moveit-mujoco.yaml` and its headless variant change only
through the config fixes; their node wiring stays as it is.

## HTTP surface

Polling, not SSE. A long-lived response would race
`ThreadingHTTPServer.shutdown()` / `server_close()` in `gui.py`'s `finally`
block, and on localhost polling costs nothing.

| Route | Behavior |
| --- | --- |
| `GET /` | The page. |
| `GET /static/<name>` | Own JS. |
| `GET /web/<name>` | Submodule `web/*.js` and `package-lock.json`. |
| `GET /v2/<path>` | Model XMLs and meshes from the installed wheel. |
| `GET /state` | Polled at ~20 Hz. `{joint_positions: [16] \| null, plan: {id, state, message, num_waypoints}, execution: {...}}`. `plan.state` is one of `idle`, `planning`, `ready`, `executing`, `done`, `failed`. |
| `GET /plan/<id>` | `{id, num_waypoints, num_joints, waypoints: [[16], …]}` for one plan, fetched when `plan.id` changes. 404 once superseded. |
| `POST /command` | One of `{"targets": {"left": …, "right": …}}`, `{"plan": {"auto_execute": bool, "groups": ["left", "right"]}}`, `{"execute": true}`. |

All three static routes resolve the request path against a fixed root and
reject anything that escapes it, including symlinks. `gui.py` does not
currently read from disk per request, so this is a new surface and needs the
guard from the start. The server stays bound to `127.0.0.1` by default.

## Gate state machine

`goal_planner.py` holds:

- `current` — latest 16-value state from `bridge/joint_positions`.
- `goal` — latest 16-value goal assembled from the IK node's
  `position_left` / `position_right`; seeded from `current` before any IK
  result arrives.
- `plan_id`, `held_trajectory`, `auto_execute`.

Transitions:

| Event | Behavior |
| --- | --- |
| `joint_positions` | Update `current`; seed `goal` if unset. |
| `position_left` / `position_right` | Update that half of `goal`. Never triggers a plan. |
| `plan` | Drop any held trajectory, increment `plan_id`, emit `plan_request` with `start=current` and a goal assembled per side — `goal`'s half for a side named in the command's `groups`, `current`'s half for a side that is not — and record `auto_execute`. |
| `trajectory` | Store as `held_trajectory` for the current `plan_id`. If `auto_execute`, forward immediately. |
| `execute` | Forward `held_trajectory` if present; otherwise emit a status saying there is nothing to execute. |

A plan request is emitted only on an explicit `plan` command. Both halves of
`goal` are already current when the command arrives, so there is no
partial-goal window and no duplicate request.

## Defects fixed

1. **GUI status inputs unwired.** `gui.py` handles `plan_status` and
   `execution_status`, but the dataflow only wired `tick`, so the browser
   never saw a planning failure or an execution finishing. The new dataflow
   wires them.
2. **Duplicate and partial plan requests.** `goal_planner`'s `dirty` flag was
   set by a single side's IK result, and a `tick` landing between the IK
   node's `position_right` and `position_left` outputs sent a plan built from
   a stale half and then a second plan on the next tick. The explicit `plan`
   command removes the trigger entirely.
3. **`eef_pose.html:264` throws the wrong error.** The template literal
   references `field`, which is out of scope outside the `some()` callback,
   so a non-finite input reported `field is not defined`. The file is
   deleted.
4. **`openarm_ik.py` duplication and a dead monkeypatch.** The file is a copy
   of `nodes/dora-openarm-kinematics/src/dora_openarm_kinematics/ik.py` plus
   `_patch_configuration_limit()`, which reimplements openarm-control's
   `ArmConfigurationLimit.__init__` to dodge a NumPy/enum comparison. On
   mujoco 3.11.0 that comparison already returns `True`
   (`m.jnt_type[0] in (mjJNT_HINGE, mjJNT_SLIDE)` was verified against the
   real model), so the patch is dead weight that would silently drift from
   openarm-control. The file and its entry point are removed and the
   dataflow uses the kinematics node. It also removes the module-level
   import after a function call that `ruff --select E402` flags.
5. **`ARM_BASE_TRANSFORMS` missing.** `OpenArmConfig` fails
   `isinstance(c, DualArmConfig)` and `get_arm_config` raises
   `AttributeError`. Add
   `{"left_arm": {"xyz": [0, 0.0935, 0], "rpy": [0, 0, 0]},
   "right_arm": {"xyz": [0, -0.0935, 0], "rpy": [0, 0, 0]}}`, measured from
   the model in the `arm_origin` frame.
6. **README overstates the planner.** Upstream's
   `planner_ompl_with_collision_op.py` has `is_state_valid` returning `True`
   unconditionally and start/goal validation behind `if False:`, so
   collision checking is entirely disabled and `COLLISION_GEOMETRY` /
   `COLLISION_MARGIN` are inert. `is_motion_valid` is likewise always true,
   so RRT-Connect connects on its first iteration and the path is
   start → one random 0.2 rad waypoint → goal. Separately, the shared joint
   limits are the union of the two chains' asymmetric ranges (left joint1 is
   `[-3.4907, 1.3963]`, right is `[-1.3963, 3.4907]`), so a sampled waypoint
   can exceed one arm's true range and get clipped by MuJoCo's `ctrlrange`.
   The README states both.
7. **`SAFE_CONFIG == HOME_CONFIG`.** The named pose `safe` was identical to
   `home`. Give it a genuinely retracted pose.

## Testing

**pytest** (`nodes/dora-openarm-moveit/tests/`)

- Existing `test_layout.py` and `test_pose.py` keep passing.
- `test_gate.py` — the state machine as a pure object, exercised without
  dora: no plan before a `plan` command; `execute` before a trajectory is a
  no-op with a status; a held trajectory is forwarded exactly once;
  `auto_execute` forwards on arrival; a new `plan` invalidates the held
  trajectory.
- `test_static.py` — static-route resolution: known files resolve, `..`
  segments, absolute paths, and symlinks escaping the root are rejected.
- `test_config.py` — `OpenArmConfig` satisfies `DualArmConfig` and
  `get_arm_config` returns both base transforms.

**node --test** (`nodes/dora-openarm-moveit/tests/js/`)

- `planning.test.mjs` — `planning.js`'s pure helpers. No `npm install`;
  `planning.js` imports nothing.

Decision logic lives in Python or in `planning.js` so it is testable;
`app.js` stays a rendering shell.

**Manual**

`dora run dataflow-openarm-moveit-gui-mujoco.yaml --uv`, then check in the
browser: the scene loads, the ghost tracks the simulation, the gizmo and the
keyboard both move the goal state, Plan shows a path without moving the
robot, Execute moves it, and a failed plan surfaces its message.

## Implementation order

1. Defect fixes 3–7 and the config change, with the existing tests staying
   green. `openarm_ik.py` is removed and the dataflows point at the
   kinematics node.
2. Add the submodule; extend `gui.py` with static serving and `/state`.
3. Gate `goal_planner`; rewire the two GUI dataflows (defects 1 and 2).
4. Browser: scene load → current-state ghost → gizmo → teleop → planned-path
   preview.
5. README.

## Risks

- **CDN availability.** A machine without internet cannot load the page.
  Accepted; the fallback is the numeric form, which stays in the panel.
- **Submodule pin drift.** The pin is an untagged master commit, so bumping
  it is a deliberate act with no release notes to read. Serving models from
  the wheel keeps the model correct regardless; what can drift is `ik.js`'s
  behaviour against a newer `@mujoco/mujoco`.
- **WASM memory.** `model-vfs.js` deletes a half-built VFS on failure, but
  repeated scene switching in one page needs the existing `disposeScene()`
  path to be preserved when adapting `main.js`.


## Addendum: goals that were not reached (2026-08-24)

Reported after the first implementation: the robot sometimes did not get
anywhere near the target pose. Three separate things were measured, two of
them defects.

**1. The IK never converged on a one-shot target.** The kinematics node runs
differential IK, and a single solve moves the arms a bounded ~0.09 rad
towards the target regardless of distance. One solve covered 99.5% of a
0.04 m move but 6.5% of a 0.176 m one, so the plan goal — which *is* that
solve's output — sat 117 mm short of the request. Every other dataflow in
this repository feeds the node from a continuously streaming pose source, so
it converges there; this GUI was the first consumer to send one discrete
target. Fixed by repeating the target until consecutive solutions stop
changing.

Detecting "stopped changing" needed a second condition: the node keeps
reporting its last solution, so results arriving right after a new target
still answer the *previous* one and are identical to each other. Counting
those as convergence made the pump settle in the minimum three ticks on every
goal after the first. A side must now be seen moving once before its
stillness counts.

**2. The executor never commands the last waypoint.** On the tick that
finishes a trajectory it sets `is_executing = False` and returns the measured
joint state, which still lags the command, then holds there. Against a simple
lag plant the goal was never commanded at all and 34.6% of the move was lost.
Fixed by dwelling on the goal (`DWELL_WAYPOINTS`), so what the executor
discards is a duplicate. Measured gain in MuJoCo is smaller than the
synthetic case suggested — 23.9 mm to 21.2 mm mean — because the actuators
are fast relative to a 200 ms waypoint.

**3. Steady-state droop, which is not a defect.** The remainder, 5-20 mm, is
the MuJoCo position actuators holding each joint at a static offset under
gravity; the MJCF has no gravity compensation and the MuJoCo node has no
option for it. It is visible at startup before anything is planned. The GUI
now reports it as "reached: N mm from plan goal" so it does not read as a
planning failure.

Measured end to end over a seven-goal sequence, worst total error went from
172 mm to 18 mm, with IK error 0.0 mm on every goal.

**A note on the measurement.** The first "after the fix" numbers looked worse
than the baseline. That was the harness, not the system: it waited on the
gate's state field, which still held `done` from the previous goal, so it
broke out immediately and compared the *previous* plan against the new
target. Waiting for the trajectory id to change fixed it. Numbers taken
before that fix should be disregarded.
