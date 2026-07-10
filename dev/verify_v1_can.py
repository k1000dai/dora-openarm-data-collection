#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Read OpenArm v1 joint positions over CAN for the first-run bring-up checks.

Uses the openarm_can library (the same low-level interface as openarm-can-demo)
to read raw motor positions [rad] for one arm, using the motor/CAN layout from
configs/openarm_v1.yaml. The motors are left backdrivable (zero-torque MIT), so
you can hand-move each joint and confirm, against the printed values:

  * 0-pose alignment  — at the model's zero pose, every joint reads ~0.
  * per-joint sign     — moving a joint in the model's + direction makes its
                         printed value increase (not decrease).

This is the read path the teleop dataflow omits (it drops request_state). It
does NOT command any motion; it only reads. Run it BEFORE trusting the arm under
dataflow-vr-v1.yaml.

Example:
    python dev/verify_v1_can.py --side right
    python dev/verify_v1_can.py --side left --config configs/openarm_v1.yaml
"""

import argparse
import time
from dataclasses import dataclass

import yaml

JOINT_LABELS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


@dataclass
class MotorArgs:
    """Motor/CAN arguments for one arm, split into arm joints and gripper."""

    can: str
    arm_types: list
    arm_send_ids: list
    arm_recv_ids: list
    gripper_type: str
    gripper_send_id: int
    gripper_recv_id: int


def motor_args_from_config(cfg: dict, side: str) -> MotorArgs:
    """Derive one arm's motor/CAN arguments from an openarm_driver config dict.

    The 8-entry motor_config is split the same way openarm_driver does: the first
    seven motors are the arm joints and the last is the gripper.
    """
    mc = cfg["motor_config"]
    return MotorArgs(
        can=cfg["can_interface"][f"{side}_arm"],
        arm_types=list(mc["types"][:-1]),
        arm_send_ids=list(mc["send_ids"][:-1]),
        arm_recv_ids=list(mc["recv_ids"][:-1]),
        gripper_type=mc["types"][-1],
        gripper_send_id=mc["send_ids"][-1],
        gripper_recv_id=mc["recv_ids"][-1],
    )


def _build_openarm(oa, args: MotorArgs):
    """Initialize and enable an openarm_can.OpenArm for reading positions."""
    openarm = oa.OpenArm(args.can, True)  # CAN-FD
    arm_types = [getattr(oa.MotorType, t) for t in args.arm_types]
    openarm.init_arm_motors(arm_types, args.arm_send_ids, args.arm_recv_ids)
    openarm.init_gripper_motor(
        getattr(oa.MotorType, args.gripper_type),
        args.gripper_send_id,
        args.gripper_recv_id,
    )
    openarm.set_callback_mode_all(oa.CallbackMode.STATE)
    openarm.enable_all()
    openarm.recv_all(2000)
    return openarm


def main():
    """Read and print v1 joint positions over CAN for the bring-up checks."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--side", choices=["right", "left"], default="right")
    parser.add_argument("--config", default="configs/openarm_v1.yaml")
    parser.add_argument(
        "--hz", type=float, default=10.0, help="Print rate [Hz] (default: 10)."
    )
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    motor_args = motor_args_from_config(cfg, args.side)

    print("!! SAFETY: motors will be ENABLED but zero-torque (backdrivable).")
    print("!! Keep the workspace clear and an e-stop reachable.")
    print(f"Reading {args.side} arm on {motor_args.can}. Ctrl-C to stop.\n")

    import openarm_can as oa

    openarm = _build_openarm(oa, motor_args)
    # Zero-torque MIT so the arm is backdrivable while we read (no holding force).
    zero = [oa.MITParam(0, 0, 0, 0, 0) for _ in motor_args.arm_types]
    period = 1.0 / args.hz
    try:
        while True:
            openarm.get_arm().mit_control_all(zero)
            openarm.get_gripper().mit_control_all([oa.MITParam(0, 0, 0, 0, 0)])
            openarm.refresh_all()
            openarm.recv_all()
            arm_pos = [m.get_position() for m in openarm.get_arm().get_motors()]
            grip_pos = [m.get_position() for m in openarm.get_gripper().get_motors()]
            cells = [f"{lbl}={p:+.3f}" for lbl, p in zip(JOINT_LABELS, arm_pos)]
            cells.append(f"gripper={grip_pos[0]:+.3f}" if grip_pos else "gripper=?")
            print("  ".join(cells))
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        openarm.disable_all()
        openarm.recv_all()
        print("\nmotors disabled.")


if __name__ == "__main__":
    main()
