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

"""Convert the IK gripper element from v1 model meters to gripper-motor radians."""

import argparse

import dora
import pyarrow as pa


def convert_position(position, out_open, in_open):
    """Rescale the gripper element (index 7) from meters to radians.

    Arm joints 0..6 pass through unchanged. The gripper (index 7) is a v1 model
    prismatic position in meters (closed 0, open ``in_open``); it is mapped
    linearly to radians so closed stays 0 and full-open maps to ``out_open``
    (the v2 gripper-motor convention).
    """
    result = list(position)
    result[-1] = out_open * (result[-1] / in_open)
    return result


def main():
    """Rescale IK gripper meters to gripper-motor radians for the followers."""
    parser = argparse.ArgumentParser(
        description="Convert the v1 IK gripper (prismatic meters) to motor radians.",
    )
    parser.add_argument(
        "--in-open",
        type=float,
        default=0.044,
        help="v1 model finger open position [m] (default: 0.044).",
    )
    parser.add_argument(
        "--out-open-right",
        type=float,
        default=-0.785,
        help="Right gripper motor angle at full open [rad] (default: -0.785).",
    )
    parser.add_argument(
        "--out-open-left",
        type=float,
        default=0.785,
        help="Left gripper motor angle at full open [rad] (default: 0.785).",
    )
    args = parser.parse_args()

    out_open = {
        "position_right": args.out_open_right,
        "position_left": args.out_open_left,
    }

    node = dora.Node()
    for event in node:
        if event["type"] != "INPUT":
            continue

        event_id = event["id"]
        if event_id not in out_open:
            continue

        position = event["value"].to_pylist()
        converted = convert_position(position, out_open[event_id], args.in_open)
        node.send_output(
            event_id,
            pa.array(converted, type=pa.float32()),
            event["metadata"],
        )


if __name__ == "__main__":
    main()
