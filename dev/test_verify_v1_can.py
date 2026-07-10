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


import verify_v1_can as v


CFG = {
    "can_interface": {"right_arm": "can0", "left_arm": "can1"},
    "motor_config": {
        "types": [
            "DM8009",
            "DM8009",
            "DM4340",
            "DM4340",
            "DM4310",
            "DM4310",
            "DM4310",
            "DM4310",
        ],
        "send_ids": [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
        "recv_ids": [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18],
    },
}


def test_arm_gets_first_seven_motors():
    args = v.motor_args_from_config(CFG, "right")

    assert args.arm_types == [
        "DM8009",
        "DM8009",
        "DM4340",
        "DM4340",
        "DM4310",
        "DM4310",
        "DM4310",
    ]
    assert args.arm_send_ids == [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    assert args.arm_recv_ids == [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]


def test_gripper_gets_last_motor():
    args = v.motor_args_from_config(CFG, "right")

    assert args.gripper_type == "DM4310"
    assert args.gripper_send_id == 0x08
    assert args.gripper_recv_id == 0x18


def test_can_interface_is_per_side():
    assert v.motor_args_from_config(CFG, "right").can == "can0"
    assert v.motor_args_from_config(CFG, "left").can == "can1"
