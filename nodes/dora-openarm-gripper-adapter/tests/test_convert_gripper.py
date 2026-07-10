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


import pytest


import dora_openarm_gripper_adapter.main as main_module  # noqa: E402


def test_closed_gripper_maps_to_zero_on_both_sides():
    # 0 m (jaw closed) -> 0 rad regardless of the open radian.
    position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.0]

    right = main_module.convert_position(position, out_open=-0.785, in_open=0.044)
    left = main_module.convert_position(position, out_open=0.785, in_open=0.044)

    assert right[-1] == pytest.approx(0.0)
    assert left[-1] == pytest.approx(0.0)


def test_full_open_meters_maps_to_open_radian():
    # 0.044 m (jaw fully open) -> the per-side open radian.
    position = [0.0] * 7 + [0.044]

    right = main_module.convert_position(position, out_open=-0.785, in_open=0.044)
    left = main_module.convert_position(position, out_open=0.785, in_open=0.044)

    assert right[-1] == pytest.approx(-0.785)
    assert left[-1] == pytest.approx(0.785)


def test_half_open_scales_linearly():
    position = [0.0] * 7 + [0.022]  # half of 0.044

    result = main_module.convert_position(position, out_open=-0.785, in_open=0.044)

    assert result[-1] == pytest.approx(-0.3925)


def test_arm_joints_pass_through_unchanged():
    position = [-1.396263, 0.5, -0.7, 1.5, 0.1, -0.2, 0.3, 0.044]

    result = main_module.convert_position(position, out_open=-0.785, in_open=0.044)

    assert result[:7] == pytest.approx(position[:7])


def test_output_has_eight_elements():
    position = [0.0] * 8

    result = main_module.convert_position(position, out_open=-0.785, in_open=0.044)

    assert len(result) == 8
