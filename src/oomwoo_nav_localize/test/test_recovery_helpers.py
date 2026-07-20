# Copyright 2026 Jayadev Rana
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
"""Unit tests for the kidnap-recovery pose/grid helpers."""

import math

import numpy as np

from oomwoo_nav_localize.kidnap_recovery_node import (
    _dilate_bool,
    _pose_differs,
)


def test_pose_differs_on_distance():
    assert _pose_differs((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), d_m=1.0)
    assert not _pose_differs((0.0, 0.0, 0.0), (0.5, 0.0, 0.0), d_m=1.0)


def test_pose_differs_on_yaw():
    assert _pose_differs((0.0, 0.0, 0.0), (0.0, 0.0, 2.0), d_yaw=1.0)
    assert not _pose_differs((0.0, 0.0, 0.0), (0.0, 0.0, 0.5), d_yaw=1.0)


def test_pose_differs_yaw_wraps_at_pi():
    # headings of +pi-0.05 and -pi+0.05 are only 0.1 rad apart, not ~2*pi
    a = (0.0, 0.0, math.pi - 0.05)
    b = (0.0, 0.0, -math.pi + 0.05)
    assert not _pose_differs(a, b, d_yaw=1.0), \
        'yaw difference must wrap through +/-pi'


def test_dilate_bool_square_growth():
    mask = np.zeros((7, 7), dtype=bool)
    mask[3, 3] = True
    out = _dilate_bool(mask, 1)
    assert bool(out[2, 3]) and bool(out[3, 2]) and bool(out[3, 4])
    assert int(mask.sum()) == 1, 'input mask must not be mutated'
