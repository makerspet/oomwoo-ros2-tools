# Copyright 2026 OOMWOO
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
"""Unit-test the dynamic-blob clustering the placeholder detector uses."""

import numpy as np

from oomwoo_perception.dynamic_object_detector_node import cluster_dynamic


def test_contiguous_dynamic_rays_form_one_blob():
    dynamic = np.array([False] * 5 + [True] * 6 + [False] * 5)
    n = dynamic.size
    xs = np.arange(n) * 0.05      # 5 cm apart, within gap
    ys = np.zeros(n)
    blobs = cluster_dynamic(dynamic, xs, ys, 0.20, 3)
    assert len(blobs) == 1
    cx, _cy, cnt = blobs[0]
    assert cnt == 6
    assert abs(cx - xs[5:11].mean()) < 1e-9


def test_too_small_run_is_ignored():
    dynamic = np.array([False] * 5 + [True] * 2 + [False] * 5)
    n = dynamic.size
    xs = np.arange(n) * 0.05
    ys = np.zeros(n)
    assert cluster_dynamic(dynamic, xs, ys, 0.20, 3) == []
