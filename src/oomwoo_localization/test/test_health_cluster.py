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
"""
Unit-test the outlier clustering that drives dynamic-obstacle filtering.

A contiguous run of unmatched beams (a box/chair) must become one cluster, so
its beams get stripped from /scan_filtered; a stray beam or two must stay noise,
so a real wall is never blanked. Calls the node's _label directly with a stub
self, so no ROS/sim is needed.
"""

import numpy as np

from oomwoo_localization import localization_health_node as lh


class _Params:

    cluster_gap = 0.20
    min_cluster = 4


def test_contiguous_outliers_form_one_cluster():
    inlier = np.array([True] * 10 + [False] * 6 + [True] * 10)
    n = inlier.size
    mx = np.arange(n) * 0.05      # 5 cm apart, within cluster_gap
    my = np.zeros(n)
    labels = lh.LocalizationHealth._label(_Params(), inlier, mx, my)
    assert (labels[:10] == lh.LBL_INLIER).all()
    assert (labels[10:16] == lh.LBL_CLUSTER0).all()    # the box -> one cluster
    assert (labels[16:] == lh.LBL_INLIER).all()


def test_short_outlier_run_stays_noise():
    inlier = np.array([True] * 10 + [False] * 2 + [True] * 10)
    n = inlier.size
    mx = np.arange(n) * 0.05
    my = np.zeros(n)
    labels = lh.LocalizationHealth._label(_Params(), inlier, mx, my)
    assert (labels[10:12] == lh.LBL_OUTLIER).all()     # too short -> not a cluster


def test_static_score_is_bounded_and_falls_off():
    s = lh.static_score(np.array([0.0, 0.10, np.inf]), 0.10)
    assert abs(s[0] - 1.0) < 1e-6      # endpoint on a wall -> fully static
    assert 0.5 < s[1] < 0.7            # one sigma out -> exp(-0.5) ~ 0.61
    assert s[2] < 0.01                # dynamic / off-map return -> ~0
