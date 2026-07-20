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
"""Unit tests for the revisit-ratio re-clean classifier."""

from oomwoo_sim_support.coverage_meter_node import _chunk_is_reclean


# 0.25 m chunk, 0.05 m cells -> 5 cells of travel; rad 4 cells.
# virgin expectation = 2*rad*(chunk_len/res) = 2*4*5 = 40 new cells.
RAD = 4
RES = 0.05
CHUNK = 0.25


def test_virgin_sweep_is_not_reclean():
    # a full virgin swath stamps ~40 new cells -> well above the 0.25 threshold
    assert not _chunk_is_reclean(CHUNK, chunk_new=40, rad=RAD, res=RES)
    assert not _chunk_is_reclean(CHUNK, chunk_new=12, rad=RAD, res=RES)


def test_driving_over_cleaned_floor_is_reclean():
    # zero or a handful of new cells over a 0.25 m chunk = re-cleaning
    assert _chunk_is_reclean(CHUNK, chunk_new=0, rad=RAD, res=RES)
    assert _chunk_is_reclean(CHUNK, chunk_new=5, rad=RAD, res=RES)


def test_threshold_is_a_quarter_of_virgin():
    # expected virgin = 40; boundary at 0.25*40 = 10
    assert _chunk_is_reclean(CHUNK, chunk_new=9, rad=RAD, res=RES)
    assert not _chunk_is_reclean(CHUNK, chunk_new=10, rad=RAD, res=RES)


def test_scales_with_chunk_length():
    # a longer chunk expects proportionally more new cells before it counts
    # as virgin, so the same new-cell count can flip from virgin to re-clean
    assert not _chunk_is_reclean(0.25, chunk_new=11, rad=RAD, res=RES)
    assert _chunk_is_reclean(0.50, chunk_new=11, rad=RAD, res=RES)
