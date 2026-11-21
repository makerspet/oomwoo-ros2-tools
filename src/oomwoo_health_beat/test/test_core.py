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
"""Tests for the heartbeat payload builder."""

import json

from oomwoo_health_beat.core import BeatBuilder, stamp_from_nanoseconds, to_json

import pytest


def test_payload_fields():
    b = BeatBuilder('dock_detector')
    p = b.build(12.5, 'ok', 'found')
    assert p == {'component_id': 'dock_detector', 'health': 'ok',
                 'stamp_sec': 12.5, 'sequence': 1, 'detail': 'found'}


def test_sequence_counts_up_per_builder():
    b = BeatBuilder('a')
    assert [b.build(t)['sequence'] for t in (1.0, 2.0, 3.0)] == [1, 2, 3]
    assert BeatBuilder('b').build(1.0)['sequence'] == 1


def test_defaults_are_ok_and_empty_detail():
    p = BeatBuilder('a').build(1.0)
    assert p['health'] == 'ok' and p['detail'] == ''


def test_json_round_trip_keeps_types():
    p = BeatBuilder('a').build(3, 'warn')
    back = json.loads(to_json(p))
    assert back == p
    assert isinstance(back['stamp_sec'], float)
    assert isinstance(back['sequence'], int)


@pytest.mark.parametrize('bad', ['', None])
def test_component_id_is_required(bad):
    with pytest.raises(ValueError):
        BeatBuilder(bad)


def test_unknown_level_rejected_and_sequence_not_consumed():
    b = BeatBuilder('a')
    with pytest.raises(ValueError):
        b.build(1.0, 'fine')
    assert b.build(1.0)['sequence'] == 1


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf')])
def test_non_finite_time_rejected(bad):
    with pytest.raises(ValueError):
        BeatBuilder('a').build(bad)


def test_stamp_equals_the_monitor_clock_on_the_same_nanosecond():
    # the reference monitor reads its clock as nanoseconds / 1e9 and counts
    # age < 0 as stale; on a shared sim-time tick the age must be 0, not -1 ulp
    ticks = [50_000_000 * k for k in range(1, 36001)] + [1_795_033_482_122_512_600]
    assert all(ns / 1_000_000_000.0 - stamp_from_nanoseconds(ns) == 0.0 for ns in ticks)
    assert stamp_from_nanoseconds(150_000_000) == 0.15
    assert 150_000_000 * 1e-9 > 0.15      # the product overshoots by one ulp
