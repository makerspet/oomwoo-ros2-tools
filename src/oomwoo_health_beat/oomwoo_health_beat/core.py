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
Payload builder for work-path heartbeats (no ROS imports).

A heartbeat says "one unit of real work just completed". The payload is the
JSON object the stack health monitor expects on /oomwoo/health/component:

  component_id  str    which roster entry this beat satisfies
  health        str    ok | healthy | warn | degraded | error
  stamp_sec     float  the publishing node's ROS clock, seconds
  sequence      int    1, 2, 3, ... per component (a gap means a lost beat)
  detail        str    free text, may be empty

The builder owns no timer and no clock: the caller passes the time, so a beat
can only follow work that was actually done.
"""

import json
import math

HEALTH_LEVELS = frozenset({'ok', 'healthy', 'warn', 'degraded', 'error'})
INITIAL_SEQUENCE = 1


class BeatBuilder:
    """Build heartbeat payloads for one component."""

    def __init__(self, component_id: str) -> None:
        """Remember the component id; reject an empty one."""
        if not component_id:
            raise ValueError('component_id is required')
        self.component_id = component_id
        self._next_sequence = INITIAL_SEQUENCE

    def build(self, now_sec: float, level: str = 'ok', detail: str = '') -> dict:
        """Return the payload for one completed unit of work at time now_sec."""
        if level not in HEALTH_LEVELS:
            raise ValueError('unknown health level: %r' % (level,))
        if not math.isfinite(now_sec):
            raise ValueError('now_sec must be finite')
        payload = {
            'component_id': self.component_id,
            'health': level,
            'stamp_sec': float(now_sec),
            'sequence': self._next_sequence,
            'detail': detail,
        }
        self._next_sequence += 1
        return payload


def stamp_from_nanoseconds(nanoseconds: int) -> float:
    """
    Convert a ROS time in nanoseconds to stamp_sec.

    True division, the same expression the reference health monitor uses for
    its own clock. nanoseconds * 1e-9 comes out one unit in the last place
    higher for about 4 values in 10 (150 ms gives 0.15000000000000002), so a
    beat stamped on the monitor's own sim-time tick would read as newer than
    the monitor's clock (age < 0), which the monitor counts as stale.
    """
    return nanoseconds / 1e9


def to_json(payload: dict) -> str:
    """Serialise a payload for the std_msgs/String transport."""
    return json.dumps(payload, sort_keys=True)
