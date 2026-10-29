# Copyright 2026 Valdi320
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

"""ROS2 Jazzy launch graph checks: no Gazebo spawn or motor activity."""
import importlib.util
import tempfile
import unittest
from pathlib import Path

import yaml

from oomwoo_sim_support.keepout_config import _pgm

try:
    from launch import LaunchContext
    from launch.actions import OpaqueFunction, SetLaunchConfiguration
except ImportError:
    LaunchContext = None


@unittest.skipIf(LaunchContext is None, "ROS2 Jazzy launch is unavailable")
class KeepoutLaunchGraphTests(unittest.TestCase):
    """Evaluate launch argument branching without starting the sim."""

    def setUp(self):
        self.pkg = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location(
            "oomwoo_sim_keepout_launch", self.pkg / "launch/sim_bringup.launch.py")
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.description = self.module.generate_launch_description()
        self.opaque = [a for a in self.description.entities
                       if isinstance(a, OpaqueFunction)]
        self.assertEqual(len(self.opaque), 1)

    def evaluate(self, mask):
        ctx = LaunchContext()
        for key, value in {
            "robot_model": "oomwoo_one",
            "keepout_mask": mask,
            "map": str(self.pkg / "maps/test_room.yaml"),
            "nav2_params_file": str(self.pkg / "config/nav2_params.yaml"),
            "x_pose": "0.0", "y_pose": "0.0", "yaw": "0.0",
            "world": "unused", "with_nav": "true", "gui": "false",
        }.items():
            ctx.launch_configurations[key] = value
        return ctx, self.opaque[0].execute(ctx)

    def test_default_graph_does_not_enable_filters(self):
        _, actions = self.evaluate("")
        self.assertFalse(any(isinstance(a, SetLaunchConfiguration)
                             for a in actions))
        self.assertEqual(len(actions), 6)

    def test_optin_graph_generates_both_costmap_filters(self):
        with tempfile.TemporaryDirectory() as tmp_name:
            tmp = Path(tmp_name)
            mapdata = yaml.safe_load(
                (self.pkg / "maps/test_room.yaml").read_text())
            width, height, _ = _pgm(self.pkg / "maps" / mapdata["image"])
            pix = bytearray([255] * (width * height))
            pix[(height // 2) * width + width // 2] = 0
            (tmp / "mask.pgm").write_bytes(
                f"P5\n{width} {height}\n255\n".encode() + pix)
            mapdata["image"] = "mask.pgm"
            (tmp / "mask.yaml").write_text(yaml.safe_dump(mapdata))
            ctx, actions = self.evaluate(str(tmp / "mask.yaml"))
            setters = [a for a in actions
                       if isinstance(a, SetLaunchConfiguration)]
            self.assertEqual(len(setters), 1)
            setters[0].execute(ctx)
            params = yaml.safe_load(Path(
                ctx.launch_configurations["nav2_params_file"]).read_text())
            for name in ("global_costmap", "local_costmap"):
                self.assertEqual(
                    params[name][name]["ros__parameters"]["filters"],
                    ["keepout_filter", "keepout_inflation"])
            self.assertEqual(len(actions), 11)
