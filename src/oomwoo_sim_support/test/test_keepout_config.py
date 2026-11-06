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

"""Pure-Python tests for the opt-in simulator keepout overlay."""
import tempfile
import unittest
from pathlib import Path

import yaml
from oomwoo_sim_support.keepout_config import build_keepout_overlay


class KeepoutConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.map = self.root / "map.yaml"
        self.mask = self.root / "mask.yaml"
        self.nav = self.root / "nav.yaml"
        self.output = self.root / "output"
        self.output.mkdir()
        self.pgm("map.pgm", bytes([255] * 12))
        self.pgm("mask.pgm", bytes([255] * 5 + [0] + [255] * 6))
        for path, image in ((self.map, "map.pgm"), (self.mask, "mask.pgm")):
            path.write_text(yaml.safe_dump({
                "image": image, "mode": "trinary", "resolution": .05,
                "origin": [-.1, -.2, 0], "negate": 0,
                "occupied_thresh": .65, "free_thresh": .25,
            }), encoding="utf-8")
        self.nav.write_text(yaml.safe_dump({
            n: {n: {"ros__parameters": {
                "robot_radius": .1745,
                "plugins": ["static_layer", "inflation_layer"],
            }}} for n in ("global_costmap", "local_costmap")
        }), encoding="utf-8")

    def pgm(self, name, pixels, width=4, height=3):
        (self.root / name).write_bytes(
            f"P5\n{width} {height}\n255\n".encode() + pixels)

    def build(self):
        return build_keepout_overlay(self.nav, self.map, self.mask,
                                     output_dir=self.output)

    def test_both_costmaps_and_unchanged_stock(self):
        original = self.nav.read_bytes()
        out, n = self.build()
        self.assertEqual(n, 1)
        self.assertEqual(self.nav.read_bytes(), original)
        data = yaml.safe_load(out.read_text())
        for name in ("global_costmap", "local_costmap"):
            params = data[name][name]["ros__parameters"]
            self.assertEqual(params["filters"],
                             ["keepout_filter", "keepout_inflation"])
            self.assertEqual(params["keepout_filter"]["filter_info_topic"],
                             "/oomwoo/keepout_filter_info")
            self.assertGreater(params["keepout_inflation"]["inflation_radius"],
                               params["robot_radius"])
            self.assertEqual(params["plugins"],
                             ["static_layer", "inflation_layer"])

    def test_dimensions_differ_fail_closed(self):
        self.pgm("mask.pgm", bytes([255] * 19 + [0]), 5, 4)
        with self.assertRaisesRegex(ValueError, "dimensions"):
            self.build()

    def test_origin_differ_fail_closed(self):
        meta = yaml.safe_load(self.mask.read_text())
        meta["origin"][0] += .05
        self.mask.write_text(yaml.safe_dump(meta))
        with self.assertRaisesRegex(ValueError, "origin/resolution"):
            self.build()

    def test_unknown_gray_mask_fail_closed(self):
        self.pgm("mask.pgm", bytes([0, 127] + [255] * 10))
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            self.build()

    def test_blank_and_total_mask_fail_closed(self):
        for pixels in (bytes([255] * 12), bytes([0] * 12)):
            self.pgm("mask.pgm", pixels)
            with self.assertRaisesRegex(ValueError, "both forbidden and free"):
                self.build()

    def test_truncated_pgm_fail_closed(self):
        self.pgm("mask.pgm", bytes([255] * 4))
        with self.assertRaisesRegex(ValueError, "truncated"):
            self.build()

    def test_unsafe_image_path_fail_closed(self):
        meta = yaml.safe_load(self.mask.read_text())
        meta["image"] = "../mask.pgm"
        self.mask.write_text(yaml.safe_dump(meta))
        with self.assertRaisesRegex(ValueError, "unsafe image path"):
            self.build()

    def test_existing_filter_not_overridden(self):
        params = yaml.safe_load(self.nav.read_text())
        params["local_costmap"]["local_costmap"]["ros__parameters"]["filters"] = ["other"]
        self.nav.write_text(yaml.safe_dump(params))
        with self.assertRaisesRegex(ValueError, "already has filters"):
            self.build()

    def test_no_overwrite_derived_yaml(self):
        self.build()
        with self.assertRaisesRegex(ValueError, "refuse to overwrite"):
            self.build()

    def test_missing_source_file_fails_closed(self):
        self.map.unlink()
        with self.assertRaisesRegex(ValueError, "missing map YAML"):
            self.build()


if __name__ == "__main__":
    unittest.main()
