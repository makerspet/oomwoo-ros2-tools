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

"""Opt-in Nav2 KeepoutFilter overlay for the OOMWOO headless simulator.

Does not modify source params, does not drive hardware, and does not
authorize reactive cmd_vel coverage. Python 3.10+/ROS2 Jazzy.
"""
from __future__ import annotations

import math
from pathlib import Path

import yaml

MASK_TOPIC = "/oomwoo/keepout_mask"
INFO_TOPIC = "/oomwoo/keepout_filter_info"


def _metadata(path: Path) -> dict:
    """Read map metadata with no escaping image path and finite geometry."""
    if not path.is_file():
        raise ValueError(f"missing map YAML: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("invalid YAML map")
    if not all(k in data for k in ("image", "resolution", "origin")):
        raise ValueError("missing map metadata fields")
    image = Path(str(data["image"]))
    if image.is_absolute() or ".." in image.parts:
        raise ValueError("unsafe image path")
    target = (path.parent / image).resolve()
    if path.parent.resolve() not in target.parents:
        raise ValueError("image escapes map directory")
    data["_image_path"] = target
    r, origin = data["resolution"], data["origin"]
    if (isinstance(r, bool) or not isinstance(r, (int, float))
            or not math.isfinite(r) or r <= 0
            or not isinstance(origin, (list, tuple)) or len(origin) != 3
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in origin)
            or abs(origin[2]) > 1e-9):
        raise ValueError("nonfinite or rotated map geometry")
    return data


def _pgm(path: Path) -> tuple[int, int, bytes]:
    """Read bounded P5 (8-bit), including comments in the text header."""
    if not path.is_file() or path.stat().st_size > 20_000_128:
        raise ValueError("missing or oversized PGM")
    raw = path.read_bytes()
    pos = 0

    def token():
        nonlocal pos
        while pos < len(raw):
            if raw[pos] in b" \t\r\n":
                pos += 1
            elif raw[pos] == 35:
                end = raw.find(b"\n", pos)
                pos = len(raw) if end < 0 else end + 1
            else:
                break
        start = pos
        while pos < len(raw) and raw[pos] not in b" \t\r\n":
            pos += 1
        if start == pos:
            raise ValueError("truncated PGM header")
        return raw[start:pos]

    if token() != b"P5":
        raise ValueError("only P5 PGM supported")
    try:
        width, height, maxval = int(token()), int(token()), int(token())
    except ValueError as ex:
        raise ValueError("invalid PGM dimensions") from ex
    if (width < 1 or height < 1 or width * height > 20_000_000
            or maxval != 255 or pos >= len(raw)
            or raw[pos] not in b" \t\r\n"):
        raise ValueError("invalid PGM header")
    pixels = raw[pos + 1:]
    if len(pixels) != width * height:
        raise ValueError("truncated or excess PGM pixel data")
    return width, height, pixels


def build_keepout_overlay(
    nav_params_file: str | Path,
    map_yaml: str | Path,
    mask_yaml: str | Path,
    *,
    output_dir: str | Path,
) -> tuple[Path, int]:
    """Validate mask/map pair and write a NEW Nav2 params file, never overwrite."""
    source = _metadata(Path(map_yaml).resolve())
    mask = _metadata(Path(mask_yaml).resolve())
    sw, sh, _ = _pgm(source["_image_path"])
    mw, mh, pixels = _pgm(mask["_image_path"])
    if (sw, sh) != (mw, mh):
        raise ValueError("mask dimensions differ from map")
    if (abs(source["resolution"] - mask["resolution"]) > 1e-9
            or any(abs(float(x) - float(y)) > 1e-8
                   for x, y in zip(source["origin"], mask["origin"]))):
        raise ValueError("mask origin/resolution differ from map")
    if (mask.get("mode") != "trinary" or mask.get("negate") != 0
            or mask.get("occupied_thresh") != 0.65
            or mask.get("free_thresh") != 0.25):
        raise ValueError("mask must be exact trinary, non-negated 0/255")
    if any(v not in (0, 255) for v in pixels):
        raise ValueError("ambiguous gray or unknown mask pixels")
    forbidden = pixels.count(0)
    if forbidden == 0 or forbidden == len(pixels):
        raise ValueError("mask requires both forbidden and free cells")

    nav = yaml.safe_load(Path(nav_params_file).read_text(encoding="utf-8"))
    if not isinstance(nav, dict):
        raise ValueError("invalid Nav2 parameters")
    for costmap in ("global_costmap", "local_costmap"):
        try:
            params = nav[costmap][costmap]["ros__parameters"]
            radius = float(params["robot_radius"])
        except (KeyError, TypeError, ValueError) as ex:
            raise ValueError(f"missing {costmap} robot radius") from ex
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError("invalid robot radius")
        if params.get("filters"):
            raise ValueError(f"{costmap} already has filters")
        params["filters"] = ["keepout_filter", "keepout_inflation"]
        params["keepout_filter"] = {
            "plugin": "nav2_costmap_2d::KeepoutFilter",
            "enabled": True,
            "filter_info_topic": INFO_TOPIC,
            "override_lethal_cost": False,
            "transform_tolerance": 0.3,
        }
        params["keepout_inflation"] = {
            "plugin": "nav2_costmap_2d::InflationLayer",
            "enabled": True,
            "inflation_radius": round(radius + 0.06, 4),
            "cost_scaling_factor": 5.0,
        }
    target = Path(output_dir)
    if not target.is_dir():
        raise ValueError("output directory must exist")
    outfile = target / "nav2_params_with_keepout.yaml"
    if outfile.exists():
        raise ValueError("refuse to overwrite existing overlay")
    outfile.write_text(yaml.safe_dump(nav, sort_keys=False), encoding="utf-8")
    return outfile, forbidden
