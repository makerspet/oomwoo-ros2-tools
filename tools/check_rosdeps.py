#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Static dependency lint: every package.xml key must resolve, both ways.

Two checks, exit 1 on any finding — wire it into CI:

1. DECLARED -> RESOLVABLE: every <depend> in every package.xml must be either
   a ROS package released in the target distro or a rosdep key (base.yaml /
   python.yaml). Catches e.g. `numpy` (a pip name, not a rosdep key —
   `python3-numpy` is the key), which breaks `rosdep install` on any clean
   machine even though `colcon build` sails past it.
2. IMPORTED -> DECLARED: every third-party module imported by the package's
   Python sources must be covered by a declared dependency. Catches the
   mirror bug: code that works only because some other package dragged the
   dependency in.

Known false-pass classes (this lint is narrower than real rosdep resolution):
check 1 counts a rosdistro repo as "released" even when it has no versioned
release section (source/doc-only repos), so a depend on such a repo name passes
here but still fails `rosdep install` on a clean machine; and the depend regex
never scans <buildtool_depend>/<build_export_depend> tags, which `rosdep
install` does resolve.

Usage:  python3 tools/check_rosdeps.py [--distro jazzy] [src_dir]
Needs network on first run (fetches the rosdep db + distro index to /tmp).
"""
import argparse
import glob
import os
import re
import sys
import urllib.request

import yaml

ROSDISTRO = 'https://raw.githubusercontent.com/ros/rosdistro/master'
# stdlib + relative imports that never need declaring
IGNORE_IMPORTS = {
    'argparse', 'collections', 'dataclasses', 'enum', 'functools', 'glob',
    'itertools', 'json', 'math', 'os', 'random', 're', 'subprocess', 'sys',
    'tempfile', 'time', 'typing', 'unittest', 'xml',
}
# import name -> the dependency key that provides it
IMPORT_TO_KEY = {'numpy': 'python3-numpy', 'yaml': 'python3-yaml',
                 'PIL': 'python3-pil', 'scipy': 'python3-scipy',
                 'pytest': 'python3-pytest'}


def fetch(url, dest):
    if not os.path.exists(dest):
        urllib.request.urlretrieve(url, dest)
    return yaml.safe_load(open(dest))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src', nargs='?', default='src')
    ap.add_argument('--distro', default='jazzy')
    a = ap.parse_args()

    rosdep = {}
    rosdep.update(fetch(f'{ROSDISTRO}/rosdep/base.yaml', '/tmp/_rosdep_base.yaml'))
    rosdep.update(fetch(f'{ROSDISTRO}/rosdep/python.yaml', '/tmp/_rosdep_python.yaml'))
    dist = fetch(f'{ROSDISTRO}/{a.distro}/distribution.yaml',
                 f'/tmp/_dist_{a.distro}.yaml')
    ros_pkgs = set()
    for repo, data in dist.get('repositories', {}).items():
        for p in (data.get('release', {}) or {}).get('packages', [repo]):
            ros_pkgs.add(p)

    findings = []
    for pkg_xml in sorted(glob.glob(os.path.join(a.src, '*', 'package.xml'))):
        pkg_dir = os.path.dirname(pkg_xml)
        pkg = os.path.basename(pkg_dir)
        deps = set(re.findall(r'<(?:build_|exec_|test_)?depend[^>]*>([^<]+)</',
                              open(pkg_xml).read()))

        for d in sorted(deps):                       # check 1
            if d not in ros_pkgs and d not in rosdep:
                findings.append(
                    f'{pkg_xml}: "{d}" is neither a {a.distro} package nor a '
                    f'rosdep key — rosdep install will fail on it')

        imports = set()                              # check 2
        for py in glob.glob(os.path.join(pkg_dir, '**', '*.py'), recursive=True):
            if os.path.basename(py) == 'setup.py':
                continue                             # packaging, not runtime
            for line in open(py, encoding='utf-8', errors='replace'):
                if 'dep-optional' in line:
                    continue     # guarded import with a working fallback
                m = re.match(r'\s*(?:import|from)\s+([A-Za-z_][A-Za-z0-9_]*)', line)
                if m:
                    imports.add(m.group(1))
        for mod in sorted(imports):
            if mod in IGNORE_IMPORTS or mod == pkg or mod.startswith('_'):
                continue
            key = IMPORT_TO_KEY.get(mod, mod)
            if key not in deps and mod not in deps:
                findings.append(
                    f'{pkg}: imports "{mod}" but declares neither "{mod}" '
                    f'nor "{key}" in {pkg_xml}')

    for f in findings:
        print(f'FAIL {f}')
    if findings:
        sys.exit(1)
    print(f'OK: all declared deps resolve ({a.distro}) and all imports are declared')


if __name__ == '__main__':
    main()
