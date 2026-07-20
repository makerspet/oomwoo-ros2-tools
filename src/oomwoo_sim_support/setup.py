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

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'oomwoo_sim_support'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jayadev Rana',
    maintainer_email='jayadevrana@users.noreply.github.com',
    description='Headless sim bringup, ground-truth measurement, and regression tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth = oomwoo_sim_support.ground_truth_node:main',
            'coverage_meter = oomwoo_sim_support.coverage_meter_node:main',
            'kidnap_injector = oomwoo_sim_support.kidnap_injector_node:main',
            'initialpose_pub = oomwoo_sim_support.initialpose_pub_node:main',
            'reloc_regression_runner = oomwoo_sim_support.reloc_regression_runner:main',
            'coverage_regression_runner = oomwoo_sim_support.coverage_regression_runner:main',
        ],
    },
)
