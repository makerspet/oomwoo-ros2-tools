# OOMWOO M1 — coverage + relocalization image (Jayadev Rana)
# SPDX-License-Identifier: Apache-2.0
#
# Fork of makerspet/oomwoo-install. Follows the oomwoo-install conventions:
# packages live in /ros_ws/src and are built in the stock /ros_ws workspace
# alongside the upstream packages — no separate overlay workspace to hunt down.
# The base image (makerspet/oomwoo:jazzy-dev) already provides ROS 2 Jazzy,
# Nav2, Gazebo, AMCL, slam_toolbox, the kaiaai platform and oomwoo_one.
#
# Build:
#   docker build -t jayadevrana/oomwoo-m1:jazzy \
#     --build-arg PKG_REPO=https://github.com/jayadevrana/oomwoo-m1-ros2.git \
#     -f deploy/Dockerfile .
#
# Run (interactive, like the makerspet tutorials):
#   docker run -it --name oom jayadevrana/oomwoo-m1:jazzy
#   # then at the container's bash prompt:
#   bash /ros_ws/src/oomwoo-m1/deploy/run_coverage_regression.sh
#
# Two ways to get the packages in, selected by USE_LOCAL:
#   * USE_LOCAL=0 (default): git clone PKG_REPO   (the self-hosted repo)
#   * USE_LOCAL=1          : COPY ./src           (local build context)

ARG BASE_IMAGE=makerspet/oomwoo:jazzy-dev
FROM ${BASE_IMAGE}

ARG PKG_REPO=https://github.com/jayadevrana/oomwoo-m1-ros2.git
ARG PKG_BRANCH=main
ARG USE_LOCAL=0

# Packages go into the stock workspace, per oomwoo-install convention.
WORKDIR /ros_ws
COPY deploy/_get_pkgs.sh /tmp/_get_pkgs.sh
COPY src /tmp/local_src
RUN chmod +x /tmp/_get_pkgs.sh && /tmp/_get_pkgs.sh

RUN . /opt/ros/jazzy/setup.sh \
 && . /ros_ws/install/setup.sh \
# rosdep must FAIL the build if a key doesn't resolve — masking it is how a
# bad dependency key ships. The base image has rosdep initialized already.
 && rosdep install --from-paths src/oomwoo-m1 --ignore-src -y \
 && colcon build --symlink-install \
      --packages-select oomwoo_coverage oomwoo_nav_localize oomwoo_sim_support \
 && rm -rf log /tmp/local_src

WORKDIR /ros_ws
