FROM ghcr.io/sdustio/ros2:base

COPY . /openspot/src/openspot_teleop_kb

RUN set -ex; \
    cd /openspot; \
    . "/opt/ros/$ROS_DISTRO/setup.sh"; \
    rosdep install --from-paths src --ignore-src -y; \
    colcon build; \
    rm -rf build
