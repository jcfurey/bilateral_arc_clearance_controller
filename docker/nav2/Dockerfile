# ROS_DISTRO selects the ROS 2 distribution to verify against: jazzy or
# lyrical, the two the hosted CI builds. It is the same argument
# docker/nav2/verify.sh passes from BAC_ROS_DISTRO.
ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Resolve dependencies from the manifest in a separate layer. Source changes do
# not invalidate the ROS/Nav2 dependency installation. ROS_DISTRO is set by the
# base image, so the manifest's distribution conditions resolve here exactly as
# they do in the hosted CI.
COPY package.xml /tmp/bac-src/bilateral_arc_clearance_controller/package.xml
RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install \
      --from-paths /tmp/bac-src \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      --as-root apt:false \
      -r -y \
    && rm -rf /tmp/bac-src /var/lib/apt/lists/*

CMD ["bash"]
