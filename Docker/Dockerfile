# Choose the base image at build time. Defaults to the second option.
ARG BASE_IMAGE="pyrobosim_ros:jazzy"

# You can refer to ARGs in FROM (supported by modern Docker)
FROM ${BASE_IMAGE} AS base

# (Optional) re-declare the arg if you want to use it in later instructions
ARG BASE_IMAGE

# Set default user/group variables
# ARG USER_NAME=coghri
# ARG GROUP_NAME=coghri-devs
# ARG USER_ID=1001
# ARG GROUP_ID=1001

ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------
# Install system dependencies & development tools
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    terminator \
    gosu  \
    # ros-${ROS_DISTRO}-usb-cam \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /roscon25_ws/src

# Copy requirements.txt and install Python dependencies using uv
# COPY requirements.txt /tmp/requirements.txt
# RUN /bin/bash -c "source /opt/ros_python_env/bin/activate && uv pip install --no-cache -r /tmp/requirements.txt"
# RUN uv pip install --system --no-cache --break-system-packages -r requirements.txt

COPY deps/BehaviorTree.CPP /roscon25_ws/src/deps/BehaviorTree.CPP
COPY deps/behaviortree_eut_plugins /roscon25_ws/src/deps/behaviortree_eut_plugins
COPY deps/rosx_introspection /roscon25_ws/src/deps/rosx_introspection
COPY deps/BehaviorTree.ROS2 /roscon25_ws/src/deps/BehaviorTree.ROS2

# ------------------------------------------------------------
# Setup rosdep
# (init system-wide, update for this user)
# ------------------------------------------------------------

WORKDIR /roscon25_ws
# Optional: fix rosdep init issue gracefully
RUN [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ] /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash;source /pyrobosim_ws/install/setup.bash;sudo rosdep init" || echo "rosdep already initialized"
RUN apt-get update && \ 
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; source /pyrobosim_ws/install/setup.bash;\
    rosdep update;"


# ------------------------------------------------------------
# Install dependencies
# (from workspace package.xmls via rosdep)
# ------------------------------------------------------------
RUN rosdep install --from-paths src --ignore-src -r -y

# ------------------------------------------------------------
# Build workspace
# ------------------------------------------------------------
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash;source /pyrobosim_ws/install/setup.bash; colcon build --symlink-install --event-handlers console_direct+;"


# Copy and set up the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]
