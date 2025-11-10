# Use ROS Noetic base image
FROM ros:noetic-ros-core

RUN rm /etc/apt/sources.list.d/ros1-latest.list \
  && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg


RUN apt-get update \
  && apt-get install -y ca-certificates curl


RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    g++ \
    build-essential \
    libcurl4-openssl-dev \
 && rm -rf /var/lib/apt/lists/*


# Set workspace
WORKDIR /workspace
RUN mkdir -p src

# Copy repo into src
COPY . /workspace/src

RUN /ros_entrypoint.sh catkin_make

# Set environment for ROS
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_PACKAGE_PATH=/workspace/src:$ROS_PACKAGE_PATH
ENV ROS_WORKSPACE=/workspace

# Run the test node automatically
CMD ["/ros_entrypoint.sh", "bash", "-c", "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && rosrun robot_base_test robot_base_test_node"]
