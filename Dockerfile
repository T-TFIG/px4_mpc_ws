FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV PX4_VERSION=v1.17.0

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    wget \
    curl \
    nano \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# --- PX4-Autopilot: pinned release, build deps + Gazebo Harmonic via PX4's own setup script,
# then compile the default SITL config (this does NOT launch Gazebo/GUI, just builds the binaries
# that `make px4_sitl gz_x500` will later run from inside the container).
RUN git clone --recursive --branch ${PX4_VERSION} --depth 1 \
    https://github.com/PX4/PX4-Autopilot.git /PX4-Autopilot

WORKDIR /PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx
RUN make px4_sitl_default

# --- Micro XRCE-DDS Agent: bridges PX4 uORB topics to ROS2 DDS ---
RUN git clone -b v2.4.3 --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /Micro-XRCE-DDS-Agent \
    && cd /Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig /usr/local/lib/

# --- CasADi for the MPC formulation (Ubuntu 24.04 needs --break-system-packages for pip) ---
RUN pip3 install --no-cache-dir --break-system-packages casadi numpy matplotlib

# --- ROS 2 workspace ---
WORKDIR /px4_mpc_ws
COPY ./src ./src

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /px4_mpc_ws/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
