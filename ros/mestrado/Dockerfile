ARG ROS_DISTRO=noetic
ARG LIBREALSENSE_VERSION=2.51.1-0~realsense0.7526


# From base ros perception image
FROM ros:$ROS_DISTRO-perception


# The default shell during build is not bash
# Needed to allow later build steps to source the workspace
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install librealsense
ARG LIBREALSENSE_VERSION
RUN apt-get update \
    # Install build tools
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
        build-essential \
        cmake \
        ninja-build \
        software-properties-common \
    # Install librealsense
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --no-install-recommends \
        librealsense2-dkms \
        librealsense2=$LIBREALSENSE_VERSION \
        librealsense2-gl=$LIBREALSENSE_VERSION \
        librealsense2-net=$LIBREALSENSE_VERSION \
        librealsense2-dev=$LIBREALSENSE_VERSION \
        librealsense2-utils=$LIBREALSENSE_VERSION \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
ARG ROS_DISTRO
ENV ROS_DISTRO=$ROS_DISTRO
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        python3-osrf-pycommon \
        python3-catkin-tools \
        # delivery_hardware dependencies
        ros-$ROS_DISTRO-resource-retriever \
        ros-$ROS_DISTRO-audio-common \
        ros-$ROS_DISTRO-ddynamic-reconfigure \
        ros-$ROS_DISTRO-depth-image-proc \
        ros-$ROS_DISTRO-graph-msgs \
        ros-$ROS_DISTRO-image-proc \
        ros-$ROS_DISTRO-libuvc-camera \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-teleop-twist-joy \
        ros-$ROS_DISTRO-tf-conversions\
        ros-$ROS_DISTRO-usb-cam \
        ros-$ROS_DISTRO-xacro \
        ros-$ROS_DISTRO-rosmon \
        ros-$ROS_DISTRO-vision-msgs \
        ros-$ROS_DISTRO-rtabmap-ros \
    # Link python executable to python3 executable
    && if [ ! -f "/usr/bin/python" ]; then \
        ln -s /usr/bin/python3 /usr/bin/python; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# Copy delivery_hardware repository
ENV MESTRADO_WS=/root/mestrado_ws
COPY . $MESTRADO_WS/src

# Build delivery_hardware

WORKDIR $MESTRADO_WS
RUN catkin config \
        --extend /opt/ros/${ROS_DISTRO} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build --no-status mestrado -j$(($(nproc)-1))
    # Install all python dependencies
    RUN for file in $(find ./src -type f -name "requirements.txt"); do \
        echo "Installing python requirements from $file"; \
        pip3 install -r $file; \
    done

RUN echo "source \$MESTRADO_WS/devel/setup.bash" >> /root/.bashrc
COPY ros_entrypoint.sh /

WORKDIR $MESTRADO_WS
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]