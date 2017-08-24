FROM osrf/ros:kinetic-desktop-full

MAINTAINER W. Nicholas Greene "wng@csail.mit.edu"

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Install general software dependencies.
RUN apt-get update
RUN apt-get install -y apt-utils lsb-release
RUN apt-get install -y git openssh-client wget
RUN apt-get install -y sudo && rm -rf /var/lib/apt/lists/*

# Install flame dependencies.
RUN apt-get install -y libboost-all-dev libpcl-dev

# Install catkin_tools.
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y python-catkin-tools

# Create a catkin workspace.
RUN mkdir -p flame_ws/src

# Clone the repo into the docker container.
RUN cd flame_ws/src && git clone git@github.com:robustrobotics/flame.git
RUN cd flame_ws/src && git clone git@github.com:robustrobotics/flame_ros.git

# Initialize the workspace.
RUN cd flame_ws && catkin init

# Install rosdeps.
RUN /bin/bash -c "source /opt/ros/kinetic/setup.sh && cd flame_ws && rosdep install -iy --from-paths ./src"

# Install Eigen and Sophus.
RUN mkdir -p flame_ws/dependencies/src
RUN flame_ws/src/flame/scripts/eigen.sh /flame_ws/dependencies/src /flame_ws/dependencies
RUN flame_ws/src/flame/scripts/sophus.sh /flame_ws/dependencies/src /flame_ws/dependencies
RUN cp flame_ws/src/flame/scripts/env.sh /flame_ws/dependencies/

# Now build flame and flame_ros.
RUN /bin/bash -c "source /opt/ros/kinetic/setup.sh && cd flame_ws && source ./dependencies/env.sh && catkin build"

# Add sourcing commands bashrc.
RUN echo "source /flame_ws/devel/setup.bash" >> ~/.bashrc

# Download and extract EuRoC dataset.
RUN wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip
RUN mkdir data && cd data && unzip ../V1_01_easy.zip

# Edit the configuarion yaml to point to the extracted data.
RUN sed 's#/home/wng/Projects/rrg/data/euroc_mav_datasets/V1_01_easy#/data#g' -i /flame_ws/src/flame_ros/cfg/flame_offline_asl.yaml

