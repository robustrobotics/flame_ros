# flame_ros
**FLaME** (Fast Lightweight Mesh Estimation) is a lightweight, CPU-only method
for dense online monocular depth estimation. Given a sequence of camera images
with known poses, **FLaME** is able to reconstruct dense 3D meshes of the
environment by posing the depth estimation problem as a variational optimization
over a Delaunay graph that can be solved at framerate, even on computationally
constrained platforms.

The `flame_ros` repository contains the ROS bindings, visualization code, and
offline frontends for the algorithm. The core library can be
found [here](https://github.com/robustrobotics/flame.git).

<p align="center">
    <a href="https://www.youtube.com/watch?v=vB_F-Sj0AX0">
    <img src="https://img.youtube.com/vi/vB_F-Sj0AX0/0.jpg" alt="FLaME">
    </a>
</p>

### Related Publications:
* [**FLaME: Fast Lightweight Mesh Estimation using Variational Smoothing on Delaunay Graphs**](https://groups.csail.mit.edu/rrg/papers/greene_iccv17.pdf),
*W. Nicholas Greene and Nicholas Roy*, ICCV 2017.

## Author
- W. Nicholas Greene (wng@csail.mit.edu)

## Quickstart
Build the provided [Docker](https://www.docker.com/) image and run an example
dataset (requires [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) for
rviz):
```bash
# Build the image.
cd flame_ros
docker build --rm -t flame -f scripts/Dockerfile .

# Run an example dataset.
./scripts/flame_docker_example.sh
```
You may need to run `xhost +local:root` in order to forward rviz outside the container.

## Dependencies
- Ubuntu 16.04
- ROS Kinetic
- OpenCV 3.2
- Boost 1.54
- PCL 1.7
- Eigen 3.2
- Sophus (SHA: b474f05f839c0f63c281aa4e7ece03145729a2cd)
- [flame](https://github.com/robustrobotics/flame.git)
- catkin_tools (optional)

## Installation
**NOTE:** These instructions assume you are running ROS Kinetic on Ubuntu 16.04
and are interested in installing both `flame` and `flame_ros`. See the
installation instructions for `flame` if you only wish to install `flame`.

1. Install `apt` dependencies:
```bash
sudo apt-get install libboost-all-dev libpcl-dev python-catkin-tools
```

2. Create a Catkin workspace using `catkin_tools`:
```bash
# Source ROS.
source /opt/ros/kinetic/setup.sh

# Create workspace source folder.
mkdir -p flame_ws/src

# Checkout flame and flame_ros into workspace.
cd flame_ws/src
git clone https://github.com/robustrobotics/flame.git
git clone https://github.com/robustrobotics/flame_ros.git

# Initialize workspace.
cd ..
catkin init

# Install ROS dependencies using rosdep.
rosdep install -iy --from-paths ./src
```

3. Install Eigen 3.2 and Sophus using the scripts provided with `flame`:
```bash
# Create a dependencies folder.
mkdir -p flame_ws/dependencies/src

# Checkout Eigen and Sophus into ./dependencies/src and install into ./dependencies.
cd flame_ws
./src/flame/scripts/eigen.sh ./dependencies/src ./dependencies
./src/flame/scripts/sophus.sh ./dependencies/src ./dependencies

# Copy and source environment variable script:
cp ./src/flame/scripts/env.sh ./dependencies/
source ./dependencies/env.sh
```

4. Build workspace:
```bash
# Build!
catkin build

# Source workspace.
source ./devel/setup.sh
```

## Offline Processing
Two types of offline nodes are provided, one to process video from
the
[EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and
one to process video from
the
[TUM RGB-D SLAM Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset).

### EuRoC Data
First, download and extract one of the ASL
datasets
[here](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (the
Vicon Room datasets should work well).

Next, update the parameter file in `flame_ros/cfg/flame_offline_asl.yaml` to
point to where you extracted the data:
```bash
pose_path: <path_to_dataset>/mav0/state_groundtruth_estimate0
rgb_path: <path_to_dataset>/mav0/cam0
```

Finally, to process the data launch:
```bash
roslaunch flame_ros flame_offline_asl.launch
```
The mesh should be published on the `/flame/mesh` topic. To visualize this topic
in rviz, consult the the [Visualization](#Visualization) section.

### TUM Data
First, download and extract one of the
datasets [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)
(`fr3/structure_texture_far` or `fr3/long_office_household` should work well).
Use the `associate.py`
script [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools) to
associate the pose (`groundtruth.txt`) and RGB (`rgb.txt`) files (you can
associate the depthmaps as well).

A ROS-compliant camera calibration YAML file will be needed. You can use the one
provided in `flame_ros/cfg/kinect.yaml`, which has the default parameters for
the Microsoft Kinect used to collect the TUM datasets.

Next, update the parameter file in `flame_ros/cfg/flame_offline_tum.yaml` to
point to where you extracted the data:
```bash
input_file: <path_to_dataset>/groundtruth_rgb.txt
calib_file: <path_to_flame_ros>/cfg/kinect.yaml
```

Finally, to process the data launch:
```bash
roslaunch flame_ros flame_offline_tum.launch
```
The mesh should be published on the `/flame/mesh` topic. To visualize this topic
in rviz, consult the the [Visualization](#Visualization) section.

## Online Processing
The online nodelet can be launched using `flame_nodelet.launch`:
```bash
roslaunch flame_ros flame_nodelet.launch image:=/image
```
where `/image` is your live rectified/undistorted image stream. The `frame_id` of
this topic must correspond to a Right-Down-Forward frame attached to the
camera. The `tf` tree must be complete such that the pose of the camera in the
world frame can be resolved by `tf`.

The mesh should be published on the `/flame/mesh` topic. To visualize this topic
in rviz, consult the the [Visualization](#Visualization) section.

The `flame_nodelet.launch` launch file loads the parameters listed in
`flame_ros/cfg/flame_nodelet.yaml`. You may need to update the
`input/camera_frame_id` param for your data. See the [Parameters](#Parameters)
section for more parameter information.

## Visualization
You can use the provided configuration file (`flame_ros/cfg/flame.rviz`) to
visualize the output data in rviz. This approach uses a custom rviz plugin to
render the `/flame/mesh` messages. `flame_ros` can also publish the depth data
in other formats (e.g. as a `sensor_msgs/PointCloud2` or a `sensor_msgs/Image`),
which can be visualized by enabling the corresponding plugins.

## Parameters
There are many parameters that control processing, but only a handful are
particularly important:

- `output/*`: Controls what type of output messages are produced. If you are
  concerned about speed, you should prefer publishing only the mesh data
  (`output/mesh: True`), but other types of output can be enabled here.

- `debug/*`: Controls what type of debug images are published. Creating these
  images is relatively expensive, so disable for real-time operation.

- `threading/openmp/*`: Controls OpenMP-accelerated sections. You may wish to
  tune the number of threads per parallel section
  (`threading/openmp/num_threads`) or the number of chunks per thread
  (`threading/openmp/chunk_size`) for your processor.

- `features/detection/min_grad_mag`: Controls the minimum gradient magnitude for
  detected features.

- `features/detection/win_size`: Features are detected by dividing the image
  domain into `win_size x win_size` blocks and selecting the best trackable
  pixel inside each block. Set to a large number (e.g. 32) for coarse, but fast
  reconstructions, and a small number (e.g. 8) for finer reconstructions.

- `regularization/nltgv2/data_factor`: Controls the balance between smoothing
  and data-fitting in the regularizer. It should be set in relation to the
  detection window size. Some good values are 0.1-0.25.

### Performance Tips
For best results use a high framerate (>= 30 Hz) camera with VGA-sized
images. Higher resolution images will require more accurate poses. The feature
detection window size (`features/detection/win_size`) and the data scaling term
(`regularization/nltgv2/data_factor`) are the primary knobs for tuning
performance. The default parameters should work well in most cases, but you may
need to tune for your specific data.

By default, `flame_ros` will publish several debug images. While helpful to
observe during operation, they will slow down the pipeline. Disable them if you
are trying to increase throughput.

The usual tips for monocular SLAM/depth estimation systems also apply:
- Prefer slow translational motion
- Avoid fast rotations when possible
- Use an accurate pose source (e.g. one of the many available visual
  odometry/SLAM packages)
- Prefer texture-rich environments
- Prefer environments with even lighting
