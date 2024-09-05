README
======

Welcome to OKVIS2: Open Keyframe-based Visual-Inertial SLAM.

This is the Author's implementation of [1]. It is further based on work presented in [2-4].

[1] Stefan Leutenegger. [OKVIS2: Realtime Scalable Visual-Inertial SLAM with Loop
    Closure](https://arxiv.org/pdf/2202.09199). arXiv, 2022.

[2] Stefan Leutenegger, Simon Lynen, Michael Bosse, Roland Siegwart and Paul Timothy Furgale.
    Keyframe-based visual-inertial odometry using nonlinear optimization. The International Journal
    of Robotics Research, 2015.

[3] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for Efficient and Robust
    Autonomous Operation. Doctoral dissertation, 2014.

[4] Stefan Leutenegger, Paul Timothy Furgale, Vincent Rabaud, Margarita Chli, Kurt Konolige, Roland
    Siegwart. Keyframe-Based Visual-Inertial SLAM using Nonlinear Optimization. In Proceedings of
    Robotics: Science and Systems, 2013.

Note that the codebase that you are provided here is free of charge and without any warranty. This
is bleeding edge research software.

Also note that the quaternion standard has been adapted to match Eigen/ROS, thus some related
mathematical description in [2-3] will not match the implementation here.

If you publish work that relates to this software, please cite at least [1].

### License ###

The 3-clause BSD license (see file LICENSE) applies.

### How do I get set up? ###
#### Ubuntu ####

This is a pure cmake project.

You will need to install the following dependencies,

* CMake,

        sudo apt install cmake

* google-glog + gflags,

        sudo apt install libgoogle-glog-dev

* BLAS & LAPACK,

        sudo apt install libatlas-base-dev

* Eigen3,

        sudo apt install libeigen3-dev

* SuiteSparse and CXSparse,

        sudo apt install libsuitesparse-dev

* Boost,

        sudo apt install libboost-dev libboost-filesystem-dev

* OpenCV 2.4-4: follow the instructions on http://opencv.org/ or install
  via

        sudo apt install libopencv-dev

* LibTorch: if you want to run the segmentation CNN to remove Sky points etc, install with
  instructions from the link below. Get the C++ version with C++11 ABI with or without CUDA
  (depending on availability on your machine):

    https://pytorch.org/get-started/locally/

    Also, depending on where you downloaded it to, you may want to tell cmake in your `~.bashrc`:

        export Torch_DIR=/path/to/libtorch

    Furthermore, you can turn on the NVIDIA GPU to be used for inference, if you have one, with
    `USE_GPU=ON`.

    In case you absolutely do not want to use `LibTorch`, you may disable with `USE_NN=OFF`.

* Optional: you can use this package with a Realsense D435i or D455.
Follow the instructions on:
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

* Optional: if you need to use openBLAS library, please use the openMP version of it:

        sudo apt-get install libopenblas-openmp-dev

* Optional: you can use this package as part of a ROS2 workspace. In this case set `BUILD_ROS2=ON`.
  Assuming you are running Ubuntu 24.04, follow the steps here:
  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

#### OSX ####
The following has not extensively been tested, but runs on OSX Monterey-Sonoma (Intel and M1).

This is a pure cmake project.

You will need to install the following dependencies,

* XCode and command line tools.

* homebrew: follow the instructions here https://brew.sh/index_de

* CMake,

        brew install cmake

* google-glog + gflags,

        brew install glog

* Eigen3,

        brew install eigen

* SuiteSparse and CXSparse,

        brew install suitesparse

* Boost,

        brew install boost

* OpenCV 2.4-4,

        brew install opencv

* LibTorch: if you want to run the segmentation CNN to remove Sky points etc, install with
  instructions from the link below. Get the C++ version with C++11 ABI with or without CUDA
  (depending on availability on your machine):

        brew install libtorch

    Furthermore, you can turn on the NVIDIA GPU to be used for inference, if you have one, with
    `USE_GPU=ON`.

    In case you absolutely do not want to use `LibTorch`, you may disable with `USE_NN=OFF`. Also,
    in that case you'll really want to get OpenMP either with `brew` or by using LLVM that is
    shipped with it.

* Optional: you can use this package with a Realsense D435i or D455.
        
        brew install openssl
        brew install librealsense
        brew uninstall --ignore-dependencies libusb
        brew install libusb --HEAD

* Optional: you can use this package as part of a ROS2 workspace. In this case set `BUILD_ROS2=ON`.
  Getting ROS2 to work on a mac, especially with Apple Silicon, can be a bit of a headache.
  If you are upe for it, follow the steps here:
  https://docs.ros.org/en/jazzy/Installation/Alternatives/macOS-Development-Setup.html

### Building the project (no ROS2) ###

Clone the OKVIS2 repository:

    git clone --recurse-submodules git@github.com:smartroboticslab/okvis2.git

If you forgot the `--recurse-submodules` flag when cloning run the following command in the
repository root:

    git submodule update --init --recursive

To change the cmake build type for the whole project use:

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j8

NOTE: if you want to use the library, install the project (default or somewhere else), so the
dependencies can be resolved.

    make install

### Running the demo application ###

You will find a demo application in okvis_apps. It can process datasets in the EuRoC (ASL/ETH)
format.

In order to run a minimal working example, follow the steps below:

1. Download a dataset of your choice from
   http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets.
   Assuming you downloaded `MH_01_easy/`.
   You will find a corresponding calibration / estimator configuration in the
   config folder.

2. Run the app as

         ./okvis_app_synchronous <path/to/euroc.yaml> <path/to/MH_01_easy/mav0/>

     If you built with librealsense installed, you can connect a Realsense D455 (or D435i) and run

         ./okvis_app_realsense <path/to/realsense_D455.yaml>

     Furthermore, there is a utility to record dataset in EuRoC format as above

         ./okvis_app_realsense_record <path/to/config.yaml> <path/to/folder>

     If you are annoyed by info log messages, you can disable them in the terminal (assuming bash):

         `export GLOG_minloglevel="2"`

### Outputs and frames

**Fundamentally, the library is meant to be used in a way that you maintain an `okvis::Trajectory`
object in your client code via two callbacks that you need to implement (and where it is your
responsibility to ensure threadsafe access and modification):**

1. Implement the callback that is called by the estimator whenever there are updates. The updates
   may involve many states, especially upon loop closure. Register it with
   `okvis::ViInterface::setOptimisedGraphCallback` and let it trigger the update to the
   `okvis::Trajectory` via `okvis::Trajectory::update`.
2. If you would like to have the high-rate, most up-to-date states available via prediction with the
   newest IMU measurements, then also register an IMU callback (last) to
   `okvis::ViSensorBase::setImuCallback` and let it trigger the update to the
   `okvis::Trajectory` via `okvis::Trajectory::addImuMeasurement`.

This mechanism allows availability of a consistent trajectory, `okvis::Trajectory`, on which you can
query states for any time (at estimated, image timestamps, or otherwise).

In terms of coordinate frames and notation,

* `W` denotes the OKVIS World frame $`\underrightarrow{\mathcal{F}}_W`$ (z up)
* `C_i` denotes the i-th camera frame $`\underrightarrow{\mathcal{F}}_{C_i}`$
* `S` denotes the IMU sensor frame $`\underrightarrow{\mathcal{F}}_S`$
* `B` denotes a (user-specified / robot) body frame $`\underrightarrow{\mathcal{F}}_B`$

States contain the pose `T_WS` ($`\boldsymbol{T}_{WS}`$) as a position `r_WS`
($`_W\mathbf{r}_{S}`$) and quaternion `q_WS` ($`\mathbf{q}_{WS}`$), followed by the velocity
in World frame `v_W` ($`_{W}\mathbf{v}`$) and gyro biases `b_g` ($`\mathbf{b}_g`$) as well as
accelerometer biases `b_a` ($`\mathbf{b}_a`$).

**See the example applications to understand how to use the estimator, sensor, callbacks, etc!**

### Configuration files ###

The config folder contains example configuration files. Please read the documentation of the
individual parameters in the yaml file carefully. You have various options to trade-off accuracy and
computational expense as well as to enable online calibration.

### Building the project with ROS2 ###

Create a workspace e.g.

    mkdir -p ~/okvis_ws/src
    cd ~/okvis_ws/src

Now you can clone the repo

    git clone --recurse-submodules git@github.com:smartroboticslab/okvis2.git

You may want to run rosdep to make sure all dependencies are installed.

Next, you can build:

    cd ~/okvis_ws
    colcon build

Make sure to turn on the ROS2 specific build: it's on by default, but you may add
`--cmake-args -DBUILD_ROS2=ON` to force it.

### Running ROS2 nodes ###

After sourcing the workspace, you can launch the live `realsense` node as

    ros2 launch okvis okvis_node_realsense.launch.xml config_filename:=<config.yaml>

This will also bring up rviz that visualises the output.

Alternatively, you can run the live node connected via subscription to topics. To experience this,
you can run the realsense driver node as

    ros2 launch okvis okvis_node_realsense_publisher.launch.xml config_filename:=<config.yaml>

... and launch the subscriber in a separate terminal as

    ros2 launch okvis okvis_node_subscriber.launch.xml config_filename:=<config.yaml>

Note that you may use either `okvis_node_realsense` or `okvis_node_realsense_publisher` to record a
bag, minimally with

    ros2 bag record /okvis/imu0 /okvis/cam0/image_raw /okvis/cam1/image_raw

Finally, you can also process datasets. Currently, the EuRoC format is supported by default, as well
as ROS2 bags (db3/mcap/..). Run the respective synchronous (blocking) processing as

    ros2 launch okvis okvis_node_synchronous.launch.xml config_filename:=<cfg.yaml> path:=<folder>

### HEALTH WARNING: calibration ###

If you would like to run the software/library on your own hardware setup, be aware that good results
(or results at all) may only be obtained with appropriate calibration of the

* camera intrinsics,
* camera extrinsics (poses relative to the IMU),
* knowledge about the IMU noise parameters,
* and ACCURATE TIME SYNCHRONISATION OF ALL SENSORS.

To perform a calibration yourself, we recommend the following:

* Get Kalibr by following the instructions here
  https://github.com/ethz-asl/kalibr/wiki/installation . If you decide to build
  from source and you run ROS indigo checkout pull request 3:

      git fetch origin pull/3/head:request3
      git checkout request3

* Follow https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration to calibrate intrinsic
  and extrinsic parameters of the cameras. If you receive an error message that the tool was unable
  to make an initial guess on focal length, make sure that your recorded dataset contains frames
  that have the whole calibration target in view.

* Follow https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration to get estimates for the
spatial parameters of the cameras with respect to the IMU.

### Using the library

Here's a minimal example of your CMakeLists.txt to build a project using OKVIS.

    # To include OKVIS in your project if it's installed on the system add:
    find_package(okvis 2 REQUIRED)
    # OR if OKVIS is a git submodule of the current project add:
    add_subdirectory(okvis2)

    # Then to link some target with it add:
    target_link_libraries(MY_TARGET_NAME ${okvis_LIBRARIES})


### Contribution guidelines ###

* Contact stefan.leutenegger@tum.de to request access to the github repository.

* Programming guidelines: please follow
  https://github.com/ethz-asl/programming_guidelines/wiki/Cpp-Coding-Style-Guidelines.

* Writing tests: please write unit tests (gtest).

* Code review: please create a pull request for all changes proposed. The pull request will be
  reviewed by an admin before merging.

### Support ###

The developpers will be happy to assist you or to consider bug reports / feature requests. But
questions that can be answered reading this document will be ignored. Please contact
stefan.leutenegger@tum.de.
