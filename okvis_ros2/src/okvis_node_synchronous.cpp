/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file okvis_node_synchronous.cpp
 * @brief This file includes the ROS node implementation: synchronous (blocking) dataset processing.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <execinfo.h>
#include <Eigen/Core>
#include <fstream>

#include <boost/filesystem.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma GCC diagnostic pop

#include <okvis/TrajectoryOutput.hpp>
#include <okvis/ViParametersReader.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/DatasetReader.hpp>
#include <okvis/RpgDatasetReader.hpp>
#include <okvis/ros2/RosbagReader.hpp>
#include <okvis/TrajectoryOutput.hpp>
#include <okvis/ros2/Publisher.hpp>

/// \brief Main
/// \param argc argc.
/// \param argv argv.
int main(int argc, char **argv)
{

  // ros2 setup
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("okvis_node_synchronous");

  // publisher
  okvis::Publisher publisher(node);

  // logging
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // Setting up paramaters
  okvis::Duration deltaT(0.0);
  bool rpg = false;
  bool rgb = false;
  bool rosbag = false;
  std::string configFilename("");
  std::string path("");

  node->declare_parameter("rpg", false);
  node->declare_parameter("rgb", false);
  node->declare_parameter("config_filename", "");
  node->declare_parameter("path", "");
  node->declare_parameter("imu_propagated_state_publishing_rate", 0.0);

  node->get_parameter("rpg", rpg);
  node->get_parameter("rgb", rgb);
  node->get_parameter("config_filename", configFilename);
  node->get_parameter("path", path);
  if (configFilename.compare("")==0){
    LOG(ERROR) << "ros parameter 'config_filename' not set";
    return EXIT_FAILURE;
  }
  if (path.compare("")==0){
    LOG(ERROR) << "ros parameter 'path' not set";
    return EXIT_FAILURE;
  }
  double imu_propagated_state_publishing_rate = 0.0;
  node->get_parameter("imu_propagated_state_publishing_rate", imu_propagated_state_publishing_rate);

  okvis::ViParametersReader viParametersReader(configFilename);
  okvis::ViParameters parameters;
  viParametersReader.getParameters(parameters);

  // dataset reader
  std::shared_ptr<okvis::DatasetReaderBase> datasetReader;
  
  // check if ros2 bag
  std::ifstream f(path+"/metadata.yaml");
  if(f.good()) {
    rosbag = true;
  }
  if(rosbag) {
    datasetReader.reset(new okvis::RosbagReader(
      path, int(parameters.nCameraSystem.numCameras()),
      parameters.camera.sync_cameras, deltaT));
  } else if(rpg) {
    datasetReader.reset(new okvis::RpgDatasetReader(
      path, deltaT, int(parameters.nCameraSystem.numCameras())));
  } else {
    datasetReader.reset(new okvis::DatasetReader(
      path, int(parameters.nCameraSystem.numCameras()),
      parameters.camera.sync_cameras, deltaT));
  }

  // also check DBoW2 vocabulary
  boost::filesystem::path executable(argv[0]);
  std::string dBowVocDir = executable.remove_filename().string() + "/../../share/okvis/resources/";
  std::ifstream infile(dBowVocDir+"/small_voc.yml.gz");
  if(!infile.good()) {
    LOG(ERROR)<<"DBoW2 vocaublary " << dBowVocDir << "/small_voc.yml.gz not found.";
    return EXIT_FAILURE;
  }

  okvis::ThreadedSlam estimator(parameters, dBowVocDir);
  estimator.setBlocking(true);

  // write logs
  std::string mode = "slam";
  if(!parameters.estimator.do_loop_closures) {
    mode = "vio";
  }
  if(parameters.camera.online_calibration.do_extrinsics) {
    mode = mode+"-calib";
  }

  // setup publishing
  publisher.setCsvFile(path + "/okvis2-" + mode + "-live_trajectory.csv", rpg);
  estimator.setFinalTrajectoryCsvFile(path+"/okvis2-" + mode + "-final_trajectory.csv", rpg);
  estimator.setMapCsvFile(path+"/okvis2-" + mode + "-final_map.csv");
  estimator.setOptimisedGraphCallback(
    std::bind(&okvis::Publisher::publishEstimatorUpdate, &publisher,
              std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3, std::placeholders::_4));
  publisher.setBodyTransform(parameters.imu.T_BS);
  publisher.setOdometryPublishingRate(imu_propagated_state_publishing_rate);
  publisher.setupImageTopics(parameters.nCameraSystem);

  // connect reader to estimator
  datasetReader->setImuCallback(
    std::bind(&okvis::ThreadedSlam::addImuMeasurement, &estimator,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  datasetReader->setImagesCallback(
    std::bind(&okvis::ThreadedSlam::addImages, &estimator, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3));

  // start
  okvis::Time startTime = okvis::Time::now();
  datasetReader->startStreaming();
  int progress = 0;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    estimator.processFrame();
    std::map<std::string, cv::Mat> images;
    estimator.display(images);
    publisher.publishImages(images);

    // check if done
    if(!datasetReader->isStreaming()) {
      estimator.stopThreading();
      std::cout << "\rFinished!" << std::endl;
      if(parameters.estimator.do_final_ba) {
        LOG(INFO) << "final full BA...";
        cv::Mat topView;
        estimator.doFinalBa();
      }
      estimator.writeFinalTrajectoryCsv();
      if(parameters.estimator.do_final_ba) {
        estimator.saveMap();
      }
      LOG(INFO) <<"total processing time " << (okvis::Time::now() - startTime) << " s" << std::endl;
      break;
    }

    // display progress
    int newProgress = int(datasetReader->completion()*100.0);
#ifndef DEACTIVATE_TIMERS
    if (newProgress>progress) {
      LOG(INFO) << okvis::timing::Timing::print();
    }
#endif
    if (newProgress>progress) {
      progress = newProgress;
      LOG(INFO) << "Progress: "
                << progress << "% "
                << std::flush;
    }
  }
  return EXIT_SUCCESS;
}
