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
 * @file RpgDatasetReader.cpp
 * @brief Source file for the DatasetReader class.
 * @author Stefan Leutenegger
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>

#include <okvis/RpgDatasetReader.hpp>

namespace okvis {

RpgDatasetReader::RpgDatasetReader(const std::string & path,
                             const Duration & deltaT, int numCameras) :
  deltaT_(deltaT), numCameras_(numCameras) {
  streaming_ = false;
  setDatasetPath(path);
  counter_ = 0;
}

RpgDatasetReader::~RpgDatasetReader() {
  stopStreaming();
}

bool RpgDatasetReader::setDatasetPath(const std::string & path) {
  path_ = path;
  return true;
}

bool RpgDatasetReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool RpgDatasetReader::isStreaming()
{
  return streaming_;
}

double RpgDatasetReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool RpgDatasetReader::startStreaming() {
  OKVIS_ASSERT_TRUE(Exception, !imagesCallbacks_.empty(), "no add image callback registered")
  OKVIS_ASSERT_TRUE(Exception, !imuCallbacks_.empty(), "no add IMU callback registered")

  // open the IMU file
  std::string line;
  imuFile_.open(path_ + "/imu.txt");
  OKVIS_ASSERT_TRUE(Exception, imuFile_.good(), "no imu file found at " << path_+"/imu.txt");
  int number_of_lines = 0;
  while (std::getline(imuFile_, line))
    ++number_of_lines;
  LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;
  if (number_of_lines - 1 <= 0) {
    LOG(ERROR)<< "no imu messages present in " << path_+"/imu0/data.csv";
    return -1;
  }
  // set reading position to second line
  imuFile_.clear();
  imuFile_.seekg(0, std::ios::beg);
  std::getline(imuFile_, line);

  // now open camera files
  std::vector<okvis::Time> times;
  okvis::Time latest(0);
  int num_camera_images = 0;
  int numCameras = numCameras_;
  if(numCameras == -1) {
    numCameras = 2;
  }
  for (size_t i = 0; i < size_t(numCameras); ++i) {
    std::ifstream camDataFile;
    if(i==0) {
      camDataFile.open(path_ + "/left_images.txt");
    } else {
      camDataFile.open(path_ + "/right_images.txt");
    }
    if(!camDataFile.good()) {
      OKVIS_ASSERT_TRUE(Exception, i>0, "No camera data found");
      break;
    }
    num_camera_images = 0;
    std::vector < std::pair<Time, std::string> > imageNames;
    std::getline(camDataFile, line);
    while (std::getline(camDataFile, line)) {
      ++num_camera_images;
      std::stringstream stream(line);
      std::string s0, s1, s2, s3;
      std::getline(stream, s0, ' ');
      std::getline(stream, s1, '.');
      std::getline(stream, s2, ' ');
      std::getline(stream, s3, ' ');
      Time t(std::atol(s1.c_str()), std::atol(s2.c_str())/1000);
      imageNames.push_back(std::make_pair(t,s3));
    }
    allImageNames_.push_back(imageNames);
    LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;
    if(i==0) {
      numImages_ = num_camera_images;
    }
  }

  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&RpgDatasetReader::processing, this);

  return true;
}

bool RpgDatasetReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}

void  RpgDatasetReader::processing() {
  std::string line;
  okvis::Time start(0.0);
  const size_t numCameras = allImageNames_.size();
  std::vector < std::vector < std::pair<Time, std::string> > ::iterator
      > cam_iterators(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    cam_iterators.at(i) = allImageNames_.at(i).begin();
  }
  while (streaming_) {

    // check if at the end
    for (size_t i = 0; i < numCameras; ++i) {
      if (cam_iterators[i] == allImageNames_[i].end()) {
        streaming_ = false;
        return;
      }
      if(i>0){
        OKVIS_ASSERT_TRUE(
          Exception,
          fabs((cam_iterators.at(i)->first - cam_iterators.at(i-1)->first).toSec()) < 0.003,
          "missing images at " << cam_iterators.at(i)->second);
      }
    }

    /// \todo synchronise if needed
    /*const uint64_t tolNSec = 10000000; // 0.01 sec
    Eigen::VectorXd timestamps(numCameras,1);
    for (size_t i = 0; i < numCameras; ++i) {
      timestamps[i] = std::atol(cam_iterators.at(i)->first.c_str());
    }*/

    // add images
    okvis::Time t;
    std::map<size_t,cv::Mat> images;
    for (size_t i = 0; i < numCameras; ++i) {

      std::string filename = path_ + "/" + cam_iterators.at(i)->second;

      cv::Mat filtered = cv::imread(filename, cv::IMREAD_GRAYSCALE);

      OKVIS_ASSERT_TRUE(
            Exception, !filtered.empty(),
            "cam " << i << " missing image :" << std::endl << filename)

      t = cam_iterators.at(i)->first;

      if (start == okvis::Time(0.0)) {
        start = t;
      }

      // get all IMU measurements till then
      okvis::Time t_imu = start;
      do {
        if (!std::getline(imuFile_, line)) {
          streaming_ = false;
          return;
        }

        std::stringstream stream(line);
        std::string s0,s1, s2;
        std::getline(stream, s0, ' ');
        std::getline(stream, s1, '.');
        std::getline(stream, s2, ' ');
        uint64_t seconds = std::stol(s1.c_str());
        uint64_t nanoseconds = std::stol(s2.c_str())/1000;

        std::string s;
        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ' ');
          gyr[j] = std::stof(s);
        }

        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ' ');
          acc[j] = std::stof(s);
        }

        t_imu = Time(seconds, nanoseconds);

        // std::cout <<s0<<" " << t_imu << " " << acc.transpose() << gyr.transpose() << std::endl;

        // add the IMU measurement for (blocking) processing
        if (t_imu - start + okvis::Duration(1.0) > deltaT_) {
          for(auto & imuCallback : imuCallbacks_) {
            imuCallback(t_imu, acc, gyr);
          }
        }

      } while (t_imu <= t);

      // add the image to the frontend for (blocking) processing
      if (t - start > deltaT_) {
        images[i] = filtered;
      }

      cam_iterators[i]++;

    }
    bool add=true;
    for (size_t i = 0; i < numCameras; ++i) {
      if(!images.count(i)) {
        add=false;
      }
    }
    if(add) {
      for (auto &imagesCallback : imagesCallbacks_) {
        imagesCallback(t, images, std::map<size_t, cv::Mat>());
      }
      ++counter_;
    }
  }

  return;
}

}
