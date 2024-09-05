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
 * @file Subscriber.cpp
 * @brief Source file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */
 
#include <glog/logging.h>
#include <okvis/ros2/Subscriber.hpp>

#define OKVIS_THRESHOLD_SYNC 0.01 ///< Sync threshold in seconds.

/// \brief okvis Main namespace of this package.
namespace okvis {

Subscriber::~Subscriber()
{
}

Subscriber::Subscriber(std::shared_ptr<rclcpp::Node> node,
                       okvis::ViInterface* viInterface,
                       okvis::Publisher* publisher, 
                       const okvis::ViParameters& parameters)
{
  viInterface_ = viInterface;
  publisher_ = publisher;
  parameters_ = parameters;
  setNodeHandle(node);
}

void Subscriber::setNodeHandle(std::shared_ptr<rclcpp::Node> node)
{

  node_ = node;

  imageSubscribers_.resize(parameters_.nCameraSystem.numCameras());
  imagesReceived_.resize(parameters_.nCameraSystem.numCameras());

  // set up image reception
  imgTransport_.reset(new image_transport::ImageTransport(node));

  // set up callbacks
  for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
    imageSubscribers_[i] = imgTransport_->subscribe(
        "/okvis/cam" + std::to_string(i) +"/image_raw",
        30 * parameters_.nCameraSystem.numCameras(),
        std::bind(&Subscriber::imageCallback, this, std::placeholders::_1, i));
  }

  subImu_ = node_->create_subscription<sensor_msgs::msg::Imu>("/okvis/imu0", 1000, 
      std::bind(&Subscriber::imuCallback, this, std::placeholders::_1));
}

void Subscriber::shutdown() {
  // stop callbacks
  for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
    imageSubscribers_[i].shutdown();
  }
  subImu_.reset();
}

void Subscriber::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                               unsigned int cameraIndex)
{
  const cv::Mat raw(msg->height, msg->width, CV_8UC1,
                    const_cast<uint8_t*>(&msg->data[0]), msg->step);
  cv::Mat filtered = raw.clone();

  // timestamp
  okvis::Time t(msg->header.stamp.sec, msg->header.stamp.nanosec);

  // insert
  imagesReceived_.at(cameraIndex)[t.toNSec()] = filtered;
  
  // try sync
  std::lock_guard<std::mutex> lock(time_mutex_);
  std::set<uint64_t> allTimes;
  const int numCameras = imagesReceived_.size();
  for(int i=0; i < numCameras; ++i) {
    for(const auto & entry : imagesReceived_.at(i)) {
      allTimes.insert(entry.first);
    }
  }
  for(const auto & time : allTimes) {
    // note: ordered old to new
    std::vector<uint64_t> syncedTimes(numCameras, 0);
    std::map<size_t, cv::Mat> images;
    okvis::Time tcheck;
    tcheck.fromNSec(time);
    bool synced = true;
    for(int i=0; i < numCameras; ++i) {
      bool syncedi = false;
      for(const auto & entry : imagesReceived_.at(i)) {
        okvis::Time ti;
        ti.fromNSec(entry.first);
        if(fabs((tcheck-ti).toSec()) < OKVIS_THRESHOLD_SYNC) {
          syncedTimes.at(i) = entry.first;
          images[i] = imagesReceived_.at(i).at(entry.first);
          syncedi = true;
          break;
        } 
      }
      if(!syncedi) {
        synced = false;
        break;
      }
    }
    if(synced) {
      // add
      //std::cout << "add images" << tcheck << std::endl;
      if (!viInterface_->addImages(tcheck, images)) {
        LOG(WARNING) << "Frame not added at t="<< tcheck;
      }
      // remove all the older stuff from buffer
      for(int i=0; i < numCameras; ++i) {
        const int size0 = imagesReceived_.at(i).size();
        auto end = imagesReceived_.at(i).find(syncedTimes.at(i));
        if(end!=imagesReceived_.at(i).end()) {
          ++end;
        }
        imagesReceived_.at(i).erase(imagesReceived_.at(i).begin(), end);
        const int size1 = imagesReceived_.at(i).size();
        if (size0-size1>1) {
          LOG(WARNING) << "dropped " << size0 - size1 - 1 << " unsyncable frame(s) of camera " << i
                       << " before t=" << tcheck;
        }
      } 
    }
  }
}

void Subscriber::imuCallback(const sensor_msgs::msg::Imu& msg)
{
  // construct measurement
  okvis::Time timestamp(msg.header.stamp.sec, msg.header.stamp.nanosec);
  Eigen::Vector3d acc(msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z);
  Eigen::Vector3d gyr(msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z);                    
  
  // forward to estimator
  viInterface_->addImuMeasurement(timestamp, acc, gyr);
  
   // also forward for realtime prediction
  if(publisher_) {
    publisher_->realtimePredictAndPublish(timestamp, acc, gyr);
  }
}


} // namespace okvis
