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
 * @file RePublisher.cpp
 * @brief Source file for the RePublisher class.
 * @author Stefan Leutenegger
 */

#include <okvis/ros2/RePublisher.hpp>
#include <sensor_msgs/image_encodings.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

RePublisher::RePublisher(std::shared_ptr<rclcpp::Node> node, size_t numCams) {
  // set up node
  node_ = node;
  numCams_ = numCams;
}

void RePublisher::setTopics(const std::string& imuTopic, 
                            const std::string& camTopic,
                            const std::string& rgbTopic,
                            const std::string& depthTopic) {
  imuTopic_ =imuTopic;
  camTopic_ = camTopic;
  rgbTopic_ = rgbTopic;
  depthTopic_ = depthTopic;

  // set up the publishers
  pubImu_ = node_->create_publisher<sensor_msgs::msg::Imu>(imuTopic_, 1);
  for(size_t i=0; i<numCams_; ++i) {
    camTransportVector_.emplace_back(new image_transport::ImageTransport(node_));
    pubCamVector_.emplace_back(
      camTransportVector_.back()->advertise(camTopic_ + std::to_string(i) + "/image_raw", 1));
    if(!rgbTopic.empty()) {
      rgbTransportVector_.emplace_back(new image_transport::ImageTransport(node_));
      pubRgbVector_.emplace_back(
        rgbTransportVector_.back()->advertise(rgbTopic_ + std::to_string(i) + "/image_raw", 1));
    }
    if(!depthTopic.empty()) {
      depthTransportVector_.emplace_back(new image_transport::ImageTransport(node_));
      pubDepthVector_.emplace_back(
        depthTransportVector_.back()->advertise(depthTopic_ + std::to_string(i) + "/image_raw", 1));
    }
  }
}
    
bool RePublisher::publishImages(const okvis::Time & stamp,
                   const std::map<size_t, cv::Mat> & images,
                   const std::map<size_t, cv::Mat> & depthImages) {
  // re-publish images
  for(size_t i=0; i<numCams_; ++i) {
    sensor_msgs::msg::Image::SharedPtr msg;
    if(images.count(i)) {
      if(images.at(i).channels() == 1) {
        // grayscale
        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", images.at(i)).toImageMsg();
        msg->header.stamp = rclcpp::Time(stamp.sec, stamp.nsec);
        pubCamVector_[i].publish(msg);
      } else {
        // colour
        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", images.at(i)).toImageMsg();
        msg->header.stamp = rclcpp::Time(stamp.sec, stamp.nsec);
        pubRgbVector_[i].publish(msg);
      }
    }
    if(depthImages.count(i)) {
      // depth
      msg = cv_bridge::CvImage(std_msgs::msg::Header(),
                               sensor_msgs::image_encodings::TYPE_32FC1,
                               depthImages.at(i)).toImageMsg();
      msg->header.stamp = rclcpp::Time(stamp.sec, stamp.nsec);
      pubDepthVector_[i].publish(msg);
    }
  }
  return true;
}

bool RePublisher::publishImuMeasurement(const okvis::Time & stamp,
                           const Eigen::Vector3d & alpha,
                           const Eigen::Vector3d & omega) {
  // re-publish IMU
  auto header = std_msgs::msg::Header();
  header.stamp = rclcpp::Time(stamp.sec, stamp.nsec);
  sensor_msgs::msg::Imu msg;
  msg.header = header;
  msg.linear_acceleration.x = alpha[0];
  msg.linear_acceleration.y = alpha[1];
  msg.linear_acceleration.z = alpha[2];
  msg.angular_velocity.x = omega[0];
  msg.angular_velocity.y = omega[1];
  msg.angular_velocity.z = omega[2];
  pubImu_->publish(msg);
  return true;
}

}  // namespace okvis
