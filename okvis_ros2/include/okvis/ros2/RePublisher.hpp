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
 * @file RePublisher.hpp
 * @brief Header file for the RePublisher class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_REPUBLISHER_HPP_
#define INCLUDE_OKVIS_REPUBLISHER_HPP_

#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#pragma GCC diagnostic pop
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <okvis/ThreadedSlam.hpp>



/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief Publishes images and IMU as ROS2 messages.
class RePublisher {
public:
  /**
   * \brief              Constructor with node and number of cameras.
   * \param node         ROS2 node.
   * \param numCams      number of cameras.
   */
  RePublisher(std::shared_ptr<rclcpp::Node> node, size_t numCams);

  /**
   * \brief              Name the topics.
   * \param imuTopic     IMU topic name.
   * \param camTopic     Image topic name.
   * \param rgbTopic     RGB image topic name.
   * \param depthTopic   Depth image topic name.
   */
  void setTopics(const std::string& imuTopic, 
                 const std::string& camTopic,
                 const std::string& rgbTopic = std::string(),
                 const std::string& depthTopic = std::string());
    
  /// \name Add measurements to the algorithm
  /// \{
  /**
   * \brief              Add a new image.
   * \param stamp        The image timestamp.
   * \param images       The images.
   * \param depthImages  The depth images.
   * \return             Returns true normally. False, if the previous one has not been processed
   *                     yet.
   */
  bool publishImages(const okvis::Time &stamp,
                     const std::map<size_t, cv::Mat> &images,
                     const std::map<size_t, cv::Mat> &depthImages);

  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return Returns true normally. False if the previous one has not been processed yet.
   */
  bool publishImuMeasurement(const okvis::Time & stamp,
                         const Eigen::Vector3d & alpha,
                         const Eigen::Vector3d & omega);
                              
private:

  std::shared_ptr<rclcpp::Node> node_; ///< The node.

  size_t numCams_ = 0; ///< Number of cameras.
  std::vector<image_transport::Publisher> pubCamVector_; ///< The publisher for the images.
  std::vector<image_transport::Publisher> pubRgbVector_; ///< The publisher for the rgb images.
  std::vector<image_transport::Publisher> pubDepthVector_; ///< The publisher for the depth images.
  /// \brief The image transporters.
  std::vector<std::shared_ptr<image_transport::ImageTransport>> camTransportVector_;
  /// \brief The rgb image transporters.
  std::vector<std::shared_ptr<image_transport::ImageTransport>> rgbTransportVector_;
  /// \brief The depth image transporters.
  std::vector<std::shared_ptr<image_transport::ImageTransport>> depthTransportVector_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu_; ///< IMU publisher

  std::string imuTopic_; ///< IMU topic name.
  std::string camTopic_; ///< Camera topic name.
  std::string rgbTopic_; ///< RGB image topic name.
  std::string depthTopic_; ///< Depth image topic name.
};

}

#endif /* INCLUDE_OKVIS_REPUBLISHER_HPP_ */
