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
 * @file Subscriber.hpp
 * @brief Header file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_SUBSCRIBER_HPP_
#define INCLUDE_OKVIS_SUBSCRIBER_HPP_

#include <memory>
#include <mutex>

#include <boost/shared_ptr.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>) // requires GCC >= 5
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <okvis/Time.hpp>
#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/ThreadedSlam.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ViParametersReader.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/ros2/Publisher.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class handles all the buffering of incoming data.
 */
class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Destructor (trivial).
  ~Subscriber();
  /**
   * @brief Constructor. This will either subscribe to the relevant ROS topics or
   *        start up the sensor and register the callbacks directly there.
   * @param node The ROS node handle.
   * @param viInterfacePtr Pointer to the ViInterface.
   * @param publisher   Pointer to publisher
   * @param parameters  VI parameters.
   */
  Subscriber(std::shared_ptr<rclcpp::Node> node, okvis::ViInterface* viInterfacePtr,
             okvis::Publisher* publisher,
             const okvis::ViParameters& parameters);

  /// @brief Set the node handle. This sets up the callbacks. This is called in the constructor.
  void setNodeHandle(std::shared_ptr<rclcpp::Node> node);
    
  /// @brief stop callbacks.
  void shutdown();

 protected:
  /// @name ROS callbacks
  /// @{

  /// @brief The image callback.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                     unsigned int cameraIndex);

  /// @brief The IMU callback.
  void imuCallback(const sensor_msgs::msg::Imu& msg);

  /// @}
  /// @name Node and subscriber related
  /// @{
  std::shared_ptr<rclcpp::Node> node_; ///< The node handle.
  std::shared_ptr<image_transport::ImageTransport> imgTransport_; ///< The image transport.
  std::vector<image_transport::Subscriber> imageSubscribers_; ///< The image message subscriber.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;  ///< The IMU message subscriber.
  std::mutex time_mutex_; ///< Lock when accessing time

  /// @}
  
  okvis::ViInterface* viInterface_ = nullptr;   ///< The VioInterface. (E.g. ThreadedSlam).
  okvis::Publisher* publisher_ = nullptr;  ///< Publisher for IMU propagation.
  okvis::ViParameters parameters_;  ///< The parameters and settings.
  
  std::mutex imagesReceived_mutex_; ///< Lock when accessing buffer.
  std::vector<std::map<uint64_t, cv::Mat>> imagesReceived_; ///< Images obtained&buffered (to sync).
};
}

#endif /* INCLUDE_OKVIS_SUBSCRIBER_HPP_ */
