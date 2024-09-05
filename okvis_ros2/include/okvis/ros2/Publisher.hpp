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
 * @file Publisher.hpp
 * @brief Header file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_ROS2_PUBLISHER_HPP_
#define INCLUDE_OKVIS_ROS2_PUBLISHER_HPP_

#include <memory>

#if __has_include(<cv_bridge/cv_bridge.hpp>) // requires GCC >= 5
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h> // ros2 changed to .hpp some point...
#endif
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#pragma GCC diagnostic pop
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <okvis/ViInterface.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Time.hpp>

#include <okvis/TrajectoryOutput.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class handles the publishing to either ROS topics or files.
 */
class Publisher
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// \brief Default constructor.
  Publisher();

  /// \brief Constructor with node.
  /// \param node The ROS2 node.
  Publisher(std::shared_ptr<rclcpp::Node> node);
  ~Publisher();

  /// \name Setters
  /// \{

  /// \brief Set up the whole node.
  /// \param node The ROS2 node.
  void setupNode(std::shared_ptr<rclcpp::Node> node);

  /**
   * @brief Set the body.
   * @param T_BS Transform body-IMU.
   */
  void setBodyTransform(const okvis::kinematics::Transformation& T_BS);

  /**
   * @brief Set the realtime publishing rate.
   * @param odometryPublishingRate The rate.
   */
  void setOdometryPublishingRate(double odometryPublishingRate) {
    odometryPublishingRate_ = odometryPublishingRate;
  }

  /**
   * @brief Set CSV file.
   * @param filename Write CSV trajectory to this file.
   * @param rpg If true, uses the RPG format, otherwise the EuRoC format.
   */
  void setCsvFile(const std::string & filename, bool rpg = false);

  /// @}

  /**
   * @brief Process the updated states and publish
   *        (plus write it to trajectory file and visualise, if desired). Set as callback.
   * @param state The current state to process.
   * @param trackingState The additional tracking info to process.
   * @param updatedStates All updated states.
   * @param landmarks The currently optimised landmarks.
   */
  void publishEstimatorUpdate(const State& state, const TrackingState & trackingState,
                              std::shared_ptr<const AlignedMap<StateId, State>> updatedStates,
                              std::shared_ptr<const okvis::MapPointVector> landmarks);

  /**
   * @brief Set up the topics.
   * @param nCameraSystem Multi-camera sensor setup.
   */
  void setupImageTopics(const okvis::cameras::NCameraSystem & nCameraSystem);

  /**
   * \brief          Publish any named images as such.
   * \param images   Named images to publish.
   * \return True on success.
   */
  bool publishImages(const std::map<std::string, cv::Mat>& images) const;
  
  /**
   * \brief          Add an IMU measurement for propagation and publishing.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return True on success.
   */
  bool realtimePredictAndPublish(const okvis::Time& stamp,
                                 const Eigen::Vector3d& alpha,
                                 const Eigen::Vector3d& omega);

  private:

  TrajectoryOutput trajectoryOutput_; ///< Trajectory output in case demanded.

  /// @name Node and subscriber related
  /// @{

  std::shared_ptr<rclcpp::Node> node_; ///< The node.

  std::shared_ptr<tf2_ros::TransformBroadcaster> pubTf_;  ///< The transform broadcaster.
  /// \brief The publisher for matched points.
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubPointsMatched_;
  /// \brief The publisher for the odometry.
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pubObometry_;
  /// \brief The publisher for the path.
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pubPath_;
  /// \brief The publisher for the transform.
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TransformStamped>> pubTransform_;
  /// \brief The publisher for a robot / camera mesh.
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pubMesh_;

  /// \brief Image transporter.
  std::map<std::string, std::shared_ptr<image_transport::ImageTransport>> imagesTransport_;
  std::map<std::string, image_transport::Publisher> pubImages_; ///< Image publisher.

  /// @}
  /// @name To be published
  /// @{

  std::vector<ImuMeasurement> imuMeasurements_; ///< Buffered IMU measurements (realtime pub.).
  std::atomic_bool trajectoryLocked_; ///< Lock the trajectory object (realtime/update are async.).
  okvis::Trajectory trajectory_; ///< Underlying trajectory object for state queries.
  okvis::kinematics::Transformation T_BS_; ///< Body-IMU transform.
  visualization_msgs::msg::Marker meshMsg_; ///< Mesh message.
  double odometryPublishingRate_; ///< Publishing rate for realtime propagation.
  okvis::Time lastTime_ = okvis::Time(0); ///< Keep track of last publishing (to maintain rate).

  /// @}


};

}

#endif /* INCLUDE_OKVIS_ROS2_PUBLISHER_HPP_ */
