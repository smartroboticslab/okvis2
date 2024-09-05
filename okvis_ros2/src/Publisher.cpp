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
 * @file Publisher.cpp
 * @brief Source file for the Publisher class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include <okvis/ros2/Publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <okvis/FrameTypedefs.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Default constructor.
Publisher::Publisher(std::shared_ptr<rclcpp::Node> node)
    : trajectoryOutput_(false), trajectoryLocked_(false)
{
  setupNode(node);
}

void Publisher::setupNode(std::shared_ptr<rclcpp::Node> node)
{
  // set up node
  node_ = node;

  // set up publishers
  pubTf_.reset(new tf2_ros::TransformBroadcaster(node_));
  pubObometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("okvis_odometry", 1);
  pubPath_ = node_->create_publisher<visualization_msgs::msg::Marker>("okvis_path", 1);
  pubTransform_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>(
      "okvis_transform", 1);
  pubMesh_ = node_->create_publisher<visualization_msgs::msg::Marker>("okvis_mesh", 0 );
  pubPointsMatched_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "okvis_points_matched", 0 );

  // get the mesh, if there is one
  // where to get the mesh from
  std::string mesh_file;
  bool loaded_mesh;
  node_->declare_parameter("mesh_file", "");
  loaded_mesh = node_->get_parameter("mesh_file", mesh_file);
  if (loaded_mesh) {
    meshMsg_.mesh_resource = mesh_file;

    // fill orientation
    meshMsg_.pose.orientation.x = 0;
    meshMsg_.pose.orientation.y = 0;
    meshMsg_.pose.orientation.z = 0;
    meshMsg_.pose.orientation.w = 1;

    // fill position
    meshMsg_.pose.position.x = 0;
    meshMsg_.pose.position.y = 0;
    meshMsg_.pose.position.z = 0;

    // scale -- needed
    meshMsg_.scale.x = 1.0;
    meshMsg_.scale.y = 1.0;
    meshMsg_.scale.z = 1.0;

    meshMsg_.action = visualization_msgs::msg::Marker::ADD;
    meshMsg_.color.a = 1.0; // Don't forget to set the alpha!
    meshMsg_.color.r = 1.0;
    meshMsg_.color.g = 1.0;
    meshMsg_.color.b = 1.0;

    // embedded material / colour
    //meshMsg_.mesh_use_embedded_materials = true;
  } else {
    LOG(INFO) << "no mesh found for visualisation, set ros param mesh_file, if desired";
    meshMsg_.mesh_resource = "";
  }
}

Publisher::~Publisher()
{
}

void Publisher::setBodyTransform(const okvis::kinematics::Transformation& T_BS) {
  T_BS_ = T_BS;

  // publish pose:
  geometry_msgs::msg::TransformStamped poseMsg; // Pose message.
  poseMsg.child_frame_id = "sensor";
  poseMsg.header.frame_id = "body";
  poseMsg.header.stamp = node_->now();

  // fill orientation
  Eigen::Quaterniond q = T_BS_.q();
  poseMsg.transform.rotation.x = q.x();
  poseMsg.transform.rotation.y = q.y();
  poseMsg.transform.rotation.z = q.z();
  poseMsg.transform.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = T_BS_.r();
  poseMsg.transform.translation.x = r[0];
  poseMsg.transform.translation.y = r[1];
  poseMsg.transform.translation.z = r[2];
  static tf2_ros::StaticTransformBroadcaster br(node_);
  br.sendTransform(poseMsg);
}

void Publisher::setCsvFile(const std::string & filename, bool rpg)
{
  return trajectoryOutput_.setCsvFile(filename, rpg);
}

bool Publisher::realtimePredictAndPublish(const okvis::Time& stamp,
                         const Eigen::Vector3d& alpha,
                         const Eigen::Vector3d& omega) {

  // store in any case
  imuMeasurements_.push_back(ImuMeasurement(stamp, ImuSensorReadings(omega,alpha)));

  // add to Trajectory if possible
  bool success = false;
  State state;
  if(!trajectoryLocked_) {
    trajectoryLocked_ = true;
    for(auto & imuMeasurement : imuMeasurements_) {
      State propagatedState;
      if(trajectory_.addImuMeasurement(imuMeasurement.timeStamp,
                                       imuMeasurement.measurement.accelerometers,
                                       imuMeasurement.measurement.gyroscopes,
                                       propagatedState)) {
        state = propagatedState;
        success = true;
      }
    }
    imuMeasurements_.clear();
    trajectoryLocked_ = false;
    if(!success) {
      return false;
    }
  } else {
    return false;
  }

  // only publish according to rate
  if((state.timestamp - lastTime_).toSec()
      < (1.0/double(odometryPublishingRate_))) {
    return false;
  }
  lastTime_ = state.timestamp;

  rclcpp::Time t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
  const okvis::kinematics::Transformation T_WS = state.T_WS;
  const kinematics::Transformation T_SB = T_BS_.inverse();
  const okvis::kinematics::Transformation T_WB = T_WS * T_SB;

  // Odometry
  nav_msgs::msg::Odometry odometryMsg;  // Odometry message.
  odometryMsg.header.frame_id = "world";
  odometryMsg.child_frame_id = "body";
  odometryMsg.header.stamp = t;

  // fill orientation
  const Eigen::Quaterniond q = T_WB.q();
  odometryMsg.pose.pose.orientation.x = q.x();
  odometryMsg.pose.pose.orientation.y = q.y();
  odometryMsg.pose.pose.orientation.z = q.z();
  odometryMsg.pose.pose.orientation.w = q.w();

  // fill position
  const Eigen::Vector3d r = T_WB.r();
  odometryMsg.pose.pose.position.x = r[0];
  odometryMsg.pose.pose.position.y = r[1];
  odometryMsg.pose.pose.position.z = r[2];

  // note: velocity and angular velocity needs to be expressed in child frame, i.e. "body"
  // see http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html

  // fill velocity
  const kinematics::Transformation T_BW = T_WB.inverse();
  const Eigen::Vector3d v = T_BW.C() * state.v_W + T_BS_.C() * T_SB.r().cross(state.omega_S);
  // ...of body orig. represented in body
  odometryMsg.twist.twist.linear.x = v[0];
  odometryMsg.twist.twist.linear.y = v[1];
  odometryMsg.twist.twist.linear.z = v[2];

  // fill angular velocity
  const Eigen::Matrix3d C_BS = T_BS_.C();
  const Eigen::Vector3d omega_B = C_BS * state.omega_S; // of body represented in body
  odometryMsg.twist.twist.angular.x = omega_B[0];
  odometryMsg.twist.twist.angular.y = omega_B[1];
  odometryMsg.twist.twist.angular.z = omega_B[2];

  // publish odometry
  pubObometry_->publish(odometryMsg);
  return true;
}

void Publisher::publishEstimatorUpdate(
  const State& state, const TrackingState & trackingState,
  std::shared_ptr<const AlignedMap<StateId, State>> updatedStates,
  std::shared_ptr<const MapPointVector> landmarks) {

  // forward to existing writer
  trajectoryOutput_.processState(state, trackingState, updatedStates, landmarks);

  // pose
  rclcpp::Time t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
  const okvis::kinematics::Transformation T_WS = state.T_WS;
  const kinematics::Transformation T_SB = T_BS_.inverse();
  const okvis::kinematics::Transformation T_WB = T_WS * T_SB;

  // publish pose:
  geometry_msgs::msg::TransformStamped poseMsg; // Pose message.
  poseMsg.child_frame_id = "body";
  poseMsg.header.frame_id = "world";
  poseMsg.header.stamp = t;
  //if ((node_->now() - t).seconds() > 10.0)
  //  poseMsg.header.stamp = node_->now(); // hack for dataset processing

  // fill orientation
  Eigen::Quaterniond q = T_WB.q();
  poseMsg.transform.rotation.x = q.x();
  poseMsg.transform.rotation.y = q.y();
  poseMsg.transform.rotation.z = q.z();
  poseMsg.transform.rotation.w = q.w();

  // fill position
  Eigen::Vector3d r = T_WB.r();
  poseMsg.transform.translation.x = r[0];
  poseMsg.transform.translation.y = r[1];
  poseMsg.transform.translation.z = r[2];

  // publish current pose
  static tf2_ros::TransformBroadcaster br(node_);
  br.sendTransform(poseMsg);
  pubTransform_->publish(poseMsg);

  // also do the mesh
  meshMsg_.header.frame_id = "body";
  meshMsg_.header.stamp = t;
  //if ((node_->now() - t).seconds() > 10.0)
  //  meshMsg_.header.stamp = node_->now(); // hack for dataset processing
  meshMsg_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

  // publish mesh
  if(!meshMsg_.mesh_resource.empty())
    pubMesh_->publish(meshMsg_);  //publish stamped mesh

  // check if empty
  if(updatedStates->empty()) {
    return; // otherwise it will override landmarks and path (plus create unnecessary traffic)
  }

  // now handle all the (key)frames & path:
  for(const auto & updatedState : *updatedStates) {
    rclcpp::Time updatedState_t(state.timestamp.sec, state.timestamp.nsec); // Header timestamp.
    const okvis::kinematics::Transformation T_WS = updatedState.second.T_WS;

    geometry_msgs::msg::TransformStamped updatedStatePoseMsg; // Pose message.
    const okvis::kinematics::Transformation T_WB = T_WS * T_SB;
    updatedStatePoseMsg.child_frame_id = "body_"+std::to_string(updatedState.second.id.value());
    updatedStatePoseMsg.header.frame_id = "world";
    updatedStatePoseMsg.header.stamp = updatedState_t;
    updatedStatePoseMsg.transform.rotation.x = T_WB.q().x();
    updatedStatePoseMsg.transform.rotation.y = T_WB.q().y();
    updatedStatePoseMsg.transform.rotation.z = T_WB.q().z();
    updatedStatePoseMsg.transform.rotation.w = T_WB.q().w();
    updatedStatePoseMsg.transform.translation.x = T_WB.r()[0];
    updatedStatePoseMsg.transform.translation.y = T_WB.r()[1];
    updatedStatePoseMsg.transform.translation.z = T_WB.r()[2];

    // publish
    br.sendTransform(updatedStatePoseMsg);
  }

  // update Trajectory object
  std::set<okvis::StateId> affectedStateIds;
  while(trajectoryLocked_);
  trajectoryLocked_ = true;
  trajectory_.update(trackingState, updatedStates, affectedStateIds);
  trajectoryLocked_ = false;

  // now for the path (in segments of 1000 state IDs)
  visualization_msgs::msg::Marker path; // The path message.
  path.header.stamp = t;
  path.header.frame_id = "world";
  path.ns = "okvis_path";
  path.type = visualization_msgs::msg::Marker::LINE_STRIP; // Type of object
  path.action = 0; // 0 add/modify an object, 1 (dprcd), 2 deletes an object, 3 deletes all objects
  path.pose.position.x = 0.0;
  path.pose.position.y = 0.0;
  path.pose.position.z = 0.0;
  path.pose.orientation.x = 0.0;
  path.pose.orientation.y = 0.0;
  path.pose.orientation.z = 0.0;
  path.pose.orientation.w = 1.0;
  path.color.r = 0.8;
  path.color.g = 0.8;
  path.color.b = 0.0;
  path.color.a = 1.0;
  path.scale.x = 0.015;
  path.lifetime = rclcpp::Duration(0,0); // 0 means for
  // publish paths as batches of 1000 points.
  uint64_t firstId = affectedStateIds.begin()->value();
  uint64_t id = (firstId/1000)*1000;
  if(id==0) {
    id = 1;
  }
  path.id = (id/1000)*1000;
  if(path.id != 0) {
    // link to previous path segment
    okvis::State state;
    if(trajectory_.getState(okvis::StateId(path.id-1), state)) {
      const okvis::kinematics::Transformation T_WB_p = state.T_WS * T_SB;
      const Eigen::Vector3d& r = T_WB_p.r();
      geometry_msgs::msg::Point point;
      point.x = r[0];
      point.y = r[1];
      point.z = r[2];
      path.points.push_back(point);
    }
  }
  const uint64_t latestId = updatedStates->rbegin()->first.value();
  while(id <= latestId) {
    uint64_t roundedId = (id/1000)*1000;
    if(path.id != int64_t(roundedId)) {
      pubPath_->publish(path); // first publish finished segment
      path.id = roundedId;
      // ..object ID useful in conjunction with namespace for manipulating&deleting the object later
      geometry_msgs::msg::Point lastPoint =  path.points.back(); // save last point
      path.points.clear(); // start new segment
      path.points.push_back(lastPoint);
    }
    okvis::State state;
    if(!trajectory_.getState(okvis::StateId(id), state)) continue;
    const okvis::kinematics::Transformation T_WB_p = state.T_WS * T_SB;
    const Eigen::Vector3d& r = T_WB_p.r();
    geometry_msgs::msg::Point point;
    point.x = r[0];
    point.y = r[1];
    point.z = r[2];
    path.points.push_back(point);
    ++id;
  }
  pubPath_->publish(path); // publish last segment

  // finally the landmarks
  pcl::PointCloud<pcl::PointXYZRGB> pointsMatched; // Point cloud for matched points.
  pointsMatched.reserve(landmarks->size());

  // transform points into custom world frame:
  /// \todo properly from ros params -- also landmark thresholds below
  for (const auto & lm : *landmarks) {
    // check infinity
    if (fabs((double) (lm.point[3])) < 1.0e-8)
      continue;

    // check quality
    if (lm.quality < 0.01)
      continue;

    pointsMatched.push_back(pcl::PointXYZRGB());
    const Eigen::Vector4d point = lm.point;
    pointsMatched.back().x = point[0] / point[3];
    pointsMatched.back().y = point[1] / point[3];
    pointsMatched.back().z = point[2] / point[3];
    pointsMatched.back().g = 255 * (std::min(0.1f, (float)lm.quality) / 0.1f);
  }
  pointsMatched.header.frame_id = "world";
  sensor_msgs::msg::PointCloud2 pointsMatchedMsg;
  pcl::toROSMsg(pointsMatched, pointsMatchedMsg);
  pointsMatchedMsg.header.frame_id = "world";

#if PCL_VERSION >= PCL_VERSION_CALC(1,7,0)
  std_msgs::msg::Header header;
  header.stamp = t;
  pointsMatched.header.stamp = pcl_conversions::toPCL(header).stamp;
#else
  pointsMatched.header.stamp=_t;
#endif

  // and now publish them:
  pubPointsMatched_->publish(pointsMatchedMsg);

}

void Publisher::setupImageTopics(const okvis::cameras::NCameraSystem & nCameraSystem) {
  imagesTransport_.clear();
  pubImages_.clear();
  for(size_t i=0; i< nCameraSystem.numCameras(); ++i) {
    std::string name;
    if(nCameraSystem.cameraType(i).isColour) {
      name = "rgb"+std::to_string(i);
    } else {
      name = "cam"+std::to_string(i);
    }
    imagesTransport_[name] = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(node_));
    pubImages_[name] = imagesTransport_.at(name)->advertise(name+"_matches", 1);
  }
  std::string name = "Top Debug View";
  imagesTransport_[name] = std::shared_ptr<image_transport::ImageTransport>(
    new image_transport::ImageTransport(node_));
  pubImages_[name] = imagesTransport_.at(name)->advertise("top_debug_view", 1);
}

bool Publisher::publishImages(const std::map<std::string, cv::Mat>& images) const {
  for(const auto & image : images) {
    sensor_msgs::msg::Image::SharedPtr msg
      = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image.second).toImageMsg();
    const auto & pubIter = pubImages_.find(image.first);
    if(pubIter == pubImages_.end()) {
      LOG(WARNING) << image.first
                   << " topic not set up. Did you call Publisher::setupImageTopics (correctly)?";
    }
    pubIter->second.publish(msg);
  }
  return true;
}

}  // namespace okvis
