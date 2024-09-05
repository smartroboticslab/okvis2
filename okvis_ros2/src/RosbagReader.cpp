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
 * @file RosbagReader.cpp
 * @brief Source file for the DatasetReader class.
 * @author Stefan Leutenegger
 */

#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <okvis/ViInterface.hpp>
#include <okvis/ros2/RosbagReader.hpp>

namespace okvis {

RosbagReader::RosbagReader(const std::string& path, size_t numCameras,
                           const std::set<size_t> &syncCameras, const Duration & deltaT) :
    numCameras_(numCameras), syncCameras_(syncCameras), deltaT_(deltaT) {
  streaming_ = false;
  setDatasetPath(path);
  counter_ = 0;
}

RosbagReader::~RosbagReader() {
  stopStreaming();
}

bool RosbagReader::setDatasetPath(const std::string & path) {
  path_ = path;
  return true;
}

bool RosbagReader::setStartingDelay(const Duration &deltaT)
{
  if(streaming_) {
    LOG(WARNING)<< "starting delay ignored, because streaming already started";
    return false;
  }
  deltaT_ = deltaT;
  return true;
}

bool RosbagReader::isStreaming()
{
  return streaming_;
}

double RosbagReader::completion() const {
  if(streaming_) {
    return double(counter_)/double(numImages_);
  }
  return 0.0;
}

bool RosbagReader::startStreaming() {
  OKVIS_ASSERT_TRUE(Exception, !imagesCallbacks_.empty(), "no add image callback registered")
  OKVIS_ASSERT_TRUE(Exception, !imuCallbacks_.empty(), "no add IMU callback registered")
  
  // set options
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = path_;
  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // open bag
  reader_.open(storage_options, converter_options);
  
  // parse metadata
  const auto & metadata = reader_.get_metadata();
  int numImuMeasurements = 0;
  std::vector<int> numRgbImages(numCameras_, 0);
  std::vector<int> numCamImages(numCameras_, 0);
  std::vector<int> numDepthImages(numCameras_, 0);
  for(const auto & info : metadata.topics_with_message_count) {
    if(info.topic_metadata.name.compare("/okvis/imu0") == 0) {
      numImuMeasurements = info.message_count;
    }
    for(int i=0; i<int(numCameras_); ++i) {
      if(info.topic_metadata.name.compare("/okvis/cam"+std::to_string(i)+"/image_raw") == 0) {
        numCamImages[i] = info.message_count;
        if(i==0) {
          numImages_ =  numCamImages[i];
        }
      }
      if(info.topic_metadata.name.compare("/okvis/rgb"+std::to_string(i)+"/image_raw") == 0) {
        numRgbImages[i] = info.message_count;
      }
      if(info.topic_metadata.name.compare("/okvis/depth"+std::to_string(i)+"/image_raw") == 0) {
        numRgbImages[i] = info.message_count;
      }
    }  
  }
  
  // print info
  LOG(INFO)<< "No. IMU measurements: " << numImuMeasurements;
  if (numImuMeasurements <= 0) {
    LOG(ERROR)<< "no imu messages present in bag";
    return -1;
  }
  for(int i=0; i<int(numCameras_); ++i) {
    LOG(INFO)<< "No. cam " << i << " RGB images: " << numRgbImages[i];
    LOG(INFO)<< "No. cam " << i << " images: " << numCamImages[i];
    LOG(INFO)<< "No. cam " << i << " depth images: " << numDepthImages[i];
  }

  counter_ = 0;
  streaming_ = true;
  processingThread_ = std::thread(&RosbagReader::processing, this);

  return true;
}

bool RosbagReader::stopStreaming() {
  // Stop the pipeline
  if(processingThread_.joinable()) {
    processingThread_.join();
    streaming_ = false;
  }
  return true;
}



void  RosbagReader::processing() {
  okvis::Time start(0.0);
  okvis::Time t_imu(0.0);
  std::map<size_t, cv::Mat> images;
  std::map<size_t, cv::Mat> depthImages;
  std::map<size_t, cv::Mat> imagesSync;
  std::map<size_t, cv::Mat> depthImagesSync;
  std::map<size_t, okvis::Time> t_images;
  std::map<size_t, okvis::Time> t_depthImages;
  bool synced = false;

  while (reader_.has_next() && streaming_) {
    // serialize data
    auto serialized_message = reader_.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    auto topic = serialized_message->topic_name;
    
    // check if IMU
    if (topic.find("/okvis/imu0") != std::string::npos) {
      sensor_msgs::msg::Imu msg;
      rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_info;
      serialization_info.deserialize_message(&extracted_serialized_msg, &msg);
      // construct measurement
      t_imu = okvis::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
      Eigen::Vector3d acc(msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z);
      Eigen::Vector3d gyr(msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z); 
      
      // add it
      for (auto &imuCallback : imuCallbacks_) {
        imuCallback(t_imu, acc, gyr);
      }
    }
    
    // check if image
    okvis::Time t_unsynced(0.0);
    for(int i=0; i<int(numCameras_); ++i) {
      if (topic.find("/okvis/cam"+std::to_string(i)+"/image_raw") != std::string::npos) {
        sensor_msgs::msg::Image msg;
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization_info;
        serialization_info.deserialize_message(&extracted_serialized_msg, &msg);
        const cv::Mat raw(msg.height, msg.width, CV_8UC1,
                    const_cast<uint8_t*>(&msg.data[0]), msg.step);
        cv::Mat image = raw.clone();
        okvis::Time time(msg.header.stamp.sec, msg.header.stamp.nanosec);
      
        if(syncCameras_.count(i)) {
          if(imagesSync.count(i)) {
            LOG(WARNING) << "image " << i << " at t=" << t_images.at(i)
                         << " without correspondence -- dropping";
          }
          imagesSync[i] = image;
        } else {
          images[i] = image;
          t_unsynced = time;
        }
        t_images[i] = time;
      }
      if (topic.find("/okvis/depth"+std::to_string(i)+"/image_raw") != std::string::npos) {
        sensor_msgs::msg::Image msg;
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization_info;
        serialization_info.deserialize_message(&extracted_serialized_msg, &msg);
        const cv::Mat raw(msg.height, msg.width, CV_32FC1,
                          reinterpret_cast<float*>(&msg.data[0]), msg.step);
        cv::Mat depthImage = raw.clone();
        okvis::Time time(msg.header.stamp.sec, msg.header.stamp.nanosec);
        
        if(syncCameras_.count(i)) {
          if(depthImagesSync.count(i)) {
            LOG(WARNING) << "depth image " << i << " at t=" << t_depthImages.at(i)
                         << " without correspondence -- dropping";
          }
          depthImagesSync[i] = depthImage;
        } else {
          depthImages[i] = depthImage;
          t_unsynced = time;
        }
        t_depthImages[i] = time;
      }
    }
    
    // check if synced
    okvis::Time t_min(0.0);
    int i_min = 0;
    okvis::Time t_max(0.0);
    bool add = false;
    bool gotAllSyncImages = true;
    if(imagesSync.size() < syncCameras_.size()) {
      synced = false; // surely not synced yet, need to wait for all
    } else {
      // check timestamps
      bool first = true;
      for(int i : syncCameras_) {
        if(!imagesSync.count(i)) {
          synced = false;
          gotAllSyncImages = false;
          break;
        } else {
          if(t_images.at(i) < t_min || first) {
            t_min = t_images.at(i);
            i_min = i;
            first = false;
          }
          if(t_images.at(i) > t_max) {
            t_max = t_images.at(i);
          }
        }
      }
      if (gotAllSyncImages) {
        if (t_max - t_min > okvis::Duration(0.01)) { // 10 ms tolerance
          LOG(WARNING) << "image " << i_min << " at t=" << t_images.at(i_min)
                       << " without correspondence -- dropping";
          imagesSync.erase(i_min);
          t_images.erase(i_min);
          synced = false;
        } else {
          synced = true;
        }
      }
    }
    
    if(synced) {
      // move
      images.insert(imagesSync.begin(), imagesSync.end());
      depthImages.insert(depthImagesSync.begin(), depthImagesSync.end());
      imagesSync.clear();
      depthImagesSync.clear();
      add = true;
    } else {
      if(!images.empty() && !depthImages.empty()) {
        add = true;
      }
    }

    // add if required
    if(add) {
      okvis::Time t;
      if(synced) {
        t = t_min + okvis::Duration(0.5*(t_max-t_min).toSec());
      } else {
        t = t_unsynced;
      }
      // finally we are ready to call the image callback

      for(auto & imagesCallback : imagesCallbacks_) {
        imagesCallback(t, images, depthImages);
      }
      if(images.count(0) && !images.at(0).empty()) {
        ++counter_; // reference for counter is always image 0.
      }
      
      // clear
      synced = false;
      images.clear();
      depthImages.clear();
      t_images.clear();
      t_depthImages.clear();
    }
    
  }
  
  // done -- stop streaming
  streaming_ = false;

  return;
}

}
