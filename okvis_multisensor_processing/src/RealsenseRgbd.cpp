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
 * @file RealsenseRgbd.cpp
 * @brief Source file for the Realsense Rgbd class.
 * @author Simon Schaefer
 */

#include <okvis/RealsenseRgbd.hpp>

namespace okvis {

RealsenseRgbd::RealsenseRgbd(okvis::Realsense::SensorType sensorType, bool enableRgb, 
                             bool alignDepthToRgb, double emitterPower)
  : Realsense(sensorType, enableRgb), alignDepthToRgb_(alignDepthToRgb), 
    emitterPower_(emitterPower) {
  OKVIS_ASSERT_TRUE(Exception, sensorType == okvis::Realsense::SensorType::D455,
                    "Depth channel not supported")
}

static int ctr=0;

void RealsenseRgbd::processFrame(const rs2::frame &frame) {
  if(!streaming_) {
    return; // be sure the emitter stuff is set up first
  }
  if(!checkFrameAndUpdate(frame)) {
    return;
  }

  if(const auto &fs = frame.as<rs2::frameset>()) {
    // outputs
    bool call = false;
    std::map<size_t,cv::Mat> outFrame;
    std::map<size_t,cv::Mat> outDepthFrame;
    Time timestamp;
    
    // sometimes in the beginning, the realsense driver will get duplicate frames...
    static unsigned long long lastFrameTimeMs = 0;
    const unsigned long long  frameTimeMs = fs.get_frame_number();
    if(frameTimeMs <= lastFrameTimeMs) {
      LOG(WARNING)<< "RealSense duplicate frame time -- skip";
      return;
    }
    lastFrameTimeMs = frameTimeMs;
    
    // Process IR frames
    const auto ir0 = fs.get_infrared_frame(1);
    const auto ir1 = fs.get_infrared_frame(2);
    if(ir0 && ir1 && imuStreamStarted_) {
    
      // unfortunately, we cannot do the following, as it resets the on-off option and won't turn on
      // fast enough...
      /*if (ir0.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
        auto exposure = ir0.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE);
        const float laser_power = 640000.0f/float(exposure);
        rs2::device selected_device = profile_.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, laser_power);
        depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 1.f);
      }*/

      // Status of the emitter
      const bool supports_md_0 = ir0.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
      const bool supports_md_1 = ir1.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
      if(supportsMetadata_ && (!supports_md_0 || !supports_md_1)) {
        LOG(WARNING)<< "reading the emitter mode is not supported, turning off emitter";
        rs2::device selected_device = profile_.get_device();
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();
        depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.f);
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f);
        supportsMetadata_ = false;
        return;
      }
      bool emitter_status_0 = false;
      bool emitter_status_1 = false;
      if(supportsMetadata_) {
        emitter_status_0 = ir0.get_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
        emitter_status_1 = ir1.get_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
      }

      // Process IR or RGB frames, depending on the emitter status and frame number.
      // todo:  RS2_FRAME_METADATA_SENSOR_TIMESTAMP does not work for global time???
      if (supportsMetadata_ && !emitter_status_0 && !emitter_status_1) {
        if(processIr_(fs, outFrame, timestamp)) {
          call = true;
        }
      }
      if(!supportsMetadata_) {
        // just process every other frame
        if(ctr%2==0) {
          if(processIr_(fs, outFrame, timestamp)) {
            call = true;
          }
        }
        ctr++;
      }
    }
    
    // Process colour
    if(fs.get_color_frame() && imuStreamStarted_) {
      if(processRgb_(fs, outFrame, timestamp)) {
        call = true;
      }
    }

    // Process depth frame.
    const auto depth = fs.get_depth_frame();
    if(depth && imuStreamStarted_) {
      const bool supports_md = depth.supports_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
      if(supports_md) {
        const bool emitter_status = fs.get_frame_metadata(RS2_FRAME_METADATA_FRAME_EMITTER_MODE);
        if(emitter_status) {
          if(processDepth_(fs, outDepthFrame, timestamp)) {
            call = true;
          }
        }
      } else {
        if(ctr%2==1) {
          if(processDepth_(fs, outDepthFrame, timestamp)) {
            call = true;
          }
        }
        ctr++;
      } 
    }

    // callback!
    if(call) {
      imageReceivedCounter_++;
      // startup frames are always bad.
      if(imageReceivedCounter_ > 15) {
        for (auto &imagesCallback : imagesCallbacks_) {
          imagesCallback(timestamp, outFrame, outDepthFrame);
        }
      }
    }
  }

  // Otherwise the frame is a motion frame. Collect and process the IMU data.
  bool imu_valid = processImu_(frame);
  if(!imuStreamStarted_ && imu_valid) {
    imuStreamStarted_ = true;
  }
}

bool RealsenseRgbd::processDepth_(const rs2::frameset &frame, std::map<size_t, cv::Mat> & outFrame,
                                  Time & timestamp) {
  const auto depth_frame = frame.get_depth_frame();  // implicitly assumes that fs is not None
  if (!depth_frame)
    return false;

  computeTimeStampFromFrame_(frame, RS2_FRAME_METADATA_SENSOR_TIMESTAMP, timestamp);

  // todo: RS2_FRAME_METADATA_SENSOR_TIMESTAMP

  if(alignDepthToRgb_) {
    // depth frame aligned to the RGB frame idx 2
    rs2::align align_to_color(RS2_STREAM_COLOR);
    const auto aligned_frameset = align_to_color.process(frame);
    const auto depth_frame_aligned = aligned_frameset.get_depth_frame();

    // Convert to OpenCV Mat
    auto depth_frame_aligned_mat = frame2Mat(
          depth_frame_aligned, rgbSize_.width, rgbSize_.height, CV_16UC1).clone();
    depth_frame_aligned_mat.convertTo(depth_frame_aligned_mat, CV_32F);
    outFrame[2] = depth_frame_aligned_mat * depth_scale_; // now in m
  } else {
    // leave aligned with IR frame idx 0
    auto depth_mat = frame2Mat(depth_frame, 640, 480, CV_16UC1).clone();
    depth_mat.convertTo(depth_mat, CV_32F);
    outFrame[0] = depth_mat * depth_scale_;  // now in m
  }

  return true;
}

bool RealsenseRgbd::startStreaming() {
  bool success = startStreaming_(irSize_, 30, rgbSize_, 15);
  cfg_.enable_stream(RS2_STREAM_DEPTH, irSize_.width, irSize_.height, RS2_FORMAT_ANY, 30);
  profile_ = pipe_.start(cfg_, std::bind(&Realsense::processFrame, this, std::placeholders::_1));

  rs2::device selected_device = profile_.get_device();
  auto depth_sensor = selected_device.first<rs2::depth_sensor>();
  // Set maximum laser power [0, 360].
  depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 0.f);
  depth_sensor.set_option(RS2_OPTION_LASER_POWER, emitterPower_);
  // Enable the emitter
  depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
  // Enable alternating emitter mode
  depth_sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, 1.f);
  // Enable global timestamps, shared across the sensors.
  selected_device.first<rs2::depth_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);
  selected_device.first<rs2::motion_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);
  selected_device.first<rs2::color_sensor>().set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, true);

  // Make sure depth scale is initialised properly.
  depth_scale_ = selected_device.first<rs2::depth_sensor>().get_depth_scale();

  streaming_ = true;
  return success;
}

}
