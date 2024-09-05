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
 * @file RealsenseRgbd.hpp
 * @brief Header file for the Realsense Rgbd class.
 * @author Simon Schaefer
 */

#ifndef INCLUDE_OKVIS_REALSENSE_RGBD_HPP_
#define INCLUDE_OKVIS_REALSENSE_RGBD_HPP_

#include <glog/logging.h>

#include <okvis/assert_macros.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Realsense.hpp>

namespace okvis {

/// @brief RealsenseRGBD (D455, for now) sensor interface.
class RealsenseRgbd : public Realsense {
public:
  /// @brief Constructor. Won't start the streaming.
  /// @param sensorType Specify the sensor type. Only D455 supported for now.
  /// @param enableRgb Enable RGB camera?
  /// @param alignDepthToRgb If true, will align to RGB camera, else to left IR.
  /// @param emitterPower Set the emitter power. Unfortunately, this is not adaptive...
  RealsenseRgbd(SensorType sensorType, bool enableRgb = false,
                bool alignDepthToRgb = false, double emitterPower = 150.0);

  /// @brief Starts streaming.
  /// @return True, if successful
  virtual bool startStreaming() override final;

  /// \brief Process a frame.
  /// \param frame The frame.
  void processFrame(const rs2::frame &frame) override final;

protected:

  /// @brief Process a depth frame.
  /// @param[in] frame The frame.
  /// @param[inout] outFrame The processed output (imserted depth image).
  /// @param[out] timestamp The corresponding timestamp.
  /// \return True on success.
    bool processDepth_(const rs2::frameset &frame,
                       std::map<size_t, cv::Mat> &outFrame,
                       Time &timestamp);

private:
  double depth_scale_; ///< Device specific depth scale. depth_meters = depth_scale * pixel_value;
  bool imuStreamStarted_ = false; ///< Data stream has started.
  bool supportsMetadata_ = true; ///< Supports metadata for reading emitter mode.
  bool alignDepthToRgb_ = false; ///< Will align to RGB camera, else to left IR.
  double emitterPower_ = 150.0; ///< The emitter power. Unfortunately, this is not adaptive...
};

}

#endif // INCLUDE_OKVIS_REALSENSE_RGBD_HPP_
