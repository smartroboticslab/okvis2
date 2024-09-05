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
 * @file Realsense.hpp
 * @brief Header file for the Realsense class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_REALSENSE_HPP_
#define INCLUDE_OKVIS_REALSENSE_HPP_

#include <atomic>

#include <glog/logging.h>
#include <librealsense2/rs.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/ViSensorBase.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief Realsense (D435i, for now) sensor interface.
/// adopted from
/// https://github.com/IntelRealSense/librealsense/blob/master/examples/
class Realsense : public ViSensorBase {
public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief The supported sensor types. Only D435i and D455 for now.
  enum class SensorType {
    D435i,
    D455
  };

  /// @brief Constructor. Won't start the streaming.
  /// @param sensorType Specify the sensor type. Only D435i for now.
  /// @param enableRgb Enable RGB camera?
  Realsense(SensorType sensorType, bool enableRgb = false);

  /// @brief Alternative constructor. Won't start the streaming.
  /// @param sensorType Specify the sensor type.
  /// @param enableRgb Enable RGB camera?
  /// @param hasDeviceTimestamps Has kernel patch for device timestamps?
  /// @param rgbSize RGB image size.
  /// @param irSize IR image size.
  Realsense(SensorType sensorType,
            bool enableRgb,
            bool hasDeviceTimestamps,
            const cv::Size& rgbSize,
            const cv::Size& irSize);

  /// @brief Destructor. Will also stop the streaming, if started.
  virtual ~Realsense();

  /// @brief Set the sensor type. Only D435i for now.
  /// @param sensorType Specify the sensor type.
  void setSensorType(SensorType sensorType);

  /// @brief Manually overwrite flag for supporting metadata retrieval.
  /// @param hasDeviceTimestamps Has kernel patch for device timestamps?
  void setHasDeviceTimestamps(const bool hasDeviceTimestamps);

  /// @brief Specify IR image size. \warning Use eligible values.
  /// @param width Image width.
  /// @param height Image height.
  void setIrSize(int width, int height);

  /// @brief Specify RGB image size. \warning Use eligible values.
  /// @param width Image width.
  /// @param height Image height.
  void setRgbSize(int width, int height);

  /// @brief Starts streaming.
  /// @return True, if successful
  bool startStreaming() override;

  /// @brief Stops streaming.
  /// @return True, if successful
  bool stopStreaming() override final;

  /// @brief Check if currently streaming.
  /// @return True, if streaming.
  bool isStreaming() override final;

  /// \brief Process a frame.
  /// \param frame The frame.
  virtual void processFrame(const rs2::frame &frame);

protected:
  /// @brief Checks the support of visual and inertial sensors.
  /// @return True if supported.
  bool checkSupport();

  /// @brief Configurable internal start streaming and starting the realsense pipe for IR
  /// and RGB (if callback exists) frames.
  /// @attention streaming_ = true is not set by this function!
  bool startStreaming_(const cv::Size& irSize, const uint irFps, const cv::Size& rgbSize, const uint rgbFps);

  /// @brief Obtain the name of a Realsense device.
  /// @param dev Realsense device.
  /// @return The name string.
  static std::string getDeviceName(const rs2::device& dev);

  /// @brief Convert a Realsense frame to a cv::Mat.
  /// @param frame The input Realsense frame.
  /// @param width The image width in pixels.
  /// @param height The image height in pixels.
  /// @param format The OpenCV format of the returned matrix, e.g. CV_8UC1
  /// @return The corresponding cv::Mat.
  cv::Mat frame2Mat(const rs2::frame &frame, const int width,
                    const int height, const int format);

  /// @brief Check incoming frame and update host - device time offset.
  /// @param frame The incoming frame.
  virtual bool checkFrameAndUpdate(const rs2::frame &frame);

  /// @brief Process IR frame.
  /// @param frame The incoming frame.
  /// @param frameInOut IR image will be inserted here.
  /// @param timestamp The frame timestamp.
  virtual bool processIr_(const rs2::frameset &frame,
                          std::map<size_t, cv::Mat> &frameInOut,
                          Time &timestamp);

  /// @brief Process RGB frame.
  /// @param frame The incoming frame.
  /// @param frameInOut RGB image will be inserted here.
  /// @param timestamp The frame timestamp.
  virtual bool processRgb_(const rs2::frameset &frame,
                           std::map<size_t, cv::Mat> &frameInOut,
                           Time &timestamp);

  /// @brief Process IMU, and put into gyr / acc buffers.
  /// @param frame The incoming data frame.
  virtual bool processImu_(const rs2::frame &frame);

  /// @brief Computes the host time.
  uint64_t computeTimeStampFromFrame_(const rs2::frame &frame,
                                      const rs2_frame_metadata_value frameMetadata,
                                      okvis::Time &ts);

  std::atomic_bool streaming_; ///< True if streaming started.
  std::atomic_bool started_; ///< True if the startup process was successful.

  /// @brief Gyro buffer; guarantees monotinic increase.
  AlignedMap<uint64_t, okvis::Measurement<Eigen::Matrix<double, 3,1, Eigen::DontAlign>>> gyrBuffer_;

  /// @brief Accelerometer buffer; guarantees monotinic increase.
  AlignedMap<uint64_t, okvis::Measurement<Eigen::Matrix<double, 3,1, Eigen::DontAlign>>> accBuffer_;

  /// @brief Declare RealSense pipeline, encapsulating the actual device and sensors.
  rs2::pipeline pipe_;

  /// @brief Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg_;

  /// @brief The pipeline profile includes a device and a selection
  /// of active streams, with specific profiles.
  rs2::pipeline_profile profile_;

  int64_t firstTimeUs_ = 0; ///< Timestamp of first frame received.
  Realsense::SensorType sensorType_; ///< The Realsense sensor type.
  uint64_t hostTimeOffsetUs_ = 0; ///< Estimates the offset between host and device time.
  int frameCounter_ = 0; ///< Counts the received frames.
  int imageReceivedCounter_ = 0; ///< Counts number of received images.

  /// \brief If true, device times will be used (and converted).
  /// \warning This requires kernel patching and respective librealsense installation.
  /// If not available, host timestamps will be used after issuing a warning.
  bool hasDeviceTimestamps_ = true; ///< Device timestamps enabled?
  bool enableRgb_ = false; ///< RGB cam enabled?

  int lastImgIdx_ = -1; ///< Tracks received image sequence number.
  int lastRGBImgIdx_ = -1;  ///< Tracks received RGB image sequence number.
  int lastGyrIdx_ = 0; ///< Tracks received gyro sequence number.
  int lastAccIdx_ = 0; ///< Tracks received accelerometer sequence number.

  cv::Size irSize_ = cv::Size(640, 480); ///< Size of IR images.
  cv::Size rgbSize_ = cv::Size(1280, 720); ///< Size of RGB images.
};

}

#endif // INCLUDE_OKVIS_REALSENSE_HPP_
