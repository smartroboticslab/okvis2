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
 * @file ViParametersReader.hpp
 * @brief Header file for the ViParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_VIPARAMETERSREADER_HPP_
#define INCLUDE_OKVIS_VIPARAMETERSREADER_HPP_

#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp>
#pragma GCC diagnostic pop

#include <okvis/assert_macros.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/cameras/NCameraSystem.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * @brief This class reads and parses config file.
 */
class ViParametersReader{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief The default constructor.
  ViParametersReader();

  /**
   * @brief The constructor. This calls readConfigFile().
   * @param filename Configuration filename.
   */
  ViParametersReader(const std::string& filename);

  /// @brief Trivial destructor.
  ~ViParametersReader() = default;

  /**
   * @brief Read and parse a config file.
   *        To get the result call getParameters().
   * @param filename Configuration filename.
   */
  void readConfigFile(const std::string& filename);

  /**
   * @brief Get parameters.
   * @param[out] parameters A copy of the parameters.
   * @return True if parameters have been read from a configuration file. If it
   *         returns false then the variable \e parameters has not been changed.
   */
  bool getParameters(okvis::ViParameters& parameters) const{
    if(readConfigFile_)
      parameters = viParameters_;
    return readConfigFile_;
  }

 protected:

  /// \brief Parse yaml file entry.
  /// @param[in] file The yaml file node.
  /// @param[in] name The variable name to be read.
  /// @param[out] readValue The read variable value.
  static void parseEntry(const cv::FileNode &file, std::string name, int& readValue);

  /// \brief Parse yaml file entry.
  /// @param[in] file The yaml file node.
  /// @param[in] name The variable name to be read.
  /// @param[out] readValue The read variable value.
  static void parseEntry(const cv::FileNode& file, std::string name, double& readValue);

  /// \brief Parse yaml file entry.
  /// @param[in] file The yaml file node.
  /// @param[in] name The variable name to be read.
  /// @param[out] readValue The read variable value.
  static void parseEntry(const cv::FileNode &file, std::string name, bool& readValue);

  /// \brief Parse yaml file entry.
  /// @param[in] file The yaml file node.
  /// @param[in] name The variable name to be read.
  /// @param[out] readValue The read variable value.
  static void parseEntry(const cv::FileNode& file, std::string name, Eigen::Matrix4d& readValue);

  /// \brief Parse yaml file entry.
  /// @param[in] file The yaml file node.
  /// @param[in] name The variable name to be read.
  /// @param[out] readValue The read variable value.
  static void parseEntry(const cv::FileNode &file, std::string name, Eigen::Vector3d& readValue);

  bool readConfigFile_; ///< If readConfigFile() has been called at least once this is true.
  okvis::ViParameters viParameters_;   ///< The parameters.

  /**
   * @brief Get the camera calibration. This looks for the calibration in the
   *        configuration file first. If this fails it will directly get the calibration
   *        from the sensor, if useDriver is set to true.
   * @remark Overload this function if you want to get the calibrations via ROS for example.
   * @param calibrations The calibrations.
   * @param configurationFile The config file.
   * @return True if reading of the calibration was successful.
   */
  bool getCameraCalibration(
      std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
      cv::FileStorage& configurationFile);

  /**
   * @brief Get the camera calibration via the configuration file.
   * @param[out] calibrations Read calibration.
   * @param[in] cameraNode File node pointing to the cameras sequence.
   * @return True if reading and parsing of calibration was successful.
   */
  bool getCalibrationViaConfig(
      std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
      cv::FileNode cameraNode) const;

};

}

#endif /* INCLUDE_OKVIS_VIOPARAMETERSREADER_HPP_ */
