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
 * @file DatasetWriter.hpp
 * @brief Header file for the DatasetWriter class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_DATASETWRITER_HPP_
#define INCLUDE_OKVIS_DATASETWRITER_HPP_

#include <thread>
#include <atomic>
#include <sstream>
#include <ctime>
#include <fstream>

#include <boost/filesystem.hpp>

#include <okvis/Measurements.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/assert_macros.hpp>

#include <okvis/timing/Timer.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <okvis/ViInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief Dataset writer class.
class DatasetWriter : public ViInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// @brief Disallow default construction.
  DatasetWriter() = delete;

  /**
   * \brief Constructor.
   * \param parameters Parameters and settings.
   * \param path Path to dataset folder.
   */
  DatasetWriter(okvis::ViParameters& parameters,
                const std::string& path = "");

  /// \brief Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
  virtual ~DatasetWriter();

  /// \name Add grayscale image(s) to the algorithm.
  /// \{
  /**
   * \brief             Add a set of new image.
   * \param stamp       The image timestamp.
   * \param images      The images. Can be gray (8UC1) or RGB (8UC3).
   * \param depthImages The depth images as float32, in [m]
   * \return            Returns true normally. False, if previous one not processed yet.
   */
  virtual bool addImages(const okvis::Time & stamp,
                         const std::map<size_t, cv::Mat> & images,
                         const std::map<size_t, cv::Mat> & depthImages
                         = std::map<size_t, cv::Mat>()) override final;

  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return Returns true normally. False if the previous one has not been processed yet.
   */
  virtual bool addImuMeasurement(const okvis::Time & stamp,
                                 const Eigen::Vector3d & alpha,
                                 const Eigen::Vector3d & omega) final;

  /// \brief Indicats whether the add functions block. This only supports blocking.
  virtual void setBlocking(bool blocking) override;

  /// @brief Display some visualisation.
  virtual void display(std::map<std::string, cv::Mat> & images) override final;

 private:

  /// @brief Creates the directory structure according to the camera system.
  bool setupImagesDirectories();

  /// @brief Processing loop for IMU in respective thread.
  void processingImu();

  /// @brief Processing loop for Images in respective thread.
  void processingImages();

  std::atomic_bool shutdown_; ///< True if shutdown requested.
  std::thread imuProcessingThread_; ///< IMU processing thread.
  std::thread imagesProcessingThread_; ///< Image processing thread.

  /// @brief Camera measurement input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > cameraMeasurementsReceived_;
  /// @brief Visualisation (live video) input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > visualisations_;
  /// @brief RGB camera measurement input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > rgbCameraMeasurementsReceived_;
  /// @brief RGB Visualisation (live video) input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > visualisationsRGB_;
  /// @brief Depth camera measurement input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > depthCameraMeasurementsReceived_;
  /// @brief Depth Visualisation (live video) input queues.
  /// For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > visualisationsDepth_;
  /// @brief IMU measurement input queue.
  threadsafe::Queue<okvis::ImuMeasurement, Eigen::aligned_allocator<okvis::ImuMeasurement>>
      imuMeasurementsReceived_;

  okvis::ViParameters parameters_; ///< All VI parameters

  std::stringstream datasetDirectory_; ///< Path to the dataset.
  std::stringstream imuDirectory_; ///< Directory to the IMU data file.
  std::ofstream imuCsv_; ///< IMU data file.
  std::vector<std::string> camDirectories_; ///< Directories to the camera data.
  std::vector<std::ofstream> camCsvs_; ///< Camera data files.
  std::vector<std::string> depthCamDirectories_; ///< Directories to the depth camera data.
  std::vector<std::ofstream> depthCamCsvs_; ///< Depth data files.

};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_THREADEDSLAM3_HPP_ */
