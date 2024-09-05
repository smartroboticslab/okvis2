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
 * @file TrajectoryOutput.hpp
 * @brief Header file for the TrajectoryOutput class.
 * @author Stefan Leutenegger
 */

#ifndef OKVIS_TRAJECTORYOUTPUT_HPP
#define OKVIS_TRAJECTORYOUTPUT_HPP

#include "okvis/QueuedTrajectory.hpp"
#include <fstream>
#include <memory>

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/core.hpp>
#pragma GCC diagnostic pop

#include <okvis/assert_macros.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <okvis/ViInterface.hpp>
#include <okvis/QueuedTrajectory.hpp>


namespace okvis {

/**
 * @brief The TrajectoryOutput class: a simple writer of trajectory files and visualiser of
 * pose/trajectory.
 */
class TrajectoryOutput {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * @brief Default constructor.
   * @param draw Whether to visualise a top view.
   */
  TrajectoryOutput(bool draw = true);

  /**
   * @brief Constructor with filename to write CSV trajectory.
   * @param filename Write CSV trajectory to this file.
   * @param rpg If true, uses the RPG format, otherwise the EuRoC format.
   * @param draw Whether to visualise a top view.
   */
  TrajectoryOutput(const std::string & filename, bool rpg = false, bool draw = true);

  /**
   * @brief Set CSV file.
   * @param filename Write CSV trajectory to this file.
   * @param rpg If true, uses the RPG format, otherwise the EuRoC format.
   */
  void setCsvFile(const std::string & filename, bool rpg = false);

  /**
   * @brief Set RGB CSV file.
   * @param filename Write CSV trajectory to this file.
   */
  void setRGBCsvFile(const std::string & filename);

  /**
   * @brief Process the state (write it to trajectory file and visualise). Set as callback.
   * @param[in] state The current state to process.
   * @param[in] trackingState The additional tracking info to process.
   * @param[in] updatedStates All the states that the estimator updated.
   * @param[in] landmarks All the landmarks that the estimator updated.
   */
  void processState(const State& state, const TrackingState & trackingState,
                    std::shared_ptr<const AlignedMap<StateId, State>> updatedStates,
                    std::shared_ptr<const okvis::MapPointVector> landmarks);

  /**
   * \brief          Add an IMU measurement.
   * \param stamp    The measurement timestamp.
   * \param alpha    The acceleration measured at this time.
   * \param omega    The angular velocity measured at this time.
   * \return True on success.
   */
  bool addImuMeasurement(const okvis::Time& stamp,
                         const Eigen::Vector3d& alpha,
                         const Eigen::Vector3d& omega);

  /**
   * @brief Compute state at RGB image perception and store pose to RGB image.
   * @param timestamp RGB image recording timestamp.
   * @param image RGB image.
   */
  bool processRGBImage(const okvis::Time &, const cv::Mat &);

  /// \brief Draw the top view now.
  /// \param outImg The output image to draw into.
  void drawTopView(cv::Mat & outImg);

private:
  std::fstream csvFile_; ///< The CSV file.
  std::fstream rgbCsvFile_;  ///< The RGB CSV file.
  bool rpg_ = false; ///< Whether to use the RPG format (instead of EuRoC)

  bool draw_ = true; ///< Whether to draw a top view.

  /// \brief Helper for graph states.
  typedef std::tuple<
      State, TrackingState, std::shared_ptr<const AlignedMap<StateId, State>>,
      std::shared_ptr<const okvis::MapPointVector>> GraphStates;

  threadsafe::Queue<std::shared_ptr<GraphStates>> states_; ///< Graph states.
  threadsafe::Queue<ImuMeasurement> imuMeasurements_; ///< Graph states.

  cv::Mat _image; ///< Image.
  int _imageSize = 500; ///< Pixel size of the image.
  std::vector<cv::Point2d> _path; ///< Path in 2d.
  std::vector<double> _heights; ///< Heights on the path.
  double _scale = 1.0; ///< Scale of the visualisation.
  double _min_x = -0.5; ///< Minimum x coordinate the visualisation.
  double _min_y = -0.5; ///< Minimum y coordinate the visualisation.
  double _min_z = -0.5; ///< Minimum z coordinate the visualisation.
  double _max_x = 0.5; ///< Maximum x coordinate the visualisation.
  double _max_y = 0.5; ///< Maximum y coordinate the visualisation.
  double _max_z = 0.5; ///< Maximum z coordinate the visualisation.
  const double _frameScale = 0.2;  ///< [m]

  okvis::Trajectory trajectory_;  ///< interpolatable okvis trajectory.
  okvis::QueuedTrajectory<cv::Mat> rgbTrajectory_;  ///< RGB trajectory with Queue.

  /// \brief Convert metric coordinates to pixels.
  /// \param pointInMeters Point in [m].
  /// \return Point in [pixels].
  cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const;

  /// \brief Draw the visualisation (causal, gray).
  void drawPath();

  /// \brief Draw the visualisation (non-causal, coloured by height).
  void drawPathNoncausal();

  /// \brief Create a trajectory csv file and write its header.
  static bool createCsvFile(const std::string &filename, std::fstream& stream,
                            const bool rpg = false);

  /// \brief Write state into csv file.
  static bool writeStateToCsv(std::fstream& csvFile, const okvis::State& state,
                              const bool rpg = false);

  okvis::kinematics::Transformation _T_AW; ///< Transf. betw. this world coord. & another agent.

};

}

#endif // OKVIS_TRAJECTORYOUTPUT_HPP
