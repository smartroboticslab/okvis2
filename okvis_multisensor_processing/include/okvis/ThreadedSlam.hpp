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
 * @file ThreadedSlam.hpp
 * @brief Header file for the ThreadedSlam3 class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_THREADEDSLAM3_HPP_
#define INCLUDE_OKVIS_THREADEDSLAM3_HPP_

#include <thread>
#include <atomic>

#include <opencv2/core/core.hpp>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/Frontend.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/assert_macros.hpp>

#include <okvis/ViVisualizer.hpp>
#include <okvis/timing/Timer.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <okvis/ViSlamBackend.hpp>
#include <okvis/ViInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 *  \brief
 *  This class manages the complete data flow in and out of the algorithm, as well as
 *  between the processing threads.
 *
 *  To ensure fast return from user callbacks, new data are collected in thread save
 *  input queues and are processed in data consumer threads, one for each data source.
 *  The algorithm tasks are running in individual threads with thread save queue for
 *  message passing.
 *  For sending back data to the user, publisher threads are created for each output
 *  which ensure that the algorithm is not being blocked by slow users.
 *
 *  All the queues can be limited to size 1 to back propagate processing congestions
 *  to the user callback.
 */
class ThreadedSlam : public ViInterface {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /**
   * \brief Constructor.
   * \param parameters Parameters and settings.
   * @param dBowDir The directory to the DBoW vocabulary.
   */
  ThreadedSlam(okvis::ViParameters& parameters, std::string dBowDir);

  /// \brief Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
  virtual ~ThreadedSlam() override;

  /// \name Add measurements to the algorithm
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
                                 const Eigen::Vector3d & omega) override final;


  /// \}
  /// \name Setters
  /// \{
  /**
   * \brief Set the blocking variable that indicates whether the addMeasurement() functions
   *        should return immediately (blocking=false), or only when the processing is complete.
   */
  virtual void setBlocking(bool blocking) override final;

  /// \brief Runs main processing iteration, call in your main loop.
  bool processFrame();

  /// \brief Trigger display (needed because OSX won't allow threaded display).
  void display(std::map<std::string, cv::Mat> & images) override final;

  /// \brief Set the final trajectory CSV file.
  /// \param finalTrajectoryCsvFileName File name.
  /// \param rpg Use RPG format?
  void setFinalTrajectoryCsvFile(const std::string & finalTrajectoryCsvFileName, bool rpg = false) {
    finalTrajectoryCsvFileName_ = finalTrajectoryCsvFileName;
    rpg_ = rpg;
  }

  /// \brief Set the map CSV file.
  /// \param mapCsvFileName File name.
  void setMapCsvFile(const std::string & mapCsvFileName) {
    mapCsvFileName_ = mapCsvFileName;
  }
  
  /// \brief Stops all threading.
  void stopThreading();

  /// \brief Writes the final trajectory CSV file.
  void writeFinalTrajectoryCsv();

  /// \brief Performs a final Bundle Adjustment.
  void doFinalBa();

  /// \brief Save the map to CSV.
  bool saveMap();

  /// \}

private:
  /// \brief Start all threads.
  void startThreads();
  /// \brief Initialises settings and calls startThreads().
  void init();

  /// \brief Wait (blocking) for next synchronised multiframe.
  bool getNextFrame(MultiFramePtr &multiFrame);

  /// \brief Performs the optimisation, publishing&visualisation preps, and marginalisation.
  void optimisePublishMarginalise(MultiFramePtr multiFrame, const Eigen::Vector3d& gyroReading);

  /// \brief Performs visualisation in the background.
  void visualisationLoop();

  /// \brief Performs publishing in the background.
  void publishingLoop();

  std::atomic_bool blocking_; ///< Blocking or non-blocking operation mode?

  /// @name Measurement input queues
  /// @{

  /// Camera measurement input queues. For each camera in the configuration one.
  threadsafe::Queue<std::vector<okvis::CameraMeasurement> > cameraMeasurementsReceived_;

  /// IMU measurement input queue.
  threadsafe::Queue<okvis::ImuMeasurement, Eigen::aligned_allocator<okvis::ImuMeasurement>>
      imuMeasurementsReceived_;

  /// IMU measurement input queue for realtime propagation.
  threadsafe::Queue<okvis::ImuMeasurement, Eigen::aligned_allocator<okvis::ImuMeasurement>>
      imuMeasurementsReceivedPropagate_;

  /// The queue containing the matching data
  threadsafe::Queue<ViVisualizer::VisualizationData::Ptr> visualisationData_;

  /// The queue containing the actual overhead display images
  threadsafe::Queue<cv::Mat> overheadImages_;

  /// The queue containing the actual matching display images
  threadsafe::Queue<std::vector<cv::Mat>> visualisationImages_;

  ViVisualizer visualizer_; ///< The matching visualiser.

  /// @}

  /// @name Consumer threads
  /// @{

  std::thread visualisationThread_; ///< Thread running visualisation.
  std::thread publishingThread_; ///< Thread running publishing.

  /// @}
  /// @name Algorithm threads
  /// @{

  std::thread optimisationThread_; ///< Thread running optimisation.
  std::thread fullGraphOptimisationThread_; ///< Thread running loopclosure full graph optimisation.

  /// @}

  // publishing:
  /// \brief Realtime publishing data struct.
  struct PublicationData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    State state; ///< The current state.
    TrackingState trackingState; ///< The tracking state.
    std::shared_ptr<AlignedMap<StateId, State>> updatedStates; ///< Updated states (may be empty).
    std::shared_ptr<MapPointVector> landmarksPublish; ///< The landmarks.
  };
  threadsafe::Queue<PublicationData> lastOptimisedQueue_; ///< Queue of last optimised results.
  threadsafe::Queue<PublicationData> publicationQueue_; ///< Publication queue.
  std::set<StateId> affectedStates_; ///< Affected states (changed pose graph edges) since pub.

  // realtime publishing:
  std::atomic_bool hasStarted_; ///< Has it started yet?

  // processing loop variables:
  bool firstFrame_ = true; ///< Is it the first frame?
  ImuMeasurementDeque imuMeasurementDeque_;  ///< Stored IMU measurements to be used next.

  /// \brief Stored IMU measurements by frame.
  AlignedMap<StateId, ImuMeasurementDeque> imuMeasurementsByFrame_;

  /// @name Algorithm objects.
  /// @{

  okvis::ViSlamBackend estimator_;    ///< The backend estimator.
  okvis::Frontend frontend_;      ///< The frontend.
  /// @}

  okvis::ViParameters parameters_; ///< The parameters and settings.

  std::string finalTrajectoryCsvFileName_; ///< File name of final trajectory log.
  bool rpg_ = false; ///< Use the RPG format for logs rather than EuRoC.
  std::string mapCsvFileName_; ///< The map file name.
  
  ::ceres::Solver::Summary posegraphOptimisationSummary_; ///< Posegraph optimisation summary.

  State lastOptimisedState_; ///< Store at least pose/id/timestamp here to avoid race on estimator_.
  State preLastOptimisedState_; ///< Store at previous to last state here to avoid race.

  std::atomic_bool shutdown_; ///< Has shutdown been called?

};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_THREADEDSLAM3_HPP_ */
