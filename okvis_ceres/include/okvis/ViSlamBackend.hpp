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
 * @file okvis/ViSlamBackend.hpp
 * @brief Header file for the Estimator class. This does all the backend work.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_VISLAMBACKEND_HPP_
#define INCLUDE_OKVIS_VISLAMBACKEND_HPP_

#include <atomic>

#include <okvis/ViGraphEstimator.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

//! The estimator class
/*!
 The estimator class. This does all the backend work.
 Frames:
 W: World
 B: Body
 C: Camera
 S: Sensor (IMU)
 */
class ViSlamBackend //: public VioBackendInterface
{
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief The default constructor.
   */
  ViSlamBackend() {
    needsFullGraphOptimisation_ = false;
    isLoopClosing_ = false;
    isLoopClosureAvailable_ = false;
    components_.resize(1);
  }

  /**
   * @brief Destructor, does nothing.
   */
  virtual ~ViSlamBackend() {}

  /// @name Sensor configuration related
  ///@{
  /**
   * @brief Add a camera to the configuration. Sensors can only be added and never removed.
   * @param cameraParameters The parameters that tell how to estimate extrinsics.
   * @return Index of new camera.
   */
  int addCamera(
      const okvis::CameraParameters & cameraParameters);

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imuParameters The IMU parameters.
   * @return index of IMU.
   */
  int addImu(const okvis::ImuParameters & imuParameters);

  /**
   * @brief Add a pose to the state.
   * @param multiFrame Matched multiFrame.
   * @param imuMeasurements IMU measurements from last state to new one.
   * @param asKeyframe Is this new frame a keyframe?
   * @return True if successful.
   */
  bool addStates(okvis::MultiFramePtr multiFrame,
                 const okvis::ImuMeasurementDeque & imuMeasurements,
                 bool asKeyframe);

  /**
   * @brief Prints state information to buffer.
   * @param stateId The pose Id for which to print.
   * @param buffer The puffer to print into.
   */
  void printStates(StateId stateId, std::ostream & buffer) const;

  /**
   * @brief Add a landmark.
   * @param landmarkId ID of the new landmark.
   * @param landmark Homogeneous coordinates of landmark in W-frame.
   * @param isInitialised Signal if 3D position initialised.
   * @return True if successful.
   */
  bool addLandmark(LandmarkId landmarkId, const Eigen::Vector4d & landmark, bool isInitialised);

  /**
   * @brief Add a new landmark and get a new ID.
   * @param homogeneousPoint The point position in World frame.
   * @param initialised Defines the landmark initialisation status (depth known or not).
   * @return The ID of the newly created landmark.
   */
  LandmarkId addLandmark(const Eigen::Vector4d &homogeneousPoint, bool initialised);

  /**
   * @brief Add a landmark.
   * @param landmarkId ID of the new landmark.
   * @param landmark Homogeneous coordinates of landmark in W-frame.
   * @param isInitialised Signal if 3D position initialised.
   * @return True if successful.
   */
  bool setLandmark(LandmarkId landmarkId, const Eigen::Vector4d & landmark, bool isInitialised);

  /**
   * @brief Set landmark classification.
   * @param landmarkId ID of the new landmark.
   * @param classification The classification (e.g. by CNN).
   * @return True if successful.
   */
  bool setLandmarkClassification(LandmarkId landmarkId, int classification);

  /**
   * @brief Is this keypoint observed?
   * @param kid The keypoint identifier.
   * @return True if it is.
   */
  bool isObserved(KeypointIdentifier kid) const {
    return realtimeGraph_.observations_.count(kid) != 0;
  }

  /**
   * @brief Add an observation to a landmark.
   * \tparam GEOMETRY_TYPE The camera geometry type for this observation.
   * @param landmarkId ID of landmark.
   * @param stateId ID of state where the landmark was observed.
   * @param camIdx ID of camera frame where the landmark was observed.
   * @param keypointIdx ID of keypoint corresponding to the landmark.
   * @param useCauchy Indicate whether to use a Cauchy robustifier.
   * @return True if scceeded.
   */
  template<class GEOMETRY_TYPE>
  bool addObservation(LandmarkId landmarkId, StateId stateId,
                      size_t camIdx, size_t keypointIdx, bool useCauchy = true) {
    KeypointIdentifier kid{stateId.value(), camIdx, keypointIdx};
    OKVIS_ASSERT_TRUE_DBG(Exception, multiFrames_.count(stateId), "frame does noe exist")

    MultiFramePtr multiFrame = multiFrames_.at(stateId);
    const bool success = realtimeGraph_.addObservation<GEOMETRY_TYPE>(
          *multiFrame, landmarkId, kid, useCauchy);
    OKVIS_ASSERT_TRUE_DBG(Exception, !auxiliaryStates_.at(stateId).isPoseGraphFrame,
                      "not allowed to have observations in pose graph frames!")
    if(isLoopClosing_ || isLoopClosureAvailable_) {
      touchedStates_.insert(stateId);
      touchedLandmarks_.insert(landmarkId);
    } else {
      fullGraph_.addObservation<GEOMETRY_TYPE>(*multiFrame, landmarkId, kid, useCauchy);
    }
    return success;
  }
  /**
   * @brief Remove an observation from a landmark, if available.
   * @param stateId ID of state where the landmark was observed.
   * @param camIdx ID of camera frame where the landmark was observed.
   * @param keypointIdx ID of keypoint corresponding to the landmark.
   * @return True if observation was present and successfully removed.
   */
  bool removeObservation(StateId stateId,  size_t camIdx,
                         size_t keypointIdx);

  /// \brief Set the information of an observation.
  /// @param stateId ID of state where the landmark was observed.
  /// @param camIdx ID of camera frame where the landmark was observed.
  /// @param keypointIdx ID of keypoint corresponding to the landmark.
  /// \param information Information of the observation.
  /// \return True on success.
  bool setObservationInformation(StateId stateId, size_t camIdx,
                                 size_t keypointIdx, const Eigen::Matrix2d & information);

  /// \brief Set the detector uniformity radius (for overlap computations).
  /// \param uniformityRadius Detector uniformity radius.
  void setDetectorUniformityRadius(double uniformityRadius) {
    kptradius_ = 0.09 * uniformityRadius / 36.0;
  }

  /**
   * @brief Applies the graph edge creation / state fixation / observation dropping & IMU-merging.
   *        The new number of frames in the window will be numKeyframes+numImuFrames.
   * @param numKeyframes Number of keyframes.
   * @param numLoopClosureFrames Number of loopclosure frames to keep around.
   * @param numImuFrames Number of frames in IMU window.
   * @param affectedFrames Frames with added or removed pose graph edges.
   * @param expand Whether or not to allow the strategy to branch out to include new keyframes.
   * @return True if successful.
   */
  bool applyStrategy(size_t numKeyframes, size_t numLoopClosureFrames, size_t numImuFrames,
                     std::set<StateId> & affectedFrames,
                     bool expand = true);

  /**
   * @brief Start ceres optimisation of the realtime problem.
   * @param[in] numIter Maximum number of iterations.
   * @param[out] updatedStates The state IDs of all the updated states.
   * @param[in] numThreads Number of threads.
   * @param[in] verbose Print out optimization progress and result, if true.
   * @param[in] onlyNewestState Whether to only optimise the newest states (landmarks also fixed).
   * @param[in] isInitialised If false, will soft-constrain the position to the on of State ID 1.
   */
  void optimiseRealtimeGraph(
      int numIter, std::vector<StateId>& updatedStates,
      int numThreads = 1, bool verbose = false, bool onlyNewestState = false, bool isInitialised = true);

  /**
   * @brief Set a time limit for the realtime problem optimisation process.
   * @param[in] timeLimit Time limit in seconds. If timeLimit < 0 the time limit is removed.
   * @param[in] minIterations minimum iterations the optimisation process should do
   *            disregarding the time limit.
   * @return True if successful.
   */
  bool setOptimisationTimeLimit(double timeLimit, int minIterations);

  /**
   * @brief Checks whether the landmark is added to the estimator.
   * @param landmarkId The ID.
   * @return True if added.
   */
  bool isLandmarkAdded(LandmarkId landmarkId) const;

  /**
   * @brief Checks whether the landmark is initialised.
   * @param landmarkId The ID.
   * @return True if initialised.
   */
  bool isLandmarkInitialised(LandmarkId landmarkId) const;

  /// @name Getters
  ///\{
  /**
   * @brief Get a specific landmark.
   * @param[in]  landmarkId ID of desired landmark.
   * @param[out] mapPoint Landmark information, such as quality, coordinates etc.
   * @return True if successful.
   */
  bool getLandmark(LandmarkId landmarkId, okvis::MapPoint2& mapPoint) const;

  /**
   * @brief Get a copy of all the landmarks as a PointMap.
   * @param[out] landmarks The landmarks.
   * @return number of landmarks.
   */
  size_t getLandmarks(MapPoints &landmarks) const;

  /**
   * @brief Get a multiframe.
   * @param stateId ID of desired multiframe.
   * @return Shared pointer to multiframe.
   */
  okvis::MultiFramePtr multiFrame(StateId stateId) const {
    if(!multiFrames_.count(stateId)) {
      return nullptr;
    }
    return multiFrames_.at(stateId);
  }

  // get/set states
  /**
   * @brief Get the pose (T_WS).
   * @param id The state ID from which go get the pose.
   * @return The pose (T_WS).
   */
  const kinematics::TransformationCacheless & pose(StateId id) const {
    return realtimeGraph_.pose(id);
  }
  /**
   * @brief Get the speed/biases [v_W, b_g, b_a].
   * @param id The state ID from which go get the speed/biases.
   * @return The pose [v_W, b_g, b_a].
   */
  const SpeedAndBias & speedAndBias(StateId id) const {
    return realtimeGraph_.speedAndBias(id);
  }
  /**
   * @brief Get the extrinsics pose (T_SC).
   * @param id The state ID from which go get the extrinsics.
   * @param camIdx The camera index of the extrinsics.
   * @return The extrinsics pose (T_SC).
   */
  const kinematics::TransformationCacheless & extrinsics(StateId id, uchar camIdx) const {
    return realtimeGraph_.extrinsics(id, camIdx);
  }

  /**
   * @brief Is a state a keyframe?
   * @param id The state ID in question.
   * @return True if it is.
   */
  bool isKeyframe(StateId id) const {
    return realtimeGraph_.isKeyframe(id);
  }

  /**
   * @brief Is a state a pose graph frame (i.e. without any observations)?
   * @param id The state ID in question.
   * @return True if it is.
   */
  bool isPoseGraphFrame(StateId id) const {
    //OKVIS_CHECK_MAP(auxiliaryStates_,id);
    return auxiliaryStates_.at(id).isPoseGraphFrame;
  }

  /**
   * @brief Is a state considered for place recognition?
   * @param id The state ID in question.
   * @return True if it is.
   */
  bool isPlaceRecognitionFrame(StateId id) const {
    //OKVIS_CHECK_MAP(auxiliaryStates_,id);
    return auxiliaryStates_.at(id).isPlaceRecognitionFrame;
  }

  /**
   * @brief The tracking quality w.r.t. the map:
   * fraction of the image pixels covered with matches.
   * @param id The state ID in question.
   * @return The quality.
   */
  double trackingQuality(StateId id) const;

  /**
   * @brief Set if keyframe or not.
   * @warning Only applicable to IMU frames.
   * @param id The state ID in question.
   * @param isKeyframe Whether or not to set it to be a keyframe.
   * @return True on success.
   */
  bool setKeyframe(StateId id, bool isKeyframe);

  /**
   * @brief Has this state closed a loop / recognised a place?
   * @param id The state ID in question.
   * @return True if it has.
   */
  bool closedLoop(StateId id) const {
    //OKVIS_CHECK_MAP(auxiliaryStates_,id);
    return auxiliaryStates_.at(id).closedLoop;
  }

  /// \brief Get the recent loop closure frames.
  /// \return The recent loop closure frames.
  const std::set<StateId> & recentLoopClosureFrames(StateId id) const {
    //OKVIS_CHECK_MAP(auxiliaryStates_,id);
    return auxiliaryStates_.at(id).recentLoopClosureFrames;
  }

  /// \brief The ID of the most recent (current) frame.
  /// \return The ID.
  StateId currentStateId() const {
    return realtimeGraph_.currentStateId();
  }

  /// \brief The ID of the current keyframe, i.e. the one with most overlapping matches.
  /// \return The ID.
  StateId currentKeyframeStateId(bool considerLoopClosureFrames = true) const;

  /// \brief The ID of the most overlapping frame.
  /// \param frame Frame in question.
  /// \param considerLoopClosureFrames Should loop closure frames also be considered?
  /// \return The ID.
  StateId mostOverlappedStateId(StateId frame, bool considerLoopClosureFrames = true) const;

  /// \brief The ID of the loopclosure frame with currently most overlapping matches.
  /// \return The ID.
  StateId currentLoopclosureStateId() const;

  /// \brief The ID of a state by age (from current/newest frame).
  /// \param age The age.
  /// \return The ID.
  StateId stateIdByAge(size_t age) const {
    return realtimeGraph_.stateIdByAge(age);
  }

  /**
   * @brief Set the pose (T_WS).
   * @param id The state ID for which go set the pose.
   * @param pose The pose (T_WS).
   * @return True on success.
   */
  bool setPose(StateId id, const kinematics::TransformationCacheless & pose);
  /**
   * @brief Set the speed/biases [v_W, b_g, b_a].
   * @param id The state ID for which go set the speed/biases.
   * @param speedAndBias The the speed/biases [v_W, b_g, b_a].
   * @return True on success.
   */
  bool setSpeedAndBias(StateId id, const SpeedAndBias & speedAndBias);
  /**
   * @brief Set the extrinsics pose (T_SC).
   * @param id The state ID for which go set the extrinsics.
   * @param camIdx The camera index of the extrinsics.
   * @param extrinsics The extrinsics pose (T_SC).
   * @return True on success.
   */
  bool setExtrinsics(
      StateId id, uchar camIdx, const kinematics::TransformationCacheless & extrinsics);

  /// @brief Get the number of states/frames in the estimator.
  /// \return The number of frames.
  size_t numFrames() const {
    return multiFrames_.size();
  }

  ///@}

  /**
   * @brief Checks if a particular frame is still in the IMU window.
   * @param[in] id ID of frame to check.
   * @return True if the frame is in IMU window.
   */
  bool isInImuWindow(StateId id) const;

  /// @name Getters
  /// @{
  /**
   * @brief Get the timestamp for a particular frame.
   * @param[in] id ID of frame.
   * @return Timestamp of frame.
   */
  okvis::Time timestamp(StateId id) const;

  /// \brief Draws a debug overhead image/
  /// \param image The image to be drawn into.
  /// \param idx 0 for realtime, 1 for full, 2 for observation-less.
  void drawOverheadImage(cv::Mat & image, int idx=0) const;

  /**
   * @brief Get the co-observed frames for a particular frame.
   * @param[in] id ID of frame.
   * @param[out] observedIds IDs co-observed with frame id.
   * @return true on success.
   */
  bool getObservedIds(StateId id, std::set<StateId>& observedIds) const;

  /**
   * @brief Is this state an active loop-closure frame?
   * @param frameId The state ID in question.
   * @return True if it is.
   */
  bool isLoopClosureFrame(StateId frameId) const;

  /**
   * @brief Was this state recently used as a loop-closure frame?
   * @param frameId The state ID in question.
   * @return True if it is.
   */
  bool isRecentLoopClosureFrame(StateId frameId) const;

  /// \brief Removes landmarks that are not observed.
  /// \return The number of landmarks removed.
  int cleanUnobservedLandmarks();

  /// \brief Merge landmarks with two IDs into one, taking care of all the observations.
  /// \param fromIds Landmarks from ID (will be deleted).
  /// \param intoIds Landmarks to ID (will be kept).
  /// \return True on success.
  int mergeLandmarks(std::vector<LandmarkId> fromIds, std::vector<LandmarkId> intoIds);

  /// \brief Write the full optimised trajectory into a file.
  /// \param csvFileName Path to file to write.
  /// \param rpg Whether to use the RPG format (instead of EuRoC).
  /// \return True on success
  bool writeFinalCsvTrajectory(const std::string& csvFileName, bool rpg = false) const;

  /// \brief Attempt loop closure.
  /// \param pose_i The old frame that was recognised.
  /// \param pose_j The new pose.
  /// \param T_Si_Sj The relative pose between the IMU frames S.
  /// \param information The relative pose information.
  /// \param skipFullGraphOptimisation Whether to not subsequently optimise the full graph.
  /// \return True on success.
  bool attemptLoopClosure(StateId pose_i, StateId pose_j,
                          const kinematics::Transformation& T_Si_Sj,
                          const Eigen::Matrix<double, 6, 6>& information,
                          bool & skipFullGraphOptimisation);

  /// \brief Add a loopclosure frame (after successful attempt).
  /// \brief loopClosureFrameId The ID of the frame to be added.
  /// \brief loopClosureLandmarks The landmarks observed by the loopclosure frame.
  void addLoopClosureFrame(StateId loopClosureFrameId, std::set<LandmarkId> &loopClosureLandmarks,
                           bool skipFullGraphOptimisation);

  /// \brief Check if full graph needs to be optimised after addLoopClosureFrame
  /// (or during interrupted full graph optimisation).
  /// \return True if it needs optimisation.
  bool needsFullGraphOptimisation() const {return needsFullGraphOptimisation_; }

  /**
   * @brief Start ceres optimisation of the full graph problem.
   * @param[in] numIter Maximum number of iterations.
   * @param[out] summary Get optimisation summary back.
   * @param[in] numThreads Number of threads.
   * @param[in] verbose Print out optimization progress and result, if true.
   */
  void optimiseFullGraph(int numIter, ::ceres::Solver::Summary &summary,
                         int numThreads = 1, bool verbose = false);

  /// \brief Performs a final Bundle Adjustment.
  /// \param numIter Number of iterations.
  /// \param summary Ceres summary.
  /// \param updatedStatesBa States that were changed -- should be all of them, reall.
  /// \param extrinsicsPositionUncertainty Std dev. of extrinsics position. 0.0 for fixed.
  /// \param extrinsicsOrientationUncertainty Std dev. of extrinsics orientation. 0.0 for fixed.
  /// \param numThreads Number of solver threads to use.
  /// \param verbose Console outputs or not?
  void doFinalBa(int numIter, ::ceres::Solver::Summary &summary,
                 std::set<StateId> & updatedStatesBa,
                 double extrinsicsPositionUncertainty = 0.0,
                 double extrinsicsOrientationUncertainty = 0.0,
                 int numThreads = 1, bool verbose = false);

  /// \brief Save the map to CSV.
  /// \param path CSV file path.
  bool saveMap(std::string path);

  /// \brief Check if currently closing loop.
  /// \return True if it is.
  bool isLoopClosing() const { return isLoopClosing_; }

  /// \brief Check if a background loop closure has finished.
  /// \return True if it is.
  bool isLoopClosureAvailable() const { return isLoopClosureAvailable_ && !isLoopClosing_; }

  /// \brief After full graph optimisation, this synchronises to
  /// the realtime / observation-less graphs.
  /// \return True on success.
  bool synchroniseRealtimeAndFullGraph(std::vector<StateId>& updatedStates);

  /// \brief Is the state of fullGraph_ vs. realtimeGraph_ synchronised?
  /// \return True if it is.
  bool isSynched() const {
    return fullGraph_.isSynched(realtimeGraph_);
  }

  /// \brief Get the overlap fraction (of pixels) between two frames.
  /// \param frameA frame A.
  /// \param frameB frame B.
  /// \return The overlap fraction.
  double overlapFraction(const MultiFramePtr frameA,
                                const MultiFramePtr frameB) const;

  /// \brief Get the set of key frame IDs.
  /// \return The set of key frame IDs.
  const std::set<StateId>& keyFrames() const {return keyFrames_;}

  /// \brief Get the set of IMU frame IDs.
  /// \return The set of IMU frame IDs.
  const std::set<StateId>& imuFrames() const {return imuFrames_;}

  /// \brief Get the set of loop closure frame IDs.
  /// \return The set of loop closure frame IDs.
  const std::set<StateId>& loopClosureFrames() const {return loopClosureFrames_;}

  /// \brief Eliminate IMU frames that are too much (that are not keyframes).
  /// \param numImuFrames Number of IMU frames to keep.
  void eliminateImuFrames(size_t numImuFrames);

  /// \brief Clears the underlying graphs and resets everything (so you can re-start).
  void clear();

private:

  /// \brief Convert to pose graph edges via MST creation.
  /// \param framesToConvert The state IDs of those to be converted to pose graph states.
  /// \param framesToConsider The frames to consider for the operation (i.e. to do the MST over).
  /// \param affectedFrames Frames that with added pose graph edges.
  /// \return True on success.
    bool convertToPoseGraphMst(const std::set<StateId> &framesToConvert,
                               const std::set<StateId> &framesToConsider,
                               std::set<StateId> &affectedFrames);

    /// \brief Re-activate the neighbouring states (i.e. convert back to reprojection errors).
    /// \param keyframe The keyframe to consider.
    /// \return The number of frames re-activated.
    int expandKeyframe(StateId keyframe);

    /// \brief Delete place recognition frames with much overlap from database.
    /// \return Number of pruned frames.
    int prunePlaceRecognitionFrames();

    std::map<StateId, MultiFramePtr> multiFrames_; ///< All the multiframes added so far.

    /// \brief Helper struct to store information about states.
    struct AuxiliaryState
    {
      bool isKeyframe = false;             ///< Is it a keyframe?
      bool isImuFrame = false;             ///< Is it an IMU frame?
      bool isPoseGraphFrame = false;       ///< Is it a pose graph frame?
      bool closedLoop = false;             ///< Has it closed a loop?
      bool isLost = false;                 ///< Is it lost?
      bool isPlaceRecognitionFrame = true; ///< Is it in the DBoW database as place recognition frame?
      StateId loopId; ///< The ID of the loop (needed for unfreezing before full graph optimisation)/
      std::set<StateId> recentLoopClosureFrames; ///< These were recent loop closure frames.
  };
  std::map<StateId, AuxiliaryState> auxiliaryStates_; ///< Store information about states.

  /// \brief Store disconnected SLAM components (currently unused).
  struct Component0 {
    std::set<StateId> poseIds; ///< Pose IDs of the component.
    StateId referenceId; ///< Reference pose ID.
    StateId connectedReferenceId; ///< Connection to other component.
  };
  std::vector<Component0> components_; ///< All the created components.
  size_t currentComponentIdx_ = 0; ///< The index of the current component.

  std::set<StateId> loopClosureFrames_; ///< All the current loop closure frames.

  ///
  std::set<StateId> imuFrames_; ///< All the current IMU frames.
  std::set<StateId> keyFrames_; ///< All the current keyframes.
  ///

  // underlying graphs;
  ViGraphEstimator realtimeGraph_; ///< The realtime estimator.
  ViGraphEstimator fullGraph_; ///< The full grapf for asynchronous optimisation.

  std::atomic_bool needsFullGraphOptimisation_; ///< Do we need a full graph optimisation now?
  std::atomic_bool isLoopClosing_; ///< Is there currently a full graph optimisation running?
  std::atomic_bool isLoopClosureAvailable_; ///< New result from full graph optimisation available?
  std::set<StateId> currentLoopClosureFrames_; ///< The set of current loop closure frames.

  /// \brief Helper struct to store graph inconsistencies.
  struct AddStatesBacklog {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Time timestamp; ///< Timestamp.
    StateId id; ///< State ID.
    ImuMeasurementDeque imuMeasurements; ///< IMU measurements leading up to it.
  };

  AlignedVector<AddStatesBacklog> addStatesBacklog_; ///< Backlog of states to add to fullGraph_.
  std::map<StateId, StateId> eliminateStates_; ///< States eliminated in realtimeGraph_.
  std::set<StateId> touchedStates_; ///< States modified in realtimeGraph_.
  std::set<LandmarkId> touchedLandmarks_; ///< Landmarks modified in realtimeGraph_.

  std::set<StateId> updatedStatesLoopClosureAttempt_; ///< States updated in loop-closure attempt.

  /// \brief Helper struct for relative poses.
  struct RelPoseInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    kinematics::Transformation T_Si_Sj; ///< Relative Transform.
    Eigen::Matrix<double, 6, 6> information; ///< Relative uncertainty.
    StateId pose_i; ///< Reference pose ID.
    StateId pose_j; ///< Other pose ID.
  };

  AlignedVector<RelPoseInfo> fullGraphRelativePoseConstraints_; ///< Relative pose constraints.
  
  StateId lastFreeze_; ///< Store up to where the realtimeGraph_ states were fixed.

  double kptradius_ = 0.09; ///< Constant of how large keypoints should appear for overlap comp.

  public: /// \todo remove this horrible hack once these are properly being estimated!
  AlignedMap<StateId, AlignedMap<uint64_t, kinematics::Transformation>> T_AiS_; ///< Agent i to World pose (if available).
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VISLAMBACKEND_HPP_ */
