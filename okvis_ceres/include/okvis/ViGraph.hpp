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
 * @file okvis/ViGraph.hpp
 * @brief Header file for the ViGraph2 class. This does all the backend work.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_VIGRAPH2_HPP_
#define INCLUDE_OKVIS_VIGRAPH2_HPP_

#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ceres/cost_function.h>
#include <ceres/crs_matrix.h>
#include <ceres/evaluation_callback.h>
#include <ceres/iteration_callback.h>
#include <ceres/loss_function.h>
#include <ceres/manifold.h>
#include <ceres/ordered_groups.h>
#include <ceres/problem.h>
#include <ceres/product_manifold.h>
#include <ceres/sized_cost_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <ceres/version.h>
#pragma GCC diagnostic pop

#include <okvis/kinematics/Transformation.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/Measurements.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/RelativePoseError.hpp>
#include <okvis/ceres/TwoPoseGraphError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/CeresIterationCallback.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief A class to construct visual-inertial optimisable graphs with.
class ViGraph
{
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class ViSlamBackend;
  friend class Component;

  /**
   * @brief The constructor.
   */
  ViGraph();

  /**
   * @brief The destructor (does nothing).
   */
  ~ViGraph() {}

  /**
   * @brief Add a camera to the configuration. Sensors can only be added and never removed.
   * @param cameraParameters The parameters that tell how to estimate extrinsics.
   * @return Index of new camera.
   */
  int addCamera(const okvis::CameraParameters & cameraParameters);

  /**
   * @brief Add an IMU to the configuration.
   * @warning Currently there is only one IMU supported.
   * @param imuParameters The IMU parameters.
   * @return index of IMU.
   */
  int addImu(const okvis::ImuParameters & imuParameters);

  // add states
  /**
   * @brief Add a new state and initialise position to zero translation and orientation with IMU.
   * @param timestamp The state's corresponding timestamp.
   * @param imuMeasurements IMU measurements to initialise orientation.
   * @param nCameraSystem The N-Camera system.
   * @return The state ID of the created state.
   */
  StateId addStatesInitialise(const Time& timestamp, const ImuMeasurementDeque & imuMeasurements,
                              const cameras::NCameraSystem & nCameraSystem);
  /**
   * @brief Add a new state by propagation of IMU.
   * @param timestamp The state's corresponding timestamp.
   * @param imuMeasurements IMU measurements to be used for propagation.
   * @param isKeyframe Add state as keyframe?
   * @return The state ID of the created state.
   */
  StateId addStatesPropagate(const Time& timestamp, const ImuMeasurementDeque & imuMeasurements,
                             bool isKeyframe);
  /**
   * @brief Add a new state from an other ViGraph object.
   * @param stateId The state ID to be used in other (and which will be created).
   * @param other The other graph.
   * otherwise the same parameter block pointer will be used (aliased).
   * @return True on success.
   */
  bool addStatesFromOther(StateId stateId, const ViGraph & other);

  // add/remove landmarks
  /**
   * @brief Add a new landmark with a defined ID.
   * @param landmarkId The landmark ID to be used.
   * @param homogeneousPoint The point position in World frame.
   * @param initialised Defines the landmark initialisation status (depth known or not).
   * @return True on success.
   */
  bool addLandmark(LandmarkId landmarkId, const Eigen::Vector4d &homogeneousPoint, bool initialised);
  /**
   * @brief Add a new landmark and get a new ID.
   * @param homogeneousPoint The point position in World frame.
   * @param initialised Defines the landmark initialisation status (depth known or not).
   * @return The ID of the newly created landmark.
   */
  LandmarkId addLandmark(const Eigen::Vector4d &homogeneousPoint, bool initialised);
  /**
   * @brief Remove landmark and get a new ID.
   * @param landmarkId The ID of the landmark to be removed.
   * @return True on successful removal.
   */
  bool removeLandmark(LandmarkId landmarkId);
  /**
   * @brief Set landmark initialisation.
   * @param landmarkId The ID of the landmark to be set.
   * @param initialised The initialisation status.
   * @return True on success.
   */
  bool setLandmarkInitialised(LandmarkId landmarkId, bool initialised);
  /**
   * @brief Set landmark quality.
   * @param landmarkId The ID of the landmark to be set.
   * @param quality The Landmark quality.
   * @return True on success.
   */
  bool setLandmarkQuality(LandmarkId landmarkId, double quality) {
    //OKVIS_CHECK_MAP(landmarks_,landmarkId);
    landmarks_.at(landmarkId).quality = quality;
    return true;
  }
  /**
   * @brief Get landmark initialisation.
   * @param landmarkId The ID of the landmark.
   * @return The initialisation status.
   */
  bool isLandmarkInitialised(LandmarkId landmarkId) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param landmarkId The ID of the landmark.
   * @return The landmark position estimate (in World frame).
   */
  bool isLandmarkAdded(LandmarkId landmarkId) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param id The ID of the landmark.
   * @return The landmark position estimate (in World frame).
   */
  const Eigen::Vector4d & landmark(LandmarkId id) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param landmarkId The ID of the landmark.
   * @param mapPoint The landmark returned.
   * @return True on success.
   */
  bool getLandmark(LandmarkId landmarkId, okvis::MapPoint2& mapPoint) const;
  /**
   * @brief Get a copy of all the landmarks as a PointMap.
   * @param[out] landmarks The landmarks.
   * @return number of landmarks.
   */
  size_t getLandmarks(MapPoints & landmarks) const;
  /**
  * @brief Does the landmark exist?
  * @param landmarkId The ID of the landmark.
  * @return Whether the landmark exists.
  */
  bool landmarkExists(LandmarkId landmarkId) const;
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param id The ID of the landmark.
   * @param homogeneousPoint The landmark position estimate (in World frame).
   * @param initialised Whether the landmark is initialised.
   * @return True on success.
   */
  bool setLandmark(LandmarkId id, const Eigen::Vector4d & homogeneousPoint, bool initialised);
  /**
   * @brief Get landmark position estimate (in World frame).
   * @param id The ID of the landmark.
   * @param homogeneousPoint The landmark position estimate (in World frame).
   * @return True on success.
   */
  bool setLandmark(LandmarkId id, const Eigen::Vector4d & homogeneousPoint);

  // add/remove observations
  /**
   * @brief Add observation.
   * @tparam GEOMETRY_TYPE Camera geometry type to use.
   * @param multiFrame The multiFrame containing the keypoint measurements to choose from.
   * @param landmarkId The ID of the landmark.
   * @param keypointId The keypoint ID {Multiframe ID, Camera index, Keypoint index}.
   * @param useCauchy Whether to use Cauchy robustification.
   * @return True on success.
   */
  template<class GEOMETRY_TYPE>
  bool addObservation(const MultiFrame& multiFrame, LandmarkId landmarkId,
                      KeypointIdentifier keypointId, bool useCauchy = true) {
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark not added")

    // avoid double observations
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.at(landmarkId).observations.count(keypointId) == 0,
                          "observation already exists")
    OKVIS_ASSERT_TRUE_DBG(Exception, observations_.count(keypointId) == 0,
                          "observation already exists")
    OKVIS_ASSERT_TRUE_DBG(Exception, multiFrame.landmarkId(
                            keypointId.cameraIndex, keypointId.keypointIndex) == landmarkId.value(),
                          "observation already exists")

    // get the keypoint measurement
    Eigen::Vector2d measurement;
    multiFrame.getKeypoint(keypointId.cameraIndex, keypointId.keypointIndex, measurement);
    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    double size = 1.0;
    multiFrame.getKeypointSize(keypointId.cameraIndex, keypointId.keypointIndex, size);
    information *= 64.0 / (size * size);

    // create error term
    Observation observation;
    observation.errorTerm.reset(new ceres::ReprojectionError<GEOMETRY_TYPE>(
                    multiFrame.template geometryAs<GEOMETRY_TYPE>(keypointId.cameraIndex),
                    keypointId.cameraIndex, measurement, information));

    State& state = states_.at(StateId(keypointId.frameId));
    observation.residualBlockId = problem_->AddResidualBlock(
        observation.errorTerm.get(),
        useCauchy&&cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : nullptr,
        state.pose->parameters(), landmarks_.at(landmarkId).hPoint->parameters(),
        state.extrinsics.at(keypointId.cameraIndex)->parameters());
    observation.landmarkId = landmarkId;

    // remember everywhere
    observations_[keypointId] = observation;
    landmarks_.at(landmarkId).observations[keypointId] = observation;
    state.observations[keypointId] = observation;

    // covisibilities invalid
    covisibilitiesComputed_ = false;

    return true;
  }
  /**
   * @brief Add observation from a reprojection error term.
   * @param reprojectionError The external reprojection error.
   * @param landmarkId The ID of the landmark.
   * @param keypointId The keypoint ID {Multiframe ID, Camera index, Keypoint index}.
   * @param useCauchy Whether to use Cauchy robustification.
   * @return True on success.
   */
  bool addExternalObservation(
      const std::shared_ptr<const ceres::ReprojectionError2dBase> & reprojectionError,
       LandmarkId landmarkId, KeypointIdentifier keypointId, bool useCauchy = true) {
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.count(landmarkId), "landmark not added")

    // avoid double observations
    OKVIS_ASSERT_TRUE_DBG(Exception, landmarks_.at(landmarkId).observations.count(keypointId) == 0,
                          "observation already exists")
    OKVIS_ASSERT_TRUE_DBG(Exception, observations_.count(keypointId) == 0,
                          "observation already exists")

    // create error term
    Observation observation;
    observation.errorTerm = reprojectionError->clone();

    State& state = states_.at(StateId(keypointId.frameId));
    observation.residualBlockId = problem_->AddResidualBlock(
        observation.errorTerm.get(),
        useCauchy&&cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : nullptr,
        state.pose->parameters(), landmarks_.at(landmarkId).hPoint->parameters(),
        state.extrinsics.at(keypointId.cameraIndex)->parameters());
    observation.landmarkId = landmarkId;

    // remember everywhere
    observations_[keypointId] = observation;
    landmarks_.at(landmarkId).observations[keypointId] = observation;
    state.observations[keypointId] = observation;

    // covisibilities invalid
    covisibilitiesComputed_ = false;

    return true;
  }
  /**
   * @brief Remove observation.
   * @param keypointId The keypoint ID {Multiframe ID, Camera index, Keypoint index}.
   * @return True on success.
   */
  bool removeObservation(KeypointIdentifier keypointId);

  /// \brief Computes the co-visibilities of all observed frames.
  /// \return True on success.
  bool computeCovisibilities();

  /// \brief Get the convisibilities between two frames.
  /// \note Need to call computeCovisiblities before!
  /// \param pose_i The one state.
  /// \param pose_j The other state.
  /// \return Number of covisible landmarks.
  int covisibilities(StateId pose_i, StateId pose_j) const;

  /// \brief Add a relative pose error between two poses in the graph.
  /// \param poseId0 ID of one pose.
  /// \param poseId1 ID of the other pose.
  /// \param T_S0S1 Relative transform (measurement).
  /// \param information Associated information matrix.
  /// \return True on success.
  bool addRelativePoseConstraint(StateId poseId0, StateId poseId1,
                                 const kinematics::Transformation &T_S0S1,
                                 const Eigen::Matrix<double, 6, 6>& information);
  /// \brief Remove a relative pose error between two poses in the graph.
  /// \param poseId0 ID of one pose.
  /// \param poseId1 ID of the other pose.
  /// \return True on success.
  bool removeRelativePoseConstraint(StateId poseId0, StateId poseId1);

  // get/set states
  /**
   * @brief Get the pose (T_WS).
   * @param id The state ID from which go get the pose.
   * @return The pose (T_WS).
   */
  const kinematics::TransformationCacheless & pose(StateId id) const;
  /**
   * @brief Get the speed/biases [v_W, b_g, b_a].
   * @param id The state ID from which go get the speed/biases.
   * @return The pose [v_W, b_g, b_a].
   */
  const SpeedAndBias & speedAndBias(StateId id) const;
  /**
   * @brief Get the extrinsics pose (T_SC).
   * @param id The state ID from which go get the extrinsics.
   * @param camIdx The camera index of the extrinsics.
   * @return The extrinsics pose (T_SC).
   */
  const kinematics::TransformationCacheless & extrinsics(StateId id, uchar camIdx) const;

  /// \brief Is it a keyframe?
  /// \brief id The state ID in question.
  /// \return True if it is.
  bool isKeyframe(StateId id) const;

  /// \brief Get the timestamp.
  /// \brief id The state ID in question.
  /// \return The timestamp.
  Time timestamp(StateId id) const;

  /// \brief Find the most recent frame added.
  /// \return ID of most recent frame.
  StateId currentStateId() const;

  /// \brief Find an older frame added.
  /// \param age How far back to go.
  /// \return ID of most recent frame.
  StateId stateIdByAge(size_t age) const;

  /// \brief Make a frame a keyframe.
  /// \warning This can only be done for IMU frames.
  /// \param id The state ID in question.
  /// \param isKeyframe Whether or not it should be a keyframe.
  /// \return True on success.
  bool setKeyframe(StateId id, bool isKeyframe);

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
      StateId id, uchar camIdx, const kinematics::TransformationCacheless & extrinsics) const;

  /// \brief Sets all extrinsics to be optimised.
  /// \return True on success.
  bool setExtrinsicsVariable();

  /// \brief Gives all extrinsice a pose prior.
  /// \param posStd Position uncertainty standard deviation.
  /// \param rotStd Orientation uncertainty standard deviation.
  /// \return True on success.
  bool softConstrainExtrinsics(double posStd, double rotStd);

  // get/set ceres stuff
  /**
   * @brief Get the ceres optimisation options.
   * @return The ceres optimisation options.
   */
  const ::ceres::Solver::Options & options() const { return options_; }
  /**
   * @brief Get the ceres optimisation options (modifiable).
   * @return The ceres optimisation options (modifiable).
   */
  ::ceres::Solver::Options & options() { return options_; }
  /**
   * @brief Get the ceres optimisation summary.
   * @return The ceres optimisation summary.
   */
  const ::ceres::Solver::Summary & summary() const { return summary_; }

  /**
   * @brief Solve the optimisation problem.
   * @param[in] maxIterations Maximum number of iterations.
   * @param[in] numThreads Number of threads.
   * @param[in] verbose Print out optimisation progress and result, if true.
   */
  void optimise(int maxIterations, int numThreads, bool verbose);

  /// \brief Set a limit for realtime-ish operation.
  /// \param timeLimit Maximum time allowed [s].
  /// \param minIterations Minimum iterations to be carried out irrespective of time limit.
  /// \return True on success.
  bool setOptimisationTimeLimit(double timeLimit, int minIterations);

  /// \brief Removes landmarks that are not observed.
  /// \return The number of landmarks removed.
  int cleanUnobservedLandmarks(
      std::map<LandmarkId, std::set<KeypointIdentifier>> *removed = nullptr);

  /// \brief Update landmark quality and initialisation status using current graph/estimates.
  void updateLandmarks();

protected:

  /// \brief Check observation consistency.
  void checkObservations() const;

  /// \brief Helper struct to store specific edges.
  template<typename ErrorTermT>
  struct GraphEdge {
    ::ceres::ResidualBlockId residualBlockId = nullptr; ///< Ceres residual pointer.
    std::shared_ptr<ErrorTermT> errorTerm; ///< Error term.
  };

  /// \brief Helper struct for generic binary graph edges between poses.
  template<typename ErrorTermT>
  struct TwoStateGraphEdge : public GraphEdge<ErrorTermT>{
    StateId state0; ///< Reference state ID.
    StateId state1; ///< Other state ID.
  };

  /// \brief Helper struct for the reprojection error graph edges.
  struct Observation : public GraphEdge<ceres::ReprojectionError2dBase> {
    LandmarkId landmarkId; ///< Landmark ID.
  };

  /// \brief Helper struct for the IMU error graph edges.
  using ImuLink = GraphEdge<ceres::ImuErrorBase>;

  /// \brief Helper struct for the extrinsics pose change binary edges.
  using ExtrinsicsLink = GraphEdge<ceres::RelativePoseError>;

  /// \brief Helper struct for pose error unary edges.
  using PosePrior = GraphEdge<ceres::PoseError>;

  /// \brief Helper struct for speed and bias error unary edges.
  using SpeedAndBiasPrior = GraphEdge<ceres::SpeedAndBiasError>;

  /// \brief Helper struct for extrinsics pose error unary edges.
  using ExtrinsicsPrior = GraphEdge<ceres::PoseError>;

  /// \brief Binary pose graph edge.
  using TwoPoseLink = TwoStateGraphEdge<ceres::TwoPoseGraphError>;

  /// \brief Binary pose graph edge (const version, i.e. not convertible to/from observations).
  using TwoPoseConstLink = TwoStateGraphEdge<ceres::TwoPoseGraphErrorConst>;

  /// \brief Relative pose graph edge.
  using RelativePoseLink = TwoStateGraphEdge<ceres::RelativePoseError>;

  /// \brief Extended state info (including graph edges)
  struct State {
    // these are holding the estimates underneath
    std::shared_ptr<ceres::PoseParameterBlock> pose; ///< Pose parameter block.
    std::shared_ptr<ceres::SpeedAndBiasParameterBlock> speedAndBias; ///< Speed/bias param. block.
    std::vector<std::shared_ptr<ceres::PoseParameterBlock>> extrinsics; ///< Extinsics param. block.

    // error terms
    std::map<KeypointIdentifier, Observation> observations; ///< All observations per keypoint.
    ImuLink nextImuLink; ///< IMU link to next state.
    ImuLink previousImuLink; ///< IMU link to next previous.
    std::vector<ExtrinsicsLink> nextExtrinsicsLink; ///< Link to next extrinsics.
    std::vector<ExtrinsicsLink> previousExtrinsicsLink; ///< Link to previous extrinsics.
    PosePrior posePrior; ///< Pose prior.
    SpeedAndBiasPrior speedAndBiasPrior; ///< Speed/bias prior.
    std::vector<ExtrinsicsPrior> extrinsicsPriors; ///< Extrinsics prior.
    std::map<StateId, TwoPoseLink> twoPoseLinks; ///< All pose graph edges.
    std::map<StateId, TwoPoseConstLink> twoPoseConstLinks; ///< All pose graph edges (const).
    std::map<StateId, RelativePoseLink> relativePoseLinks; ///< All relative pose graph edges.

    // attributes
    bool isKeyframe = false; ///< Is it a keyframe?
    okvis::Time timestamp = okvis::Time(0.0); ///< The timestamp.
  };

  /// \brief Landmark helper struct.
  struct Landmark {
    std::shared_ptr<ceres::HomogeneousPointParameterBlock> hPoint; ///< Point in world coordinates.
    std::map<KeypointIdentifier, Observation> observations; ///< All observations of it.
    double quality = 0.0; ///< 3D quality.
    int classification = -1; ///< It's classification (if used / classified by the CNN already).
  };


  // parameters
  std::vector<okvis::CameraParameters,
      Eigen::aligned_allocator<okvis::CameraParameters> > cameraParametersVec_; ///< Extrinsics.
  std::vector<okvis::ImuParameters,
      Eigen::aligned_allocator<okvis::ImuParameters> > imuParametersVec_; ///< IMU parameters.

  // this stores the elements of the graph (note the redundancy for spee in the states)
  std::map<StateId, State> states_; ///< Store all states.
  std::map<LandmarkId, Landmark> landmarks_; ///< Contains all current landmarks.
  std::map<KeypointIdentifier, Observation> observations_; ///< Contains all observations.

  /// \brief Store parameterisation locally.
  okvis::ceres::HomogeneousPointManifold homogeneousPointManifold_;

  /// \brief Store parameterisation locally.
  okvis::ceres::PoseManifold poseManifold_;

  /// \brief The ceres problem
  std::shared_ptr< ::ceres::Problem> problem_;

  /// \brief Ceres options
  ::ceres::Solver::Options options_;

  /// \brief Ceres optimization summary
  ::ceres::Solver::Summary summary_;

  // loss function for reprojection errors
  std::shared_ptr< ::ceres::LossFunction> cauchyLossFunctionPtr_; ///< Cauchy loss.
  std::shared_ptr< ::ceres::LossFunction> huberLossFunctionPtr_; ///< Huber loss.

  // ceres iteration callback object
  std::unique_ptr<ceres::CeresIterationCallback> ceresCallback_; ///< If callback registered, store.

  std::map<uint64_t, std::map<uint64_t, int>> coObservationCounts_; ///< Covisibilities cached.

  /// \brief If computeCovisibilities was called.
  /// Init with true since no observations in the beginning.
  bool covisibilitiesComputed_ = true;
  std::set<StateId> visibleFrames_; ///< Currently visible frames.

  /// \brief Helper struct for any state (i.e. keyframe or not).
  struct AnyState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StateId keyframeId; ///< Reference keyframe.
    Time timestamp; ///< The time.
    kinematics::Transformation T_Sk_S; ///< Transformation to keyframe.
    Eigen::Vector3d v_Sk; ///< Speed in keyframe coordinates
  };
  AlignedMap<StateId, AnyState> anyState_; ///< All states (including non-keyframes).
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VIGRAPH_HPP_ */
