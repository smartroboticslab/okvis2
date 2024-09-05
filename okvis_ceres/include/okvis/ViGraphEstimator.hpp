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
 * @file okvis/ViGraphEstimator.hpp
 * @brief Header file for the ViGraphEstimator class. This does all the backend work.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_VIGRAPHESTIMATOR_HPP_
#define INCLUDE_OKVIS_VIGRAPHESTIMATOR_HPP_

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

#include <opencv2/core.hpp>

#include <okvis/ViGraph.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief ViGraph with additional functionalities to build a finished estimator.
class ViGraphEstimator : public ViGraph
{
 public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class ViSlamBackend;
  friend class Component;

  /**
   * @brief The constructor, does nothing.
   */
  ViGraphEstimator() {}

  /**
   * @brief The destructor, does nothing.
   */
  ~ViGraphEstimator() {}

  /// \brief Remove all observations on a specific state.
  /// \param stateId The state ID to perform the operation on.
  /// \return True on success.
  bool removeAllObservations(StateId stateId);

  /// \brief Remove all observations and merge/append IMU errors.
  /// \param stateId The state ID to perform the operation on.
  /// \param refId the reference state ID.
  /// \return True on success.
  bool eliminateStateByImuMerge(StateId stateId, StateId refId);

  /// \brief Merge landmarks with two IDs into one, taking care of all the observations.
  /// \param fromId Landmark from ID (will be deleted).
  /// \param intoId Landmark to ID (will be kept).
  /// \param multiFrames (Such that the Landmark Id in the frame can be kept consistent.
  /// \todo: refactor this.)
  /// \return True on success.
  bool mergeLandmark(LandmarkId fromId, LandmarkId intoId,
                     std::map<StateId,MultiFramePtr> &multiFrames);

  // freeze/unfreeze
  /// \brief Fixes poses from the first (ID 1) to stateId.
  /// \param stateId The newest state to be fixed.
  /// \param removeInCeres Removes the parameterBlock in ceres -- irreversible!
  /// \return True on success.
  bool freezePosesUntil(StateId stateId, bool removeInCeres = false);

  /// \brief Makes poses variable from stateId to the newest ID.
  /// \param stateId The oldest state to be made variable.
  /// \return True on success.
  bool unfreezePosesFrom(StateId stateId);

  /// \brief Fixes speed and biases from the first (ID 1) to stateId.
  /// \param stateId The newest state to be fixed.
  /// \param removeInCeres Removes the parameterBlock in ceres -- irreversible!
  /// \return True on success.
  bool freezeSpeedAndBiasesUntil(StateId stateId, bool removeInCeres = false);

  /// \brief Makes speed and biases variable from stateId to the newest ID.
  /// \param stateId The oldest state to be made variable.
  /// \return True on success.
  bool unfreezeSpeedAndBiasesFrom(StateId stateId);

  /// \brief Fixes extrinsics from the first (ID 1) to stateId.
  /// \param stateId The newest state to be fixed.
  /// \param removeInCeres Removes the parameterBlock in ceres -- irreversible!
  /// \return True on success.
  bool freezeExtrinsicsUntil(StateId stateId, bool removeInCeres = false);

  /// \brief Makes extrinsics variable from stateId to the newest ID.
  /// \param stateId The oldest state to be made variable.
  /// \return True on success.
  bool unfreezeExtrinsicsFrom(StateId stateId);

  /// \brief Helper struct to access underlying two-pose pose graph links.
  struct PoseGraphEdge {
    std::shared_ptr<ceres::TwoPoseGraphError> poseGraphError; ///< Error term.
    StateId referenceId; ///< The reference.
    StateId otherId; ///< The other state.
  };

  /// \brief Convert the observations to a set of states into a set of binary pose graph errors
  ///        via construction of a Maximum Spanning Tree (MST).
  ///        Takes care of underlying duplication of multiple observations used in
  ///        Different links by progressively halving the observation information.
  /// \param states The states that have to become entirely observation-free.
  /// \param statesToConsider The states to be considered/linked via pose graph errors.
  /// \param createdPoseGraphEdges The links created.
  /// \param removedTwoPoseErrors The previously existing links that were removed in the process.
  /// \param removedObservations These observations were removed in the process.
  /// \return True on success.
  bool convertToPoseGraphMst(
      const std::set<StateId>& states, const std::set<StateId> &statesToConsider,
      std::vector<PoseGraphEdge>* createdPoseGraphEdges = nullptr,
      std::vector<std::pair<StateId, StateId> > *removedTwoPoseErrors = nullptr,
      std::vector<KeypointIdentifier> *removedObservations = nullptr);

  /// \brief Get the full Maximum Spanning Tree (MST) pose graph from all observations present.
  /// \note This will not change the underlying graph. \todo Rewrite to be made const.
  /// \param poseGraphEdges The returned edges.
  /// \return True on success.
  bool obtainPoseGraphMst(std::vector<PoseGraphEdge>& poseGraphEdges);

  /// \brief Add an externally known/created two-pose pose graph error.
  /// \param twoPoseError The external link.
  /// \param referenceId Reference pose.
  /// \param otherId The other pose.
  /// \return True on success.
  bool addExternalTwoPoseLink(const std::shared_ptr<ceres::TwoPoseGraphErrorConst> &twoPoseError,
                         StateId referenceId, StateId otherId);

  /// \brief Remove all two-pose pose graph error terms for a state.
  /// \param convertStateId The state from which to remove.
  /// \return True on success.
  bool removeTwoPoseConstLinks(StateId convertStateId);

  /// \brief Remove a specific two-pose pose graph error term (order won't matter).
  /// \param pose_i The one state from which to remove.
  /// \param pose_j The other state from which to remove.
  /// \return True on success.
  bool removeTwoPoseConstLink(StateId pose_i, StateId pose_j);

  /// \brief Convert a set of two-pose pose graph errors back into observations.
  ///        Also takes care of the re-merging of duplicated observations.
  /// \param convertStateId The two-pose pose graph errors of this state will be converted.
  /// \param createdLandmarks These landmarks were (re-)created in the process.
  /// \param connectedStates These states were originally connected to convertStateId.
  /// \param observations These observations were created.
  /// \return The number of observations created.
  int convertToObservations(
      StateId convertStateId,
      std::set<LandmarkId> *createdLandmarks = nullptr,
      std::set<StateId> *connectedStates = nullptr,
      std::vector<ceres::TwoPoseGraphError::Observation> *observations = nullptr);

  /// \brief Remove a specific speed and bias prior.
  /// \param stateId The one state from which to remove.
  /// \return True on success.
  bool removeSpeedAndBiasPrior(StateId stateId);

  /// \brief Clears this graph and resets everything.
  void clear();

protected:
  /// \brief Construct the MST of a set of frames.
  int buildMst(const std::set<StateId>& states, bool keyframesOnly=true);

  /// \brief Check if the graph is synched with an other one.
  /// \param other The other graph.
  /// \return True if synched.
  /// \warning slow, just for debug.
  bool isSynched(const ViGraphEstimator &other) const;

  /// \brief Check if the observationless graph is synched with an other one.
  /// \param other The other graph.
  /// \return True if synched.
  /// \warning slow, just for debug.
  bool isObservationlessSynched(const ViGraphEstimator & other) const;

  /// \brief Check if the only the states are synched with an other graph.
  /// \param other The other graph.
  /// \warning slow, just for debug.
  void checkSynchedStates(const ViGraphEstimator & other) const; // slow, just for debug

  std::vector<uint64_t> mstFrameIds_; ///< MST frame IDs.
  std::map<uint64_t, size_t> mstGraphIdxs_; ///< MST graph indices.
  std::vector<std::pair<int,int>> mstEdges_; ///< MST edges.
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_VIGRAPHESTIMATOR_HPP_ */
