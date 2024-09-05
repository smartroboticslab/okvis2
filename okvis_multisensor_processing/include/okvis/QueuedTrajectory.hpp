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
 * @file QueuedTrajectory.hpp
 * @brief Header file for the QueuedTrajectory class.
 * @author Simon Schaefer
 */

#ifndef OKVIS_QUEUEDTRAJECTORY_HPP
#define OKVIS_QUEUEDTRAJECTORY_HPP

#include <vector>
#include <deque>
#include <Eigen/Core>

#include <okvis/threadsafe/ThreadsafeQueue.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ViInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief A class to store measurements in a queue and get the according state when available.
///        The class is templated with the measurement type, e.g. cv::Mat for images.
template<class MEASUREMENT_T>
class QueuedTrajectory {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  typedef Measurement<MEASUREMENT_T> MeasurementTyped; ///< Helper Measurem. typed on MEASUREMENT_T.

  /// @brief For each measurement in the internal queue, get the corresponding state from the
  ///        trajectory based on the measurement's timestamp. If the state is available, return the
  ///        state-measurement pair and remove the measurement from the queue. Otherwise, return an
  ///        empty vector.
  /// @param trajectory The trajectory to get the states from. 
  std::vector<std::pair<MeasurementTyped, okvis::State>> getStates(okvis::Trajectory& trajectory);

  /// @brief Enqueues a new measurement to the measurement queue.
  /// @param measurement measurement to queue.
  void enqueue(const MeasurementTyped& measurement);

  /// @brief Enqueues new raw measurement data and timestamp. Therefore, first a MeasurementTyped
  ///        object is created from the timestamp and measurement data. Then, the MeasurementTyped
  ///        object is enqueued.
  /// @param data measurement data.
  /// @param timestamp measurement timestamp.
  void enqueue(const MEASUREMENT_T& data, const okvis::Time& timestamp);

private:
  /// \brief The internal measurement queue.
  std::deque<MeasurementTyped, Eigen::aligned_allocator<MeasurementTyped> > measurementQueue_;

};

#include "implementation/QueuedTrajectory.hpp"

}

#endif // OKVIS_QUEUEDTRAJECTORY_HPP
