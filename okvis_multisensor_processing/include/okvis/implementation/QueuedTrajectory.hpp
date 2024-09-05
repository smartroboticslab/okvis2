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
 * @brief Source file for the QueuedTrajectory class.
 * @author Simon Schaefer
 */

#pragma once

#include <okvis/QueuedTrajectory.hpp>


template<class MEASUREMENT_T>
std::vector<std::pair<okvis::Measurement<MEASUREMENT_T>, okvis::State>>
okvis::QueuedTrajectory<MEASUREMENT_T>::getStates(okvis::Trajectory& trajectory) {
  std::vector<std::pair<MeasurementTyped, okvis::State>> states;
  while (!measurementQueue_.empty()) {
    okvis::State state;
    MeasurementTyped measurement = measurementQueue_.back();
    bool valid_transform = trajectory.getState(measurement.timeStamp, state);

    // The queue is sorted in time. If the current iteration's transform is invalid, the next
    // iterations with an even larger timestamp will be invalid, too.
    if (!valid_transform) {
      break;
    }

    // Otherwise, push the state-measurement-pair to the output vector and remove the measurement
    // from the queue.
    measurementQueue_.pop_back();
    std::pair<MeasurementTyped, okvis::State> state_measurement_pair(measurement, state);
    states.push_back(state_measurement_pair);
  }
  return states;
}

template<class MEASUREMENT_T>
void okvis::QueuedTrajectory<MEASUREMENT_T>::enqueue(
   const okvis::Measurement<MEASUREMENT_T>& measurement) {
  measurementQueue_.push_front(measurement);
  while (measurementQueue_.size() > 20) {
    LOG(WARNING) << "RGB Image queue overloaded, popping last element";
    measurementQueue_.pop_back();
  }
}

template<class MEASUREMENT_T>
void okvis::QueuedTrajectory<MEASUREMENT_T>::enqueue(
  const MEASUREMENT_T& data, const okvis::Time& timestamp) {
  MeasurementTyped measurement(timestamp, data);
  enqueue(measurement);
}
