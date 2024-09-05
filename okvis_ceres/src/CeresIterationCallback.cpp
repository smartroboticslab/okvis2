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
 * @file CeresIterationCallback.cpp
 * @brief Source file for the CeresIterationCallback class.
 * @author Stefan Leutenegger
 */

#include <okvis/ceres/CeresIterationCallback.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

CeresIterationCallback::CeresIterationCallback(double timeLimit, int iterationMinimum)
  : timeLimit_(timeLimit),
    iterationMinimum_(iterationMinimum) {
}

::ceres::CallbackReturnType CeresIterationCallback::operator()(
    const ::ceres::IterationSummary &summary) {
  // assume next iteration takes the same time as current iteration
  if (summary.iteration >= iterationMinimum_
      && summary.cumulative_time_in_seconds
      + summary.iteration_time_in_seconds > timeLimit_) {
    return ::ceres::SOLVER_TERMINATE_SUCCESSFULLY;
  }
  return ::ceres::SOLVER_CONTINUE;
}

void CeresIterationCallback::setTimeLimit(double timeLimit) {
  timeLimit_ = timeLimit;
}

void CeresIterationCallback::setMinimumIterations(int iterationMinimum) {
  iterationMinimum_ = iterationMinimum;
}


}  // namespace ceres
}  // namespace okvis

