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
 * @file ReprojectionErrorBase.hpp
 * @brief Header file for the ReprojectionErrorBase class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_REPROJECTIONERRORBASE_HPP_
#define INCLUDE_OKVIS_CERES_REPROJECTIONERRORBASE_HPP_

#include <ceres/sized_cost_function.h>

#include <okvis/ceres/ErrorInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief Reprojection error base class.
template<int ...T>
class ReprojectionErrorBase_ :
    public ::ceres::SizedCostFunction<T...>,
    public ErrorInterface {
 public:

  /// \brief The base type.
  typedef ::ceres::SizedCostFunction<T...> ceres_base_t;

  /// \brief Camera ID.
  uint64_t cameraId() const {
    return cameraId_;
  }

  /// \brief Set camera ID.
  /// @param[in] cameraId ID of the camera.
  void setCameraId(uint64_t cameraId) {
    cameraId_ = cameraId;
  }

  uint64_t cameraId_; ///< ID of the camera.
};

/// \brief 2D keypoint reprojection error base class.
template<int ...T>
class ReprojectionError2dBase_ : public ReprojectionErrorBase_<T...> {
 public:

  /// \brief The base type.
  typedef ReprojectionErrorBase_<T...> base_t;

  /// \brief The ceres base type.
  typedef typename ReprojectionErrorBase_<T...>::ceres_base_t ceres_base_t;

  /// \brief Measurement type (2D).
  typedef Eigen::Vector2d measurement_t;

  /// \brief Covariance / information matrix type (2x2).
  typedef Eigen::Matrix2d covariance_t;

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement vector (2d).
  virtual void setMeasurement(const measurement_t& measurement) = 0;

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix (2x2).
  virtual void setInformation(const covariance_t& information) = 0;

  // getters
  /// \brief Get the measurement.
  /// \return The measurement vector (2d).
  virtual const measurement_t& measurement() const = 0;

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix (2x2).
  virtual const covariance_t& information() const = 0;

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  virtual const covariance_t& covariance() const = 0;

  /// \brief Get a clone of this error term.
  /// \return The clone.
  virtual std::shared_ptr<ReprojectionError2dBase_<T...>> clone() const = 0;

};

/// \brief The reprojection error base type.
typedef ReprojectionErrorBase_<
2 /* number of residuals */,
7 /* size of first parameter */,
4 /* size of second parameter */,
7 /* size of third parameter (camera extrinsics) */>
ReprojectionErrorBase;

/// \brief The 2D reprojection error base type.
typedef ReprojectionError2dBase_<
2 /* number of residuals */,
7 /* size of first parameter */,
4 /* size of second parameter */,
7 /* size of third parameter (camera extrinsics) */>
ReprojectionError2dBase;

}

}

#endif /* INCLUDE_OKVIS_CERES_REPROJECTIONERRORBASE_HPP_ */
