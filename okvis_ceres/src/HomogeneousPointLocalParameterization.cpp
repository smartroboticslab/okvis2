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
 * @file HomogeneousPointLocalParameterization.cpp
 * @brief Source file for the HomogeneousPointLocalParameterization class.
 * @author Stefan Leutenegger
 */

#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/assert_macros.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

bool HomogeneousPointManifold::plus(
    const double* x, const double* delta, double* x_plus_delta)
{
  Eigen::Map<const Eigen::Vector3d> delta_(delta);
  Eigen::Map<const Eigen::Vector4d> x_(x);
  Eigen::Map<Eigen::Vector4d> x_plus_delta_(x_plus_delta);

  // Euclidean style
  x_plus_delta_ = x_ + Eigen::Vector4d(delta_[0], delta_[1], delta_[2], 0);

  return true;
}
bool HomogeneousPointManifold::Plus(
    const double* x, const double* delta, double* x_plus_delta) const {
  return plus(x, delta, x_plus_delta);
}

bool HomogeneousPointManifold::plusJacobian(const double* /*x*/, double* jacobian)
{
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > Jp(jacobian);

  // Euclidean-style
  Jp.setZero();
  Jp.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

  return true;
}
bool HomogeneousPointManifold::PlusJacobian(const double* x, double* jacobian) const {
  return plusJacobian(x, jacobian);
}

bool HomogeneousPointManifold::minus(
    const double* y, const double* x, double* y_minus_x)
{

  Eigen::Map<Eigen::Vector3d> delta_(y_minus_x);
  Eigen::Map<const Eigen::Vector4d> x_(x);
  Eigen::Map<const Eigen::Vector4d> x_plus_delta_(y);

  // Euclidean style
  delta_ = (x_plus_delta_ - x_).head<3>();

  return true;
}
bool HomogeneousPointManifold::Minus(
    const double* y, const double* x, double* y_minus_x) const {
  return minus(y, x, y_minus_x);
}


bool HomogeneousPointManifold::minusJacobian(const double* /*x*/, double* jacobian)
{
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> > Jp(jacobian);

  // Euclidean-style
  Jp.setZero();
  Jp.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();

  return true;
}
bool HomogeneousPointManifold::MinusJacobian(const double* x, double* jacobian) const {
  return minusJacobian(x, jacobian);
}

}  // namespace ceres
}  // namespace okvis
