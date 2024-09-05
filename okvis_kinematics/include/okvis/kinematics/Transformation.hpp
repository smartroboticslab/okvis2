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
 * @file kinematics/Transformation.hpp
 * @brief Header file for the TransformationT class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_TRANSFORMATION_HPP_
#define INCLUDE_OKVIS_TRANSFORMATION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief kinematics Namespace for kinematics functionality, i.e. TransformationTs and stuff.
namespace kinematics {

/// \brief Implements sin(x)/x for all x in R.
/// @param[in] x The argument of the sinc function.
/// \return The result.
double sinc(double x);

/// \brief Implements the exponential map for quaternions.
/// @param[in] dAlpha a axis*angle (minimal) input in tangent space.
/// \return The corresponding Quaternion.
Eigen::Quaterniond deltaQ(const Eigen::Vector3d& dAlpha);

/// \brief Right Jacobian, see Forster et al. RSS 2015 eqn. (8)
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d & PhiVec);

/// \brief A class that does homogeneous Transformations.
/// This relates a frame A and B: T_AB; it consists of
///   translation r_AB (represented in frame A) and
///   Quaternion q_AB (as an Eigen Quaternion).
/// see also the RSS'13 / IJRR'14 paper or the Thesis.
/// Follows some of the functionality of the SchweizerMesser library by Paul Furgale,
/// but uses Eigen quaternions underneath.
/// \tparam CACHE_C Whether to cache the rotation matrix or not.
/// \warning This means the convention is different to SchweizerMesser
///          and the RSS'13 / IJRR'14 paper / the Thesis
template<bool CACHE_C = true>
class TransformationT
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class TransformationT<!CACHE_C>;

  /// \brief Default constructor: initialises a unit TransformationT.
  TransformationT();

  /// \brief Copy constructor: nothing fancy but takes care
  ///        of not doing bad stuff with internal caching.
  TransformationT(const TransformationT<CACHE_C> & other);

  /// \brief Copy constructor: nothing fancy but takes care
  ///        of not doing bad stuff with internal caching.
  TransformationT(const TransformationT<!CACHE_C> & other);

  /// \brief Move constructor: nothing fancy but takes care
  ///        of not doing bad stuff with internal caching.
  TransformationT(TransformationT<CACHE_C> && other);

  /// \brief Move constructor: nothing fancy but takes care
  ///        of not doing bad stuff with internal caching.
  TransformationT(TransformationT<!CACHE_C> && other);

  /// \brief Construct from a translation and quaternion.
  /// @param[in] r_AB The translation r_AB (represented in frame A).
  /// @param[in] q_AB The Quaternion q_AB (as an Eigen Quaternion).
  TransformationT(const Eigen::Vector3d & r_AB, const Eigen::Quaterniond& q_AB);

  /// \brief Construct from a homogeneous TransformationT matrix.
  /// @param[in] T_AB The homogeneous TransformationT matrix.
  explicit TransformationT(const Eigen::Matrix4d & T_AB);

  /// \brief Trivial destructor.
  ~TransformationT();

  /// \brief Parameter setting, all 7.
  /// \tparam Derived_coeffs Deducible matrix type.
  /// @param[in] coeffs The parameters as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen internal convention).
  template<typename Derived_coeffs>
  bool setCoeffs(const Eigen::MatrixBase<Derived_coeffs> & coeffs);

  /// \brief Parameter setting, all 7.
  /// \tparam Derived_coeffs Deducible matrix type.
  /// @param[in] parameters The parameters as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen internal convention).
  template<typename Derived_coeffs>
  bool setParameters(const Eigen::MatrixBase<Derived_coeffs> & parameters)
  {
    return setCoeffs(parameters);
  }

  /// \brief The underlying homogeneous TransformationT matrix.
  Eigen::Matrix4d T() const;

  /// \brief Returns the rotation matrix (cached).
  Eigen::Matrix3d C() const;

  /// \brief Returns the translation vector r_AB (represented in frame A).
  const Eigen::Map<Eigen::Vector3d> & r() const;

  /// \brief Returns the Quaternion q_AB (as an Eigen Quaternion).
  const Eigen::Map<Eigen::Quaterniond> & q() const;

  /// \brief Get the upper 3x4 part of the homogeneous TransformationT matrix T_AB.
  Eigen::Matrix<double, 3, 4> T3x4() const;

  /// \brief The coefficients (parameters) as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen internal conv.).
  const Eigen::Matrix<double, 7, 1> & coeffs() const
  {
    return parameters_;
  }

  /// \brief The coefficients (parameters) as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen internal conv.).
  Eigen::Matrix<double, 7, 1> & coeffs()
  {
    if(CACHE_C) throw(std::runtime_error("can't access writable parameters in cached mode."));
    return parameters_;
  }

  /// \brief The parameters (coefficients) as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen internal conv.).
  const Eigen::Matrix<double, 7, 1> & parameters() const
  {
    return parameters_;
  }

  /// \brief The parameters (coefficients) as [r_AB,q_AB], q_AB as [x,y,z,w] (Eigen internal conv.).
  Eigen::Matrix<double, 7, 1> & parameters()
  {
    if(CACHE_C) throw(std::runtime_error("can't access writable parameters in cached mode."));
    return parameters_;
  }

  /// \brief Get the parameters --- support for ceres.
  /// \warning USE WITH CARE!
  const double* parameterPtr() const
  {
    return &parameters_[0];
  }

  /// \brief Set this to a random TransformationT.
  void setRandom();

  /// \brief Set this to a random TransformationT with bounded rotation and translation.
  /// @param[in] translationMaxMeters Maximum translation [m].
  /// @param[in] rotationMaxRadians Maximum rotation [rad].
  void setRandom(double translationMaxMeters, double rotationMaxRadians);

  /// \brief Set from a homogeneous TransformationT matrix.
  /// @param[in] T_AB The homogeneous TransformationT matrix.
  void set(const Eigen::Matrix4d & T_AB);

  /// \brief Set from a translation and quaternion.
  /// @param[in] r_AB The translation r_AB (represented in frame A).
  /// @param[in] q_AB The Quaternion q_AB (as an Eigen Quaternion).
  void set(const Eigen::Vector3d & r_AB, const Eigen::Quaternion<double>& q_AB);

  /// \brief Set this TransformationT to identity
  void setIdentity();

  /// \brief Get an identity TransformationT
  static TransformationT<CACHE_C> Identity();

  /// \brief Returns a copy of the TransformationT inverted.
  TransformationT<CACHE_C> inverse() const;

  // operator* (group operator)
  /// \brief Multiplication with another TransformationT object.
  /// @param[in] rhs The right-hand side TransformationT for this to be multiplied with.
  TransformationT<CACHE_C> operator*(const TransformationT<CACHE_C> & rhs) const;

  /// \brief Transform a direction as v_A = C_AB*v_B (with rhs = hp_B)..
  /// \warning This only applies the rotation!
  /// @param[in] rhs The right-hand side direction for this to be multiplied with.
  Eigen::Vector3d operator*(const Eigen::Vector3d & rhs) const;

  /// \brief Transform a homogenous point as hp_B = T_AB*hp_B (with rhs = hp_B).
  /// @param[in] rhs The right-hand side direction for this to be multiplied with.
  Eigen::Vector4d operator*(const Eigen::Vector4d & rhs) const;

  /// \brief Assignment -- copy. Takes care of proper caching.
  /// @param[in] rhs The rhs for this to be assigned to.
  TransformationT<CACHE_C>& operator=(const TransformationT<CACHE_C> & rhs);

  /// \brief Assignment -- copy. Takes care of proper caching.
  /// @param[in] rhs The rhs for this to be assigned to.
  TransformationT<CACHE_C>& operator=(const TransformationT<!CACHE_C> & rhs);

  /// \brief Apply a small update with delta being 6x1.
  /// \tparam Derived_delta Deducible matrix type.
  /// @param[in] delta The 6x1 minimal update.
  /// \return True on success.
  template<typename Derived_delta>
  bool oplus(const Eigen::MatrixBase<Derived_delta> & delta);

  /// \brief Apply a small update with delta being 6x1 --
  ///        the Jacobian is a 7 by 6 matrix.
  /// @param[in] delta The 6x1 minimal update.
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  template<typename Derived_delta, typename Derived_jacobian>
  bool oplus(const Eigen::MatrixBase<Derived_delta> & delta,
             const Eigen::MatrixBase<Derived_jacobian> & jacobian);

  /// \brief Get the Jacobian of the oplus operation (a 7 by 6 matrix).
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  template<typename Derived_jacobian>
  bool oplusJacobian(
      const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;

  /// \brief Gets the jacobian dx/dChi,
  ///        i.e. lift the minimal Jacobian to a full one (as needed by ceres).
  // @param[out] jacobian The output lift Jacobian (6 by 7 matrix).
  /// \return True on success.
  template<typename Derived_jacobian>
  bool liftJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;

 protected:
  /// \brief Update the caching of the rotation matrix.
  void updateC();
  Eigen::Matrix<double, 7, 1> parameters_;  ///< Concatenated parameters [r;q].
  Eigen::Map<Eigen::Vector3d> r_;  ///< Translation {_A}r_{B}.
  Eigen::Map<Eigen::Quaterniond> q_;  ///< Quaternion q_{AB}.
  Eigen::Matrix3d C_; ///< The cached DCM C_{AB}.
};

/// \brief Transformation class.
typedef TransformationT<true> Transformation;

/// \brief Transformation class that doesn't cache rotation matrix.
typedef TransformationT<false> TransformationCacheless;


}  // namespace kinematics
}  // namespace okvis

#include "implementation/Transformation.hpp"

#endif /* INCLUDE_OKVIS_TRANSFORMATION_HPP_ */
