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
 * @file PoseParameterBlock.hpp
 * @brief Header file for the PoseParameterBlock class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_POSEPARAMETERBLOCK_HPP_
#define INCLUDE_OKVIS_CERES_POSEPARAMETERBLOCK_HPP_

#include <Eigen/Core>
#include <okvis/ceres/ParameterBlockSized.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>

namespace okvis {
namespace ceres{

/// \brief Wraps the parameter block for a pose estimate
class PoseParameterBlock :
    public ParameterBlockSized<7,6,okvis::kinematics::TransformationCacheless>{
public:

  /// \brief The estimate type (okvis::kinematics::Transformation ).
  typedef okvis::kinematics::TransformationCacheless estimate_t;

  /// \brief The base class type.
  typedef ParameterBlockSized<7,6,estimate_t> base_t;

  /// \brief Default constructor (assumes not fixed).
  PoseParameterBlock();

  /// \brief Constructor with estimate and time.
  /// @param[in] T_WS The pose estimate as T_WS.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  PoseParameterBlock(const okvis::kinematics::Transformation& T_WS, uint64_t id,
                     const okvis::Time& timestamp = okvis::Time(0));

  /// \brief Trivial destructor.
  virtual ~PoseParameterBlock() override;

  // setters

  /// @param[in] timestamp The timestamp of this state.
  void setTimestamp(const okvis::Time& timestamp){timestamp_=timestamp;}

  /// @brief Set exact parameters of this parameter block.
  /// @param[in] parameters The parameters to set this to.
  virtual void setParameters(const double* parameters) override final {
    estimate_.setCoeffs(Eigen::Map<const Eigen::Matrix<double,7,1>>(parameters));
  }

  /// @brief Get parameters -- as a pointer.
  /// \return Pointer to the parameters allocated in here.
  virtual double* parameters() override final {
    return estimate_.coeffs().data();
  }

  /// @brief Get parameters -- as a pointer.
  /// \return Pointer to the parameters allocated in here.
  virtual const double* parameters() const override final {
    return estimate_.coeffs().data();
  }

  // getters
  /// \brief Get the time.
  /// \return The timestamp of this state.
  okvis::Time timestamp() const {return timestamp_;}

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(
      const double* x0, const double* Delta_Chi, double* x0_plus_Delta) const override final {
    PoseManifold::plus(x0,Delta_Chi,x0_plus_Delta);
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double* x0, double* jacobian) const override final {
    PoseManifold::plusJacobian(x0,jacobian);
  }

  // Delta_Chi=x0_plus_Delta[-]x0
  /// \brief Computes the minimal difference between a variable x and a perturbed variable
  /// x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  virtual void minus(
      const double* x0, const double* x0_plus_Delta, double* Delta_Chi) const override final {
    PoseManifold::minus(x0_plus_Delta, x0, Delta_Chi);
  }

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by
  /// ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  virtual void liftJacobian(const double* x0, double* jacobian) const override final {
    PoseManifold::minusJacobian(x0,jacobian);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const override final {return "PoseParameterBlock";}

private:
  okvis::Time timestamp_; ///< Time of this state.
};

} // namespace ceres
} // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_POSEPARAMETERBLOCK_HPP_ */
