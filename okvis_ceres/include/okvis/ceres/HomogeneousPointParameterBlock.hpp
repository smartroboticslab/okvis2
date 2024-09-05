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
 * @file HomogeneousPointParameterBlock.hpp
 * @brief Header file for the HomogeneousPointParameterBlock class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_HOMOGENEOUSPOINTPARAMETERBLOCK_HPP_
#define INCLUDE_OKVIS_CERES_HOMOGENEOUSPOINTPARAMETERBLOCK_HPP_

#include <okvis/ceres/ParameterBlockSized.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <Eigen/Core>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief Wraps the parameter block for a speed / IMU biases estimate
class HomogeneousPointParameterBlock : public ParameterBlockSized<4, 3,
    Eigen::Vector4d>
{
 public:

  /// \brief The estimate type (4D vector).
  typedef Eigen::Vector4d estimate_t;

  /// \brief The base class type.
  typedef ParameterBlockSized<4, 3, estimate_t> base_t;

  /// \brief Default constructor (assumes not fixed).
  HomogeneousPointParameterBlock();

  /// \brief Constructor with estimate and time.
  /// @param[in] point The homogeneous point estimate.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] initialized Whether or not the 3d position is considered initialised.
  HomogeneousPointParameterBlock(const Eigen::Vector4d& point, uint64_t id,
                                 bool initialized = true);

  /// \brief Constructor with estimate and time.
  /// @param[in] point The homogeneous point estimate.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] initialized Whether or not the 3d position is considered initialised.
  HomogeneousPointParameterBlock(const Eigen::Vector3d& point, uint64_t id,
                                 bool initialized = true);

  /// \brief Trivial destructor.
  virtual ~HomogeneousPointParameterBlock() override;

  /// @name Setters
  /// @{

  /// \brief Set initialisaiton status.
  /// @param[in] initialized Whether or not the 3d position is considered initialised.
  void setInitialized(bool initialized)
  {
    initialized_ = initialized;
  }

  /// @}
  ///
  /// @brief Set exact parameters of this parameter block.
  /// @param[in] parameters The parameters to set this to.
  virtual void setParameters(const double* parameters) override {
    estimate_ = Eigen::Map<const Eigen::Matrix<double,4,1>>(parameters);
  }

  /// @brief Get parameters -- as a pointer.
  /// \return Pointer to the parameters allocated in here.
  virtual double* parameters() override {
    return estimate_.data();
  }

  /// @brief Get parameters -- as a pointer.
  /// \return Pointer to the parameters allocated in here.
  virtual const double* parameters() const override {
    return estimate_.data();
  }

  /// @name Getters
  /// @{

  /// \brief Get initialisaiton status.
  /// \return Whether or not the 3d position is considered initialised.
  bool initialized() const
  {
    return initialized_;
  }

  /// @}

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(const double* x0, const double* Delta_Chi,
                    double* x0_plus_Delta) const override
  {
    HomogeneousPointManifold::plus(x0, Delta_Chi, x0_plus_Delta);
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double* x0, double* jacobian) const override
  {
    HomogeneousPointManifold::plusJacobian(x0, jacobian);
  }

  // Delta_Chi=x0_plus_Delta[-]x0
  /// \brief Computes the minimal difference between a variable x and a perturbed variable
  /// x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  virtual void minus(const double* x0, const double* x0_plus_Delta,
                     double* Delta_Chi) const override
  {
    HomogeneousPointManifold::minus(x0_plus_Delta, x0, Delta_Chi);
  }

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by
  /// ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  virtual void liftJacobian(const double* x0, double* jacobian) const override
  {
    HomogeneousPointManifold::minusJacobian(x0, jacobian);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const override
  {
    return "HomogeneousPointParameterBlock";
  }

 private:
  bool initialized_;  ///< Whether or not the 3d position is considered initialised.

};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_HOMOGENEOUSPOINTPARAMETERBLOCK_HPP_ */
