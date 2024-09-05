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
 * @file PseudoInverse.hpp
 * @brief Header file for the PseudoInverse class.
 * @author Stefan Leutenegger
 */

#ifndef OKVIS_PSEUDOINVERSE_HPP_
#define OKVIS_PSEUDOINVERSE_HPP_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <okvis/assert_macros.hpp>


/// \brief okvis Main namespace of this package.
namespace okvis {

/// @brief A class to efficiently compute Pseudo-inverses of square PSD matrices.
class PseudoInverse {
public:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
/**
 * @brief Pseudo inversion of a symmetric matrix.
 * @warning This uses Eigen-decomposition, it assumes the input is symmetric positive
 * semi-definite (negative Eigenvalues are set to zero).
 * @tparam Derived Matrix type (auto-deducible).
 * @param[in] a Input Matrix
 * @param[out] result Output, i.e. pseudo-inverse.
 * @param[in] epsilon The tolerance.
 * @param[out] rank Optional rank.
 * @return
 */
template<typename Derived>
static bool symm(
    const Eigen::MatrixBase<Derived>&a,
    const Eigen::MatrixBase<Derived>&result, double epsilon =
        std::numeric_limits<typename Derived::Scalar>::epsilon(), int * rank = nullptr) {
    OKVIS_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                          "matrix supplied is not square")

    Eigen::SelfAdjointEigenSolver<Derived> saes(a);

    typename Derived::Scalar tolerance = std::max(
          epsilon, epsilon * a.cols() * saes.eigenvalues().array().maxCoeff());

    const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
        * Eigen::VectorXd(
            (saes.eigenvalues().array() > tolerance).select(
                saes.eigenvalues().array().inverse(), 1.0/tolerance)).asDiagonal()
        * (saes.eigenvectors().transpose());

    if (rank) {
      *rank = 0;
      for (int i = 0; i < a.rows(); ++i) {
        if (saes.eigenvalues()[i] > tolerance)
          (*rank)++;
      }
    }

    return true;
}

/**
 * @brief Pseudo inversion and square root (Cholesky decomposition) of a symmetric matrix.
 * @warning This uses Eigen-decomposition, it assumes the input is symmetric positive
 * semi-definite (negative Eigenvalues are set to zero).
 * @tparam Derived Matrix type (auto-deducible).
 * @param[in] a Input Matrix
 * @param[out] result Output, i.e. the Cholesky decomposition of a pseudo-inverse.
 * @param[in] epsilon The tolerance.
 * @param[out] rank The rank, if of interest.
 * @return
 */
template<typename Derived>
static bool symmSqrt(
    const Eigen::MatrixBase<Derived>&a,
    const Eigen::MatrixBase<Derived>&result, double epsilon =
        std::numeric_limits<typename Derived::Scalar>::epsilon(),
        int* rank = nullptr) {
    OKVIS_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                          "matrix supplied is not square")

    Eigen::SelfAdjointEigenSolver<Derived> saes(a);

    typename Derived::Scalar tolerance = std::max(
          epsilon, epsilon * a.cols() * saes.eigenvalues().array().maxCoeff());

    const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
        * Eigen::VectorXd(
            Eigen::VectorXd(
                (saes.eigenvalues().array() > tolerance).select(
                    saes.eigenvalues().array().inverse(), 1.0/tolerance)).array().sqrt())
            .asDiagonal();

    if (rank) {
      *rank = 0;
      for (int i = 0; i < a.rows(); ++i) {
        if (saes.eigenvalues()[i] > tolerance)
          (*rank)++;
      }
    }

    return true;
}

/**
 * @brief Pseudo inversion and square root (Cholesky decomposition) of a symmetric matrix.
 * @warning This uses Eigen-decomposition, it assumes the input is symmetric positive
 * semi-definite (negative Eigenvalues are set to zero).
 * @tparam Derived Matrix type (auto-deducible).
 * @param[in] a Input Matrix
 * @param[out] result Output, i.e. the Cholesky decomposition of a pseudo-inverse.
 * @param[in] epsilon The tolerance.
 * @param[out] rank The rank, if of interest.
 * @return
 */
template<typename Derived>
static bool symmSqrtU(
    const Eigen::MatrixBase<Derived>&a,
    const Eigen::MatrixBase<Derived>&result, double epsilon =
        std::numeric_limits<typename Derived::Scalar>::epsilon(),
        int* rank = nullptr) {
    OKVIS_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                          "matrix supplied is not square")

    Eigen::SelfAdjointEigenSolver<Derived> saes(a);

    typename Derived::Scalar tolerance = std::max(
          epsilon, epsilon * a.cols() * saes.eigenvalues().array().maxCoeff());

    const_cast<Eigen::MatrixBase<Derived>&>(result) = Eigen::VectorXd(
            Eigen::VectorXd(
                (saes.eigenvalues().array() > tolerance).select(
                    saes.eigenvalues().array().inverse(), 1.0/tolerance)).array().sqrt())
            .asDiagonal() * (saes.eigenvectors().transpose());

    if (rank) {
      *rank = 0;
      for (int i = 0; i < a.rows(); ++i) {
        if (saes.eigenvalues()[i] > tolerance)
          (*rank)++;
      }
    }

    return true;
}
};

}

#endif // OKVIS_PSEUDOINVERSE_HPP_
