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
 * @file TwoPoseGraphError.cpp
 * @brief Source file for the TwoPoseGraphError class.
 * @author Stefan Leutenegger
 */

#include <okvis/PseudoInverse.hpp>
#include <okvis/ceres/TwoPoseGraphError.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/timing/Timer.hpp>

//#define USE_NEW_LINEARIZATION_POINT

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Default constructor. Initialises a new okvis::ceres::Map.
TwoPoseGraphError::TwoPoseGraphError() {
  errorComputationValid_ = false;
  H00_.setZero();
  b0_.setZero();
  J_.setZero();
  DeltaX_.setZero();
  extrinsicsParameterBlockId2idx_.resize(10); /// \todo make this generic...
}

TwoPoseGraphError::TwoPoseGraphError(StateId referencePoseId, StateId otherPoseId) :
  referencePoseId_(referencePoseId), otherPoseId_(otherPoseId)
{
  errorComputationValid_ = false;
  H00_.setZero();
  b0_.setZero();
  J_.setZero();
  DeltaX_.setZero();
  extrinsicsParameterBlockId2idx_.resize(10); /// \todo make this generic...
}

// Add some residuals to this marginalisation error. This means, they will get linearised.
bool TwoPoseGraphError::addObservation(
    const KeypointIdentifier & keypointIdentifier,
    const std::shared_ptr<const ReprojectionError2dBase> & reprojectionError,
    ::ceres::LossFunction* lossFunction, const std::shared_ptr<ceres::PoseParameterBlock> & pose,
    const std::shared_ptr<ceres::HomogeneousPointParameterBlock> & hPoint,
    const std::shared_ptr<ceres::PoseParameterBlock> & extrinsics, bool isDuplication,
    double weight) {

  errorComputationValid_ = false;  // flag that the error computation is invalid

  // check poses
  OKVIS_ASSERT_TRUE_DBG(Exception, referencePoseId_.isInitialised(), "reference pose not set")
  OKVIS_ASSERT_TRUE_DBG(Exception, otherPoseId_.isInitialised(), "other pose not set")

  // add observation to bookkeeping
  const size_t ci = reprojectionError->cameraId();
  if(weight != 1.0) { // very very hacky.
    // clone the reprojection error into a new object with half the information...
    auto reprojectionErrorClone = reprojectionError->clone();
    reprojectionErrorClone->setInformation(weight*reprojectionErrorClone->information());
    observations_.push_back(
          Observation{keypointIdentifier, pose, hPoint, extrinsics,
                      reprojectionErrorClone, lossFunction, false, isDuplication});
  } else {
    if(isDuplication) {
      // clone the reprojection error into a new object with half the information...
      auto reprojectionErrorClone = reprojectionError->clone();
      reprojectionErrorClone->setInformation(reprojectionErrorClone->information());
      observations_.push_back(Observation{keypointIdentifier, pose, hPoint, extrinsics,
                                          reprojectionErrorClone, lossFunction, false, true});
    } else {
      // clone the reprojection error into a new object with half the information...
      auto reprojectionErrorClone = reprojectionError->clone();
      observations_.push_back(Observation{keypointIdentifier, pose, hPoint, extrinsics,
                                          reprojectionErrorClone, lossFunction, false, false});
    }
  }

  // first parameter: pose
  if(pose->id() == referencePoseId_.value()) {
    if(!poseParameterBlockInfos_[0].parameterBlock) {
      poseParameterBlockInfos_[0] = ParameterBlockInfo<7>(pose, 0);
    }
  } else if (pose->id() == otherPoseId_.value()) {
    if(!poseParameterBlockInfos_[1].parameterBlock) {
      poseParameterBlockInfos_[1] = ParameterBlockInfo<7>(pose, 0);
    }
  } else {
    OKVIS_THROW(Exception, "pose not registered")
    return false;
  }

  // second parameter: landmark
  if(landmarkParameterBlockId2idx_.find(hPoint->id()) == landmarkParameterBlockId2idx_.end()) {
    // needs adding
    landmarkParameterBlockInfos_.push_back(ParameterBlockInfo<4>(hPoint, sparseSize_));
    landmarkParameterBlockId2idx_[hPoint->id()] = landmarkParameterBlockInfos_.size() - 1;
    sparseSize_ += 3;
    // no base_t adding, since we will marginalise all landmarks
  }

  // third parameter: extrinsics
  if(!extrinsicsParameterBlockId2idx_.at(ci).count(extrinsics->id())) {
    // needs adding
    extrinsicsParameterBlockInfos_.push_back(ParameterBlockInfo<7>(extrinsics, extrinsicsSize_));
    extrinsicsParameterBlockId2idx_.at(ci)[extrinsics->id()] =
        extrinsicsParameterBlockInfos_.size() - 1;
  }

  return true;
}

bool TwoPoseGraphError::compute() {
  if(isComputed_) {
    return true; // noting to be done...
  }

  OKVIS_ASSERT_TRUE(Exception, poseParameterBlockInfos_[0].parameterBlock,
      "reference pose not observed")
  OKVIS_ASSERT_TRUE(Exception, poseParameterBlockInfos_[1].parameterBlock,
      "other pose not observed")

  // the reference pose
  const okvis::kinematics::Transformation T_WS0 = std::static_pointer_cast<PoseParameterBlock>(
        poseParameterBlockInfos_[0].parameterBlock)->estimate();
  const okvis::kinematics::Transformation T_S0W = T_WS0.inverse();

  // temporarily allocate sparse and sparse-dense part
  Eigen::Matrix<double,6,Eigen::Dynamic> H01(6, sparseSize_);
  H01.setZero();
  //Eigen::Matrix<double,6,Eigen::Dynamic> H00(6, 2*sparseSize_);
  //H00.setZero();
  //Eigen::VectorXd b0(2*sparseSize_);
  //b0.setZero();
  Eigen::Matrix3Xd H11(3, sparseSize_);
  H11.setZero();
  Eigen::VectorXd b1(sparseSize_);
  b1.setZero();

  Eigen::Matrix<double,6,Eigen::Dynamic> H00_backup(6, 2*sparseSize_);
  H00_backup.setZero();
  Eigen::VectorXd b0_backup(2*sparseSize_);
  b0_backup.setZero();

  // go through all the error terms and construct GN system
  bool relPoseSet = false;
  for(size_t i=0; i< observations_.size(); ++i) {
    const Observation & observation = observations_[i];
    const uint64_t id0 = observation.keypointIdentifier.frameId;
    const uint64_t id1 = observation.hPoint->id();
    const uint64_t id2 = observation.extrinsics->id();
    const size_t ci = observation.keypointIdentifier.cameraIndex;
    const ParameterBlockInfo<7> & info0 = (id0==referencePoseId_.value() ?
        poseParameterBlockInfos_[0] : poseParameterBlockInfos_[1]);
    const ParameterBlockInfo<4> & info1 =
        landmarkParameterBlockInfos_.at(landmarkParameterBlockId2idx_.at(id1));
    const ParameterBlockInfo<7> & info2 =
        extrinsicsParameterBlockInfos_.at(extrinsicsParameterBlockId2idx_.at(ci).at(id2));

    // get landmark
    const Eigen::Vector4d hp_W(
        info1.parameters[0], info1.parameters[1], info1.parameters[2], info1.parameters[3]);

    // get extrinsics
    const okvis::kinematics::Transformation T_SCi(
        Eigen::Vector3d(info2.parameters[0], info2.parameters[1], info2.parameters[2]),
        Eigen::Quaterniond(
            info2.parameters[6], info2.parameters[3], info2.parameters[4], info2.parameters[5]));

    // distinguish if reference to get pose
    const bool isReference = id0 == referencePoseId_.value();

    // residuals and jacobians
    Eigen::Vector2d residual;
    Eigen::Matrix<double,2,7,Eigen::RowMajor> jacobian0;
    Eigen::Matrix<double,2,6,Eigen::RowMajor> minimalJacobian0;
    Eigen::Matrix<double,2,4,Eigen::RowMajor> jacobian1;
    Eigen::Matrix<double,2,3,Eigen::RowMajor> minimalJacobian1;
    //Eigen::Matrix<double,2,7,Eigen::RowMajor> jacobian2;
    //Eigen::Matrix<double,2,6,Eigen::RowMajor> minimalJacobian2;
    double* jacobians[3];
    double* minimalJacobians[3];
    if(isReference) {
      jacobians[0] = nullptr;
      minimalJacobians[0] = nullptr;
    } else {
      jacobians[0] = jacobian0.data();
      minimalJacobians[0] = minimalJacobian0.data();
    }
    jacobians[1] = jacobian1.data();
    jacobians[2] = nullptr; //jacobian2.data();
    minimalJacobians[1] = minimalJacobian1.data();
    minimalJacobians[2] = nullptr; //minimalJacobian2.data();

    // transform landmark and remember
    const Eigen::Vector4d hp_S0 = T_S0W * hp_W;
    landmarks_[id1] = hp_S0;

    okvis::kinematics::Transformation T_S0S; // identity as the default (if reference)
    if(!isReference) {
      // get parameters (relative transorms and landmarks in S0 coordinates):
      const okvis::kinematics::Transformation T_WS(
          Eigen::Vector3d(info0.parameters[0], info0.parameters[1], info0.parameters[2]),
          Eigen::Quaterniond(
              info0.parameters[6], info0.parameters[3], info0.parameters[4], info0.parameters[5]));
      T_S0S = T_S0W*T_WS;
    }

    // remember parameters
    if(!isReference && !relPoseSet) {
      linearisationPoint_T_S0S1_ = T_S0S;
      relPoseSet = true;
    }
    if(!linearisationPoint_T_SCi_.count(id2)) {
      linearisationPoint_T_SCi_[id2] = T_SCi;
    }

    // assemble parameter pointer
    const double* params[3];
    PoseParameterBlock pose(T_S0S, id0, okvis::Time(0));
    params[0] = pose.parameters();
    params[1] = hp_S0.data();
    params[2] = info2.parameters.data();

    // evaluate
    observations_[i].reprojectionError->EvaluateWithMinimalJacobians(
                params, residual.data(), jacobians, minimalJacobians);

    // ignore obvious outliers
    if(residual.norm()>3.0) {
      continue;
    }

    {
      //robustify!!
      const ::ceres::LossFunction* lossFunction = observation.lossFunction;
      if (lossFunction) {

        // following ceres in internal/ceres/corrector.cc
        const double sq_norm = residual.transpose() * residual;
        double rho[3];
        lossFunction->Evaluate(sq_norm, rho);
        const double sqrt_rho1 = sqrt(rho[1]);
        double residual_scaling;
        double alpha_sq_norm;
        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
          residual_scaling = sqrt_rho1;
          alpha_sq_norm = 0.0;

        } else {
          // Calculate the smaller of the two solutions to the equation
          //
          // 0.5 *  alpha^2 - alpha - rho'' / rho' *  z'z = 0.
          //
          // Start by calculating the discriminant D.
          const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];

          // Since both rho[1] and rho[2] are guaranteed to be positive at
          // this point, we know that D > 1.0.

          const double alpha = 1.0 - sqrt(D);

          // Calculate the constants needed by the correction routines.
          residual_scaling = sqrt_rho1 / (1 - alpha);
          alpha_sq_norm = alpha / sq_norm;
        }

        // correct Jacobians (Equation 11 in BANS)
        minimalJacobian0 = sqrt_rho1 * (minimalJacobian0 - alpha_sq_norm * residual
                                        * (residual.transpose() * minimalJacobian0));
        minimalJacobian1 = sqrt_rho1 * (minimalJacobian1 - alpha_sq_norm * residual
                                        * (residual.transpose() * minimalJacobian1));
        //minimalJacobian2 = sqrt_rho1 * (minimalJacobian2 - alpha_sq_norm * residual
        //                                * (residual.transpose() * minimalJacobian2));

        // correct residuals (caution: must be after "correct Jacobians"):
        residual *= residual_scaling;
      }

      const int start0 = int(info0.idx);
      const int start1 = int(info1.idx);

      // construct GN system: add
      if(!isReference) {
        H00_ += minimalJacobian0.transpose()*minimalJacobian0;
        b0_ -= minimalJacobian0.transpose()*residual;
        H01.block<6,3>(start0,start1) += minimalJacobian0.transpose()*minimalJacobian1;

        H00_backup.block<6,6>(0,2*start1) += minimalJacobian0.transpose()*minimalJacobian0;
        b0_backup.segment<6>(2*start1) -=minimalJacobian0.transpose()*residual;
      }

      H11.block<3,3>(0,start1) += minimalJacobian1.transpose()*minimalJacobian1;
      b1.segment<3>(start1) -= minimalJacobian1.transpose()*residual;
    }

    // book-keeping: remove/mark internal
    observations_[i].isMarginalised = true;
  }

  // now marginalise out
  //const double epsilon = std::numeric_limits<double>::epsilon();
  Eigen::Matrix<double,6,6> mH = Eigen::Matrix<double,6,6>::Zero();
  Eigen::Matrix<double,6,1> mb = Eigen::Matrix<double,6,1>::Zero();
  for(int i=0; i<int(sparseSize_); i+=3) {
    Eigen::Matrix<double,6,3> W = H01.block<6,3>(0,i);
    const Eigen::Matrix3d V = H11.block<3,3>(0,i);
    Eigen::Matrix3d V_inv_sqrt;
    int rank;
    PseudoInverse::symmSqrt(V, V_inv_sqrt, 1.0e-8, &rank);
    if(rank<3) {
      // remove
      H00_ -= H00_backup.block<6,6>(0,2*i);
      b0_ -= b0_backup.segment<6>(2*i);
      // rotation only:
      H00_.block<3,3>(3,3) += H00_backup.block<3,3>(3,2*i+3);
      b0_.tail<3>() += b0_backup.segment<3>(2*i+3);
      //W.block<3,3>(0,0).setZero();
    } else {
    const Eigen::Matrix<double,6,3> M = W*V_inv_sqrt;
      mH +=  M*M.transpose();
      mb +=  M*(V_inv_sqrt.transpose()*b1.segment<3>(i));
    }
  }
  H00_ -= mH;
  b0_ -= mb;

  // Compute virtual error and relative Covariance
  // this might be a singular system and we need a decomposition to supply a Jacobian to ceres;
  // therefore we use eigendecomposition
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H00_);
  const double tolerance = 1.0e-8 * double(H00_.cols()) * saes.eigenvalues().array().maxCoeff();
  const Eigen::VectorXd D_sqrt = Eigen::VectorXd((saes.eigenvalues().array() > tolerance).select(
                saes.eigenvalues().array().sqrt(), 0));
  const Eigen::VectorXd D_inv_sqrt =
      Eigen::VectorXd((saes.eigenvalues().array() > tolerance).select(
                        saes.eigenvalues().array().inverse().sqrt(), 0));

  J_ = D_sqrt.asDiagonal() * saes.eigenvectors().transpose();
  const Eigen::Matrix<double,6,6> M = saes.eigenvectors() * D_inv_sqrt.asDiagonal();
  DeltaX_ = -M * (M.transpose() * b0_);

  // book-keeping: sizes
  sparseSize_ = 0;

  // book-keeping: internal sparse part
  landmarkParameterBlockId2idx_.clear();

  // remember we have computed
  isComputed_ = true;

  return true;
}

bool TwoPoseGraphError::convertToReprojectionErrors(std::vector<Observation> & observations,
                                                 std::vector<KeypointIdentifier> & duplicates) {
  // obtain referene pose
  const okvis::kinematics::Transformation T_WS0 = std::static_pointer_cast<PoseParameterBlock>(
              poseParameterBlockInfos_[0].parameterBlock)->estimate();

  // go through all the observations and add again...
  for(std::vector<Observation>::iterator it = observations_.begin();
      it != observations_.end(); ++it) {
    // check if marginalised
    if(!it->isMarginalised) {
      continue; // nothing to do.
    }

    // check if duplication
    if(it->isDuplication) {
      // communicate duplicate original information
      duplicates.push_back(it->keypointIdentifier);
      // return nevertheless
    }

    // see if landmark already present
    const uint64_t landmarkId = it->hPoint->id();
    std::shared_ptr<HomogeneousPointParameterBlock> landmark;
    // change landmark coordinate to world
    Eigen::Vector4d hp_W = T_WS0*landmarks_.at(landmarkId);
    it->hPoint.reset(new ceres::HomogeneousPointParameterBlock(hp_W, landmarkId, true));
    /// \todo init status

    // finally add observation
    observations.push_back(*it);
  }

  // clear all
  landmarkParameterBlockId2idx_.clear();
  landmarkParameterBlockInfos_.clear();
  observations_.clear();
  landmarks_.clear();
  linearisationPoint_T_SCi_.clear();

  // resize all
  J_.setZero();
  H00_.setZero();
  b0_.setZero();
  sparseSize_ = 0;
  return true;
}

void TwoPoseGraphError::getLandmarks(std::set<uint64_t> & landmarks) const {
  for(auto it=landmarkParameterBlockId2idx_.begin();
      it!=landmarkParameterBlockId2idx_.end(); ++it) {
    landmarks.insert(it->first);
  }
}

//This evaluates the error term and additionally computes the Jacobians.
bool TwoPoseGraphError::Evaluate(double const* const * parameters,
                                    double* residuals,
                                    double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

bool TwoPoseGraphError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                  double* residuals, double** jacobians,
                                  double** jacobiansMinimal) const {
  if (!isComputed_) {
    return false;
  }

  // don't evaluate constant cost (**hack**)
  Eigen::Map<Eigen::Matrix<double,6,1>> error_weighted(residuals, 6);
  if(!jacobians) {
    error_weighted.setZero();
    return true;
  }

  // get the reference pose T_WS0
  const size_t refIdx = 0;
  const okvis::kinematics::Transformation T_WS0(
        Eigen::Vector3d(parameters[refIdx][0],parameters[refIdx][1],parameters[refIdx][2]),
        Eigen::Quaterniond(parameters[refIdx][6],parameters[refIdx][3],parameters[refIdx][4],
      parameters[refIdx][5]).normalized());
  const okvis::kinematics::Transformation T_S0W = T_WS0.inverse();

  // the difference to the linearisation point:
  Eigen::Matrix<double,6,1> DeltaX_linearisationPoint = Eigen::Matrix<double,6,1>::Zero();

  // parse other pose and evaluate deviation from linearisation point
  const size_t idx = 1;
  const size_t startIdx = 0;
  // compute difference to relative pose stored as linearisation point
  const okvis::kinematics::Transformation T_WSi(
        Eigen::Vector3d(parameters[idx][0],parameters[idx][1],parameters[idx][2]),
        Eigen::Quaterniond(parameters[idx][6],parameters[idx][3],parameters[idx][4],
      parameters[idx][5]).normalized());
  const okvis::kinematics::Transformation T_S0Si = T_S0W * T_WSi;
  DeltaX_linearisationPoint.segment<3>(startIdx) =
      T_S0Si.r() - linearisationPoint_T_S0S1_.r();
  DeltaX_linearisationPoint.segment<3>(startIdx + 3) =
      2.0*(T_S0Si.q() * linearisationPoint_T_S0S1_.q().inverse()).coeffs().head<3>();

  // the unweighted error term
  const Eigen::Matrix<double,6,1> error = DeltaX_ + DeltaX_linearisationPoint;

  // and with the already factored information matrix:
  error_weighted = J_ * error;

  /// Jacobians...
  /// the structure is (with J_ the dense sqrt coviariance):
  ///       S0 S1 S2 S3 S4 C0 C1
  ///
  ///       X  X  0  0  0  0  0
  ///       X  0  X  0  0  0  0
  ///  J_*  X  0  0  X  0  0  0
  ///       X  0  0  0  X  0  0
  ///       0  0  0  0  0  X  0
  ///       0  0  0  0  0  0  X
  ///
  if(jacobians || jacobiansMinimal) {
    // allocate the jacobian w.r.t. the reference pose separately:
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> JerrRef =
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();

    // handle non-reference first
    const size_t idx = 1;
    if(jacobians[idx]) {

      const size_t startIdx = 0;

      // the minimal jacobian
      Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jmin;

      // handle non-reference
      const okvis::kinematics::Transformation T_WS(Eigen::Vector3d(parameters[idx][0],
                                                                   parameters[idx][1],
                                                                   parameters[idx][2]),
                                                   Eigen::Quaterniond(parameters[idx][6],
                                                                      parameters[idx][3],
                                                                      parameters[idx][4],
                                                                      parameters[idx][5]));
      Eigen::Matrix<double,6,6> Jerr = Eigen::Matrix<double,6,6>::Zero();
      Jerr.topLeftCorner<3,3>() = T_S0W.C();
      Jerr.bottomRightCorner<3, 3>() = (okvis::kinematics::plus(T_WS0.q().inverse())
                                        * okvis::kinematics::oplus(
                                          T_WS.q() * linearisationPoint_T_S0S1_.q().inverse()))
                                         .topLeftCorner<3, 3>();
      Jmin = J_.block<6,6>(0,startIdx)*Jerr;

      // we also handle the reference here
      if(jacobians[refIdx]) {
        JerrRef.block<3,3>(startIdx, 0) = -T_S0W.C();
        JerrRef.block<3,3>(startIdx, 0+3) = T_S0W.C()*okvis::kinematics::crossMx(T_WS.r()-T_WS0.r());
        JerrRef.block<3,3>(startIdx+3, 0+3) = -Jerr.bottomRightCorner<3,3>();
      }

      // map minimal Jacobian if requested
      if(jacobiansMinimal){
        if(jacobiansMinimal[idx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> JminMapped(
                jacobiansMinimal[idx]);
          JminMapped = Jmin;
        }
      }

      // and assign the actual Jacobian
      if(jacobians[idx]) {
        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseManifold::minusJacobian(parameters[idx], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[idx]);
        J = Jmin * J_lift;
      }
    }

    // now handle the overall Jacobian of the reference pose separately:
    if(jacobians[refIdx]) {
      // the minimal jacobian
      const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jmin =  J_ * JerrRef;

      // map if requested
      if(jacobiansMinimal) {
        if(jacobiansMinimal[refIdx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> JminMapped(
                jacobiansMinimal[refIdx], J_.rows(), 6);
          JminMapped = Jmin;
        }
      }

      // map actual reference Jacobian if requested
      if(jacobians[refIdx]) {
        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseManifold::minusJacobian(parameters[refIdx], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic, 7, Eigen::RowMajor>> J(
              jacobians[refIdx], J_.rows(), 7);
        J = Jmin * J_lift;
      }
    }
  }

  return true;
}

std::shared_ptr<TwoPoseGraphErrorConst> TwoPoseGraphError::cloneTwoPoseGraphErrorConst() const {
  return std::shared_ptr<TwoPoseGraphErrorConst>(
        new TwoPoseGraphErrorConst(
          DeltaX_, J_, linearisationPoint_T_S0S1_, linearisationPoint_T_SCi_));
}

double TwoPoseGraphError::strength() const {
  if(isComputed_) {
    Eigen::Matrix<double, 6, 6> H00inv;
    int rank;
    PseudoInverse::symm(H00_, H00inv, 1.0e-6, &rank);
    if(rank<6) {
      return 0.0;
    }
    return 1.0/sqrt(H00inv.norm()); // inverse position standard deviation
  }
  return 0.0;
}

bool TwoPoseGraphErrorConst::Evaluate(
    const double * const *parameters, double *residuals, double **jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

bool TwoPoseGraphErrorConst::EvaluateWithMinimalJacobians(
    const double * const *parameters, double *residuals, double **jacobians,
    double **jacobiansMinimal) const
{

  // don't evaluate constant cost (**hack**)
  Eigen::Map<Eigen::Matrix<double,6,1>> error_weighted(residuals, 6);
  if(!jacobians) {
    error_weighted.setZero();
    return true;
  }

  // get the reference pose T_WS0
  const size_t refIdx = 0;
  const okvis::kinematics::Transformation T_WS0(
        Eigen::Vector3d(parameters[refIdx][0],parameters[refIdx][1],parameters[refIdx][2]),
        Eigen::Quaterniond(parameters[refIdx][6],parameters[refIdx][3],parameters[refIdx][4],
      parameters[refIdx][5]).normalized());
  const okvis::kinematics::Transformation T_S0W = T_WS0.inverse();

  // the difference to the linearisation point:
  Eigen::Matrix<double,6,1> DeltaX_linearisationPoint = Eigen::Matrix<double,6,1>::Zero();

  // parse other pose and evaluate deviation from linearisation point
  const size_t idx = 1;
  const size_t startIdx = 0;
  // compute difference to relative pose stored as linearisation point
  const okvis::kinematics::Transformation T_WSi(
        Eigen::Vector3d(parameters[idx][0],parameters[idx][1],parameters[idx][2]),
        Eigen::Quaterniond(parameters[idx][6],parameters[idx][3],parameters[idx][4],
      parameters[idx][5]).normalized());
  const okvis::kinematics::Transformation T_S0Si = T_S0W * T_WSi;
  DeltaX_linearisationPoint.segment<3>(startIdx) =
      T_S0Si.r() - linearisationPoint_T_S0S1_.r();
  DeltaX_linearisationPoint.segment<3>(startIdx + 3) =
      2*(T_S0Si.q() * linearisationPoint_T_S0S1_.q().inverse()).coeffs().head<3>();

  // the unweighted error term
  const Eigen::Matrix<double,6,1> error = DeltaX_ + DeltaX_linearisationPoint;

  // and with the already factored information matrix:
  error_weighted = J_ * error;

  /// Jacobians...
  /// the structure is (with J_ the dense sqrt coviariance):
  ///       S0 S1 S2 S3 S4 C0 C1
  ///
  ///       X  X  0  0  0  0  0
  ///       X  0  X  0  0  0  0
  ///  J_*  X  0  0  X  0  0  0
  ///       X  0  0  0  X  0  0
  ///       0  0  0  0  0  X  0
  ///       0  0  0  0  0  0  X
  ///
  if(jacobians || jacobiansMinimal) {
    // allocate the jacobian w.r.t. the reference pose separately:
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> JerrRef =
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();

    // handle non-reference first
    const size_t idx = 1;
    if(jacobians[idx]) {

      const size_t startIdx = 0;

      // the minimal jacobian
      Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jmin;

      // handle non-reference
      const okvis::kinematics::Transformation T_WS(Eigen::Vector3d(parameters[idx][0],
                                                                   parameters[idx][1],
                                                                   parameters[idx][2]),
                                                   Eigen::Quaterniond(parameters[idx][6],
                                                                      parameters[idx][3],
                                                                      parameters[idx][4],
                                                                      parameters[idx][5]));
      Eigen::Matrix<double,6,6> Jerr = Eigen::Matrix<double,6,6>::Zero();
      Jerr.topLeftCorner<3,3>() = T_S0W.C();
      Jerr.bottomRightCorner<3, 3>() = (okvis::kinematics::plus(T_WS0.q().inverse())
                                        * okvis::kinematics::oplus(
                                          T_WS.q() * linearisationPoint_T_S0S1_.q().inverse()))
                                         .topLeftCorner<3, 3>();
      Jmin = J_.block<6,6>(0,startIdx)*Jerr;

      // we also handle the reference here
      if(jacobians[refIdx]) {
        JerrRef.block<3,3>(startIdx, 0) = -T_S0W.C();
        JerrRef.block<3,3>(startIdx, 0+3) = T_S0W.C()*okvis::kinematics::crossMx(T_WS.r()-T_WS0.r());
        JerrRef.block<3,3>(startIdx+3, 0+3) = -Jerr.bottomRightCorner<3,3>();
      }

      // map minimal Jacobian if requested
      if(jacobiansMinimal){
        if(jacobiansMinimal[idx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> JminMapped(
                jacobiansMinimal[idx]);
          JminMapped = Jmin;
        }
      }

      // and assign the actual Jacobian
      if(jacobians[idx]) {
        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseManifold::minusJacobian(parameters[idx], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobians[idx]);
        J = Jmin * J_lift;
      }
    }

    // now handle the overall Jacobian of the reference pose separately:
    if(jacobians[refIdx]) {
      // the minimal jacobian
      const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jmin =  J_ * JerrRef;

      // map if requested
      if(jacobiansMinimal) {
        if(jacobiansMinimal[refIdx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> JminMapped(
                jacobiansMinimal[refIdx], J_.rows(), 6);
          JminMapped = Jmin;
        }
      }

      // map actual reference Jacobian if requested
      if(jacobians[refIdx]) {
        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseManifold::minusJacobian(parameters[refIdx], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic, 7, Eigen::RowMajor>> J(
              jacobians[refIdx], J_.rows(), 7);
        J = Jmin * J_lift;
      }
    }
  }

  return true;
}

} // namespace ceres
} // namespace okvis
