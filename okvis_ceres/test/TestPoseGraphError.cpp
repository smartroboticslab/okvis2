/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
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
 *
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include "glog/logging.h"
#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/HomogeneousPointError.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/PoseGraphError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, PoseGraphError){
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  // different cases of camera extrinsics;
  for (size_t c = 0; c < 2; ++c) {

    OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

    const double DURATION = 10.0;  // 10 seconds motion
    std::cout << "case " << c % 2 << ", " << c / 2 << std::endl;

    // create the map
    std::shared_ptr<okvis::ceres::Map> mapPtr(new okvis::ceres::Map);

    // camera extrinsics:
    okvis::kinematics::Transformation T_SC_0(Eigen::Vector3d(0,0.0,0),
                                             Eigen::Quaterniond(sqrt(0.5),0,sqrt(0.5),0));
    okvis::kinematics::Transformation T_SC_1(Eigen::Vector3d(0,0.1,0),
                                             Eigen::Quaterniond(sqrt(0.5),0,sqrt(0.5),0));
    std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsics0(
        new okvis::ceres::PoseParameterBlock(T_SC_0,okvis::IdProvider::instance().newId()));
    std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsics1(
        new okvis::ceres::PoseParameterBlock(T_SC_1,okvis::IdProvider::instance().newId()));
    mapPtr->addParameterBlock(extrinsics0, okvis::ceres::Map::Pose6d);
    mapPtr->addParameterBlock(extrinsics1, okvis::ceres::Map::Pose6d);
    if(c==0) {
      // set constant in one case only
      mapPtr->setParameterBlockConstant(extrinsics0->id());
      mapPtr->setParameterBlockConstant(extrinsics1->id());
    } else {
      // soft-constrain
      std::shared_ptr<okvis::ceres::PoseError> poseError0(
            new okvis::ceres::PoseError(T_SC_0, 0.1, 0.01));
      std::shared_ptr<okvis::ceres::PoseError> poseError1(
            new okvis::ceres::PoseError(T_SC_1, 0.1, 0.01));
      mapPtr->addResidualBlock(poseError0, nullptr, extrinsics0);
      mapPtr->addResidualBlock(poseError1, nullptr, extrinsics1);
    }

    // set up camera with intrinsics
    typedef okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion> CameraModel;
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry0(
        CameraModel::createTestObject());
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry1(
        CameraModel::createTestObject());

    // create landmark grid
    const okvis::kinematics::Transformation T_WS_0;
    okvis::AlignedVector<Eigen::Vector4d> homogeneousPoints;
    std::vector<uint64_t> lmIds;
    for (double y = -10.0; y <= DURATION * T_SC_1.r()[1] + 10.0; y += 0.5) {
      for (double z = -10.0; z <= 10.0; z += 0.5) {
        homogeneousPoints.push_back(Eigen::Vector4d(3.0, y, z, 1));
        lmIds.push_back(okvis::IdProvider::instance().newId());
        std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> hPointParameterBlock(
              new okvis::ceres::HomogeneousPointParameterBlock(
                homogeneousPoints.back(), lmIds.back()));
        mapPtr->addParameterBlock(hPointParameterBlock, okvis::ceres::Map::HomogeneousPoint);
      }
    }

    const size_t K = 6;
    Eigen::Vector3d speed(0, 1, 0);
    std::vector<std::shared_ptr<okvis::ceres::PoseParameterBlock>> poseParameterBlocks;
    std::vector<::ceres::ResidualBlockId> obsIds;
    std::vector<size_t> keypointIndices;
    for (size_t k = 0; k < K + 1; ++k) {
      // calculate the transformation
      okvis::kinematics::Transformation T_WS(
          T_WS_0.r() + speed * double(k) * DURATION / double(K), T_WS_0.q());

      // instantiate new pose parameter block
      std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock(
            new okvis::ceres::PoseParameterBlock(T_WS, okvis::IdProvider::instance().newId()));
      poseParameterBlocks.push_back(poseParameterBlock);
      mapPtr->addParameterBlock(poseParameterBlock, okvis::ceres::Map::Pose6d);

      // soft-fix first pose
      if(k==0) {
        std::shared_ptr<okvis::ceres::PoseError> poseError(
              new okvis::ceres::PoseError(T_WS, 0.1, 0.01));
        mapPtr->addResidualBlock(poseError, nullptr, poseParameterBlock);
      }

      // now let's add landmark observations
      for (size_t j = 0; j < homogeneousPoints.size(); ++j) {
        Eigen::Vector2d projection0, projection1;
        Eigen::Vector4d point_C0 = T_SC_0.inverse()
            * T_WS.inverse() * homogeneousPoints[j];
        Eigen::Vector4d point_C1 = T_SC_1.inverse()
            * T_WS.inverse() * homogeneousPoints[j];
        okvis::cameras::CameraBase::ProjectionStatus status0 =
            cameraGeometry0->projectHomogeneous(point_C0, &projection0);
        okvis::cameras::CameraBase::ProjectionStatus status1 =
            cameraGeometry1->projectHomogeneous(point_C1, &projection1);
        if (status0 == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
          Eigen::Vector2d measurement0(projection0 + Eigen::Vector2d::Random());
          std::shared_ptr<okvis::ceres::ReprojectionErrorBase> reprojectionError0(
                new okvis::ceres::ReprojectionError<CameraModel>(
                  std::dynamic_pointer_cast<const CameraModel>(cameraGeometry0), 0, measurement0,
                  Eigen::Matrix2d::Identity()));
          obsIds.push_back(mapPtr->addResidualBlock(reprojectionError0, nullptr, poseParameterBlock,
                                   mapPtr->parameterBlockPtr(lmIds[j]), extrinsics0));
          keypointIndices.push_back(0);
        }
        if (status1 == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
          Eigen::Vector2d measurement1(projection1 + Eigen::Vector2d::Random());
          std::shared_ptr<okvis::ceres::ReprojectionErrorBase> reprojectionError1(
                new okvis::ceres::ReprojectionError<CameraModel>(
                  std::dynamic_pointer_cast<const CameraModel>(cameraGeometry1), 1, measurement1,
                  Eigen::Matrix2d::Identity()));
          obsIds.push_back(mapPtr->addResidualBlock(reprojectionError1, nullptr, poseParameterBlock,
                                   mapPtr->parameterBlockPtr(lmIds[j]), extrinsics1));
          keypointIndices.push_back(0);
        }
      }
      // run the optimization briefly (to create some deviation from ground truth)
      mapPtr->options.minimizer_progress_to_stdout = false;
      mapPtr->options.max_num_iterations = 2;
      mapPtr->solve();
    }

    // inspect convergence:
    okvis::kinematics::Transformation T_WS(
         T_WS_0.r() + speed * DURATION,
         T_WS_0.q());
    okvis::kinematics::Transformation T_WS_est = poseParameterBlocks.back()->estimate();
    OKVIS_ASSERT_TRUE(Exception, 2*(T_WS.q()*T_WS_est.q().inverse()).vec().norm()<1e-2,
                "quaternions not close enough");
    OKVIS_ASSERT_TRUE(Exception, (T_WS.r() - T_WS_est.r()).norm()<1e-1,
                "translation not close enough");

    std::cout << "create pose graph error..." << std::endl;

    // now create pose graph error
    std::shared_ptr<okvis::ceres::PoseGraphError> poseGraphError(
          new okvis::ceres::PoseGraphError(*mapPtr));
    poseGraphError->setReferencePoseId(poseParameterBlocks.front()->id());

    // compute it from observations
    poseGraphError->addObservations(obsIds, keypointIndices);
    poseGraphError->compute();

    // add to map
    std::vector<std::shared_ptr<okvis::ceres::ParameterBlock>> parameterBlocks;
    poseGraphError->getParameterBlockPtrs(parameterBlocks);
    auto poseGraphErrorId = mapPtr->addResidualBlock(poseGraphError, nullptr, parameterBlocks);

    // check Jacobian
    OKVIS_ASSERT_TRUE(Exception, mapPtr->isJacobianCorrect(poseGraphErrorId),
                      "Pose graph Jacobian does not verify");

    // and now solve again
    std::cout << "solve again..." << std::endl;
    mapPtr->options.minimizer_progress_to_stdout = true;
    mapPtr->options.max_num_iterations = 10;
    mapPtr->solve();

    // check correctness
    T_WS_est = poseParameterBlocks.back()->estimate();
    std::cout << "estimated T_WS: " << std::endl << T_WS_est.T() << std::endl;
    std::cout << "correct T_WS: " << std::endl << T_WS.T() << std::endl;

    // check correctness
    OKVIS_ASSERT_TRUE(Exception, 2*(T_WS.q()*T_WS_est.q().inverse()).vec().norm()<2e-2,
                "quaternions not close enough");
    OKVIS_ASSERT_TRUE(Exception, (T_WS.r() - T_WS_est.r()).norm()<1e-1,
                "translation not close enough");

    // provide additional pose constraint simulating loop closure
    std::cout << "simulating loop closure pose constraint and re-solve..." << std::endl;
    std::shared_ptr<okvis::ceres::PoseError> poseError(
          new okvis::ceres::PoseError(T_WS, 0.1, 0.01));
    mapPtr->addResidualBlock(poseError, nullptr, poseParameterBlocks.back());
    mapPtr->solve();

    // check correctness
    T_WS_est = poseParameterBlocks.back()->estimate();
    std::cout << "estimated T_WS: " << std::endl << T_WS_est.T() << std::endl;
    std::cout << "correct T_WS: " << std::endl << T_WS.T() << std::endl;

    // check correctness
    OKVIS_ASSERT_TRUE(Exception, 2*(T_WS.q()*T_WS_est.q().inverse()).vec().norm()<1e-2,
                "quaternions not close enough");
    OKVIS_ASSERT_TRUE(Exception, (T_WS.r() - T_WS_est.r()).norm()<1e-1,
                "translation not close enough");

    // convert back to observations
    std::cout << "convert back to observations and re-solve..." << std::endl;
    poseGraphError->convertToReprojectionErrors();
    mapPtr->removeResidualBlock(poseGraphErrorId);
    mapPtr->solve();

    // check correctness
    T_WS_est = poseParameterBlocks.back()->estimate();
    std::cout << "estimated T_WS: " << std::endl << T_WS_est.T() << std::endl;
    std::cout << "correct T_WS: " << std::endl << T_WS.T() << std::endl;

    // check correctness
    OKVIS_ASSERT_TRUE(Exception, 2*(T_WS.q()*T_WS_est.q().inverse()).vec().norm()<1e-2,
                "quaternions not close enough");
    OKVIS_ASSERT_TRUE(Exception, (T_WS.r() - T_WS_est.r()).norm()<1e-1,
                "translation not close enough");
  }
}
