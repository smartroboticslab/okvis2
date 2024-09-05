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
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/HomogeneousPointError.hpp>
#include <okvis/ceres/RelativeReprojectionError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, RelativeReprojectionError){
	// initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

	// Build the problem.
  okvis::ceres::Map map;

	// set up a random geometry
	std::cout<<"set up a random geometry... "<<std::flush;
  okvis::kinematics::Transformation T_WS0; // world to sensor 0
  T_WS0.setRandom(10.0,M_PI);
  okvis::kinematics::Transformation T_SC; // sensor to camera
  T_SC.setRandom(0.2,M_PI);
  okvis::kinematics::Transformation T_C0C1(Eigen::Vector3d(0.2,0.01,-0.02),
                                           Eigen::Quaterniond(1,0,0,0));
  okvis::kinematics::Transformation T_WS1 = T_WS0*T_SC*T_C0C1*T_SC.inverse(); // world to sensor 1
	okvis::kinematics::Transformation T_disturb;
  T_disturb.setRandom(0.2,0.01);
  okvis::kinematics::Transformation T_WS0_init=T_WS0*T_disturb; // world to sensor 0 init
  T_disturb.setRandom(0.19,0.009);
  okvis::kinematics::Transformation T_WS1_init=T_WS1*T_disturb; // world to sensor 1 init

  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock0(
        new okvis::ceres::PoseParameterBlock(T_WS0_init,1,okvis::Time(0)));
  //std::cout << "est0 " << poseParameterBlock0->estimate().q().coeffs().transpose() <<std::endl;
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock1(
        new okvis::ceres::PoseParameterBlock(T_WS1_init,2,okvis::Time(0)));
  //std::cout << "est1 " << poseParameterBlock1->estimate().q().coeffs().transpose() <<std::endl;
  std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicsParameterBlock(
        new okvis::ceres::PoseParameterBlock(T_SC,3,okvis::Time(0)));
  map.addParameterBlock(poseParameterBlock0,okvis::ceres::Map::Parameterization::Pose6d);
  map.addParameterBlock(poseParameterBlock1,okvis::ceres::Map::Parameterization::Pose6d);
  map.addParameterBlock(extrinsicsParameterBlock,okvis::ceres::Map::Parameterization::Pose6d);
  map.setParameterBlockConstant(extrinsicsParameterBlock); // do not optimize...
	std::cout<<" [ OK ] "<<std::endl;

  // pose constraint to pose 0
  std::shared_ptr<okvis::ceres::PoseError> poseError(
        new okvis::ceres::PoseError(T_WS0, 0.01, 0.0001));
  map.addResidualBlock(poseError, NULL, poseParameterBlock0);

	// set up a random camera geometry
  std::cout << "set up a random camera geometry... " << std::flush;
  typedef okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>
      DistortedPinholeCameraGeometry;
  std::shared_ptr<const DistortedPinholeCameraGeometry> cameraGeometry =
  std::static_pointer_cast<const DistortedPinholeCameraGeometry>(
        DistortedPinholeCameraGeometry::createTestObject());
  std::cout << " [ OK ] " << std::endl;

  // custom exception for tests:
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

	// get some random points and build error terms
	const size_t N=100;
  std::cout << "create N="<< N
            <<" visible points and add respective reprojection error terms... " << std::flush;
	for (size_t i=1; i<100; ++i){

	  Eigen::Vector4d point = cameraGeometry->createRandomVisibleHomogeneousPoint(double(i%10)*3+2.0);
    std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> homogeneousPointParameterBlock(
        new okvis::ceres::HomogeneousPointParameterBlock(T_SC*point,i+3));
    map.addParameterBlock(homogeneousPointParameterBlock,
                              okvis::ceres::Map::Parameterization::HomogeneousPoint);

    // get randomized projections
    Eigen::Vector2d kp0 = Eigen::Vector2d::Zero();
    auto status0 = cameraGeometry->projectHomogeneous(point,&kp0);
    kp0 += Eigen::Vector2d::Random();

    Eigen::Vector2d kp1 = Eigen::Vector2d::Zero();
    auto status1 = cameraGeometry->projectHomogeneous(T_C0C1.inverse()*point,&kp1);
    kp0 += Eigen::Vector2d::Random();

    // Set up the reprojection errors into both cameras.
	  Eigen::Matrix2d information=Eigen::Matrix2d::Identity();
    std::shared_ptr<::ceres::CostFunction> reprojectionError0(new
        okvis::ceres::RelativeReprojectionError<DistortedPinholeCameraGeometry,true>(
        cameraGeometry,1, kp0,information));
    std::shared_ptr<::ceres::CostFunction> reprojectionError1(new
        okvis::ceres::RelativeReprojectionError<DistortedPinholeCameraGeometry,false>(
        cameraGeometry,1, kp1,information));
    ::ceres::ResidualBlockId id0, id1;
    if(status0 == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
      id0 = map.addResidualBlock(reprojectionError0, NULL,
                                 homogeneousPointParameterBlock,
                                 extrinsicsParameterBlock);
      // verify there are no errors in the Jacobians
      OKVIS_ASSERT_TRUE(Exception,map.isJacobianCorrect(id0),"Jacobian(s) incorrect");
      //std::cout << "id0 ok"<<std::endl;
    }
    if(status1 == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
      id1 = map.addResidualBlock(reprojectionError1, NULL, poseParameterBlock1,
                                 homogeneousPointParameterBlock,
                                 extrinsicsParameterBlock,
                                 poseParameterBlock0);
      //std::cout << "est " << poseParameterBlock1->estimate().q().coeffs().transpose() <<std::endl;
      // verify there are no errors in the Jacobians
      OKVIS_ASSERT_TRUE(Exception,map.isJacobianCorrect(id1),"Jacobian(s) incorrect");
      //std::cout << "id1 ok"<<std::endl;
    }
  }
	std::cout<<" [ OK ] "<<std::endl;

	// Run the solver!
	std::cout<<"run the solver... "<<std::endl;
  map.options.minimizer_progress_to_stdout = true;
  map.solve();

	// print some infos about the optimization
  std::cout << map.summary.BriefReport() << "\n";
  std::cout << "initial T_WS : " << T_WS1_init.T() << "\n"
      << "optimized T_WS : " << poseParameterBlock1->estimate().T() << "\n"
      << "correct T_WS : " << T_WS1.T() << "\n";

	// make sure it converged
  OKVIS_ASSERT_TRUE(Exception,
                    2*(T_WS1.q()*poseParameterBlock1->estimate().q().inverse()).vec().norm()<1e-2,
                    "quaternions not close enough");
  OKVIS_ASSERT_TRUE(Exception,
                    (T_WS1.r()-poseParameterBlock1->estimate().r()).norm()<1e-1,
                    "translation not close enough");
}
