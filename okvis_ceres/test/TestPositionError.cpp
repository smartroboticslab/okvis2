/*
 * TestPositionError.cpp
 *
 *  Created on: 25 Jul 2015
 *      Author: lestefan
 */

#include "glog/logging.h"
#include "ceres/ceres.h"
#include <gtest/gtest.h>
#include <okvis/ceres/Map.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PositionError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, PositionError){
  // initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  // check errors
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error);

  // the optimisation problem
  okvis::ceres::Map map;

  // set the imu parameters
  okvis::ImuParameters imuParameters;
  imuParameters.a0.setZero();
  imuParameters.g = 9.81;
  imuParameters.a_max = 1000.0;
  imuParameters.g_max = 1000.0;
  imuParameters.rate = 1000; // 1 kHz
  imuParameters.sigma_g_c = 6.0e-4;
  imuParameters.sigma_a_c = 2.0e-3;
  imuParameters.sigma_gw_c = 3.0e-6;
  imuParameters.sigma_aw_c = 2.0e-5;
  imuParameters.tau = 3600.0;

  // actual speed
  Eigen::Vector3d v_W(10.0, 0.0, 0.0);

  // actual biases
  Eigen::Vector3d b_g = Eigen::Vector3d(0.0001,-0.0002,0.0003);
  Eigen::Vector3d b_a = Eigen::Vector3d(-0.001,0.002,0.003);

  // the ground truth:
  const double Dt = 1.02; // [sec]
  const double dt = 0.005; // 200 Hz
  const double omega = 2 * M_PI / (Dt-0.02); // one revolution in Dt
  const okvis::kinematics::Transformation T_WS_0;
  const okvis::kinematics::Transformation T_WS_1(v_W*(Dt-0.02),Eigen::Quaterniond::Identity());
  okvis::kinematics::Transformation T_GW;
  T_GW.setRandom(100.0,2*M_PI);
  okvis::SpeedAndBias zeroSpeedAndBias = okvis::SpeedAndBias::Zero();
  zeroSpeedAndBias.head<3>() = v_W;
  const Eigen::Vector3d p_SP_S(1.0, 0.5, -0.2);
  const Eigen::Matrix3d positionMeasurementCovariance = Eigen::Matrix3d::Identity()*1.0e-4;
  const okvis::PositionSensorParameters positionSensorParameters = {p_SP_S, false};

  // create parameter blocks
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock0(
       new okvis::ceres::PoseParameterBlock(T_WS_0, 1, okvis::Time(0.01)));
  std::shared_ptr<okvis::ceres::SpeedAndBiasParameterBlock> speedAndBiasParameterBlock0(
         new okvis::ceres::SpeedAndBiasParameterBlock(zeroSpeedAndBias, 2, okvis::Time(0.01)));
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock1(
       new okvis::ceres::PoseParameterBlock(T_WS_1, 3, okvis::Time(Dt-0.01)));
  std::shared_ptr<okvis::ceres::SpeedAndBiasParameterBlock> speedAndBiasParameterBlock1(
       new okvis::ceres::SpeedAndBiasParameterBlock(zeroSpeedAndBias, 4, okvis::Time(Dt-0.01)));
  std::shared_ptr<okvis::ceres::PoseParameterBlock> alignmentParameterBlock(
        new okvis::ceres::PoseParameterBlock(okvis::kinematics::Transformation(), 5, okvis::Time(0.0)));

  // add to map
  map.addParameterBlock(poseParameterBlock0, okvis::ceres::Map::Pose4d);
  map.setParameterBlockConstant(poseParameterBlock0);
  map.addParameterBlock(speedAndBiasParameterBlock0);
  std::shared_ptr<::ceres::CostFunction> speedAndBiasError(
      new okvis::ceres::SpeedAndBiasError(zeroSpeedAndBias,0.0001,0.0001,0.01));
  map.addResidualBlock(speedAndBiasError,NULL,speedAndBiasParameterBlock0);
  map.addParameterBlock(poseParameterBlock1, okvis::ceres::Map::Pose6d);
  map.addParameterBlock(speedAndBiasParameterBlock1);
  map.addParameterBlock(alignmentParameterBlock, okvis::ceres::Map::Pose6d);

  // generate the measurements
  okvis::ImuMeasurementDeque imuMeasurements;
  int ctr = 0;
  ceres::ResidualBlockId positionMeasurementId = 0;
  for(double t = 0; t < Dt; t+=dt) {
    // current orientation:
    const double alpha = omega*t;
    Eigen::Matrix3d C_WS;
    C_WS <<
        cos(alpha), -sin(alpha), 0.0,
        sin(alpha),  cos(alpha), 0.0,
               0.0,         0.0, 1.0;
    // generate IMU measurements
    Eigen::Vector3d gyr = Eigen::Vector3d(0.0,0.0,omega) + b_g
        + imuParameters.sigma_g_c/sqrt(dt) * Eigen::Vector3d::Random();
    Eigen::Vector3d acc = C_WS.inverse()*(Eigen::Vector3d(0,0,imuParameters.g)) + b_a
        + imuParameters.sigma_a_c/sqrt(dt) * Eigen::Vector3d::Random();
    imuMeasurements.push_back(okvis::ImuMeasurement(okvis::Time(t),okvis::ImuSensorReadings(gyr, acc)));
    //std::cout << gyr.transpose() << " " << acc.transpose() << std::endl;
    // generate position measurements - 20 Hz only
    if(ctr % 10 == 0 && t>0.01 && t<Dt-0.01){
      Eigen::Vector3d r_P_W = T_GW.r() + T_GW.C() * ( t*v_W + C_WS*p_SP_S);
      std::shared_ptr<::ceres::CostFunction> positionError(new okvis::ceres::PositionError(
          r_P_W, positionMeasurementCovariance, positionSensorParameters,
          imuMeasurements, imuParameters, okvis::Time(0.01), okvis::Time(t)));
      //std::cout << r_P_W.transpose() << std::endl;
      positionMeasurementId = map.addResidualBlock(positionError, NULL,
                             poseParameterBlock0, speedAndBiasParameterBlock0, alignmentParameterBlock);
    }
    ctr++;
  }

  // create and add an IMU error
  std::shared_ptr<::ceres::CostFunction> imuError(new okvis::ceres::ImuError(
      imuMeasurements, imuParameters, okvis::Time(0.01), okvis::Time(Dt-0.01)));
  ceres::ResidualBlockId imuId = map.addResidualBlock(imuError, NULL,
                       poseParameterBlock0, speedAndBiasParameterBlock0,
                       poseParameterBlock1, speedAndBiasParameterBlock1);

  // solve
  std::cout << "run the solver... " << std::endl;
  map.options.trust_region_strategy_type = ::ceres::DOGLEG;
  map.options.minimizer_progress_to_stdout = true;
  map.options.max_num_iterations = 100;
  FLAGS_stderrthreshold = google::INFO;  // enable console warnings (Jacobian verification)
  map.solve();

  // check Jacobians
  OKVIS_ASSERT_TRUE(Exception, map.isJacobianCorrect(imuId, 1.0e-3),
                    "IMU error Jacobian did not verify");
  OKVIS_ASSERT_TRUE(Exception, map.isJacobianCorrect(positionMeasurementId, 1.0e-3),
                    "Position error Jacobian did not verify");

  // check convergence
  /*std::cout << poseParameterBlock0->estimate().T() << std::endl;
  std::cout << poseParameterBlock1->estimate().T() << std::endl;
  std::cout << speedAndBiasParameterBlock1->estimate().transpose() << std::endl;
  std::cout << T_GW.T() << std::endl;
  std::cout << alignmentParameterBlock->estimate().T() << std::endl;*/

  OKVIS_ASSERT_TRUE(Exception,
                    2*(T_GW.q()*alignmentParameterBlock->estimate().q().inverse()).vec().norm()<1e-2,
                    "quaternions not close enough");
  OKVIS_ASSERT_TRUE(Exception,
                    (T_GW.r()-alignmentParameterBlock->estimate().r()).norm()<1e-1,
                    "translation not close enough");

}
