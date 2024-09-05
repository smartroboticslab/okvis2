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
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <gtest/gtest.h>
#include <okvis/ViGraphEstimator.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/RelativePoseError.hpp>
#include <okvis/assert_macros.hpp>


TEST(okvisTestSuite, ViGraph2) {
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  // different cases of camera extrinsics;
  for (size_t c = 0; c < 4; ++c) {

    OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

    const double DURATION = 10.0;  // 10 seconds motion
    const double IMU_RATE = 100.0;  // 1 kHz
    const double DT = 1.0 / IMU_RATE;  // time increments

    // set the imu parameters
    okvis::ImuParameters imuParameters;
    imuParameters.a0.setZero();
    imuParameters.g = 9.81;
    imuParameters.a_max = 1000.0;
    imuParameters.g_max = 1000.0;
    imuParameters.rate = 1000;  // 1 kHz
    imuParameters.sigma_g_c = 6.0e-4;
    imuParameters.sigma_a_c = 2.0e-3;
    imuParameters.sigma_gw_c = 3.0e-6;
    imuParameters.sigma_aw_c = 2.0e-5;
    imuParameters.sigma_bg = 0.01;
    imuParameters.sigma_ba = 0.01;
    imuParameters.tau = 3600.0;
    std::cout << "case " << c % 2 << ", " << c / 2 << std::endl;

    // let's generate a really stupid motion: constant translation
    okvis::SpeedAndBias speedAndBias;
    speedAndBias.setZero();
    speedAndBias.head<3>() = Eigen::Vector3d(1, 0, 0);
    okvis::ImuMeasurementDeque imuMeasurements;
    okvis::ImuSensorReadings nominalImuSensorReadings(
        Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, imuParameters.g));
    okvis::Time t0 = okvis::Time::now();
    for (size_t i = 0; i <= DURATION * IMU_RATE; ++i) {
      Eigen::Vector3d gyr = nominalImuSensorReadings.gyroscopes
          + Eigen::Vector3d::Random() * imuParameters.sigma_g_c * sqrt(DT);
      Eigen::Vector3d acc = nominalImuSensorReadings.accelerometers
          + Eigen::Vector3d::Random() * imuParameters.sigma_a_c * sqrt(DT);
      imuMeasurements.push_back(
          okvis::ImuMeasurement(t0 + okvis::Duration(DT * i),
                                okvis::ImuSensorReadings(gyr, acc)));
    }

    // camera extrinsics:
    std::shared_ptr<const okvis::kinematics::Transformation> T_SC_0(
        new okvis::kinematics::Transformation(Eigen::Vector3d(0,0.0,0),
                                              Eigen::Quaterniond(-sqrt(0.5),0,0,sqrt(0.5))));
    std::shared_ptr<const okvis::kinematics::Transformation> T_SC_1(
        new okvis::kinematics::Transformation(Eigen::Vector3d(0.1,0,0),
                                              Eigen::Quaterniond(-sqrt(0.5),0,0,sqrt(0.5))));

    // some parameters on how to do the online estimation:
    okvis::ExtrinsicsEstimationParameters extrinsicsEstimationParameters;
    extrinsicsEstimationParameters.sigma_absolute_translation = 1.0e-3
        * (c % 2);
    extrinsicsEstimationParameters.sigma_absolute_orientation = 1.0e-4
        * (c % 2);
    extrinsicsEstimationParameters.sigma_c_relative_translation = 1e-8
        * (c / 2);
    extrinsicsEstimationParameters.sigma_c_relative_orientation = 1e-7
        * (c / 2);

    // set up camera with intrinsics
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry0(
        okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());
    std::shared_ptr<const okvis::cameras::CameraBase> cameraGeometry1(
        okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>::createTestObject());

    // create an N-camera system:
    std::shared_ptr<okvis::cameras::NCameraSystem> cameraSystem(
        new okvis::cameras::NCameraSystem);
    cameraSystem->addCamera(T_SC_0, cameraGeometry0,
                            okvis::cameras::NCameraSystem::DistortionType::Equidistant);
    cameraSystem->addCamera(T_SC_1, cameraGeometry1,
                            okvis::cameras::NCameraSystem::DistortionType::Equidistant);

    // create an Estimator
    okvis::ViGraphEstimator viGraph2;

    // create landmark grid
    const okvis::kinematics::Transformation T_WS_0;
    std::vector<Eigen::Vector4d,
        Eigen::aligned_allocator<Eigen::Vector4d> > homogeneousPoints;
    std::vector<okvis::LandmarkId> lmIds;
    for (double x = -10.0; x <= DURATION * speedAndBias[1] + 10.0; x += 0.5) {
      for (double z = -10.0; z <= 10.0; z += 0.5) {
        homogeneousPoints.push_back(Eigen::Vector4d(x, 3.0, z, 1));
        lmIds.push_back(viGraph2.addLandmark(homogeneousPoints.back(), true));
      }
    }

    // add sensors
    viGraph2.addCamera(extrinsicsEstimationParameters);
    viGraph2.addCamera(extrinsicsEstimationParameters);
    viGraph2.addImu(imuParameters);

    const size_t K = 8;
    okvis::StateId id;
    okvis::kinematics::Transformation T_WS_est;
    okvis::SpeedAndBias speedAndBias_est;
    for (size_t k = 0; k < K + 1; ++k) {
      // calculate the transformation
      okvis::kinematics::Transformation T_WS(
          T_WS_0.r() + speedAndBias.head<3>() * double(k) * DURATION / double(K),
          T_WS_0.q());

      // assemble a multi-frame
      std::shared_ptr<okvis::MultiFrame> mf(new okvis::MultiFrame);
      mf->setTimestamp(t0 + okvis::Duration(double(k) * DURATION / double(K)));

      // add frames
      mf->resetCameraSystemAndFrames(*cameraSystem);

      // add it in the window to create a new time instance
      if(k==0) {
        id = viGraph2.addStatesInitialise(mf->timestamp(), imuMeasurements, *cameraSystem);
      } else {
        const bool isKeyframe = (k%3 == 0); // every third as keyframe
        id = viGraph2.addStatesPropagate(mf->timestamp(), imuMeasurements, isKeyframe);
      }
      std::cout << "Frame " << k << " successfully added. ID="<< id.value() << std::endl;
      mf->setId(id.value());

      T_WS_est = viGraph2.pose(okvis::StateId(mf->id()));

      // now let's add also landmark observations
      std::vector<cv::KeyPoint> keypoints;
      for (size_t j = 0; j < homogeneousPoints.size(); ++j) {
        for (size_t i = 0; i < mf->numFrames(); ++i) {
          Eigen::Vector2d projection;
          Eigen::Vector4d point_C = mf->T_SC(i)->inverse()
              * T_WS.inverse() * homogeneousPoints[j];
          okvis::cameras::CameraBase::ProjectionStatus status = mf
              ->geometryAs<
                  okvis::cameras::PinholeCamera<
                      okvis::cameras::EquidistantDistortion>>(i)->projectHomogeneous(
              point_C, &projection);
          if (status == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
            Eigen::Vector2d measurement(projection + Eigen::Vector2d::Random());
            keypoints.push_back(cv::KeyPoint(static_cast<float>(measurement[0]),
                                static_cast<float>(measurement[1]), 8.0));
            mf->resetKeypoints(i,keypoints);
            okvis::KeypointIdentifier kid(mf->id(), i, mf->numKeypoints(i)-1);
            viGraph2.addObservation<
                okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion>>(
                  *mf, lmIds[j], kid);
          }
        }
      }
      // run the optimization
      viGraph2.optimise(2, 4, true);

      // also test the elimination of non-keyframes and posegraph converting
      if(k==7) {
        viGraph2.removeAllObservations(okvis::StateId(1));
        viGraph2.eliminateStateByImuMerge(okvis::StateId(1));
        viGraph2.removeAllObservations(okvis::StateId(2));
        viGraph2.eliminateStateByImuMerge(okvis::StateId(2));
        std::set<okvis::StateId> ids{okvis::StateId(1),okvis::StateId(3)};
        std::set<okvis::StateId> idsConsidered{okvis::StateId(1),okvis::StateId(2),
              okvis::StateId(3), okvis::StateId(4)};
        viGraph2.convertToPoseGraphMst(ids, idsConsidered);
      }
    }

    // run the optimization
    std::cout << "== LAST OPTIMIZATION ==" << std::endl;
    viGraph2.optimise(10, 4, true);
    //std::cout << viGraph2.summary().FullReport() << std::endl;

    // get the estimates
    T_WS_est = viGraph2.pose(okvis::StateId(id));
    speedAndBias_est = viGraph2.speedAndBias(okvis::StateId(id));

    // inspect convergence:
    okvis::kinematics::Transformation T_WS(
         T_WS_0.r() + speedAndBias.head<3>() * DURATION,
         T_WS_0.q());

    std::cout << "estimated T_WS: " << std::endl << T_WS_est.T() << std::endl;
    std::cout << "correct T_WS: " << std::endl << T_WS.T() << std::endl;

    OKVIS_ASSERT_TRUE(Exception, (speedAndBias_est - speedAndBias).norm() < 0.04,
                      "speed and biases not close enough")
    OKVIS_ASSERT_TRUE(Exception, 2*(T_WS.q()*T_WS_est.q().inverse()).vec().norm()<1e-2,
                      "quaternions not close enough")
    OKVIS_ASSERT_TRUE(Exception, (T_WS.r() - T_WS_est.r()).norm()<1e-1,
                      "translation not close enough")
  }
}
