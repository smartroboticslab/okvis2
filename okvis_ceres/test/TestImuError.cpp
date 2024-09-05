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

#include "glog/logging.h"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ceres/cost_function.h>
#include <ceres/crs_matrix.h>
#include <ceres/evaluation_callback.h>
#include <ceres/iteration_callback.h>
#include <ceres/loss_function.h>
#include <ceres/manifold.h>
#include <ceres/ordered_groups.h>
#include <ceres/problem.h>
#include <ceres/product_manifold.h>
#include <ceres/sized_cost_function.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <ceres/version.h>
#pragma GCC diagnostic pop

#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/SpeedAndBiasParameterBlock.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

double sinc_test(double x){
	if(fabs(x)>1e-10) {
		return sin(x)/x;
	}
	else{
		static const double c_2=1.0/6.0;
		static const double c_4=1.0/120.0;
		static const double c_6=1.0/5040.0;
		const double x_2 = x*x;
		const double x_4 = x_2*x_2;
		const double x_6 = x_2*x_2*x_2;
		return 1.0 - c_2*x_2 + c_4*x_4 - c_6*x_6;
	}
}

TEST(okvisTestSuite, ImuError){
	// initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

	// Build the problem.
	::ceres::Problem problem;

  // check errors
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

	// set the imu parameters
	okvis::ImuParameters imuParameters;
	imuParameters.a0.setZero();
	imuParameters.g = 9.81;
	imuParameters.a_max = 1000.0;
	imuParameters.g_max = 1000.0;
  const double imuRate = 1000; // 1 kHz
  imuParameters.sigma_g_c = 6.0e-4;
	imuParameters.sigma_a_c = 2.0e-3;
	imuParameters.sigma_gw_c = 3.0e-6;
	imuParameters.sigma_aw_c = 2.0e-5;

	// generate random motion
	const double w_omega_S_x = Eigen::internal::random(0.1,10.0); // circular frequency
	const double w_omega_S_y = Eigen::internal::random(0.1,10.0); // circular frequency
	const double w_omega_S_z = Eigen::internal::random(0.1,10.0); // circular frequency
	const double p_omega_S_x = Eigen::internal::random(0.0,M_PI); // phase
	const double p_omega_S_y = Eigen::internal::random(0.0,M_PI); // phase
	const double p_omega_S_z = Eigen::internal::random(0.0,M_PI); // phase
	const double m_omega_S_x = Eigen::internal::random(0.1,1.0); // magnitude
	const double m_omega_S_y = Eigen::internal::random(0.1,1.0); // magnitude
	const double m_omega_S_z = Eigen::internal::random(0.1,1.0); // magnitude
	const double w_a_W_x = Eigen::internal::random(0.1,10.0);
	const double w_a_W_y = Eigen::internal::random(0.1,10.0);
	const double w_a_W_z = Eigen::internal::random(0.1,10.0);
	const double p_a_W_x = Eigen::internal::random(0.1,M_PI);
	const double p_a_W_y = Eigen::internal::random(0.1,M_PI);
	const double p_a_W_z = Eigen::internal::random(0.1,M_PI);
	const double m_a_W_x = Eigen::internal::random(0.1,10.0);
	const double m_a_W_y = Eigen::internal::random(0.1,10.0);
	const double m_a_W_z = Eigen::internal::random(0.1,10.0);

	// generate randomized measurements - duration 10 seconds
	const double duration = 1.0;
	okvis::ImuMeasurementDeque imuMeasurements;
	okvis::kinematics::Transformation T_WS;
	//T_WS.setRandom();

	// time increment
  const double dt=1.0/double(imuRate); // time discretization

	// states
	Eigen::Quaterniond q=T_WS.q();
	Eigen::Vector3d r=T_WS.r();
	okvis::SpeedAndBias speedAndBias;
	speedAndBias.setZero();
	Eigen::Vector3d v=speedAndBias.head<3>();

	// start
	okvis::kinematics::Transformation T_WS_0;
	okvis::SpeedAndBias speedAndBias_0;
	okvis::Time t_0;

	// end
	okvis::kinematics::Transformation T_WS_1;
	okvis::SpeedAndBias speedAndBias_1;
	okvis::Time t_1;

  for(size_t i=0; i<size_t(duration*imuRate); ++i){
    double time = double(i)/imuRate;
	  if (i==10){ // set this as starting pose
		  T_WS_0 = T_WS;
		  speedAndBias_0=speedAndBias;
		  t_0=okvis::Time(time);
	  }
    if (i==size_t(duration*imuRate)-10){ // set this as starting pose
		  T_WS_1 = T_WS;
		  speedAndBias_1=speedAndBias;
		  t_1=okvis::Time(time);
	  }

	  Eigen::Vector3d omega_S(m_omega_S_x*sin(w_omega_S_x*time+p_omega_S_x),
			  m_omega_S_y*sin(w_omega_S_y*time+p_omega_S_y),
			  m_omega_S_z*sin(w_omega_S_z*time+p_omega_S_z));
	  Eigen::Vector3d a_W(m_a_W_x*sin(w_a_W_x*time+p_a_W_x),
				  m_a_W_y*sin(w_a_W_y*time+p_a_W_y),
				  m_a_W_z*sin(w_a_W_z*time+p_a_W_z));

	  //omega_S.setZero();
	  //a_W.setZero();

	  Eigen::Quaterniond dq;

	  // propagate orientation
	  const double theta_half = omega_S.norm()*dt*0.5;
	  const double sinc_theta_half = sinc_test(theta_half);
	  const double cos_theta_half = cos(theta_half);
	  dq.vec()=sinc_theta_half*0.5*dt*omega_S;
	  dq.w()=cos_theta_half;
	  q = q * dq;

	  // propagate speed
	  v+=dt*a_W;

	  // propagate position
	  r+=dt*v;

	  // T_WS
	  T_WS = okvis::kinematics::Transformation(r,q);

	  // speedAndBias - v only, obviously, since this is the Ground Truth
	  speedAndBias.head<3>()=v;

	  // generate measurements
	  Eigen::Vector3d gyr = omega_S + imuParameters.sigma_g_c/sqrt(dt)
			  *Eigen::Vector3d::Random();
    Eigen::Vector3d acc = T_WS.inverse().C()*(a_W+Eigen::Vector3d(0,0,imuParameters.g))
        + imuParameters.sigma_a_c/sqrt(dt)
				*Eigen::Vector3d::Random();
    imuMeasurements.push_back(
          okvis::ImuMeasurement(okvis::Time(time),okvis::ImuSensorReadings(gyr,acc)));
	}

	// create the pose parameter blocks
	okvis::kinematics::Transformation T_disturb;
	T_disturb.setRandom(1,0.02);
	okvis::kinematics::Transformation T_WS_1_disturbed=T_WS_1*T_disturb; //
	okvis::ceres::PoseParameterBlock poseParameterBlock_0(T_WS_0,0,t_0); // ground truth
	okvis::ceres::PoseParameterBlock poseParameterBlock_1(T_WS_1_disturbed,2,t_1); // disturbed...
  problem.AddParameterBlock(poseParameterBlock_0.parameters(),
                            okvis::ceres::PoseParameterBlock::Dimension);
  problem.AddParameterBlock(poseParameterBlock_1.parameters(),
                            okvis::ceres::PoseParameterBlock::Dimension);
	//problem.SetParameterBlockConstant(poseParameterBlock_0.parameters());

	// create the speed and bias
	okvis::ceres::SpeedAndBiasParameterBlock speedAndBiasParameterBlock_0(speedAndBias_0,1,t_0);
	okvis::ceres::SpeedAndBiasParameterBlock speedAndBiasParameterBlock_1(speedAndBias_1,3,t_1);
  problem.AddParameterBlock(speedAndBiasParameterBlock_0.parameters(),
                            okvis::ceres::SpeedAndBiasParameterBlock::Dimension);
  problem.AddParameterBlock(speedAndBiasParameterBlock_1.parameters(),
                            okvis::ceres::SpeedAndBiasParameterBlock::Dimension);

	// let's use our own local quaternion perturbation
	std::cout<<"setting local parameterization for pose... "<<std::flush;
  ::ceres::Manifold* poseManifold = new okvis::ceres::PoseManifold;
  problem.SetManifold(poseParameterBlock_0.parameters(), poseManifold);
  problem.SetManifold(poseParameterBlock_1.parameters(), poseManifold);
	std::cout<<" [ OK ] "<<std::endl;

	// create the Imu error term
  okvis::ceres::ImuError* cost_function_imu =
      new okvis::ceres::ImuError(imuMeasurements, imuParameters,t_0, t_1);
  auto id = problem.AddResidualBlock(cost_function_imu, nullptr,
		  poseParameterBlock_0.parameters(), speedAndBiasParameterBlock_0.parameters(),
		  poseParameterBlock_1.parameters(), speedAndBiasParameterBlock_1.parameters());

	// let's also add some priors to check this alongside
  ::ceres::CostFunction* cost_function_pose =
      new okvis::ceres::PoseError(T_WS_0, 1e-12, 1e-4); // pose prior...
  problem.AddResidualBlock(cost_function_pose, nullptr,poseParameterBlock_0.parameters());
  ::ceres::CostFunction* cost_function_speedAndBias =
      new okvis::ceres::SpeedAndBiasError(speedAndBias_0, 1e-12, 1e-12, 1e-12); // speed/bias prior
  problem.AddResidualBlock(
        cost_function_speedAndBias, nullptr,speedAndBiasParameterBlock_0.parameters());

  // check Jacobians
  OKVIS_ASSERT_TRUE(
        Exception, okvis::ceres::jacobiansCorrect(&problem, id), "Jacobian verification failed")

	// Run the solver!
	std::cout<<"run the solver... "<<std::endl;
	::ceres::Solver::Options options;
  //options.check_gradients=true;
  //options.gradient_check_numeric_derivative_relative_step_size = 1e-6;
  //options.gradient_check_relative_precision=1e-6;
	options.minimizer_progress_to_stdout = false;
	::FLAGS_stderrthreshold=google::WARNING; // enable console warnings (Jacobian verification)
	::ceres::Solver::Summary summary;
	::ceres::Solve(options, &problem, &summary);

	// print some infos about the optimization
  //std::cout << summary.FullReport() << "\n";
	std::cout << "initial T_WS_1 : " << T_WS_1_disturbed.T() << "\n"
			<< "optimized T_WS_1 : " << poseParameterBlock_1.estimate().T() << "\n"
			<< "correct T_WS_1 : " << T_WS_1.T() << "\n";

	// make sure it converged
  OKVIS_ASSERT_TRUE(Exception, summary.final_cost<1e-2,"cost not reducible")
  OKVIS_ASSERT_TRUE(Exception,
                    2*(T_WS_1.q()*poseParameterBlock_1.estimate().q().inverse()).vec().norm()<1e-2,
                    "quaternions not close enough")
  OKVIS_ASSERT_TRUE(Exception,
                    (T_WS_1.r()-poseParameterBlock_1.estimate().r()).norm()<0.04,
                    "translation not close enough")
}



