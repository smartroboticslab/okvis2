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

#include <memory>
#include <glog/logging.h>

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

#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/ceres/HomogeneousPointError.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>

TEST(okvisTestSuite, HomogeneousPointError) {
  // initialize random number generator
  //srand((unsigned int) time(0)); // disabled: make unit tests deterministic...

  // Build the problem.
  ::ceres::Problem::Options problemOptions;
  problemOptions.manifold_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
      ::ceres::Ownership::TAKE_OWNERSHIP;
  ::ceres::Problem problem(problemOptions);
  okvis::ceres::HomogeneousPointManifold homogeneousPointManifold;

  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  std::vector<std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock>>
      homogeneousPointParameterBlocks(100);
  for (size_t i = 0; i < homogeneousPointParameterBlocks.size(); ++i) {
    // create point
    Eigen::Vector4d point;
    point.head<3>().setRandom();
    point *= 100;
    point[3] = 1.0;

    // create parameter block
    homogeneousPointParameterBlocks.at(i) =
          std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock>(
        new okvis::ceres::HomogeneousPointParameterBlock(point, i));
    // add it as optimizable thing.
    problem.AddParameterBlock(
          homogeneousPointParameterBlocks.at(i)->parameters(), 4, &homogeneousPointManifold);
    problem.SetParameterBlockVariable(homogeneousPointParameterBlocks.at(i)->parameters());

    // invent a point error
    okvis::ceres::HomogeneousPointError* homogeneousPointError =
        new okvis::ceres::HomogeneousPointError(
            homogeneousPointParameterBlocks.at(i)->estimate(), 0.1);

    // add it
    ::ceres::ResidualBlockId id = problem.AddResidualBlock(
        homogeneousPointError, nullptr, homogeneousPointParameterBlocks.at(i)->parameters());

    // disturb
    Eigen::Vector4d point_disturbed = point;
    point_disturbed.head<3>() += 0.2 * Eigen::Vector3d::Random();
    homogeneousPointParameterBlocks.at(i)->setEstimate(point_disturbed);

    // check Jacobian
    OKVIS_ASSERT_TRUE(Exception, okvis::ceres::jacobiansCorrect(&problem, id),
                   "Jacobian verification on homogeneous point error failed.")
  }

  // Run the solver!
  ::ceres::Solver::Options options;
  ::ceres::Solver::Summary summary;
  options.minimizer_progress_to_stdout = false;
  std::cout << "run the solver... " << std::endl;
  ::ceres::Solve(options, &problem, &summary);

  // print some infos about the optimization
  //std::cout << map.summary.BriefReport() << "\n";

  // check convergence. this must converge to zero, since it is not an overdetermined system.
  OKVIS_ASSERT_TRUE(
      Exception, summary.final_cost < 1.0e-10,
      "No convergence. this must converge to zero, since it is not an overdetermined system.")
}
