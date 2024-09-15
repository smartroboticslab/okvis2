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
 * @file ViVisualizer.cpp
 * @brief Source file for the ViVisualizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */


#include <okvis/kinematics/Transformation.hpp>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/FrameTypedefs.hpp>

#include "okvis/ViVisualizer.hpp"

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

ViVisualizer::ViVisualizer(ViParameters &parameters)
    : parameters_(parameters) {
  if (parameters.nCameraSystem.numCameras() > 0) {
    init(parameters);
  }
}

ViVisualizer::~ViVisualizer() {
}

void ViVisualizer::init(ViParameters &parameters) {
  parameters_ = parameters;
}

cv::Mat ViVisualizer::drawMatches(VisualizationData::Ptr& data,
                                   size_t image_number) {

  const kinematics::Transformation T_WS = data->T_WS;
  const kinematics::Transformation T_SC = data->T_SCi.at(image_number);
  const kinematics::Transformation T_WC = T_WS*T_SC;
  const kinematics::Transformation T_CW = T_WC.inverse();

  std::shared_ptr<okvis::MultiFrame> frame = data->currentFrames;

  // allocate an image
  const int im_cols = frame->image(image_number).cols;
  const int im_rows = frame->image(image_number).rows;

  cv::Mat outimg(im_rows, im_cols, CV_8UC3);
  cv::Mat current = outimg;

  cv::cvtColor(frame->image(image_number), current, cv::COLOR_GRAY2BGR);

  // loop-closure frame?
  cv::Mat tmp(im_rows, im_cols, CV_8UC3);
  if(data->recognisedPlace) {
    cv::Scalar colour(255,0,0);
    tmp.setTo(colour);
    cv::addWeighted(tmp, 0.5, current, 1.0 - 0.5, 0, current);
    cv::putText(current, "Place recognised", cv::Point2f(200,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                colour, 1, cv::LINE_AA);
  }

  // keyframe?
  if(data->isKeyframe) {
    cv::Scalar colour(255,255,0);
    tmp.setTo(colour);
    cv::addWeighted(tmp, 0.3, current, 1.0 - 0.3, 0, current);
    cv::putText(current, "Keyframe", cv::Point2f(120,10), cv::FONT_HERSHEY_COMPLEX, 0.3, colour, 1,
                cv::LINE_AA);
  }

  // Quality
  cv::Scalar trackingColour;
  if(data->trackingQuality == VisualizationData::TrackingQuality::Lost) {
    trackingColour = cv::Scalar(0,0,255);
    cv::putText(current, "TRACKING LOST", cv::Point2f(5,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                trackingColour, 1, cv::LINE_AA);
  } else if (data->trackingQuality == VisualizationData::TrackingQuality::Marginal) {
    trackingColour = cv::Scalar(0,255,255);
    cv::putText(current, "Tracking marginal", cv::Point2f(5,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                trackingColour, 1, cv::LINE_AA);
  } else {
    trackingColour = cv::Scalar(0,255,0);
    cv::putText(current, "Tracking good", cv::Point2f(5,10), cv::FONT_HERSHEY_COMPLEX, 0.3,
                trackingColour, 1, cv::LINE_AA);
  }

  // find distortion type
  okvis::cameras::NCameraSystem::DistortionType distortionType = parameters_.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < parameters_.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == parameters_.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet")
  }

  for (auto it = data->observations.begin(); it != data->observations.end();
      ++it) {
    if (it->cameraIdx != image_number)
      continue;

    cv::Scalar color;

    if (it->landmarkId != 0) {
      color = cv::Scalar(255, 0, 0);  // blue
    } else {
      color = cv::Scalar(0, 0, 255);  // red
    }

    // draw matches
    auto keypoint = it->keypointMeasurement;
    if (fabs(it->landmark_W[3]) > 1.0e-8) {
      Eigen::Vector4d hPoint = it->landmark_W;
      if (it->isInitialized) {
        color = cv::Scalar(0, 255, 0);  // green
        if (it->classification == 10 || it->classification == 11) {
            color = cv::Scalar(0, 80, 0);  // dark green
        }
      } else {
        color = cv::Scalar(0, 255, 255);  // yellow
        if (it->classification == 10 || it->classification == 11) {
            color = cv::Scalar(0, 50, 50);  // dark yellew -> olive
        }
      }
      Eigen::Vector2d projection;
      bool isVisible = false;
      Eigen::Vector4d hP_C = T_CW * hPoint;
      switch (distortionType) {
        case okvis::cameras::NCameraSystem::RadialTangential: {
          if (frame
              ->geometryAs<
                  okvis::cameras::PinholeCamera<
                      okvis::cameras::RadialTangentialDistortion>>(image_number)
              ->projectHomogeneous(hP_C, &projection)
              == okvis::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        case okvis::cameras::NCameraSystem::Equidistant: {
          if (frame
              ->geometryAs<
                  okvis::cameras::PinholeCamera<
                      okvis::cameras::EquidistantDistortion>>(image_number)
              ->projectHomogeneous(hP_C, &projection)
              == okvis::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        case okvis::cameras::NCameraSystem::RadialTangential8: {
          if (frame
              ->geometryAs<
                  okvis::cameras::PinholeCamera<
                      okvis::cameras::RadialTangentialDistortion8>>(
              image_number)->projectHomogeneous(hP_C, &projection)
              == okvis::cameras::ProjectionStatus::Successful)
            isVisible = true;
          break;
        }
        default:
          OKVIS_THROW(Exception, "Unsupported distortion type.")
          break;
      }
      if (fabs(hP_C[3]) > 1.0e-8) {
        if (hP_C[2] / hP_C[3] < 0.03) {
          isVisible = false;
        }
      }

      // draw projection
      if(isVisible) {
        cv::Point2f projectionCv(projection[0], projection[1]);
        cv::line(current, projectionCv,
            cv::Point2f(float(keypoint[0]), float(keypoint[1])), color, 1, cv::LINE_AA);
        cv::circle(current, projectionCv, 2,
            cv::Scalar(255,255,0), cv::FILLED, cv::LINE_AA);
        std::stringstream idText;
        idText << it->landmarkId;
        cv::putText(current, idText.str(),
                    cv::Point2f(float(projection[0]), float(projection[1])) + cv::Point2f(10,5),
                     cv::FONT_HERSHEY_COMPLEX, 0.35, cv::Scalar(255,255,0), 1, cv::LINE_AA);
      }
    }
    // draw keypoint
    const double r = 0.5 * it->keypointSize;
    cv::circle(current, cv::Point2f(float(keypoint[0]), float(keypoint[1])), int(r), color, 1,
        cv::LINE_AA);
    cv::KeyPoint cvKpt;
    frame->getCvKeypoint(it->cameraIdx, it->keypointIdx, cvKpt);
    const float angle = cvKpt.angle / 180.0f * float(M_PI);
    cv::line(outimg, cvKpt.pt,
             cv::Point2f(cvKpt.pt.x + float(r) * cos(angle), cvKpt.pt.y + float(r) * sin(angle)),
             color, 1, cv::LINE_AA);
  }
  return outimg;
}


cv::Mat ViVisualizer::drawKeypoints(VisualizationData::Ptr& data,
                                     size_t cameraIndex) {

  std::shared_ptr<okvis::MultiFrame> currentFrames = data->currentFrames;
  cv::Mat currentImage = currentFrames->image(cameraIndex);

  cv::Mat outimg;
  cv::cvtColor(currentImage, outimg, cv::COLOR_GRAY2BGR);
  cv::Scalar greenColor(0, 255, 0);  // green

  cv::KeyPoint keypoint;
  for (size_t k = 0; k < currentFrames->numKeypoints(cameraIndex); ++k) {
    currentFrames->getCvKeypoint(cameraIndex, k, keypoint);

    const float radius = keypoint.size;
    const float angle = keypoint.angle / 180.0f * float(M_PI);

    cv::circle(outimg, keypoint.pt, int(radius), greenColor);
    cv::line(
        outimg,
        keypoint.pt,
        cv::Point2f(keypoint.pt.x + radius * cos(angle),
                    keypoint.pt.y - radius * sin(angle)),
        greenColor);
  }

  return outimg;
}

} /* namespace okvis */
