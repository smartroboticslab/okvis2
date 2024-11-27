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
 * @file TrajectoryOutput.cpp
 * @brief Source file for the TrajectoryOutput class.
 * @author Stefan Leutenegger
 */

#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/TrajectoryOutput.hpp>


okvis::TrajectoryOutput::TrajectoryOutput(bool draw) : draw_(draw) {
  if(draw_) {
    _image.create(_imageSize, _imageSize, CV_8UC3);
    _image.setTo(cv::Scalar(0,0,0));
  }
}

okvis::TrajectoryOutput::TrajectoryOutput(const std::string &filename, bool rpg, bool draw)
    : draw_(draw) {
  setCsvFile(filename, rpg);
  if(draw_) {
    _image.create(_imageSize, _imageSize, CV_8UC3);
    _image.setTo(cv::Scalar(0,0,0));
  }
}

void okvis::TrajectoryOutput::setCsvFile(const std::string &filename, bool rpg) {
  rpg_ = rpg;
  createCsvFile(filename, csvFile_, rpg);
}

void okvis::TrajectoryOutput::setRGBCsvFile(const std::string & filename) {
  createCsvFile(filename, rgbCsvFile_, rpg_);
}

void okvis::TrajectoryOutput::processState(
    const State& state, const TrackingState & trackingState,
    std::shared_ptr<const AlignedMap<StateId, State>> updatedStates,
    std::shared_ptr<const MapPointVector> landmarks) {
  writeStateToCsv(csvFile_, state, rpg_);

  if(draw_ && !updatedStates->empty()) {
    std::shared_ptr<GraphStates> statePtr(
          new GraphStates{state, trackingState, updatedStates, landmarks});
    states_.PushBlockingIfFull(statePtr, 1000);
  }
}

bool okvis::TrajectoryOutput::addImuMeasurement(
  const okvis::Time &stamp, const Eigen::Vector3d &alpha, const Eigen::Vector3d &omega)
{
  if(draw_) {
    return !imuMeasurements_.PushNonBlockingDroppingIfFull(
      ImuMeasurement(stamp, ImuSensorReadings(omega,alpha)), 1000);
  }
  return false;
}

bool okvis::TrajectoryOutput::processRGBImage(const okvis::Time& timestamp, const cv::Mat& image) {
  if (!rgbCsvFile_) {
    LOG(WARNING) << "Cannot write RGB trajectory, since no CSV file is set";
    return false;
  }
  rgbTrajectory_.enqueue(image, timestamp);
  auto state_pairs = rgbTrajectory_.getStates(trajectory_);
  for (auto state_pair : state_pairs) {
    writeStateToCsv(rgbCsvFile_, state_pair.second, rpg_);
  }
  return true;
}

void okvis::TrajectoryOutput::drawTopView(cv::Mat& outImg) {
  std::shared_ptr<GraphStates> graphStates;
  while(states_.PopNonBlocking(&graphStates)) {

    // do the update here to avoid race conditions
    std::set<okvis::StateId> affectedStateIds;
    trajectory_.update(std::get<1>(*graphStates), std::get<2>(*graphStates), affectedStateIds);
    State state = std::get<0>(*graphStates);

    // also propagate IMU, if requested
    if(states_.Empty()) {
      ImuMeasurement imuMeasurement;
      while(imuMeasurements_.PopNonBlocking(&imuMeasurement)) {
        State propagatedState;
        if(trajectory_.addImuMeasurement(imuMeasurement.timeStamp,
                                      imuMeasurement.measurement.accelerometers,
                                      imuMeasurement.measurement.gyroscopes,
                                      propagatedState)) {
          state = propagatedState;
        }
      }
    }

    // append the path
    Eigen::Vector3d r = state.T_WS.r();
    Eigen::Matrix3d C = state.T_WS.C();
    _path.push_back(cv::Point2d(r[0], r[1]));
    _heights.push_back(r[2]);

    // maintain scaling
    if (r[0] - _frameScale < _min_x)
      _min_x = r[0] - _frameScale;
    if (r[1] - _frameScale < _min_y)
      _min_y = r[1] - _frameScale;
    if (r[2] < _min_z)
      _min_z = r[2];
    if (r[0] + _frameScale > _max_x)
      _max_x = r[0] + _frameScale;
    if (r[1] + _frameScale > _max_y)
      _max_y = r[1] + _frameScale;
    if (r[2] > _max_z)
      _max_z = r[2];
    _scale = std::min(_imageSize / (_max_x - _min_x), _imageSize / (_max_y - _min_y));

    // reset image
    _image.setTo(cv::Scalar(10, 10, 10));

    // First the landmarks
    const MapPointVector& landmarks = *std::get<3>(*graphStates);
    for(const auto & lm : landmarks) {
      if(fabs(lm.point[3])<1e-12) continue;
      if(lm.quality<0.0001) continue;
      Eigen::Vector3d pt3d = lm.point.head<3>()/lm.point[3];
      cv::Point2d pt = convertToImageCoordinates(cv::Point2d(pt3d[0], pt3d[1]));
      if (pt.x<0 || pt.y<0 || pt.x > _imageSize-1 || pt.y > _imageSize-1) continue;
      const double colourScale = std::min(1.0,lm.quality/0.1);
      cv::circle(_image, pt, 1.0, colourScale*cv::Scalar(0,255,0), cv::FILLED, cv::LINE_AA);
    }

    // draw the path
    drawPath();

    // draw non-causal trajectory
    drawPathNoncausal();

    // draw IMU frame axes
    Eigen::Vector3d e_x = C.col(0);
    Eigen::Vector3d e_y = C.col(1);
    Eigen::Vector3d e_z = C.col(2);
    cv::line(
          _image,
          convertToImageCoordinates(_path.back()),
          convertToImageCoordinates(
            _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
        cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::line(
          _image,
          convertToImageCoordinates(_path.back()),
          convertToImageCoordinates(
            _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(
          _image,
          convertToImageCoordinates(_path.back()),
          convertToImageCoordinates(
            _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
        cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

    // some text:
    std::stringstream postext;
    postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
    cv::putText(_image, postext.str(), cv::Point(15,15),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    std::stringstream veltext;
    veltext << "velocity = [" << state.v_W[0] << ", " << state.v_W[1] << ", " << state.v_W[2]
            << "]";
    cv::putText(_image, veltext.str(), cv::Point(15,35),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    std::stringstream gyroBiasText;
    gyroBiasText << "gyro bias = [" << state.b_g[0] << ", " << state.b_g[1] << ", " << state.b_g[2]
                 << "]";
    cv::putText(_image, gyroBiasText.str(), cv::Point(15,55),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    std::stringstream accBiasText;
    accBiasText << "acc bias = [" << state.b_a[0] << ", " << state.b_a[1] << ", " << state.b_a[2]
                << "]";
    cv::putText(_image, accBiasText.str(), cv::Point(15,75),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);

    // Quality
    const TrackingState& tracking = std::get<1>(*graphStates);
    cv::Scalar trackingColour;
    if(tracking.trackingQuality == TrackingQuality::Lost) {
      trackingColour = cv::Scalar(0,0,255);
      cv::putText(_image, "TRACKING LOST", cv::Point2f(5,_imageSize-10), cv::FONT_HERSHEY_COMPLEX,
                  0.3, trackingColour, 1, cv::LINE_AA);
    } else if (tracking.trackingQuality == TrackingQuality::Marginal) {
      trackingColour = cv::Scalar(0,255,255);
      cv::putText(_image, "Tracking marginal", cv::Point2f(5,_imageSize-10),
                  cv::FONT_HERSHEY_COMPLEX, 0.3, trackingColour, 1, cv::LINE_AA);
    } else {
      trackingColour = cv::Scalar(0,255,0);
      cv::putText(_image, "Tracking good", cv::Point2f(5,_imageSize-10), cv::FONT_HERSHEY_COMPLEX,
                  0.3, trackingColour, 1, cv::LINE_AA);
    }
    if(tracking.recognisedPlace) {
      cv::putText(_image, "Recognised place", cv::Point2f(160,_imageSize-10),
                  cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255,0,0), 1, cv::LINE_AA);
    }

    // hack: agent i pose / world frame axes
    for (const auto &T_AW : state.T_AiW) {
      _T_AW = T_AW.second;
    }
    e_x = _T_AW.inverse().C().col(0);
    e_y = _T_AW.inverse().C().col(1);
    e_z = _T_AW.inverse().C().col(2);
    cv::Point2d r_WA(_T_AW.inverse().r()[0], _T_AW.inverse().r()[1]);
    cv::line(
      _image,
      convertToImageCoordinates(r_WA),
      convertToImageCoordinates(r_WA + cv::Point2d(e_x[0], e_x[1]) * 2.0 * _frameScale),
      cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::line(
      _image,
      convertToImageCoordinates(r_WA),
      convertToImageCoordinates(r_WA + cv::Point2d(e_y[0], e_y[1]) * 2.0 *_frameScale),
      cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(
      _image,
      convertToImageCoordinates(r_WA),
      convertToImageCoordinates(r_WA + cv::Point2d(e_z[0], e_z[1]) * 2.0 *_frameScale),
      cv::Scalar(255, 0, 0), 1, cv::LINE_AA);


    // output
    outImg = _image;
  }
}

cv::Point2d okvis::TrajectoryOutput::convertToImageCoordinates(
  const cv::Point2d &pointInMeters) const
{
  cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
  return cv::Point2d(pt.x, _imageSize - pt.y); // reverse y for more intuitive top-down plot
}

void okvis::TrajectoryOutput::drawPath()
{
  for (size_t i = 0; i + 1 < _path.size(); ) {
    cv::Point2d p0 = convertToImageCoordinates(_path[i]);
    cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
    cv::Point2d diff = p1-p0;
    if(diff.dot(diff)<2.0){
      _path.erase(_path.begin() + i + 1);  // clean short segment
      _heights.erase(_heights.begin() + i + 1);
      continue;
    }
    cv::line(_image, p0, p1, cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    i++;
  }
}

void okvis::TrajectoryOutput::drawPathNoncausal()
{
  std::vector<cv::Point2d> path; // Path in 2d.
  std::vector<double> heights; // Heights on the path.
  std::vector<bool> isKeyframe; // Keyframe status on the path.
  auto stateIds = trajectory_.stateIds();
  for(const auto & id : stateIds) {
    State state;
    trajectory_.getState(id, state);
    Eigen::Vector3d r = state.T_WS.r();
    path.push_back(cv::Point2d(r[0],r[1]));
    heights.push_back(r[2]);
    isKeyframe.push_back(state.isKeyframe);

    // draw covisibilities, too

    if (state.isKeyframe) {
      cv::Point2d p0 = convertToImageCoordinates(cv::Point2d(r[0],r[1]));
      for (const auto &id : state.covisibleFrameIds) {
        State state1;
        trajectory_.getState(id, state1);
        Eigen::Vector3d r1 = state1.T_WS.r();
        cv::Point2d p1 = convertToImageCoordinates(cv::Point2d(r1[0],r1[1]));
        cv::line(_image, p0, p1, cv::Scalar(0, 127, 127), 1, cv::LINE_AA);
      }
    }
  }
 
  for (size_t i = 0, j = 1; j < path.size(); ) {
    cv::Point2d p0 = convertToImageCoordinates(path[i]);
    cv::Point2d p1 = convertToImageCoordinates(path[j]);
    if(isKeyframe[j]) {
      cv::circle(_image, p1, 1, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);
    }
    if(i==0) {
      cv::circle(_image, p0, 1, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);
    }
    cv::Point2d diff = p1-p0;
    if(diff.dot(diff)<2.0){
      ++j;
      continue;
    }
    double rel_height = (heights[i] - _min_z + heights[j] - _min_z)
                        * 0.5 / (_max_z - _min_z);
    cv::line(_image, p0, p1, rel_height * cv::Scalar(255, 0, 0)
            + (1.0 - rel_height) * cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    i = j;
    j++;
  }
}

bool okvis::TrajectoryOutput::createCsvFile(
  const std::string &filename, std::fstream& stream, const bool rpg) {
  stream.open(filename.c_str(), std::ios_base::out);
  OKVIS_ASSERT_TRUE(Exception, stream.good(), "couldn't create trajectory file at " << filename)
  if(rpg) {
    stream << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  } else {
    stream << "timestamp" << ", " << "p_WS_W_x" << ", " << "p_WS_W_y" << ", "
           << "p_WS_W_z" << ", " << "q_WS_x" << ", " << "q_WS_y" << ", "
           << "q_WS_z" << ", " << "q_WS_w" << ", " << "v_WS_W_x" << ", "
           << "v_WS_W_y" << ", " << "v_WS_W_z" << ", " << "b_g_x" << ", "
           << "b_g_y" << ", " << "b_g_z" << ", " << "b_a_x" << ", " << "b_a_y"
           << ", " << "b_a_z" << std::endl;
  }
  return true;
}


bool okvis::TrajectoryOutput::writeStateToCsv(
  std::fstream& csvFile, const okvis::State& state, const bool rpg) {
  if (!csvFile.good()) {
    return false;
  }
  Eigen::Vector3d p_WS_W = state.T_WS.r();
  Eigen::Quaterniond q_WS =state. T_WS.q();
  if(rpg) {
    csvFile << std::setprecision(19) << state.timestamp.toSec() << " "
             << p_WS_W[0] << " " << p_WS_W[1] << " " << p_WS_W[2] << " "
             << q_WS.x() << " " << q_WS.y() << " " << q_WS.z() << " " << q_WS.w() << std::endl;
  } else {
    std::stringstream time;
    time << state.timestamp.sec << std::setw(9)
         << std::setfill('0') <<  state.timestamp.nsec;
    csvFile << time.str() << ", " << std::scientific
            << std::setprecision(18)
            << p_WS_W[0] << ", " << p_WS_W[1] << ", " << p_WS_W[2] << ", "
            << q_WS.x() << ", " << q_WS.y() << ", " << q_WS.z() << ", " << q_WS.w() << ", "
            << state.v_W[0] << ", " << state.v_W[1] << ", " << state.v_W[2] << ", "
            << state.b_g[0] << ", " << state.b_g[1] << ", " << state.b_g[2] << ", "
            << state.b_a[0] << ", " << state.b_a[1] << ", " << state.b_a[2] << ", " << std::endl;
  }
  return true;
}
