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
 * @file ThreadedSlam.cpp
 * @brief Source file for the ThreadedSlam3 class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <map>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

#include <okvis/ThreadedSlam.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ImuError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis
{

static const int cameraInputQueueSize = 2;

// overlap of imu data before and after two consecutive frames [seconds]:
static const double imuTemporalOverlap = 0.02;


// Constructor.
ThreadedSlam::ThreadedSlam(ViParameters &parameters, std::string dBowDir) :
  visualizer_(parameters),
  hasStarted_(false),
  frontend_(parameters.nCameraSystem.numCameras(), dBowDir),
  parameters_(parameters)
{
  setBlocking(false);
  init();

  ///// HACK: multi-session and multi-agent //////
  //frontend_.loadComponent(
  //  "/Users/leuteneg/Documents/datasets/euroc/V2_03_difficult/mav0/okvis2-slam-final_map.g2o",
  //  parameters_.imu, parameters_.nCameraSystem);
  ////////////////////////////////////////////////
}

// Initialises settings and calls startThreads().
void ThreadedSlam::init()
{
  assert(parameters_.nCameraSystem.numCameras() > 0);
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  shutdown_ = false;

  // setup frontend
  frontend_.setBriskDetectionOctaves(size_t(parameters_.frontend.octaves));
  frontend_.setBriskDetectionThreshold(parameters_.frontend.detection_threshold);
  frontend_.setBriskDetectionAbsoluteThreshold(parameters_.frontend.absolute_threshold);
  frontend_.setBriskMatchingThreshold(parameters_.frontend.matching_threshold);
  frontend_.setBriskDetectionMaximumKeypoints(size_t(parameters_.frontend.max_num_keypoints));
  frontend_.setKeyframeInsertionOverlapThreshold(float(parameters_.frontend.keyframe_overlap));

  // setup estimator
  estimator_.addImu(parameters_.imu);
  for (size_t im = 0; im < numCameras; ++im) {
    // parameters_.camera_extrinsics is never set (default 0's)...
    // do they ever change?
    estimator_.addCamera(parameters_.camera);
  }
  estimator_.setDetectorUniformityRadius(parameters_.frontend.detection_threshold); 

  // time limit if requested
  if(parameters_.estimator.enforce_realtime) {
    estimator_.setOptimisationTimeLimit(
          parameters_.estimator.realtime_time_limit,
          parameters_.estimator.realtime_min_iterations);
  }

  startThreads();
}

// Start all threads.
void ThreadedSlam::startThreads()
{

  // visualisation
  if(parameters_.output.display_matches) {
    visualisationThread_ = std::thread(&ThreadedSlam::visualisationLoop, this);
  }

  //setMainThreadPriority(SCHED_RR, 99);

  // publishing
  publishingThread_ = std::thread(&ThreadedSlam::publishingLoop, this);

}

// Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
ThreadedSlam::~ThreadedSlam()
{

  // shutdown and join threads
  stopThreading();
}

// Add a new image.
bool ThreadedSlam::addImages(const okvis::Time & stamp,
                             const std::map<size_t, cv::Mat> & images,
                             const std::map<size_t, cv::Mat> & depthImages)
{
  // assemble frame
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  std::vector<okvis::CameraMeasurement> frames(numCameras);
  frames.at(0).timeStamp = stamp; // slight hack -- always have the timestamp here.

  bool useFrame = false;
  for(const auto & image : images) {
    if(parameters_.nCameraSystem.isCameraConfigured(image.first) && 
       parameters_.nCameraSystem.cameraType(image.first).isUsed) {
      if(image.second.channels()==1) {
        frames.at(image.first).measurement.image = image.second;
      } else {
        cv::cvtColor(image.second, frames.at(image.first).measurement.image, cv::COLOR_BGR2GRAY);
      }
      frames.at(image.first).timeStamp = stamp;
      frames.at(image.first).sensorId = image.first;
      frames.at(image.first).measurement.deliversKeypoints = false;
      useFrame = true;
    }
  }

  for(const auto & depthImage : depthImages) {
    if(parameters_.nCameraSystem.cameraType(depthImage.first).isUsed) {
      frames.at(depthImage.first).measurement.depthImage = depthImage.second;
      frames.at(depthImage.first).timeStamp = stamp;
      frames.at(depthImage.first).sensorId = depthImage.first;
      frames.at(depthImage.first).measurement.deliversKeypoints = false;
    }
  }

  if(!useFrame) {
    return true; // the frame is not used, so quitely ignore.
  }

  if (blocking_)
  {
    return cameraMeasurementsReceived_.PushBlockingIfFull(frames,1);
  }
  else
  {
    if(cameraMeasurementsReceived_.PushNonBlockingDroppingIfFull(frames, cameraInputQueueSize)) {
      LOG(WARNING) << "frame drop ";
      return false;
    }
    return true;
  }
}

// Add an IMU measurement.
bool ThreadedSlam::addImuMeasurement(const okvis::Time& stamp,
                                     const Eigen::Vector3d& alpha,
                                     const Eigen::Vector3d& omega)
{
  static bool warnOnce = true;
  if (!parameters_.imu.use) {
    if (warnOnce) {
      LOG(WARNING) << "imu measurement added, but IMU disabled";
      warnOnce = false;
    }
    return false;
  }
  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers = alpha;
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  const int imuQueueSize = 500;

  if(realtimePropagation_) {
     imuMeasurementsReceivedPropagate_.PushNonBlockingDroppingIfFull(
           imu_measurement, size_t(imuQueueSize));
  }

  if (blocking_)
  {
    return imuMeasurementsReceived_.PushBlockingIfFull(imu_measurement, size_t(imuQueueSize));
  }
  else
  {
    if(imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(
         imu_measurement, size_t(imuQueueSize))) {
      LOG(WARNING) << "imu measurement drop ";
      return false;
    }
    return true;
  }

}


// Set the blocking variable that indicates whether the addMeasurement() functions
// should return immediately (blocking=false), or only when the processing is complete.
void ThreadedSlam::setBlocking(bool blocking)
{
  blocking_ = blocking;
  // disable time limit for optimization
  if(blocking_)
  {
    /// \todo Lock estimator
    //estimator_.setOptimizationTimeLimit(-1.0,parameters_.optimization.max_iterations);
  }
}

bool ThreadedSlam::getNextFrame(MultiFramePtr &multiFrame) {
  std::vector<okvis::CameraMeasurement> frames;
  if(!cameraMeasurementsReceived_.getCopyOfFront(&frames)) {
    return false;
  }
  const size_t numCameras = frames.size();
  multiFrame.reset(new okvis::MultiFrame(parameters_.nCameraSystem, frames.at(0).timeStamp, 0));
  bool success = false;
  for(size_t im = 0; im < numCameras; ++im)
  {
    cv::Mat filtered = frames.at(im).measurement.image;
    if(!filtered.empty()) {
      success = true; // require at least one image, depth-only is not considered a success.
    }
    multiFrame->setImage(im, filtered);
    multiFrame->setDepthImage(im, frames.at(im).measurement.depthImage);
  }
  return success;
}

bool ThreadedSlam::processFrame() {
  MultiFramePtr multiFrame;
  ImuMeasurement imuMeasurement;
  const size_t numCameras = parameters_.nCameraSystem.numCameras();

  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBias;

  bool ranDetection = false;
  if(firstFrame_) {
    // in the very beginning, we need to wait for the IMU
    if(parameters_.imu.use && !imuMeasurementsReceived_.getCopyOfFront(&imuMeasurement)) {
      return false;
    }

    // now get the frames (synchronised timestamp)
    if(!getNextFrame(multiFrame)) {
      return false;
    }

    if(parameters_.imu.use) {
      if(multiFrame->timestamp()-Duration(imuTemporalOverlap) <= imuMeasurement.timeStamp) {
        // that's bad, we have frames without IMU measurements. Discard too old frames
        LOG(WARNING) << "startup: dropping frame because IMU measurements are newer, t="
                     << imuMeasurement.timeStamp;
        std::vector<okvis::CameraMeasurement> frames;
        cameraMeasurementsReceived_.PopBlocking(&frames);
        return false;
      }

      // also make sure we have enough IMU measuerements
      if(!imuMeasurementsReceived_.getCopyOfBack(&imuMeasurement)) {
        return false;
      }
      if(imuMeasurement.timeStamp < multiFrame->timestamp() + Duration(imuTemporalOverlap)) {
        return false; // wait for more IMU measurements
      }

      // now get all relevant IMU measurements we have received thus far
      do {
        if(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement))
        {
          imuMeasurementDeque_.push_back(imuMeasurement);
        } else {
          return false;
        }
      } while(imuMeasurementDeque_.back().timeStamp
              < multiFrame->timestamp() + Duration(imuTemporalOverlap));
    }

    firstFrame_ = false;
  } else {
    // wait for next frame
    if(!getNextFrame(multiFrame)) {
      if(optimisationThread_.joinable()) {
        // in the very beginning, we can't join because it was not started
        optimisationThread_.join();
      }

      return false;
    }
    // now get all relevant IMU measurements we have received thus far
    if(parameters_.imu.use) {
      while(!shutdown_ && imuMeasurementDeque_.back().timeStamp <
            multiFrame->timestamp() + Duration(imuTemporalOverlap))
      {
        if(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement))
        {
          imuMeasurementDeque_.push_back(imuMeasurement);
        } else {
          return false;
        }
      }
    }
  }
  if (!lastOptimisedState_.id.isInitialised()) {
    // initial state
    if (parameters_.imu.use) {
      bool success = ceres::ImuError::initPose(imuMeasurementDeque_, T_WS);
      OKVIS_ASSERT_TRUE_DBG(Exception,
                            success,
                            "pose could not be initialized from imu measurements.")
      (void) (success); // avoid warning on unused variable
    } else {
      // otherwise we assume the camera is vertical & upright
      kinematics::Transformation T_WC;
      T_WC.set(T_WC.r(), T_WC.q() * Eigen::Quaterniond(-sqrt(2), sqrt(2), 0, 0));
      T_WS = T_WC * parameters_.nCameraSystem.T_SC(0)->inverse();
      //LOG(WARNING) << std::endl << T_WC.C();
    }

    // detection -- needed to check if we can start up
    TimerSwitchable detectTimer("1 DetectAndDescribe");
    if(parameters_.frontend.parallelise_detection) {
      std::vector<std::shared_ptr<std::thread>> detectionThreads;
      for(size_t im = 1; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          detectionThreads.emplace_back(
              new std::thread(&Frontend::detectAndDescribe, &frontend_,
                              im,  multiFrame, T_WC, nullptr));
        }
      }
      kinematics::Transformation T_WC0 = T_WS * (*parameters_.nCameraSystem.T_SC(0));
      if(!multiFrame->image(0).empty()) {
        frontend_.detectAndDescribe(0, multiFrame, T_WC0, nullptr);
      }
      for(size_t i = 0; i < detectionThreads.size(); ++i) {
        detectionThreads[i]->join();
      }
    } else {
      for(size_t im = 0; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          frontend_.detectAndDescribe(im,  multiFrame, T_WC, nullptr);
        }
      }
    }
    if(!frontend_.isInitialized() && multiFrame->numKeypoints() < 15) {
      LOG(WARNING) << "Not enough keypoints (" << multiFrame->numKeypoints()
                   << ") -- cannot initialise yet.";
      std::vector<okvis::CameraMeasurement> frames;
      cameraMeasurementsReceived_.PopBlocking(&frames);
      return false;
    }
    ranDetection = true;
    detectTimer.stop();

  } else {
    // propagate to have a sensible pose estimate (as needed for detection)
    if (parameters_.imu.use) {
      // NOTE: we are not allowed to access the Estimator object here, as optimisation might run.
      T_WS = lastOptimisedState_.T_WS;
      kinematics::Transformation T_WS_prev = T_WS;
      speedAndBias.head<3>() = lastOptimisedState_.v_W;
      speedAndBias.segment<3>(3) = lastOptimisedState_.b_g;
      speedAndBias.tail<3>() = lastOptimisedState_.b_a;
      ceres::ImuError::propagation(imuMeasurementDeque_,
                                   parameters_.imu,
                                   T_WS,
                                   speedAndBias,
                                   lastOptimisedState_.timestamp,
                                   multiFrame->timestamp());
    } else {
      T_WS = lastOptimisedState_.T_WS;
      if (preLastOptimisedState_.id.isInitialised()) {
        const double r = (multiFrame->timestamp() - lastOptimisedState_.timestamp).toSec()
        / (lastOptimisedState_.timestamp - preLastOptimisedState_.timestamp).toSec();
        kinematics::Transformation T_WS_m1 = preLastOptimisedState_.T_WS;
        Eigen::Vector3d dr = r * (T_WS.r() - T_WS_m1.r());
        Eigen::AngleAxisd daa(T_WS.q() * T_WS_m1.q().inverse());
        daa.angle() *= r;
        T_WS.set(T_WS.r() + dr, T_WS.q() * Eigen::Quaterniond(daa));
        //LOG(WARNING) << "---\n" << T_WS_m1.T() << "\n" << T_WS.T() << "\n===";
        //LOG(WARNING) << dr.norm() << " : " << 2.0 * acos(dq.w());
      }
    }
  }
  // success, frame processed. Need to remove from the queue, as getNextFrame only obtains a copy.
  std::vector<okvis::CameraMeasurement> frames;
  cameraMeasurementsReceived_.PopBlocking(&frames);

  // detection
  if(!ranDetection) {
    TimerSwitchable detectTimer("1 DetectAndDescribe");
    if(parameters_.frontend.parallelise_detection) {
      std::vector<std::shared_ptr<std::thread>> detectionThreads;
      for(size_t im = 1; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          detectionThreads.emplace_back(
              new std::thread(&Frontend::detectAndDescribe, &frontend_,
                              im,  multiFrame, T_WC, nullptr));
        }
      }
      kinematics::Transformation T_WC0 = T_WS * (*parameters_.nCameraSystem.T_SC(0));
      if(!multiFrame->image(0).empty()) {
        frontend_.detectAndDescribe(0, multiFrame, T_WC0, nullptr);
      }
      for(size_t i = 0; i < detectionThreads.size(); ++i) {
        detectionThreads[i]->join();
      }
    } else {
      for(size_t im = 0; im < numCameras; ++im) {
        kinematics::Transformation T_WC = T_WS * (*parameters_.nCameraSystem.T_SC(im));
        if(!multiFrame->image(im).empty()) {
          frontend_.detectAndDescribe(im,  multiFrame, T_WC, nullptr);
        }
      }
    }
    if(!frontend_.isInitialized() && multiFrame->numKeypoints() < 15) {
      LOG(WARNING) << "Not enough keypoints (" << multiFrame->numKeypoints()
                   << ") -- cannot initialise yet.";
      return true;
    }
    detectTimer.stop();
  }

  // IMPORTANT: the matcher needs the optimiser to be finished:
  if(optimisationThread_.joinable()) {
    // in the very beginning, we can't join because it was not started
    optimisationThread_.join();
  }

  // now store last optimised state for later use
  if (estimator_.numFrames() > 0) {
    const StateId currentId = estimator_.currentStateId();
    lastOptimisedState_.T_WS = estimator_.pose(currentId);
    SpeedAndBias speedAndBias = estimator_.speedAndBias(currentId);
    lastOptimisedState_.v_W = speedAndBias.head<3>();
    lastOptimisedState_.b_g = speedAndBias.segment<3>(3);
    lastOptimisedState_.b_a = speedAndBias.tail<3>();
    lastOptimisedState_.id = currentId;
    lastOptimisedState_.timestamp = estimator_.timestamp(currentId);
    if (estimator_.numFrames() > 1) {
      const StateId previousId(currentId.value() - 1);
      preLastOptimisedState_.T_WS = estimator_.pose(previousId);
      preLastOptimisedState_.id = previousId;
      preLastOptimisedState_.timestamp = estimator_.timestamp(previousId);
    }
  }

  // break here since all local threads joined
  if(shutdown_) {
    return false;
  }

  // remove imuMeasurements from deque
  if (estimator_.numFrames() > 0) {
    if(parameters_.imu.use) {
      while (!shutdown_
             && (imuMeasurementDeque_.front().timeStamp
                 < lastOptimisedState_.timestamp - Duration(imuTemporalOverlap))) {
        auto imuMeasurement = imuMeasurementDeque_.front();
        imuMeasurementDeque_.pop_front();
        if (imuMeasurementDeque_.empty()
            || (imuMeasurementDeque_.front().timeStamp
                > lastOptimisedState_.timestamp - Duration(imuTemporalOverlap))) {
          imuMeasurementDeque_.push_front(imuMeasurement); // re-add
          break; // finished popping
        }
      }
    }
  }

  // start the matching
  Time matchingStart = Time::now();
  TimerSwitchable matchTimer("2 Match");
  bool asKeyframe = false;
      
  if (!estimator_.addStates(multiFrame, imuMeasurementDeque_, asKeyframe)) {    
    LOG(ERROR)<< "Failed to add state! will drop multiframe.";
    matchTimer.stop();
    return true;
  }
  imuMeasurementsByFrame_[StateId(multiFrame->id())] = imuMeasurementDeque_;

  // call the matcher
  if(!frontend_.dataAssociationAndInitialization(
        estimator_, parameters_, multiFrame, &asKeyframe) && !frontend_.isInitialized()) {
    LOG(WARNING) << "Not enough matches, cannot initialise";
    frontend_.clear();
    estimator_.clear();
    return false;
  }
  estimator_.setKeyframe(StateId(multiFrame->id()), asKeyframe);
  matchTimer.stop();

  // start optimise, publish&visualise, marginalise:
  Eigen::Vector3d gyr(0,0,0);
  auto riter = imuMeasurementDeque_.rbegin();
  while(riter!=imuMeasurementDeque_.rend() && riter->timeStamp > multiFrame->timestamp()) {
    gyr = riter->measurement.gyroscopes;
    ++riter;
  }
  Time now = Time::now();
  double dt = parameters_.estimator.realtime_time_limit-(now-matchingStart).toSec();
  if(dt < 0.0) {
    dt = 0.01;
  }
  if(parameters_.estimator.enforce_realtime) {
    estimator_.setOptimisationTimeLimit(dt,
          parameters_.estimator.realtime_min_iterations);
  }
  optimisationThread_ = std::thread(&ThreadedSlam::optimisePublishMarginalise,
                                    this, multiFrame, gyr);

  // kick off posegraph optimisation, if needed, too...
  if(estimator_.needsFullGraphOptimisation()) {
    if(fullGraphOptimisationThread_.joinable()) {
      fullGraphOptimisationThread_.join();
    }
    // hack: call full graph optimisation
    fullGraphOptimisationThread_ = std::thread(
          &ViSlamBackend::optimiseFullGraph, &estimator_,
          parameters_.estimator.full_graph_iterations,
          std::ref(posegraphOptimisationSummary_),
          parameters_.estimator.full_graph_num_threads, false);
  }

  return true;
}

void ThreadedSlam::optimisePublishMarginalise(MultiFramePtr multiFrame,
                                              const Eigen::Vector3d& gyroReading) {
  kinematics::Transformation T_WS;
  SpeedAndBias speedAndBiases;
  const size_t numCameras = parameters_.nCameraSystem.numCameras();

  // optimise (if initialised)
  TimerSwitchable optimiseTimer("3 Optimise");
  std::vector<StateId> updatedStatesRealtime;
  estimator_.optimiseRealtimeGraph(
      parameters_.estimator.realtime_max_iterations, updatedStatesRealtime,
      parameters_.estimator.realtime_num_threads,
      false, false, frontend_.isInitialized());
  optimiseTimer.stop();

  // import pose graph optimisation
  std::vector<StateId> updatedStatesSync;
  if(estimator_.isLoopClosureAvailable()) {
    OKVIS_ASSERT_TRUE(Exception,
                      !estimator_.isLoopClosing(),
                      "loop closure available, but still loop closing -- bug")
    TimerSwitchable synchronisationTimer("5 Import full optimisation");
    estimator_.synchroniseRealtimeAndFullGraph(updatedStatesSync);
    synchronisationTimer.stop();
  }

  // prepare for publishing
  TimerSwitchable publishTimer("4 Prepare publishing");
  T_WS = estimator_.pose(StateId(multiFrame->id()));
  speedAndBiases = estimator_.speedAndBias(StateId(multiFrame->id()));
  StateId id(multiFrame->id());
  State state;
  state.id = id;
  state.T_WS = T_WS;
  state.v_W = speedAndBiases.head<3>();
  state.b_g = speedAndBiases.segment<3>(3);
  state.b_a = speedAndBiases.tail<3>();
  state.omega_S = gyroReading - speedAndBiases.segment<3>(3);
  state.timestamp = multiFrame->timestamp();
  state.previousImuMeasurements = imuMeasurementsByFrame_.at(id);
  state.isKeyframe = estimator_.isKeyframe(id);

  AlignedMap<uint64_t, kinematics::Transformation> T_AiW;
  if (estimator_.T_AiS_.count(id)) {
    AlignedMap<uint64_t, kinematics::Transformation> T_AiS = estimator_.T_AiS_.at(id);
    for (const auto &T_AS : T_AiS) {
      T_AiW[T_AS.first] = T_AS.second * T_WS.inverse();
      // std::cout << "@@@@ T_WAi= \n" << T_AiW.at(T_AS.first).inverse().T() << std::endl;
    }
  }
  state.T_AiW = T_AiW;
  estimator_.getObservedIds(state.id, state.covisibleFrameIds);

  TrackingState trackingState;
  trackingState.id = id;
  trackingState.isKeyframe = estimator_.isKeyframe(id);
  trackingState.recognisedPlace = estimator_.closedLoop(id);
  const double trackingQuality = estimator_.trackingQuality(id);
  if(trackingQuality < 0.01) {
    trackingState.trackingQuality = TrackingQuality::Lost;
  } else if (trackingQuality < 0.3){
    trackingState.trackingQuality = TrackingQuality::Marginal;
  } else {
    trackingState.trackingQuality = TrackingQuality::Good;
  }
  trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(id, false);

  // re-propagate
  hasStarted_.store(true);

  // now publish
  if(optimisedGraphCallback_) {
    // current state & tracking info via State and Tracking State.
    // the graph:
    std::vector<StateId> updatedStateIds;
    PublicationData publicationData;
    publicationData.state = state;
    publicationData.trackingState = trackingState;
    publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
    if(updatedStatesSync.size()>0) {
      updatedStateIds = updatedStatesSync;
    } else {
      updatedStateIds = updatedStatesRealtime;
    }
    for(const auto & id : updatedStateIds) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      const bool isKeyframe = estimator_.isKeyframe(id);
      ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
      Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
      for(auto riter = imuMeasurements.rbegin(); riter!=imuMeasurements.rend(); ++riter) {
        if(riter->timeStamp < timestamp) {
          omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
          break;
        }
      }

      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      (*publicationData.updatedStates)[id] = State{T_WS, speedAndBias.head<3>(),
                                    speedAndBias.segment<3>(3), speedAndBias.tail<3>(),
                                    omega_S, timestamp, id, imuMeasurements, isKeyframe,
                                    AlignedMap<uint64_t, kinematics::Transformation>(),
                                    observedIds, true};
    }
    for (const auto &id : affectedStates_) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
      Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
      for (auto riter = imuMeasurements.rbegin(); riter != imuMeasurements.rend(); ++riter) {
        if (riter->timeStamp < timestamp) {
          omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
          break;
        }
      }
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      (*publicationData.updatedStates)[id]
        = State{T_WS,
                speedAndBias.head<3>(),
                speedAndBias.segment<3>(3),
                speedAndBias.tail<3>(),
                omega_S,
                timestamp,
                id,
                imuMeasurements,
                estimator_.isKeyframe(id),
                AlignedMap<uint64_t, kinematics::Transformation>(),
                observedIds, false};
    }
    affectedStates_.clear();

    // landmarks:
    publicationData.landmarksPublish.reset(new MapPointVector());
    MapPoints landmarks;
    estimator_.getLandmarks(landmarks);
    publicationData.landmarksPublish->reserve(landmarks.size());
    for(const auto & lm : landmarks) {
      publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality));
    }
    
    // now publish in separate thread. queue size 3 to ensure nothing ever lost.
    const bool overrun = publicationQueue_.PushNonBlockingDroppingIfFull(publicationData,3);
    if(overrun) {
      LOG(ERROR) << "publication (full update) overrun: dropping";
    }
    if(realtimePropagation_) {
      // pass on into IMU processing loop for realtime propagation later.
      // queue size 1 because we can afford to lose them; re-prop. just needs newest stuff.
      const bool overrun2 = lastOptimisedQueue_.PushNonBlockingDroppingIfFull(publicationData,3);
      if(overrun2) {
        LOG(WARNING) << "publication (full update) overrun 2: dropping";
      }
    }
  }
  publishTimer.stop();

  // visualise
  TimerSwitchable visualisationTimer("6 Visualising");
  if(parameters_.output.display_overhead) {
    TimerSwitchable visualisation1Timer("6.1 Visualising overhead");
    // draw debug overhead image
    cv::Mat image(580, 580, CV_8UC3);
    image.setTo(cv::Scalar(10, 10, 10));
    estimator_.drawOverheadImage(image);
    overheadImages_.PushNonBlockingDroppingIfFull(image,1);
    visualisation1Timer.stop();
  }
  if(parameters_.output.display_matches) {
    TimerSwitchable visualisation2Timer("6.2 Prepare visualising matches");
    // draw matches
    // fill in information that requires access to estimator.
    ViVisualizer::VisualizationData::Ptr visualizationDataPtr(
          new ViVisualizer::VisualizationData());
    visualizationDataPtr->observations.resize(multiFrame->numKeypoints());
    okvis::MapPoint2 landmark;
    okvis::ObservationVector::iterator it = visualizationDataPtr
        ->observations.begin();
    for (size_t camIndex = 0; camIndex < numCameras; ++camIndex) {
      for (size_t k = 0; k < multiFrame->numKeypoints(camIndex); ++k) {
        OKVIS_ASSERT_TRUE_DBG(Exception,it != visualizationDataPtr->observations.end(),
                              "Observation-vector not big enough")
        it->keypointIdx = k;
        multiFrame->getKeypoint(camIndex, k, it->keypointMeasurement);
        multiFrame->getKeypointSize(camIndex, k, it->keypointSize);
        it->cameraIdx = camIndex;
        it->frameId = multiFrame->id();
        it->landmarkId = multiFrame->landmarkId(camIndex, k);
        if (estimator_.isLandmarkAdded(LandmarkId(it->landmarkId)) &&
            estimator_.isObserved(KeypointIdentifier(it->frameId, camIndex, k))) {
          estimator_.getLandmark(LandmarkId(it->landmarkId), landmark);
          it->landmark_W = landmark.point;
          it->classification = landmark.classification;
          if (estimator_.isLandmarkInitialised(LandmarkId(it->landmarkId)))
            it->isInitialized = true;
          else
            it->isInitialized = false;
        } else {
          it->landmark_W = Eigen::Vector4d(0, 0, 0, 0);
          // set to infinity to tell visualizer that landmark is not added...
        }
        ++it;
      }
    }
    visualizationDataPtr->T_WS = estimator_.pose(estimator_.currentStateId());
    visualizationDataPtr->currentFrames = multiFrame;
    visualizationDataPtr->isKeyframe = trackingState.isKeyframe;
    visualizationDataPtr->recognisedPlace = trackingState.recognisedPlace;
    if(trackingState.trackingQuality == TrackingQuality::Lost) {
      visualizationDataPtr->trackingQuality =
          ViVisualizer::VisualizationData::TrackingQuality::Lost;
    } else if(trackingState.trackingQuality == TrackingQuality::Marginal) {
      visualizationDataPtr->trackingQuality =
          ViVisualizer::VisualizationData::TrackingQuality::Marginal;
    }
    visualisation2Timer.stop();
    visualisationData_.PushNonBlockingDroppingIfFull(visualizationDataPtr,1);
  }
  visualisationTimer.stop();

  // apply marginalisation strategy
  bool expand = true;
  TimerSwitchable marginaliseTimer("7 Marginalise");
  estimator_.applyStrategy(
        size_t(parameters_.estimator.num_keyframes),
        size_t(parameters_.estimator.num_loop_closure_frames),
    size_t(parameters_.estimator.num_imu_frames), affectedStates_, expand);
  marginaliseTimer.stop();
}

// Loop to process visualisations.
void ThreadedSlam::visualisationLoop()
{
  while (!shutdown_) {
    ViVisualizer::VisualizationData::Ptr visualisationData;
    if(visualisationData_.PopBlocking(&visualisationData)) {
      std::vector<cv::Mat> outImages(parameters_.nCameraSystem.numCameras());
      for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
        if(parameters_.nCameraSystem.cameraType(i).isUsed
           && !visualisationData->currentFrames->image(i).empty()) {
          outImages[i] = visualizer_.drawMatches(visualisationData, i);
        }
      }
      visualisationImages_.PushNonBlockingDroppingIfFull(outImages,1);
    } else {
      return;
    }
  }
}

// Loop to process publishing.
void ThreadedSlam::publishingLoop()
{
  while (!shutdown_) {
    PublicationData publicationData;
    if(publicationQueue_.PopBlocking(&publicationData)) {
      if(optimisedGraphCallback_) {
        optimisedGraphCallback_(publicationData.state, publicationData.trackingState,
                                publicationData.updatedStates, publicationData.landmarksPublish);
      }
    } else {
      return;
    }
  }
}

// trigger display (needed because OSX won't allow threaded display)
void ThreadedSlam::display(std::map<std::string, cv::Mat> &images)
{

  std::vector<cv::Mat> outImages(parameters_.nCameraSystem.numCameras());
  if(visualisationImages_.PopNonBlocking(&outImages)) {
    // draw
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); i++) {
      std::string name;
      if(parameters_.nCameraSystem.cameraType(i).isColour) {
        name = "rgb"+std::to_string(i);
      } else {
        name = "cam"+std::to_string(i);
      }
      if(!outImages[i].empty()) {
        images[name] = outImages[i];
      }
    }
  }

  // top view
  cv::Mat topDebugImg;
  if(overheadImages_.PopNonBlocking(&topDebugImg)) {
    if(!topDebugImg.empty()) {
      images["Top Debug View"] = topDebugImg;
    }
  }
}

void ThreadedSlam::stopThreading() {
  if (shutdown_) {
    return;
  }

  // process all received
  while (processFrame()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // stop CNN stuff
  frontend_.endCnnThreads();

  if(optimisationThread_.joinable()) {
    optimisationThread_.join(); // this should not be necessary after having called processFrame.
  }

  // shutdown queues
  imuMeasurementsReceived_.Shutdown();
  cameraMeasurementsReceived_.Shutdown();
  imuMeasurementsReceivedPropagate_.Shutdown();
  visualisationImages_.Shutdown();
  visualisationData_.Shutdown();
  publicationQueue_.Shutdown();
  lastOptimisedQueue_.Shutdown();

  // force background optimisation to finish
  if (fullGraphOptimisationThread_.joinable()) {
    LOG(INFO) << "wait for background optimisation to finish...";
    fullGraphOptimisationThread_.join();
    // import pose graph optimisation
    if (estimator_.isLoopClosureAvailable()) {
      OKVIS_ASSERT_TRUE(Exception,
                        !estimator_.isLoopClosing(),
                        "loop closure available, but still loop closing -- bug")
      LOG(INFO) << "import final background optimisation and publish...";
      TimerSwitchable synchronisationTimer("5 Import full optimisation");
      std::vector<StateId> updatedStates;
      estimator_.synchroniseRealtimeAndFullGraph(updatedStates);
      synchronisationTimer.stop();

      // prepare publishing
      StateId currentId = estimator_.currentStateId();
      kinematics::Transformation T_WS = estimator_.pose(currentId);
      SpeedAndBias speedAndBiases = estimator_.speedAndBias(currentId);
      State state;
      state.id = currentId;
      state.T_WS = T_WS;
      state.v_W = speedAndBiases.head<3>();
      state.b_g = speedAndBiases.segment<3>(3);
      state.b_a = speedAndBiases.tail<3>();
      state.omega_S.setZero(); // FIXME: use actual value
      state.timestamp = estimator_.timestamp(currentId);
      state.previousImuMeasurements = imuMeasurementsByFrame_.at(currentId);
      state.isKeyframe = estimator_.isKeyframe(currentId);
      TrackingState trackingState;
      trackingState.id = currentId;
      trackingState.isKeyframe = estimator_.isKeyframe(currentId);
      trackingState.recognisedPlace = estimator_.closedLoop(currentId);
      const double trackingQuality = estimator_.trackingQuality(currentId);
      if (trackingQuality < 0.01) {
        trackingState.trackingQuality = TrackingQuality::Lost;
      } else if (trackingQuality < 0.3) {
        trackingState.trackingQuality = TrackingQuality::Marginal;
      } else {
        trackingState.trackingQuality = TrackingQuality::Good;
      }
      trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(currentId, false);
      hasStarted_.store(true);

      // now publish
      if (optimisedGraphCallback_) {
        // current state & tracking info via State and Tracking State.
        PublicationData publicationData;
        publicationData.state = state;
        publicationData.trackingState = trackingState;
        publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
        for (const auto &id : updatedStates) {
          kinematics::Transformation T_WS = estimator_.pose(id);
          SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
          Time timestamp = estimator_.timestamp(id);
          ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
          Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
          for (auto riter = imuMeasurements.rbegin(); riter != imuMeasurements.rend(); ++riter) {
            if (riter->timeStamp < timestamp) {
              omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
              break;
            }
          }
          std::set<StateId> observedIds;
          estimator_.getObservedIds(id, observedIds);
          (*publicationData.updatedStates)[id] =
            State{T_WS, speedAndBias.head<3>(), speedAndBias.segment<3>(3), speedAndBias.tail<3>(),
                  omega_S, timestamp, id, imuMeasurements, estimator_.isKeyframe(id),
                  AlignedMap<uint64_t, kinematics::Transformation>(), observedIds, true};
          affectedStates_.erase(id); // do not separately publish as affected state...
        }
        for (const auto &id : affectedStates_) {
          kinematics::Transformation T_WS = estimator_.pose(id);
          SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
          Time timestamp = estimator_.timestamp(id);
          ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
          Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
          for (auto riter = imuMeasurements.rbegin(); riter != imuMeasurements.rend(); ++riter) {
            if (riter->timeStamp < timestamp) {
              omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
              break;
            }
          }
          std::set<StateId> observedIds;
          estimator_.getObservedIds(id, observedIds);
          (*publicationData.updatedStates)[id]
            = State{T_WS,
                    speedAndBias.head<3>(),
                    speedAndBias.segment<3>(3),
                    speedAndBias.tail<3>(),
                    omega_S,
                    timestamp,
                    id,
                    imuMeasurements,
                    estimator_.isKeyframe(id),
                    AlignedMap<uint64_t, kinematics::Transformation>(),
                    observedIds, false};
        }
        affectedStates_.clear();

        // landmarks:
        publicationData.landmarksPublish.reset(new MapPointVector());
        MapPoints landmarks;
        estimator_.getLandmarks(landmarks);
        publicationData.landmarksPublish->reserve(landmarks.size());
        for (const auto &lm : landmarks) {
          publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality));
        }

        // and publish
        optimisedGraphCallback_(publicationData.state,
                                publicationData.trackingState,
                                publicationData.updatedStates,
                                publicationData.landmarksPublish);
      }
    }
  }

  // end remaining threads
  shutdown_ = true;
  if(visualisationThread_.joinable()) {
    visualisationThread_.join();
  }
  if(publishingThread_.joinable()) {
    publishingThread_.join();
  }
}

void ThreadedSlam::writeFinalTrajectoryCsv()
{
  // thread safety -- join running stuff
  stopThreading();

  // trajectory writing
  if(!finalTrajectoryCsvFileName_.empty()) {
    estimator_.writeFinalCsvTrajectory(finalTrajectoryCsvFileName_, rpg_);
  }
}

void ThreadedSlam::doFinalBa()
{
  // thread safety -- join running stuff
  stopThreading();

  // now call it
  const int numThreads = parameters_.estimator.realtime_num_threads
      +parameters_.estimator.full_graph_num_threads;
  const bool do_extrinsics_final_ba =
      parameters_.camera.online_calibration.do_extrinsics_final_ba;
  const double sigma_r = do_extrinsics_final_ba ?
                         parameters_.camera.online_calibration.sigma_r :
                         0.0;
  const double sigma_alpha = do_extrinsics_final_ba ?
                             parameters_.camera.online_calibration.sigma_alpha :
                             0.0;
  std::set<StateId> updatedStatesBa;
  estimator_.doFinalBa(100, posegraphOptimisationSummary_, updatedStatesBa,
      sigma_r, sigma_alpha, numThreads, true);

  // prepare publishing
  StateId currentId = estimator_.currentStateId();
  kinematics::Transformation T_WS = estimator_.pose(currentId);
  SpeedAndBias speedAndBiases = estimator_.speedAndBias(currentId);
  State state;
  state.id = currentId;
  state.T_WS = T_WS;
  state.v_W = speedAndBiases.head<3>();
  state.b_g = speedAndBiases.segment<3>(3);
  state.b_a = speedAndBiases.tail<3>();
  state.omega_S.setZero(); // FIXME: use actual value
  state.timestamp = estimator_.timestamp(currentId);
  state.previousImuMeasurements = imuMeasurementsByFrame_.at(currentId);
  state.isKeyframe = estimator_.isKeyframe(currentId);
  TrackingState trackingState;
  trackingState.id = currentId;
  trackingState.isKeyframe = estimator_.isKeyframe(currentId);
  trackingState.recognisedPlace = estimator_.closedLoop(currentId);
  const double trackingQuality = estimator_.trackingQuality(currentId);
  if(trackingQuality < 0.01) {
    trackingState.trackingQuality = TrackingQuality::Lost;
  } else if (trackingQuality < 0.3){
    trackingState.trackingQuality = TrackingQuality::Marginal;
  } else {
    trackingState.trackingQuality = TrackingQuality::Good;
  }
  trackingState.currentKeyframeId = estimator_.mostOverlappedStateId(currentId, false);
  hasStarted_.store(true);

  // now publish
  if(optimisedGraphCallback_) {
    // current state & tracking info via State and Tracking State.
    std::vector<StateId> updatedStateIds;
    PublicationData publicationData;
    publicationData.state = state;
    publicationData.trackingState = trackingState;
    publicationData.updatedStates.reset(new AlignedMap<StateId, State>());
    for(const auto & id : updatedStatesBa) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
      Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
      for(auto riter = imuMeasurements.rbegin(); riter!=imuMeasurements.rend(); ++riter) {
        if(riter->timeStamp < timestamp) {
          omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
          break;
        }
      }
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      (*publicationData.updatedStates)[id] =
          State{T_WS, speedAndBias.head<3>(),speedAndBias.segment<3>(3),
          speedAndBias.tail<3>(),omega_S, timestamp, id, imuMeasurements,
          estimator_.isKeyframe(id),
          AlignedMap<uint64_t, kinematics::Transformation>(), observedIds, true};
    }
    for (const auto &id : affectedStates_) {
      kinematics::Transformation T_WS = estimator_.pose(id);
      SpeedAndBias speedAndBias = estimator_.speedAndBias(id);
      Time timestamp = estimator_.timestamp(id);
      ImuMeasurementDeque imuMeasurements = imuMeasurementsByFrame_.at(id);
      Eigen::Vector3d omega_S(0.0, 0.0, 0.0); // get this for real now:
      for (auto riter = imuMeasurements.rbegin(); riter != imuMeasurements.rend(); ++riter) {
        if (riter->timeStamp < timestamp) {
          omega_S = riter->measurement.gyroscopes - speedAndBias.segment<3>(3);
          break;
        }
      }
      std::set<StateId> observedIds;
      estimator_.getObservedIds(id, observedIds);
      (*publicationData.updatedStates)[id]
        = State{T_WS,
                speedAndBias.head<3>(),
                speedAndBias.segment<3>(3),
                speedAndBias.tail<3>(),
                omega_S,
                timestamp,
                id,
                imuMeasurements,
                estimator_.isKeyframe(id),
                AlignedMap<uint64_t, kinematics::Transformation>(),
                observedIds, false};
    }
    affectedStates_.clear();

    // landmarks:
    publicationData.landmarksPublish.reset(new MapPointVector());
    MapPoints landmarks;
    estimator_.getLandmarks(landmarks);
    publicationData.landmarksPublish->reserve(landmarks.size());
    for(const auto & lm : landmarks) {
      publicationData.landmarksPublish->push_back(
            MapPoint(lm.first.value(), lm.second.point, lm.second.quality));
    }

    // and publish
    optimisedGraphCallback_(publicationData.state, publicationData.trackingState,
                            publicationData.updatedStates, publicationData.landmarksPublish);
  }
}

bool ThreadedSlam::saveMap() {
  // thread safety -- join running stuff
  stopThreading();

  // now call it
  if(!finalTrajectoryCsvFileName_.empty()) {
    return estimator_.saveMap(mapCsvFileName_);
  }
  return false;
}

}  // namespace okvis
