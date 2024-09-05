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
 * @file DatasetWriter.cpp
 * @brief Source file for the DatasetWriter class.
 * @author Stefan Leutenegger
 */

#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <okvis/ViInterface.hpp>
#include <okvis/DatasetWriter.hpp>

namespace okvis {

DatasetWriter::DatasetWriter(ViParameters &parameters,
                             const std::string &path)
  : parameters_(parameters){

  // check valid path
  OKVIS_ASSERT_TRUE(Exception, boost::filesystem::is_directory(path),
                    "provided path: " << path << " not a valid directory")

      // create dataset folder with timestamp
      auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  datasetDirectory_ << path << "/" << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  boost::filesystem::create_directory(datasetDirectory_.str());

  // create imu csv file
  boost::filesystem::create_directory(datasetDirectory_.str() + "/imu0/");
  imuDirectory_ << datasetDirectory_.str() << "/imu0/";
  imuCsv_.open(imuDirectory_.str() + "data.csv");
  OKVIS_ASSERT_TRUE(Exception, imuCsv_.good(),
                    "couldn't open " << imuDirectory_.str() << "data.csv")
  imuCsv_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
          << "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"<<std::endl;

  // create image folders, CSVs and image queues
  setupImagesDirectories();

  // start processing thread
  shutdown_ = false;
  imuProcessingThread_ = std::thread(&DatasetWriter::processingImu, this);
  imagesProcessingThread_ = std::thread(&DatasetWriter::processingImages, this);
}

DatasetWriter::~DatasetWriter() {

  // shutdown queues
  imuMeasurementsReceived_.Shutdown();
  cameraMeasurementsReceived_.Shutdown();
  rgbCameraMeasurementsReceived_.Shutdown();
  depthCameraMeasurementsReceived_.Shutdown();
  visualisations_.Shutdown();
  visualisationsRGB_.Shutdown();
  visualisationsDepth_.Shutdown();

  // finish writing what's already in the queues
  shutdown_ = true;
  imuProcessingThread_.join();
  imagesProcessingThread_.join();

  // close CSV files
  imuCsv_.close();
  for(size_t i = 0; i<camCsvs_.size(); ++i) {
    camCsvs_.at(i).close();
  }
  for(size_t i = 0; i<depthCamCsvs_.size(); ++i) {
    depthCamCsvs_.at(i).close();  // close can be called even if never opened
  }
}

bool DatasetWriter::addImages(const Time &stamp,
                              const std::map<size_t, cv::Mat> & images,
                              const std::map<size_t, cv::Mat> & depthImages) {

  // assemble frame
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  std::vector<okvis::CameraMeasurement> frames(numCameras);

  for(const auto & image : images) {
    frames.at(image.first).measurement.image = image.second;
    frames.at(image.first).timeStamp = stamp;
    frames.at(image.first).sensorId = image.first;
    frames.at(image.first).measurement.deliversKeypoints = false;
  }

  for(const auto & depthImage : depthImages) {
    frames.at(depthImage.first).measurement.depthImage = depthImage.second;
    frames.at(depthImage.first).timeStamp = stamp;
    frames.at(depthImage.first).sensorId = depthImage.first;
    frames.at(depthImage.first).measurement.deliversKeypoints = false;
  }

  const int cameraInputQueueSize = 100;
  if(cameraMeasurementsReceived_.PushNonBlockingDroppingIfFull(frames, cameraInputQueueSize)) {
    LOG(WARNING) << "frame drop";
  }
  return true;
}

bool DatasetWriter::addImuMeasurement(
  const Time &stamp, const Eigen::Vector3d &alpha, const Eigen::Vector3d &omega) {
  okvis::ImuMeasurement imu_measurement;
  imu_measurement.measurement.accelerometers = alpha;
  imu_measurement.measurement.gyroscopes = omega;
  imu_measurement.timeStamp = stamp;

  const int imuQueueSize = 100;
  if(imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(
        imu_measurement, size_t(imuQueueSize))) {
    LOG(WARNING) << "imu measurement drop ";
    return false;
  }
  return true;
}

void DatasetWriter::setBlocking(bool blocking) {
  if(blocking) {
    LOG(WARNING) << "DatasetWriter cannot run in blocking mode" << std::endl;
  }
}

void DatasetWriter::display(std::map<std::string, cv::Mat> & images) {
  std::vector<CameraMeasurement> frames;
  if(visualisations_.PopNonBlocking(&frames)) {
    for(size_t i=0; i<frames.size(); ++i) {
      std::string name;
      if(parameters_.nCameraSystem.cameraType(i).isColour) {
        name = "rgb"+std::to_string(i);
      } else {
        name = "cam"+std::to_string(i);
      }
      if(!frames.at(i).measurement.image.empty()) {
        images[name] = frames.at(i).measurement.image;
      }
      if(parameters_.nCameraSystem.isDepthCamera(i)
          && !frames.at(i).measurement.depthImage.empty()) {
        images["depth"+std::to_string(i)] = frames.at(i).measurement.depthImage;
      }
    }
  }
}

void DatasetWriter::processingImu() {
  while(!shutdown_) {
    ImuMeasurement imuMeasurement;
    while(imuMeasurementsReceived_.PopNonBlocking(&imuMeasurement)) {
      imuCsv_ << imuMeasurement.timeStamp.toNSec() << ","
              << imuMeasurement.measurement.gyroscopes[0] << ","
              << imuMeasurement.measurement.gyroscopes[1] << ","
              << imuMeasurement.measurement.gyroscopes[2] << ","
              << imuMeasurement.measurement.accelerometers[0] << ","
              << imuMeasurement.measurement.accelerometers[1] << ","
              << imuMeasurement.measurement.accelerometers[2] <<std::endl;
    }
  }
}

void DatasetWriter::processingImages() {
  while(!shutdown_) {
    std::vector<CameraMeasurement> frames;
    while(cameraMeasurementsReceived_.PopNonBlocking(&frames)) {
      const size_t numCameras = frames.size();
      for (size_t i = 0; i < numCameras; ++i) {
        uint64_t timestamp = frames.at(i).timeStamp.toNSec();
        if(!frames.at(i).measurement.image.empty()) {
          // write text file data
          const std::string imageSuffix =
            parameters_.nCameraSystem.cameraType(i).isColour ? ".jpg" : ".png";
          camCsvs_.at(i) << timestamp << "," << timestamp << imageSuffix << std::endl;

          // write image
          std::stringstream imagename;
          imagename << camDirectories_.at(i) << "data/" << timestamp << imageSuffix;
        
          cv::imwrite(imagename.str(), frames.at(i).measurement.image);
        }
        
        // handle depth image, if needed
        if(parameters_.nCameraSystem.isDepthCamera(i)
            && !frames.at(i).measurement.depthImage.empty()) {
          // write text file data
          depthCamCsvs_.at(i) << timestamp << "," << timestamp << ".tif" << std::endl;

          // transform and write depth image
          std::stringstream depthImagename;
          depthImagename << depthCamDirectories_.at(i) << "data/" << timestamp << ".tif";
          expImageTransform(frames.at(i).measurement.depthImage);
          cv::imwrite(depthImagename.str(), frames.at(i).measurement.depthImage);
        }
      }
      visualisations_.PushNonBlockingDroppingIfFull(frames, 1);
    }
  }
}

bool DatasetWriter::setupImagesDirectories() {
  const size_t numCameras = parameters_.nCameraSystem.numCameras();
  for(size_t i = 0; i<numCameras; ++i) {
    auto type = parameters_.nCameraSystem.cameraType(i);
    std::string sensorName;
    if(type.isColour) {
      sensorName = "rgb";
    } else {
      sensorName = "cam";
    }
    std::stringstream camDirectory;
    camDirectory << datasetDirectory_.str() << "/" << sensorName << i << "/";
    camDirectories_.push_back(camDirectory.str());
    boost::filesystem::create_directory(camDirectory.str());
    boost::filesystem::create_directory(camDirectory.str() + "data/");
    camCsvs_.push_back(std::ofstream(camDirectory.str()+"data.csv"));
    OKVIS_ASSERT_TRUE(DatasetWriter::Exception, camCsvs_.back().good(),
                      "couldn't open " << camDirectory.str() << "data.csv");
    camCsvs_.back() << "#timestamp [ns],filename" << std::endl;

    // handle depth, if it is depth camera
    if(type.depthType.isDepthCamera){
      std::stringstream depthDirectory;
      depthDirectory << datasetDirectory_.str() << "/depth" << i << "/";
      depthCamDirectories_.push_back(depthDirectory.str());
      boost::filesystem::create_directory(depthDirectory.str());
      boost::filesystem::create_directory(depthDirectory.str() + "data/");
      depthCamCsvs_.push_back(std::ofstream(depthDirectory.str()+"data.csv"));
      OKVIS_ASSERT_TRUE(DatasetWriter::Exception, depthCamCsvs_.back().good(),
                        "couldn't open " << depthDirectory.str() << "data.csv");
      depthCamCsvs_.back() << "#timestamp [ns],filename" << std::endl;
    } else {
      // we keep the depth equivalents (albeit empty) so vector indexing always works.
      depthCamDirectories_.push_back("");
      depthCamCsvs_.push_back(std::ofstream());
    }
  }
  return true;
}

}
