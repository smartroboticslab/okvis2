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
 * @file cameras/NCameraSystem.hpp
 * @brief Header file for the NCameraSystem class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_NCAMERASYSTEM_HPP_
#define INCLUDE_OKVIS_NCAMERASYSTEM_HPP_

#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/assert_macros.hpp>
#include "okvis/cameras/CameraBase.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \class NCameraSystem
/// \brief A class that assembles multiple cameras into a system of
/// (potentially different) cameras.
class NCameraSystem
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// The enumeration of the currently supported distortion types.
  enum DistortionType  {
    Equidistant = 0, ///< Use with okvis::cameras::EquidistantDistortion.
    RadialTangential = 1, ///< Use with okvis::cameras::RadialTangentialDistortion.
    NoDistortion = 2, ///< No distortion.
    RadialTangential8 = 3 ///< Use with okvis::cameras::RadialTangentialDistortion.
  };

  /// \brief Default constructor
  inline NCameraSystem();

  /// \brief Destructor that doesn't do anything really.
  inline virtual ~NCameraSystem();

  /// \brief Helper struct to specify the camera type.
  struct CameraType {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraType(){} ///< Default constructor.
    bool isColour = true; ///< Set true for RGB8, false for mono8.
    bool isUsed = true; ///< Set false if not in use for okvis.
    /// \brief Helper struct to specify the depth camera type.
    struct DepthType {
      bool isDepthCamera = false; ///< Is it a depth camera?
      Eigen::Vector3d baseline = Eigen::Vector3d(0.0,0.0,0.0); ///< Stereo baseline [m].
      bool createVirtual = false; ///< If true, creating measurements in virtual camera.
      bool createDepth = false; ///< If true, creating measurements constraining depth.
      double sigmaPixels = 1.0; ///< Uncertainty (std. deviation) [pix].
      double sigmaDepth = 0.01; ///< Depth uncertainty (std. deviation) [m].
    };
    DepthType depthType; ///< Depth camera type.
  };

  /// \brief Append with a single camera.
  /// @param[in] T_SC extrinsics.
  /// @param[in] cameraGeometry Camera geometry.
  /// @param[in] distortionType Distortion type.
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take a while) should be
  /// performed.
  /// @param[in] cameraType The type of the camera (colour / gray / depth, etc.).
  inline void addCamera(std::shared_ptr<const okvis::kinematics::Transformation> T_SC,
                        std::shared_ptr<const cameras::CameraBase> cameraGeometry,
                        DistortionType distortionType,
                        bool computeOverlaps = true,
                        const CameraType & cameraType = CameraType());

  /// \brief Obtatin the number of cameras currently added.
  /// @return The number of cameras.
  inline size_t numCameras() const;

  /// \brief Obtatin the number of cameras currently added.
  /// @return The number of cameras.
  inline size_t numUsedCameras() const;

  /// \brief Compute all the overlaps of fields of view. Attention: can be expensive.
  void computeOverlaps();

  /// \brief Get the pose of the IMU frame S with respect to the camera cameraIndex
  /// @param[in] cameraIndex The camera index for which the extrinsics should be returned.
  /// @return T_SC, the extrinsics.
  inline std::shared_ptr<const okvis::kinematics::Transformation> T_SC(size_t cameraIndex) const;

  /// \brief Set the camera extrinsics of camera cameraIndex.
  /// @param[in] cameraIndex The camera index for which the extrinsics should be set.
  /// @param[in] T_SCi The new extrinsics.
  inline void setExtrinsics(size_t cameraIndex, kinematics::Transformation T_SCi);

  /// \brief Get the camera geometry of camera cameraIndex
  /// @param[in] cameraIndex The camera index for which the camera geometry should be returned.
  /// @return The camera geometry.
  inline std::shared_ptr<const cameras::CameraBase> cameraGeometry(size_t cameraIndex) const;

  /// \brief Get the distortion type of the camera
  /// @param[in] cameraIndex The camera index for which the distortion type should be returned.
  /// @return The distortion type
  inline DistortionType distortionType(size_t cameraIndex) const;

  /// \brief Get the overlap mask. Sorry for the weird syntax, but remember that
  /// cv::Mat is essentially a shared pointer.
  /// @param[in] cameraIndexSeenBy The camera index for one camera.
  /// @param[in] cameraIndex The camera index for the other camera.
  /// @return The overlap mask image.
  inline const cv::Mat overlap(size_t cameraIndexSeenBy,
                                 size_t cameraIndex) const;

  /// \brief Can the first camera see parts of the FOV of the second camera?
  /// @param[in] cameraIndexSeenBy The camera index for one camera.
  /// @param[in] cameraIndex The camera index for the other camera.
  /// @return True, if there is at least one pixel of overlap.
  inline bool hasOverlap(size_t cameraIndexSeenBy, size_t cameraIndex) const;

  /// \brief Is it a depth camera?
  /// \param cameraIndex The index of the camera in question.
  /// \return Ture if depth camera.
  inline bool isDepthCamera(size_t cameraIndex) const {
    return cameraTypes_.at(cameraIndex).depthType.isDepthCamera;
  }

  /// \brief The camera type.
  /// \param cameraIndex The index of the camera in question.
  /// \return The camera type.
  CameraType cameraType(size_t cameraIndex) const {
    return cameraTypes_.at(cameraIndex);
  }

  /// \brief Check if the camera type for the given index is configured.
  /// \param cameraIndex The index of the camera in question.
  /// \return True if the camera type is configured.
  bool isCameraConfigured(size_t cameraIndex) const {
    return cameraIndex < cameraTypes_.size();
  }

 protected:
  /// \brief Use this to check overlapMats_ and overlaps_ have correct sizes
  /// @return True, if valid.
  inline bool overlapComputationValid() const;
  std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> T_SC_; ///< IMU-cam trafo.
  std::vector<std::shared_ptr<const cameras::CameraBase>> cameraGeometries_;  ///< Cam geometries.
  std::vector<DistortionType> distortionTypes_; ///< Distortion types by camera.
  std::vector<std::vector<cv::Mat>> overlapMats_;  ///< Overlaps between cameras: mats.
  std::vector<std::vector<bool>> overlaps_;  ///< Overlaps between cameras: binary.
  std::vector<CameraType> cameraTypes_; ///< Camera types.
  std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> virtual_T_SC_; ///< IMU-cam trafo.
  std::vector<std::shared_ptr<const cameras::CameraBase>> virtual_cameraGeometries_;  ///< Cam geometries.
  std::vector<DistortionType> virtual_distortionTypes_; ///< Distortion types by camera.
  size_t numUsedCameras_; ///< Number of cameras used.
};

}  // namespace cameras
}  // namespace okvis

#include "okvis/cameras/implementation/NCameraSystem.hpp"

#endif /* INCLUDE_OKVIS_NCAMERASYSTEM_HPP_ */
