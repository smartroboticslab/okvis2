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
 * @file FrameTypedefs.hpp
 * @brief This file contains useful typedefs and structs related to frames.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_FRAMETYPEDEFS_HPP_
#define INCLUDE_OKVIS_FRAMETYPEDEFS_HPP_

#include <map>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief Simple named Ids.
template <typename Parameter>
class NamedId {
public:
  /// \brief Default constructor -- value set to 0 (not initialised).
  NamedId() = default;

  /// \brief Constructor with value.
  /// @param[in] value Initial valute.
  explicit NamedId(uint64_t const& value) : value_(value) {}

  /// \brief Move constructor.
  explicit NamedId(uint64_t&& value) : value_(std::move(value)) {}

  /// \brief Get the value.
  /// \return The value.
  uint64_t& value() { return value_; }

  /// \brief Get the value.
  /// \return The value.
  uint64_t const& value() const {return value_; }

  /// \brief Equal comparison.
  /// @param[in] other Compared with.
  /// \return True if equal.
  bool operator==(const NamedId<Parameter>& other) const {return value_ == other.value_; }

  /// \brief Not equal comparison.
  /// @param[in] other Compared with.
  /// \return True if not equal.
  bool operator!=(const NamedId<Parameter>& other) const {return value_ != other.value_; }

  /// \brief Bigger than comparison.
  /// @param[in] other Compared with.
  /// \return True if bigger.
  bool operator>(const NamedId<Parameter>& other) const {return value_ > other.value_; }

  /// \brief Smaller than comparison.
  /// @param[in] other Compared with.
  /// \return True if smaller.
  bool operator<(const NamedId<Parameter>& other) const {return value_ < other.value_; }

  /// \brief Addition of other value.
  /// @param[in] add Value to add.
  /// \return The Id with added value.
  NamedId operator+(uint64_t add) const {return NamedId(value_ + add);}

  /// \brief Check if value is initialised.
  /// \return True if the value is initialised.
  bool isInitialised() const {return value_ != 0; }

private:
  uint64_t value_ = 0; ///< Holds the value.
};

/// \brief Landmark IDs.
using LandmarkId = NamedId<struct LandmarkIdParameter>;

/// \brief State IDs.
using StateId = NamedId<struct StateIdParameter>;

/**
 * \brief Unique identifier for a keypoint.
 *
 * A keypoint is identified as the keypoint with index \e keypointIndex
 * in the frame with index \e cameraIndex of multiframe with ID \e frameID.
 */
struct KeypointIdentifier
{
  /**
   * @brief Constructor.
   * @param fi Multiframe ID.
   * @param ci Camera index.
   * @param ki Keypoint index.
   */
  KeypointIdentifier(uint64_t fi = 0, size_t ci = 0, size_t ki = 0)
      : frameId(fi),
        cameraIndex(ci),
        keypointIndex(ki)
  {
  }

  uint64_t frameId;     ///< Multiframe ID.
  size_t cameraIndex;   ///< Camera index.
  size_t keypointIndex; ///< Index of the keypoint

  /// \brief Get multiframe ID.
  uint64_t getFrameId()
  {
    return frameId;
  }
  /// \brief Set multiframe ID.
  void setFrameId(uint64_t fid)
  {
    frameId = fid;
  }
  /// \brief Are two identifiers identical?
  bool isBinaryEqual(const KeypointIdentifier & rhs) const
  {
    return frameId == rhs.frameId && cameraIndex == rhs.cameraIndex
        && keypointIndex == rhs.keypointIndex;
  }
  /// \brief Equal to operator.
  bool operator==(const KeypointIdentifier & rhs) const
  {
    return isBinaryEqual(rhs);
  }
  /// \brief Less than operator. Compares first multiframe ID, then camera index,
  ///        then keypoint index.
  bool operator<(const KeypointIdentifier & rhs) const
  {

    if (frameId == rhs.frameId) {
      if (cameraIndex == rhs.cameraIndex) {
        return keypointIndex < rhs.keypointIndex;
      } else {
        return cameraIndex < rhs.cameraIndex;
      }
    }
    return frameId < rhs.frameId;
  }

};

/// \brief Type to store the result of matching.
struct Match
{
  /**
   * @brief Constructor.
   * @param idxA_ Keypoint index of frame A.
   * @param idxB_ Keypoint index of frame B.
   * @param distance_ Descriptor distance between those two keypoints.
   */
  Match(size_t idxA_, size_t idxB_, float distance_)
      : idxA(idxA_),
        idxB(idxB_),
        distance(distance_)
  {
  }
  size_t idxA;    ///< Keypoint index in frame A.
  size_t idxB;    ///< Keypoint index in frame B.
  float distance; ///< Distance between the keypoints.
};

/// \brief Matches vector for OpenGV adapters.
typedef std::vector<Match> Matches;

/**
 * @brief A type to store information about a point in the world map.
 */
struct MapPoint
{
  /// \brief Default constructor. Point is at origin with quality 0.0 and ID 0.
  MapPoint()
      : id(0),
        point(Eigen::Matrix<double,4,1,Eigen::DontAlign>(0.0, 0.0, 0.0, 1.0)),
        quality(0.0)
  {
  }
  /**
   * @brief Constructor.
   * @param id        ID of the point. E.g. landmark ID.
   * @param point     Homogeneous coordinate of the point.
   * @param quality   Quality of the point. Usually between 0 and 1.
   */
  MapPoint(uint64_t id, const Eigen::Matrix<double,4,1,Eigen::DontAlign> & point,
           double quality)
      : id(id),
        point(point),
        quality(quality)
  {
  }
  uint64_t id;            ///< ID of the point. E.g. landmark ID.
  Eigen::Matrix<double,4,1,Eigen::DontAlign> point;  ///< Homogeneous coordinate of the point.
  double quality;         ///< Quality of the point. Usually between 0 and 1.
};

/**
 * @brief A type to store information about a point in the world map.
 */
struct MapPoint2
{
  LandmarkId id;            ///< ID of the point. E.g. landmark ID.
  Eigen::Matrix<double,4,1,Eigen::DontAlign> point;  ///< Homogeneous coordinate of the point.
  std::set<KeypointIdentifier> observations; ///< All observations to it.
  bool isInitialised;      ///< Quality of the point. Usually between 0 and 1.
  double quality;     ///< Quality of the point. Usually between 0 and 1.
  int classification=-1; ///< Class ID of classification (if enabled).
};

/// \brief Eigen aligned std::map.
template<typename KEY_T, typename VALUE_T>
using AlignedMap = std::map<KEY_T, VALUE_T, std::less<KEY_T>,
Eigen::aligned_allocator<std::pair<const KEY_T, VALUE_T>>>;

/// \brief Eigen aligned std::vector.
template<typename VALUE_T>
using AlignedVector = std::vector<VALUE_T, Eigen::aligned_allocator<VALUE_T>>;

/// \brief Eigen aligned std::set.
template<typename VALUE_T>
using AlignedSet = std::set<VALUE_T, Eigen::aligned_allocator<VALUE_T>>;

/// \brief Vector of map points.
typedef std::vector<MapPoint> MapPointVector;

/// \brief Map of map points.
typedef std::map<LandmarkId, MapPoint2 > MapPoints;

/// \brief Map of transformations.
typedef AlignedMap<uint64_t, okvis::kinematics::Transformation> TransformationMap;

/// \brief For convenience to pass associations - also contains the 3d points.
struct Observation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param keypointIdx Keypoint ID.
   * @param keypointMeasurement Image coordinates of keypoint. [pixels]
   * @param keypointSize Keypoint size. Basically standard deviation of the
   *                     image coordinates in pixels.
   * @param cameraIdx Camera index of observed keypoint.
   * @param frameId Frame ID of observed keypoint.
   * @param landmark_W  Associated landmark coordinates in world frame.
   * @param landmarkId  Unique landmark ID
   * @param isInitialized Is the landmark initialized?
   * @param classification ID of classification.
   */
  Observation(size_t keypointIdx,
              const Eigen::Vector2d& keypointMeasurement,
              double keypointSize,
              size_t cameraIdx,
              uint64_t frameId,
              const Eigen::Vector4d& landmark_W,
              uint64_t landmarkId, bool isInitialized, int classification=-1)
      : keypointIdx(keypointIdx),
        cameraIdx(cameraIdx),
        frameId(frameId),
        keypointMeasurement(keypointMeasurement),
        keypointSize(keypointSize),
        landmark_W(landmark_W),
        landmarkId(landmarkId),
        isInitialized(isInitialized),
        classification(classification)
  {
  }
  Observation()
      : keypointIdx(0),
        cameraIdx(std::numeric_limits<size_t>::max()),
        frameId(0),
        keypointSize(0),
        landmarkId(0),
        isInitialized(false),
        classification(-1)
  {
  }
  size_t keypointIdx; ///< Keypoint ID.
  size_t cameraIdx;  ///< index of the camera this point is observed in
  uint64_t frameId;  ///< unique pose block ID == multiframe ID
  Eigen::Vector2d keypointMeasurement;  ///< 2D image keypoint [pixels]
  double keypointSize;  ///< Keypoint size: standard deviation of the image coordinates in pixels.
  Eigen::Vector4d landmark_W;  ///< landmark as homogeneous point in body frame B
  uint64_t landmarkId;  ///< unique landmark ID
  bool isInitialized;  ///< Initialisation status of landmark
  int classification;  ///< Class ID of classification (if enabled).
};

/// \brief Vector of observations.
typedef std::vector<Observation, Eigen::aligned_allocator<Observation> > ObservationVector;

/// \brief Speed and bias state.
typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;

}  // namespace okvis

#endif /* INCLUDE_OKVIS_FRAMETYPEDEFS_HPP_ */
