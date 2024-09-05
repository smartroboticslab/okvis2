/**
 * @file FBrisk.hpp
 * @brief Header file for the Frontend class.
 *
 * License: BSD, see https://github.com/dorian3d/DBoW2/blob/master/LICENSE.txt
 * @author Dorian Galvez-Lopez
 * @author Stefan Leutenegger
 *
 */

#ifndef __D_T_F_BRISK__
#define __D_T_F_BRISK__

#include <vector>
#include <string>

#include <DBoW2/FClass.h>

/// \brief Adopt the existing DBoW2 workspace.
namespace DBoW2 {

/// Functions to manipulate SURF64 descriptors
class FBrisk: protected FClass
{
public:

  /// \brief Default destructor.
  virtual ~FBrisk() = default;

  /// Descriptor type
  typedef std::vector<unsigned char> TDescriptor;
  /// Pointer to a single descriptor
  typedef const TDescriptor *pDescriptor;
  /// Descriptor length
  static const int L = 48;

  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors vector of pointers to descriptors
   * @param mean mean descriptor
   */
  static void meanValue(const std::vector<pDescriptor> &descriptors,
    TDescriptor &mean);

  /**
   * Calculates the (squared) distance between two descriptors
   * @param a
   * @param b
   * @return (squared) distance
   */
  static double distance(const TDescriptor &a, const TDescriptor &b);

  /**
   * Returns a string version of the descriptor
   * @param a descriptor
   * @return string version
   */
  static std::string toString(const TDescriptor &a);

  /**
   * Returns a descriptor from a string
   * @param a descriptor
   * @param s string version
   */
  static void fromString(TDescriptor &a, const std::string &s);

  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors
   * @param mat (out) NxL 32F matrix
   */
  static void toMat32F(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat);

};

} // namespace DBoW2

#endif

