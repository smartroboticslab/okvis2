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
 * @file source_file_pos.hpp
 * @brief This file contains some helper functions for the assert macros.
 * @author Paul Furgale
 * @author Stefan Leutenegger
 */

#ifndef OKVIS_SOURCE_FILE_POS_HPP
#define OKVIS_SOURCE_FILE_POS_HPP

#include <string>
#include <iostream>
#include <sstream>
// A class and macro that gives you the current file position.

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief Helper class to get source file positions.
class source_file_pos {
public:
  std::string function; ///< The function.
  std::string file; ///< The file.
  int line; ///< The line.

  /// \brief Constructor.
  /// \param function The function.
  /// \param file The file.
  /// \param line The line.
  source_file_pos(std::string function, std::string file, int line) :
    function(function), file(file), line(line) {}

  /// \brief Return as string.
  /// \return String message.
  operator std::string()
  {
    return toString();
  }

  /// \brief Return as string.
  /// \return String message.
  std::string toString() const
  {
    std::stringstream s;
    s << file << ":" << line << ": " << function << "()";
    return s.str();
  }

};

}// namespace okvis

/// \brief Print to stream.
/// \param out Out stream buffer.
/// \param sfp okvis::source_file_pos object to print.
/// \return The stream object.
inline std::ostream & operator<<(std::ostream & out, const okvis::source_file_pos & sfp)
{
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}


#define OKVIS_SOURCE_FILE_POS okvis::source_file_pos(__FUNCTION__,__FILE__,__LINE__)

#endif // OKVIS_SOURCE_FILE_POS_HPP
