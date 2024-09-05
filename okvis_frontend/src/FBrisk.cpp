/**
 * @file FBrisk.cpp
 * @brief Header file for the Frontend class.
 *
 * License: BSD, see https://github.com/dorian3d/DBoW2/blob/master/LICENSE.txt
 * @author Dorian Galvez-Lopez
 * @author Stefan Leutenegger
 *
 */

#include <vector>
#include <string>
#include <sstream>

#include <DBoW2/FBrisk.hpp>

#include <brisk/brisk.h>

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void FBrisk::meanValue(const std::vector<FBrisk::pDescriptor> &descriptors,
  FBrisk::TDescriptor &mean)
{
  mean.resize(0);
  mean.resize(L, 0);

  uint64_t s = descriptors.size()/2;

  std::vector<uint64_t> sum(L*8,0);

  // sum
  vector<FBrisk::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
    const FBrisk::TDescriptor &desc = **it;
    for(size_t i = 0; i < L; i++)
    {
      for(size_t b=0; b<8; ++b)
      {
        sum[i*8+b] += (desc[i]&(0x01<<b)) ? 1 : 0;
      }
    }
  }

  // average
  for(size_t i = 0; i < L; i++)
  {
    for(int b=0; b<8; ++b)
    {
      if(sum[i*8+size_t(b)]>s)
      {
        mean[i] |= (0x01<<b);
      }
    }
  }
}

// --------------------------------------------------------------------------

double FBrisk::distance(const FBrisk::TDescriptor &a, const FBrisk::TDescriptor &b)
{
  return double(brisk::Hamming::PopcntofXORed(&a.front(),&b.front(),L/16));
}

// --------------------------------------------------------------------------

std::string FBrisk::toString(const FBrisk::TDescriptor &a)
{
  stringstream ss;
  for(size_t i = 0; i < L; ++i)
  {
    ss << int(a[i]) << " ";
  }
  return ss.str();
}

// --------------------------------------------------------------------------

void FBrisk::fromString(FBrisk::TDescriptor &a, const std::string &s)
{
  a.resize(L);

  stringstream ss(s);
  for(size_t i = 0; i < L; ++i)
  {
    int tmp;
    ss >> tmp;
    a[i] = uint8_t(tmp);
  }
}

// --------------------------------------------------------------------------

void FBrisk::toMat32F(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }

  const int N = int(descriptors.size());

  mat.create(N, L*8, CV_32F); // 8bit per byte

  for(int i = 0; i < N; ++i)
  {
    const TDescriptor& desc = descriptors[size_t(i)];
    float *p = mat.ptr<float>(i);
    for(int j = 0; j < L*8; j+=8, p+=8)
    {
      for(int b=0; b<8; ++b)
      {
        *(p+b) = (desc[size_t(j)]&(0x01<<b));
      }
    }
  }
}

// --------------------------------------------------------------------------

} // namespace DBoW2


