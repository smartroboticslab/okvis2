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
 * @file Frame.cpp
 * @brief Source file for the CameraBase class.
 * @author Stefan Leutenegger
 */

#include <okvis/Frame.hpp>
#include <okvis/internal/Network.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

#ifdef OKVIS_USE_NN
  int Frame::computeClassifications(int sizeU, int sizeV, int numThreads) {
  OKVIS_ASSERT_TRUE(Exception, network_, "network not set");
#ifndef OKVIS_USE_GPU
  // make 100% sure multithreading on
  int num_threads =torch::get_num_threads();
  torch::set_num_threads(numThreads);
#endif
  // convert image
  cv::Mat img2;
  cv::resize(image_, img2, cv::Size(sizeU, sizeV));
  cv::cvtColor(img2, img2, cv::COLOR_GRAY2RGB);
  const double scaleU = double(img2.cols)/double(image_.cols);
  const double scaleV = double(img2.rows)/double(image_.rows);
  cv::Mat img_float;
  img2.convertTo(img_float, CV_32F, 1.0 / 255);
  auto tensor_image = torch::from_blob(img_float.data,
                                       {img_float.rows, img_float.cols, img_float.channels()},
                                       torch::kFloat32);
  tensor_image = tensor_image.permute({2, 0, 1});
  std::vector<double> norm_mean = {0.485, 0.456, 0.406};
  std::vector<double> norm_std = {0.229, 0.224, 0.225};
  tensor_image = torch::data::transforms::Normalize<>(norm_mean, norm_std)(tensor_image);
  tensor_image.unsqueeze_(0);
#ifdef OKVIS_USE_GPU
#ifdef OKVIS_USE_MPS
  tensor_image = tensor_image.to(torch::kMPS);
#else
  tensor_image = tensor_image.to(torch::kCUDA);
#endif
#endif

  // forward pass
  auto outputs = network_->forward({tensor_image}).toTuple();

  // convert output
#ifdef OKVIS_USE_GPU
  torch::Tensor out1 = outputs->elements()[0].toTensor().cpu();
#else
  torch::Tensor out1 = outputs->elements()[0].toTensor();
#endif
  //torch::Tensor out2 = torch::softmax(out1, 1).squeeze(0);
  torch::Tensor out2 = out1.squeeze(0);

  // cnn classification
  // used by S-CNN: [7, 8, 11, 12, 13, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33]
  // https://github.com/mcordts/cityscapesScripts/blob/master/cityscapesscripts/helpers/labels.py
  // name                     id    trainId   category            catId     hasInstances   eval ign.
  // 'unlabeled'            ,  0 ,      255 , 'void'            , 0       , False        , True
  // 'ego vehicle'          ,  1 ,      255 , 'void'            , 0       , False        , True
  // 'rectification border' ,  2 ,      255 , 'void'            , 0       , False        , True
  // 'out of roi'           ,  3 ,      255 , 'void'            , 0       , False        , True
  // 'static'               ,  4 ,      255 , 'void'            , 0       , False        , True
  // 'dynamic'              ,  5 ,      255 , 'void'            , 0       , False        , True
  // 'ground'               ,  6 ,      255 , 'void'            , 0       , False        , True
  // 'road'                 ,  7 ,        0 , 'flat'            , 1       , False        , False
  // 'sidewalk'             ,  8 ,        1 , 'flat'            , 1       , False        , False
  // 'parking'              ,  9 ,      255 , 'flat'            , 1       , False        , True
  // 'rail track'           , 10 ,      255 , 'flat'            , 1       , False        , True
  // 'building'             , 11 ,        2 , 'construction'    , 2       , False        , False
  // 'wall'                 , 12 ,        3 , 'construction'    , 2       , False        , False
  // 'fence'                , 13 ,        4 , 'construction'    , 2       , False        , False
  // 'guard rail'           , 14 ,      255 , 'construction'    , 2       , False        , True
  // 'bridge'               , 15 ,      255 , 'construction'    , 2       , False        , True
  // 'tunnel'               , 16 ,      255 , 'construction'    , 2       , False        , True
  // 'pole'                 , 17 ,        5 , 'object'          , 3       , False        , False
  // 'polegroup'            , 18 ,      255 , 'object'          , 3       , False        , True
  // 'traffic light'        , 19 ,        6 , 'object'          , 3       , False        , False
  // 'traffic sign'         , 20 ,        7 , 'object'          , 3       , False        , False
  // 'vegetation'           , 21 ,        8 , 'nature'          , 4       , False        , False
  // 'terrain'              , 22 ,        9 , 'nature'          , 4       , False        , False
  // 'sky'                  , 23 ,       10 , 'sky'             , 5       , False        , False
  // 'person'               , 24 ,       11 , 'human'           , 6       , True         , False
  // 'rider'                , 25 ,       12 , 'human'           , 6       , True         , False
  // 'car'                  , 26 ,       13 , 'vehicle'         , 7       , True         , False
  // 'truck'                , 27 ,       14 , 'vehicle'         , 7       , True         , False
  // 'bus'                  , 28 ,       15 , 'vehicle'         , 7       , True         , False
  // 'caravan'              , 29 ,      255 , 'vehicle'         , 7       , True         , True
  // 'trailer'              , 30 ,      255 , 'vehicle'         , 7       , True         , True
  // 'train'                , 31 ,       16 , 'vehicle'         , 7       , True         , False
  // 'motorcycle'           , 32 ,       17 , 'vehicle'         , 7       , True         , False
  // 'bicycle'              , 33 ,       18 , 'vehicle'         , 7       , True         , False
  // 'license plate'        , -1 ,       -1 , 'vehicle'         , 7       , False        , True
  classifications_ = cv::Mat(keypoints_.size(), 19, CV_32FC1);
  for(size_t k=0; k<keypoints_.size(); ++k) {
    int u = std::round(scaleU*(keypoints_[k].pt.x+0.5)-0.5);
    int v = std::round(scaleV*(keypoints_[k].pt.y+0.5)-0.5);
    for(int c=0; c<19; ++c) {
      classifications_.at<float>(k,c) = out2[c][v][u].item<float>();
    }
  }
  isClassified_=true;

#ifndef OKVIS_USE_GPU
  // back to previous threading
  torch::set_num_threads(num_threads);
#endif
#else
int Frame::computeClassifications(int /*sizeU*/, int /*sizeV*/, int /*numThreads*/) {
  OKVIS_THROW(Exception, "trying to remove points with CNN, but CNN support not enabled.")
#endif
  return int(keypoints_.size());
}
}  // namespace okvis
