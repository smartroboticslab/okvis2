/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
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
 *
 *  Created on: Nov 11, 2020
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file nn_test.cpp
 * @brief This file processes a dataset.
 * @author Stefan Leutenegger
 */

#include <opencv2/opencv.hpp>

#include <torch/torch.h>
#include <torch/script.h> // One-stop header.

#include <iostream>

/// \brief Main.
/// \param argc argc.
/// \param argv argv.
int main(int argc, const char* argv[]) {
  if (argc != 3 && argc != 4) {
    std::cerr << "usage: ./nn_test path-to-exported-script-module input-image [gpu/cpu]\n";
    return -1;
  }

  // Deserialize the ScriptModule from a file using torch::jit::load().
  torch::jit::script::Module model = torch::jit::load(argv[1]);

  // make 100% sure multithreading on
  at::init_num_threads();

  // read and convert input image
  cv::Mat img = cv::imread(argv[2]);
  cv::Mat img2;
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  cv::cvtColor(img, img2, cv::COLOR_GRAY2RGB);
  cv::Mat img_float;
  img2.convertTo(img_float, CV_32F, 1.0 / 255);
  //auto tensor_image = torch::from_blob(img_float.data, {img_float.channels(), img_float.rows, img_float.cols }, torch::kFloat32);
  auto tensor_image = torch::from_blob(img_float.data, {img_float.rows, img_float.cols, img_float.channels()}, torch::kFloat32);
  tensor_image = tensor_image.permute({2, 0, 1});
  std::vector<double> norm_mean = {0.485, 0.456, 0.406};
  std::vector<double> norm_std = {0.229, 0.224, 0.225};
  tensor_image = torch::data::transforms::Normalize<>(norm_mean, norm_std)(tensor_image);
  tensor_image.unsqueeze_(0);
  //std::cout << tensor_image[0][0][0][0] << " " << tensor_image[0][1][0][0] << " " << tensor_image[0][2][0][0] << std::endl;
  bool cuda = false;
  if(argc==4 && argv[3][0]=='g') {
    if(!torch::hasCUDA()) {
      std::cerr << "Cuda requested but not available.\n";
      return -1;
    }
    cuda = true;
    tensor_image = tensor_image.to(torch::kCUDA);
    model.to(torch::kCUDA);
  } else {
    tensor_image = tensor_image.to(torch::kCPU);
    model.to(torch::kCPU);
  }
  // forward pass
  std::chrono::steady_clock::time_point start0 = std::chrono::steady_clock::now();
  auto outputs = model.forward({tensor_image}).toTuple();
  std::chrono::steady_clock::time_point end0 = std::chrono::steady_clock::now();
  std::cout << "fwd cuda=" << int(cuda) << " "
            << float(std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count())/1000.0
            << " ms" << std::endl;

  // convert output
  torch::Tensor out1 = outputs->elements()[0].toTensor().cpu();
  auto pred = torch::argmax(out1, 1).squeeze(0);
  auto predByte = pred.to(at::kByte);
  cv::Mat segmentation(img.rows, img.cols, CV_8UC1, predByte.data_ptr());

  // display
  cv::imshow("input", img2);
  cv::Mat segmentation_color;
  cv::applyColorMap(10*segmentation, segmentation_color, cv::COLORMAP_JET);
  cv::imshow("segmentation", segmentation_color);
  //cv::waitKey();

  // separate masks
  auto softmax = out1.squeeze(0);
  auto skyTorch = softmax[10].to(at::kFloat);
  auto humanTorch = softmax[11].to(at::kFloat);
  auto humanRiderTorch = softmax[12].to(at::kFloat);
  cv::Mat sky(img.rows, img.cols, CV_32FC1, skyTorch.data_ptr());
  cv::Mat human(img.rows, img.cols, CV_32FC1, humanTorch.data_ptr());
  cv::Mat humanRider(img.rows, img.cols, CV_32FC1, humanRiderTorch.data_ptr());

  cv::imshow("sky activations 3", (sky>3.0));
  cv::imshow("sky activations 4", (sky>4.0));
  //cv::imshow("human", ((human)>3.5f)*255);
  cv::imshow("sky", (segmentation==10));

  cv::waitKey();

  /*// warmup
  std::vector<torch::jit::IValue> inputs0;
  inputs0.push_back(torch::zeros({1, 3, 192, 192}));
  auto output = module.forward(inputs0);//.toTensor();

  // real forward pass
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(torch::ones({1, 3, 192, 192}));
  std::chrono::steady_clock::time_point start0 = std::chrono::steady_clock::now();
  for(size_t i=0; i<100; ++i) {
    output = module.forward(inputs);//.toTensor();
  }
  std::chrono::steady_clock::time_point end0 = std::chrono::steady_clock::now();
  std::cout << "fwd "
            << float(std::chrono::duration_cast<std::chrono::microseconds>(end0 - start0).count())/100000.0
              << " ms" << std::endl;

  //std::cout << output.sizes() << '\n';*/

  return 0;
}
