#pragma once

// C++ header
#include <vector>

// opencv header
#include <opencv2/core.hpp>

namespace yolo_detector
{

const std::vector<cv::Scalar> colors = {
  cv::Scalar(255, 255, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 255, 255),
  cv::Scalar(255, 0, 0)};

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.4;
const float CONFIDENCE_THRESHOLD = 0.4;

const int CLASS_DIMENSIONS = 85;
const int OUTPUT_ROWS = 25200;
// Details: https://user-images.githubusercontent.com/58934176/105021013-554a2600-5a48-11eb-90ef-6656a7fe63bb.png

} // namespace yolo_detector
