#ifndef VISION_LOGIC_HPP
#define VISION_LOGIC_HPP

#include <opencv2/opencv.hpp>
#include "Constants.h"

using namespace std;
using namespace cv;

void filterTooltips(const Mat& frame, Mat& colour_mask, const Scalar& lower_mask, const Scalar& upper_mask);
void findLargestContour(Mat mask, Mat& frame, Point& largest_centroid);

#endif