#ifndef COLOUR_DEPTH_ENCODING_HPP
#define COLOUR_DEPTH_ENCODING_HPP

#include <opencv2/opencv.hpp>
#include "Constants.h"

using namespace std;
using namespace cv;

Scalar hsv_to_rgb(const Vec3b& hsv_col);
void getDepthColour(const float& depth, Scalar& near_dist_colour, Scalar& far_dist_colour);
void drawColourDepthEncoding(Mat& frame, const vector<Point>& tooltip_coords, const double& depth);

#endif