#ifndef MINIMUM_DISTANCE_VISUALISATION_HPP
#define MINIMUM_DISTANCE_VISUALISATION_HPP

#include <opencv2/opencv.hpp>
#include "Constants.h"

using namespace std;
using namespace cv;

void drawMinimumDistVector(Mat& frame, const vector<Point>& tooltip_coords, const double& dist);
void addTextToLine(Mat& frame, const double& dist, const Point& tt_coords, const Point& ooi_coords, const Scalar& colour);

#endif