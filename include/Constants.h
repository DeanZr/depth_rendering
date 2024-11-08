#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Constants
const Scalar COLOUR_RED = Scalar(0,0,255);
const Scalar COLOUR_BLUE = Scalar(255,0,0);
const Scalar COLOUR_YELLOW = Scalar(255,255,0);
const Scalar COLOUR_BLACK = Scalar(0,0,0);
const double HUE_BLUE = 0.0;
const double HUE_RED = 120.0;
const double MAX_DIST = 500.0;
const double DIST_THRESHOLD = 10.0;
const int BELOW_THRESHOLD_COUNT = 5;
const int DISP_WIDTH = 1000;
const int DISP_HEIGHT = 1000;
const double CDE_THRESHOLD = 30.0;
const double COLOUR_DIST_BREAK = 20;
const int NUM_TOOLS = 2;
const Point NO_CENTROID_FOUND = Point(-1000, -1000);
const Scalar LOWER_GREEN_MASK = Scalar(40, 60, 80);
const Scalar UPPER_GREEN_MASK = Scalar(80, 255, 200);
const Scalar LOWER_PINK_MASK = Scalar(140, 120, 120);
const Scalar UPPER_PINK_MASK = Scalar(180, 255, 255);

#endif