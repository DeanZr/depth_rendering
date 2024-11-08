#ifndef VISUALISATION_HPP
#define VISUALISATION_HPP

#include <fstream>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include "Constants.h"
#include "ToolData.h"

using namespace std;
using namespace cv;

void displayStartMessage(Mat& frame, const Scalar& colour, VideoCapture& cap);
void displayCompletionMessage(Mat& frame, const Scalar& colour);
void setupTrial(const vector<ToolData>& enabledTools, const bool& is_trial, string& participant_number, string& trial_number, bool& is_col_depth, bool& is_min_dist, ofstream& csv_file);

#endif