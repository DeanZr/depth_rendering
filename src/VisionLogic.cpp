#include "VisionLogic.h"

// Finds the tooltips in the image
void filterTooltips(const Mat& frame, Mat& colour_mask, const Scalar& lower_mask, const Scalar& upper_mask) {
    
    // Convert to HSV
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // Simply mask colour first
    inRange(hsv, lower_mask, upper_mask, colour_mask);

    // Morphological operations
    Mat opening_kernel = cv::getStructuringElement(MORPH_RECT, Size(9, 9));
    cv::morphologyEx(colour_mask, colour_mask, MORPH_OPEN, opening_kernel);
}

// Finds the largest contour from a given mask. Also updates centroid of this contour
void findLargestContour(Mat mask, Mat& frame, Point& largest_centroid) {

    // Find contours in the mask
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Sort contours by area, we are only interested in the largest one
    int n_contours = contours.size();
    
    // Check for overlap scenario:
    // Sets centroid out of frame to avoid rendering issues
    if (n_contours == 0) {
        largest_centroid = NO_CENTROID_FOUND;
        return;
    }
    double max_area = 0;
    int max_index = -1;
    for (int i = 0; i < n_contours; i++) {
        if (max_area < cv::contourArea(contours[i])) {
            max_area = cv::contourArea(contours[i]);
            max_index = i;
        }
    }

    // Calculate the centroid (center of mass) of the contour
    Moments moments = cv::moments(contours[max_index]);
    Point centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);
    largest_centroid = centroid;
}