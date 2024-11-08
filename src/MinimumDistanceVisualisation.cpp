#include "MinimumDistanceVisualisation.h"

// Draws minimum distance vector on given frame
void drawMinimumDistVector(Mat& frame, const vector<Point>& tooltip_coords, const double& dist) {

    // Check there is at least one tooltip and target in frame
    if (tooltip_coords.size() != 2) {
        return;
    }

    Point tooltip1 = tooltip_coords.at(0);
    Point tooltip2 = tooltip_coords.at(1);

    // Assume at least one of the tooltips will be found
    // (since overlap will only obstruct one tooltip)
    if (tooltip1 == NO_CENTROID_FOUND) {
        tooltip1 = tooltip2;
    }
    if (tooltip2 == NO_CENTROID_FOUND) {
        tooltip2 = tooltip1;
    }

    // Draw line between points on frame
    line(frame, tooltip1, tooltip2, COLOUR_BLUE, 2);

    // Add text to line
    addTextToLine(frame, dist, tooltip1, tooltip2, COLOUR_BLUE);
}

// Adds text to the minimum distance vector
void addTextToLine(Mat& frame, const double& dist, const Point& tt_coords, const Point& ooi_coords, const Scalar& colour) {

    stringstream dist_str;
    dist_str << fixed << setprecision(2) << dist;
    String text = dist_str.str();

    // Get midpoint
    Point midpoint((tt_coords.x + ooi_coords.x) / 2, (tt_coords.y + ooi_coords.y) / 2);

    // Add text to frame
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int thickness = 3;
    int baseline = 0;
    Size text_size = getTextSize(to_string(dist), font_face, font_scale, thickness, &baseline);
    int vertical_buffer = 30;
    Point text_pos(midpoint.x - text_size.width / 2, midpoint.y - vertical_buffer);    
    putText(frame, text, text_pos, font_face, font_scale, colour, thickness);
}