#include "ColourDepthEncoding.h"

// Convert HSV colour to RGB
Scalar hsv_to_rgb(const Vec3b& hsv_col) {

    // Convert HSV to RGB
    Mat3b hsv(1, 1, hsv_col);
    Mat3b rgb;
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
    return Scalar(rgb(0, 0)[2], rgb(0, 0)[1], rgb(0, 0)[0]);
}

// Logic to get the colour used to represent depth
void getDepthColour(const float& depth, Scalar& near_dist_colour, Scalar& far_dist_colour) {

    double MIN_HUE = HUE_BLUE;
    double MAX_HUE = HUE_RED;
    double scale_factor;
    double half_hue;
    double near_hue;
    double far_hue;

    // Black and white only
    MIN_HUE = 0;
    MAX_HUE = 255;
    scale_factor = min(abs(depth) / CDE_THRESHOLD, 1.0);
    half_hue = (MIN_HUE + MAX_HUE) / 2;
    near_hue = half_hue + half_hue * scale_factor;
    far_hue = half_hue - half_hue * scale_factor;
    near_dist_colour = hsv_to_rgb(Vec3b(100, 0, near_hue));
    far_dist_colour = hsv_to_rgb(Vec3b(100, 0, far_hue));
}

// Draws colour-indicating relative depth along the occluded axis
// Blue indicates closer to the camera, red indicates further
void drawColourDepthEncoding(Mat& frame, const vector<Point>& tooltip_coords, const double& depth) {

    // Check there are two tooltips in frame
    if (tooltip_coords.size() != 2) {
        return;
    }

    // Now get the colour for the tooltips
    Scalar near_dist_colour;
    Scalar far_dist_colour;
    getDepthColour(depth, near_dist_colour, far_dist_colour);

    // Draw depth indicators. Negative depth indicates tooltip1 is closer. Closer is more white.
    // Note that when overlap occurs a circle is drawn out of frame, as intended.
    int indicator_radius = 10;
    if (depth < 0) {
        circle(frame, tooltip_coords[0], indicator_radius, near_dist_colour, -1);
        circle(frame, tooltip_coords[1], indicator_radius, far_dist_colour, -1);
    } else {
        circle(frame, tooltip_coords[0], indicator_radius, far_dist_colour, -1);
        circle(frame, tooltip_coords[1], indicator_radius, near_dist_colour, -1);
    }

    // Add reminder to frame on colour codes
    int font_face = FONT_HERSHEY_SIMPLEX;
    vector<String> text_strs = {"The lighter tip is closer to the viewpoint",
                                "The darker tip is further away from the viewpoint"};
    double font_scale = 1.0;
    int thickness = 2;
    int baseline = 0;
    int vertical_buffer = 30;
    for (int i = 0; i < 2; i++) {
        String text_str = text_strs.at(i);
        Size text_size = getTextSize(text_str, font_face, font_scale, thickness, &baseline);
        Point text_pos(20,frame.rows - 70 + i * vertical_buffer);
        putText(frame, text_str, text_pos, font_face, font_scale, COLOUR_BLACK, thickness);
    }

}