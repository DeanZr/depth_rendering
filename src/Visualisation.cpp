#include "Visualisation.h"

// Display message on frame upon completion
void displayStartMessage(Mat& frame, const Scalar& colour, VideoCapture& cap) {
    
    // Text properties
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 2.0;
    int thickness = 3;
    int baseline = 0;
    Point text_pos(20,70);    
    std::string text = "The visualisation is ready to be run. Hit any key to begin!";
    putText(frame, text, text_pos, font_face, font_scale, colour, thickness);
    imshow("Original Frame", frame);
    waitKey();
    int timer = 5;
    for (;;) {
        cap.read(frame);
        text = "The task will start in: " + std::to_string(timer);
        putText(frame, text, text_pos, font_face, font_scale, colour, thickness);
        imshow("Original Frame", frame);
        
        if (waitKey(1000) >= 0) {
            break; 
        }

        timer--;
        if (timer == 0) {
            break;
        }
    }
}

// Display message on frame upon completion
void displayCompletionMessage(Mat& frame, const Scalar& colour) {

    String text = "Task Complete! This window will close now.";

    // Add text to frame
    int font_face = FONT_HERSHEY_SIMPLEX;
    double font_scale = 2.0;
    int thickness = 3;
    int baseline = 0;
    Point text_pos(20,70);    
    putText(frame, text, text_pos, font_face, font_scale, colour, thickness);

}

// Setup csv file (if required) and visualisation booleans
void setupTrial(const vector<ToolData>& enabledTools, const bool& is_trial, string& participant_number, string& trial_number, bool& is_col_depth, bool& is_min_dist, ofstream& csv_file) {
    
    if (is_trial) {
        std::cout << "Enter participant number: ";
        std::cin >> participant_number;
        std::cout << "Enter trial number: ";
        std::cin >> trial_number;
        std::cout << "Is CDE enabled? (0 for no, 1 for yes): ";
        std::cin >> is_col_depth;
        std::cout << "Is MDV enabled? (0 for no, 1 for yes): ";
        std::cin >> is_min_dist;
        
        std::string file_name = "p" + participant_number + "t" + trial_number + "_cde" + 
            std::to_string(is_col_depth) + "_mdv" + std::to_string(is_min_dist) + ".csv";
        std::cout << file_name << std::endl;
        

        // Create file to record data
        // First check if file exists, and return if it does to prevent overwriting data
        struct stat buffer;
        if (stat(file_name.c_str(), &buffer) == 0) {
            std::cout << "File already exists! Check parameters." << std::endl;
            return;
        }
        std::cout << "Creating data file." << std::endl;
        
        // Print header information to the first line of the output file
        csv_file.open(file_name.c_str());
        csv_file << "Time,Distance,Depth,#Tools";
        for (int t = 0; t < enabledTools.size(); t++)
        {
            csv_file << ",ToolInfo,Frame#,PortHandle,Face#,TransformStatus,Q0,Qx,Qy,Qz,Tx,Ty,Tz,Error,#Markers";
            for (int m = 0; m < enabledTools[t].markers.size(); m++)
            {
                csv_file << ",Marker" << m << ".Status,Tx,Ty,Tz";
            }
        }
        csv_file << std::endl;
    } else {
        is_col_depth = true;
        is_min_dist = true;
    }
}