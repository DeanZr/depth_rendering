#define ACCESS access
#include <unistd.h>     // for POSIX sleep(sec), and access()
#include <sys/ioctl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <chrono>
#include <sys/stat.h>
#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"
#include "VisionLogic.h"
#include "Constants.h"
#include "ColourDepthEncoding.h"
#include "MinimumDistanceVisualisation.h"
#include "Visualisation.h"

static CombinedApi capi = CombinedApi();
static bool apiSupportsBX2 = false;
static bool apiSupportsStreaming = false;
static bool useEncryption = false;
static std::string cipher = "";
static bool useUDP = false;

using namespace std;
using namespace cv;

/**
 * @brief There's no standard cross platform sleep() keystroke detection
 */
int detectKeystroke()
{
#ifdef _WIN32
    return _kbhit();
#else
    int bytesWaiting;
    ioctl(0, FIONREAD, &bytesWaiting); //stdin=0
    return bytesWaiting;
#endif
}

/**
 * @brief There's no standard cross platform sleep() method prior to C++11
 */
void sleepSeconds(unsigned numSeconds)
{
#ifdef _WIN32
    Sleep((DWORD)1000 * numSeconds); // Sleep(ms)
#else
    sleep(numSeconds); // sleep(sec)
#endif
}

/**
 * @brief Prints a debug message if a method call failed.
 * @details To use, pass the method name and the error code returned by the method.
 *          Eg: onErrorPrintDebugMessage("capi.initialize()", capi.initialize());
 *          If the call succeeds, this method does nothing.
 *          If the call fails, this method prints an error message to stdout.
 */
void onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
    if (errorCode < 0)
    {
        std::cout << methodName << " failed: " << capi.errorToString(errorCode) << std::endl;
    }
}

/**
* @brief Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output
*/
std::string getToolInfo(std::string toolHandle)
{
    // Get the port handle info from PHINF
    PortHandleInfo info = capi.portHandleInfo(toolHandle);

    // Return the ID and SerialNumber the desired string format
    std::string outputString = info.getToolId();
    outputString.append(" s/n:").append(info.getSerialNumber());
    return outputString;
}

/**
* @brief Returns a string representation of the data in CSV format.
* @details The CSV format is: "Frame#,ToolHandle,Face,TransformStatus,q0,qx,qy,qz,tx,ty,tz,error,#markers,[Marker1:status,x,y,z],[Marker2..."
*/
std::string toolDataToCSV(const ToolData& toolData)
{
    std::stringstream stream;
    stream << std::setprecision(toolData.PRECISION) << std::setfill('0');
    stream << "" << static_cast<unsigned>(toolData.frameNumber) << ","
           << "Port:" << static_cast<unsigned>(toolData.transform.toolHandle) << ",";
    stream << static_cast<unsigned>(toolData.transform.getFaceNumber()) << ",";

    if (toolData.transform.isMissing())
    {
        stream << "Missing,,,,,,,,";
    }
    else
    {
        stream << TransformStatus::toString(toolData.transform.getErrorCode()) << ","
               << toolData.transform.q0 << "," << toolData.transform.qx << "," << toolData.transform.qy << "," << toolData.transform.qz << ","
               << toolData.transform.tx << "," << toolData.transform.ty << "," << toolData.transform.tz << "," << toolData.transform.error;
    }

    // Each marker is printed as: status,tx,ty,tz
    stream << "," << toolData.markers.size();
    for ( int i = 0; i < toolData.markers.size(); i++)
    {
        stream << "," << MarkerStatus::toString(toolData.markers[i].status);
        if (toolData.markers[i].status == MarkerStatus::Missing)
        {
            stream << ",,,";
        }
        else
        {
            stream << "," << toolData.markers[i].x << "," << toolData.markers[i].y << "," << toolData.markers[i].z;
        }
    }
    return stream.str();
}

/* The streamed reply, may be used to quit streaming if there is an ERROR */
std::string streamedReply = "";

/*
 * @brief Streams a command using insecure or secure TCP
 */
void streamTcp()
{
    std::string streamId = useEncryption ? "tlsStream1" : "tcpStream1";
    capi.startStreaming("TX 0801", streamId, useEncryption ? Protocol::SecureTCP : Protocol::TCP, cipher);

    // Wait until a keystroke, or if there's an "ERROR" in the stream
    while (detectKeystroke() == 0 && streamedReply.rfind( "ERROR", 0 ) != 0 )
    {
        streamedReply = capi.readStream(streamId);
    }
    capi.stopStreaming(streamId, useEncryption ? Protocol::SecureTCP : Protocol::TCP );
}

/*
 * @brief Streams a command using insecure or secure UDP
 */
void streamUdp()
{
    std::string streamId = useEncryption ? "dtlsStream1" : "udpStream1";
    capi.startStreaming("TX 0801", streamId, useEncryption ? Protocol::SecureUDP : Protocol::UDP, cipher);

    // Wait until a keystroke, or if there's an "ERROR" in the stream
    while ( detectKeystroke() == 0 && streamedReply.rfind( "ERROR", 0 ) != 0 )
    {
        streamedReply = capi.readStream(streamId);
    }
    capi.stopStreaming(streamId, useEncryption ? Protocol::SecureUDP : Protocol::UDP );
}

/**
 * @brief Initialize and enable loaded tools. This is the same regardless of tool type.
 */
void initializeAndEnableTools(std::vector<ToolData>& enabledTools)
{
    std::cout << std::endl << "Initializing and enabling tools..." << std::endl;

    // Initialize and enable tools
    std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        onErrorPrintDebugMessage("capi.portHandleInitialize()", capi.portHandleInitialize(portHandles[i].getPortHandle()));
        onErrorPrintDebugMessage("capi.portHandleEnable()", capi.portHandleEnable(portHandles[i].getPortHandle()));
    }

    // Print all enabled tools
    portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    for (int i = 0; i < portHandles.size(); i++)
    {
        std::cout << portHandles[i].toString() << std::endl;
    }

    // Lookup and store the serial number for each enabled tool
    for (int i = 0; i < portHandles.size(); i++)
    {
        enabledTools.push_back(ToolData());
        enabledTools.back().transform.toolHandle = (uint16_t)capi.stringToInt(portHandles[i].getPortHandle());
        enabledTools.back().toolInfo = getToolInfo(portHandles[i].getPortHandle());
    }
}

/**
 * @brief Loads a tool from a tool definition file (.rom)
 */
void loadTool(const char* toolDefinitionFilePath)
{
    // Request a port handle to load a passive tool into
    int portHandle = capi.portHandleRequest();
    onErrorPrintDebugMessage("capi.portHandleRequest()", portHandle);

    // Load the .rom file using the previously obtained port handle
    capi.loadSromToPort(toolDefinitionFilePath, portHandle);
}

/**
 * @brief Demonstrate detecting active tools.
 * @details Active tools are connected through a System Control Unit (SCU) with physical wires.
 */
void configureActiveTools(std::string scuHostname)
{
    // Setup the SCU connection for demonstrating active tools
    std::cout << std::endl << "Configuring Active Tools - Setup SCU Connection" << std::endl;
    onErrorPrintDebugMessage("capi.setUserParameter()", capi.setUserParameter("Param.Connect.SCU Hostname", scuHostname));
    std::cout << capi.getUserParameter("Param.Connect.SCU Hostname") << std::endl;

    // Wait a few seconds for the SCU to detect any wired tools plugged in
    std::cout << std::endl << "Demo Active Tools - Detecting Tools..." << std::endl;
    sleepSeconds(2);

    // Print all port handles
    std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        std::cout << portHandles[i].toString() << std::endl;
    }
}

/**
 * @brief Demonstrate loading an active wireless tool.
 * @details Active wireless tools are battery powered and emit IR in response to a chirp from the illuminators.
 */
void configureActiveWirelessTools()
{
    // Load an active wireless tool definitions from a .rom files
    std::cout << std::endl << "Configuring an Active Wireless Tool - Loading .rom File..." << std::endl;
    loadTool("sroms/active-wireless.rom");
}

/**
 * @brief Demonstrate loading dummy tools of each tool type.
 * @details Dummy tools are used to report 3Ds in the absence of real tools.
 *          Dummy tools should not be loaded with regular tools of the same type.
 *          TSTART will fail if real and dummy tools are enabled simultaneously.
 */
void configureDummyTools()
{
    std::cout << std::endl << "Loading passive, active-wireless, and active dummy tools..." << std::endl;
    onErrorPrintDebugMessage("capi.loadPassiveDummyTool()", capi.loadPassiveDummyTool());
    onErrorPrintDebugMessage("capi.loadActiveWirelessDummyTool()", capi.loadActiveWirelessDummyTool());
    onErrorPrintDebugMessage("capi.loadActiveDummyTool()", capi.loadActiveDummyTool());
}

/**
 * @brief Demonstrate getting/setting user parameters.
 */
void configureUserParameters()
{
    std::cout << capi.getUserParameter("Param.User.String0") << std::endl;
    onErrorPrintDebugMessage("capi.setUserParameter(Param.User.String0, customString)", capi.setUserParameter("Param.User.String0", "customString"));
    std::cout << capi.getUserParameter("Param.User.String0") << std::endl;
    onErrorPrintDebugMessage("capi.setUserParameter(Param.User.String0, emptyString)", capi.setUserParameter("Param.User.String0", ""));
}

/**
 * @brief Sets the user parameter "Param.Simulated Alerts" to test communication of system alerts.
 * @details This method does nothing if simulatedAlerts is set to 0x00000000.
 */
void simulateAlerts(uint32_t simulatedAlerts = 0x00000000)
{
    // Simulate alerts if any were requested
    if (simulatedAlerts > 0x0000)
    {
        std::cout << std::endl << "Simulating system alerts..." << std::endl;
        std::stringstream stream;
        stream << simulatedAlerts;
        onErrorPrintDebugMessage("capi.setUserParameter(Param.Simulated Alerts, alerts)", capi.setUserParameter("Param.Simulated Alerts", stream.str()));
        std::cout << capi.getUserParameter("Param.Simulated Alerts") << std::endl;
    }
}

/**
 * @brief Determines whether an NDI device supports the BX2 command by looking at the API revision
 */
void determineApiSupportForBX2()
{
    // Lookup the API revision
    std::string response = capi.getApiRevision();

    // Refer to the API guide for how to interpret the APIREV response
    char deviceFamily = response[0];
    int majorVersion = capi.stringToInt(response.substr(2,3));

    // As of early 2017, the only NDI device supporting BX2 is the Vega
    // Vega is a Polaris device with API major version 003
    if ( deviceFamily == 'G' && majorVersion >= 3)
    {
        apiSupportsBX2 = true;
        apiSupportsStreaming = true;
    }
}

void printHelp()
{
    std::cout << "CAPIsample Ver " << capi.getVersion() << std::endl
        << "usage: ./capisample <hostname> --tools=[file1.rom],[file2.rom] [args]" << std::endl
        << "or   : ./capisample <input gbf file> <output txt file>" << std::endl
        << "where:" << std::endl
        << "    <hostname>      The measurement device's hostname, IP address, or serial port." << std::endl
        << "    --tools=[file1.rom],[file2.rom]...  A comma delimited list of tools to load." << std::endl
        << "    [args]          (optional) Any other arguments such as tools to load, or SCU to connect to." << std::endl
        << "    <input gbf file> a file that contains GBF data recorded by ToolBox." << std::endl
        << "    <output txt file> destination output file to contain the text representation of the input GBF file data." << std::endl
        << "example hostnames:" << std::endl
        << "    Connecting to device by IP address: 169.254.8.50" << std::endl
        << "    Connecting to device by hostname: P9-B0103.local" << std::endl
        << "    Connecting to serial port varies by operating system:" << std::endl
        << "        COM10 (Windows), /dev/ttyUSB0 (Linux), /dev/cu.usbserial-001014FA (Mac)" << std::endl
        << "Optional arguments:" << std::endl
#ifdef OPENSSL
        << "    --cipher=[ciphersuite] Limit OpenSSL to use a single TLS/DTLS ciphersuite. Only valid when --encrypted argument is also passed. For example --cipher=ECDHE-RSA-CAMELLIA256-SHA384." << std::endl
        << "    --encrypted Encrypt communcation with the device. Can specify ciphersuite with --cipher but not required." << std::endl
#endif
        << "    --help Displays the help information." << std::endl
        << "    --scu=[scu_hostname] A System Control Unit (SCU) hostname, used to connect active tools." << std::endl
        << "    --udp Stream data over UDP instead of TCP (default)" << std::endl;
}

/**
 * @brief Run depth perception visualisation
 */
void runVisualisation(std::vector<ToolData>& enabledTools) {

    // Get trial info
    bool is_trial;
    std::string participant_number;
    std::string trial_number;
    bool is_col_depth;
    bool is_min_dist;
    std::ofstream csv_file;
    std::cout << "Is this a trial? (0 for no, 1 for yes): ";
    std::cin >> is_trial;

    // Setup csv file (if required) and visualisation booleans
    setupTrial(enabledTools, is_trial, participant_number, trial_number, is_col_depth, is_min_dist, csv_file);

    // Frame for display & centre coordinates
    Mat frame;
    
    // Get video capture
    VideoCapture cap;
    int deviceID = 0;
    int apiID = cv::CAP_ANY;

    // Open selected camera using selected API
    cap.open(deviceID, apiID);
    
    // Check if successful
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return;
    } else {
        cap.read(frame);
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            return;
        }
        imshow("Original Frame", frame);
    }

    // Define green and pink colour mask
    Mat green_mask;
    Mat pink_mask;

    // Initialise variables
    std::vector<std::vector<double>> toolLoc = {{0,0,0},{0,0,0}};
    double distData = 0;
    double prevDistData = MAX_DIST;
    double depthData = 0;
    double prevDepthData = MAX_DIST;
    int distBelowThresholdCount = 0;
    bool taskIsComplete = false;

    // Loop to gather tracking data
    int previousFrameNumber = 0; // use this variable to avoid printing duplicate data with BX

    // Have the user confirm they are ready, then count down until they can begin
    if (is_trial) {
        displayStartMessage(frame, COLOUR_BLUE, cap);
    }

    // Start stopwatch for timer
    auto stopwatch_begin = std::chrono::high_resolution_clock::now();

    // Main loop
    for (;;) {

        // Get new frame from camera and store it into frame
        cap.read(frame);
        
        // Check if successful
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        // Get new tool data using BX2
        std::vector<ToolData> newToolData = apiSupportsBX2 ? capi.getTrackingDataBX2("--6d=tools --3d=tools --sensor=none --1d=buttons") :
                                                             capi.getTrackingDataBX(TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms);

        // Update enabledTools array with new data
        for (int t = 0; t < enabledTools.size(); t++)
        {
            for (int j = 0; j < newToolData.size(); j++)
            {
                if (enabledTools[t].transform.toolHandle == newToolData[j].transform.toolHandle)
                {
                    // Copy the new tool data
                    newToolData[j].toolInfo = enabledTools[t].toolInfo; // keep the serial number
                    enabledTools[t] = newToolData[j]; // use the new data
                }
            }
        }

        // If we're using BX2 there's extra work to do because BX2 and BX use opposite philosophies.
        // BX will always return data for all enabled tools, but some of the data may be old: #linesWritten == # BX commands
        // BX2 never returns old data, but cannot guarantee new data for all enabled tools with each call: #linesWritten <= # BX2 commands
        // We want a CSV file with data for all enabled tools in each line, but this requires multiple BX2 calls.
        if (apiSupportsBX2)
        {
            // Count the number of tools that have new data
            int newDataCount = 0;
            for (int t = 0; t < enabledTools.size(); t++)
            {
                if (enabledTools[t].dataIsNew)
                {
                    newDataCount++;
                }
            }

            // Send another BX2 if some tools still have old data
            if (newDataCount < enabledTools.size())
            {
                continue;
            }
        }
        else
        {
            if (previousFrameNumber == enabledTools[0].frameNumber)
            {
                // If the frame number didn't change, don't print duplicate data to the CSV, send another BX
                continue;
            }
            else
            {
                // This frame number is different, so we'll print a line to the CSV, but remember it for next time
                previousFrameNumber = enabledTools[0].frameNumber;
            }
        }

        // Print if all enabled tools have new data
        for (int t = 0; t < enabledTools.size(); t++)
        {
            auto toolData = enabledTools[t];
            
            auto x = toolData.transform.tx;
            auto y = toolData.transform.ty;
            auto z = toolData.transform.tz;
            
            toolLoc[t] = {x,y,z};

            enabledTools[t].dataIsNew = false; // once printed, the data becomes "old"
        }
        // 1 Dimensional distance for obstructed view (x-axis)
        // If this value is positive the right hand tooltip is in front
        depthData = toolLoc[0][0] - toolLoc[1][0]; // Parallel axis
        // depthData = toolLoc[0][2] - toolLoc[1][2]; // Perpendicular axis

        // Straight line distance between the tooltips
        distData = std::sqrt(std::pow(toolLoc[0][0] - toolLoc[1][0], 2) +
                    std::pow(toolLoc[0][1] - toolLoc[1][1], 2) +
                    std::pow(toolLoc[0][2] - toolLoc[1][2], 2));
        
        // Get time at this point
        auto stopwatch_tick = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(stopwatch_tick - stopwatch_begin);
        
        // Control for sensor not scanning - use previous value
        if (enabledTools[0].transform.isMissing() || enabledTools[1].transform.isMissing()) {
            distData = prevDistData;
            depthData = prevDepthData;
        } else {
            prevDistData = distData;
            prevDepthData = depthData;
        }

        if (is_trial) {
            // Write to csv
            csv_file << elapsed.count();
            csv_file << "," << distData;
            csv_file << "," << depthData;
            csv_file << "," << std::dec << enabledTools.size();
            for (int t = 0; t < enabledTools.size(); t++)
            {
                csv_file << "," << enabledTools[t].toolInfo << "," << toolDataToCSV(enabledTools[t]);
                enabledTools[t].dataIsNew = false; // once printed, the data becomes "old"
            }
            csv_file << std::endl;
        }

        // Count how many frames the distance is below the threshold
        if (distData < DIST_THRESHOLD) {
            distBelowThresholdCount++;
        } else {
            distBelowThresholdCount = 0;
        }
        // Inform user the task is complete
        if (distBelowThresholdCount > BELOW_THRESHOLD_COUNT) {
            taskIsComplete = true;
            displayCompletionMessage(frame, COLOUR_BLUE);
            std::cout << "Task Completed!" << std::endl;
        }

        // Filter image for tooltips
        filterTooltips(frame,green_mask,LOWER_GREEN_MASK,UPPER_GREEN_MASK);
        filterTooltips(frame,pink_mask,LOWER_PINK_MASK,UPPER_PINK_MASK);

        // Find contours and centroids of tooltips from the masks
        Point tooltip1; // GREEN TOOLTIP
        Point tooltip2; // PINK TOOLTIP
        findLargestContour(green_mask, frame, tooltip1);
        findLargestContour(pink_mask, frame, tooltip2);
        vector<Point> tooltip_coords = vector<Point>{tooltip1,tooltip2};

        if (is_col_depth) {
            
            // Colour encode markers on the tooltips to indicate relative depth along the obstructed axis
            drawColourDepthEncoding(frame, tooltip_coords, depthData);
        }      
        if (is_min_dist) {

            // Draw the minimum distance vector between the tooltips
            drawMinimumDistVector(frame, tooltip_coords, distData);
        }  

        // Display the frame
        imshow("Original Frame", frame);

        // Get key to toggle view mode or quit program
        auto key = waitKey(5);
        
        // Used for test modes to toggle visualisations. Not to be used in trials
        if (key == '1') {
            is_min_dist = !is_min_dist;
        }
        else if (key == '2') {
            is_col_depth = !is_col_depth;
        }
        else if (key >= 0) {
            break;
        } else if (taskIsComplete && is_trial) {
            sleep(5);
            break;
        }
    }
    csv_file.close();
}

/**
 * @brief   The entry point for the CAPIsample application.
 * @details The invocation of CAPIsample is expected to pass a few arguments: ./CAPIsample [hostname] [args]
 *          arg(0) - (default)  The path to this application (ignore this)
 *          arg(1) - (required) The measurement device's hostname, IP address, or serial port.
 *          Eg: Connecting to device by IP address: "169.254.8.50"
 *          Eg: Connecting to device by zeroconf hostname: "P9-B0103.local"
 *          Eg: Connecting to serial port varies by OS: "COM10" (Win), "/dev/ttyUSB0" (Linux), "/dev/cu.usbserial-001014FA" (Mac)
 *          arg(2+) - (optional) Specify options such as: --encrypted, --scu=[scu_hostname], --tools=[file1.rom],[file2.rom]
 */
int main(int argc, char* argv[])
{
    // Print help information if the user has not specified any arguments
    if (argc < 2)
    {
        printHelp();
        return -1;
    }

    std::string inputName = argv[1];

    // if doesn't start with COM or dev
    if ( inputName.substr( 0, 3 ).compare( "COM" ) != 0 && inputName.substr( 0, 4 ).compare( "/dev" ) != 0 )
    {
        // Instead of a hostname, argv[1] may be a path to a GBF file to convert
        std::ifstream input( argv[1], std::ios::in | std::ios::binary );
        if ( input )
        {
            input.close();
            if ( argc == 3 )
            {
                if ( capi.convertGbfFileToText( argv[1], argv[2] ) == 0 )
                {
                    std::cout << "Saved file to " << argv[2] << " successfully." << std::endl;
                    // quit
                    return 0;
                }
                else
                {
                    // Failure
                    return -1;
                }
            }
            else
            {
                std::cerr << "GBF-to-text conversion requires an input and an output file to be specified." << std::endl;
                return -1;
            }
        }
    }

    // Assign the hostname
    std::string hostname = std::string( argv[1] );

    // Look for optional arguments
    std::string scu_hostname = "";
    std::vector<std::string> toolDefinitions = std::vector<std::string>();
    for (int i = 2; i < argc; i++)
    {
        std::string arg(argv[i]);
        if (arg.substr(0, 6).compare("--run") == 0)
        {
            std::cerr << "Running Visualisation Software." << std::endl;
        }
        else
        {
            std::cerr << "Unrecognized option: " << arg << std::endl;
            return -1;
        }
    }

    // Open the command channel to the device
    if (capi.connect(hostname, useEncryption ? Protocol::SecureTCP : Protocol::TCP, cipher) != 0)
    {
        // Print the error and exit if we can't connect to a device
        std::cout << "Connection Failed!" << std::endl;
        std::cout << "Press Enter to continue...";
        std::cin.ignore();
        return -1;
    }
    std::cout << "Connected!" << std::endl;

    // Wait a second - needed to support connecting to LEMO Vega
    sleepSeconds(1);

    // Print the firmware version for debugging purposes
    std::cout << capi.getUserParameter("Features.Firmware.Version") << std::endl;

    // Determine if the connected device supports the BX2 command
    determineApiSupportForBX2();

    // Initialize the system. This clears all previously loaded tools, unsaved settings etc...
    onErrorPrintDebugMessage("capi.initialize()", capi.initialize());

    // Demonstrate error handling by asking for tracking data in the wrong mode
    std::cout << capi.getTrackingDataTX() << std::endl;

    // Demonstrate getting/setting user parameters
    configureUserParameters();

    // Load any passive tool definitions from a .rom files
    if (toolDefinitions.size() > 0)
    {
        std::cout << "Loading Tool Definitions (.rom files) ..." << std::endl;
        for (int f = 0; f < toolDefinitions.size(); f++)
        {
            std::cout << "Loading: " << toolDefinitions[f] << std::endl;
            loadTool(toolDefinitions[f].c_str());
        }
    }

    // Wired tools are connected through a System Control Unit (SCU)
    if (scu_hostname.length() > 0)
    {
        configureActiveTools(scu_hostname);
    }

    // Dummy tools are used to report 3Ds in the absence of real tools.
    // TSTART will fail if real and dummy tools of the same type are enabled simultaneously.
    // To experiment with dummy tools, comment out the previous tool configurations first.
    // configureDummyTools();

    // Once loaded or detected, tools are initialized and enabled the same way
    // PHSR can be time consuming, so store the tool metadata immediately (eg. port handle, serial number)
    std::vector<ToolData> enabledTools = std::vector<ToolData>();
    initializeAndEnableTools(enabledTools);

    // Print an error if no tools were specified
    if (enabledTools.size() == 0)
    {
        std::cout << "No tools detected. To load passive tools, specify: --tools=[tool1.rom],[tool2.rom]" << std::endl;
    }

    // Spoofing system faults is handy to see how system alerts are handled, but faults can
    // prevent tracking data from being returned. This sample application has valued code simplicity
    // above robustness, so it does not gracefully handle every error case. You can experiment with
    // system faults, but attempting to do other tasks (eg. writing a .csv of tracking data) may fail
    // depending on what faults have been simulated.
    // simulateAlerts(0x7FFFFFFF);

    // Once the system is put into tracking mode, data is returned for whatever tools are enabled
    std::cout << std::endl << "Entering tracking mode..." << std::endl;
    onErrorPrintDebugMessage("capi.startTracking()", capi.startTracking());

    if (!apiSupportsStreaming)
    {
        // Get stream of tracking data and run the visualisation
        runVisualisation(enabledTools);

    }
    else
    {
        // Stream tracking data to the terminal on another thread
        std::thread streamingThread(useUDP ? streamUdp : streamTcp);

        // Command/response is possible while data is streaming.
        // Keep the command channel alive by issuing commands.
        // Param.Connect.Idle Timeout=300s by default will terminate the connection if there is no activity.
        // If there's an ERROR in the streamed replies, quit out of the polling loop
        while (detectKeystroke() == 0 && streamedReply.rfind( "ERROR", 0 ) != 0 )
        {
            // Periodically read the gravity vector
            sleepSeconds(1);
            capi.getUserParameter("Info.Status.Gravity Vector");
        }

        // Wait for the streaming thread to stop and cleanup
        streamingThread.join();
    }

    // Stop tracking (back to configuration mode)
    std::cout << std::endl << "Leaving tracking mode and returning to configuration mode..." << std::endl;
    onErrorPrintDebugMessage("capi.stopTracking()", capi.stopTracking());

    std::cout << "CAPI demonstration complete. Press any key to exit." << std::endl;
    return 0;
}