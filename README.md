# Dean Zworestine's Undergraduate Thesis
This is a project which is built for an undergraduate thesis titled ***Augmented Reality for the Operation of Surgical Robots***. It utilises Aurora NDI sensors and openCV to provide depth perception to users of UNSW Mechatronics Laboratory's surgical robot.

Much of this code has been adapted from NDI's CAPI Sample, which can be found at: https://github.com/Oct19/NDI-API-Sample-v1.4.0/tree/master

Note that this code is intended to be run in VSCode on Mac.

## Setup:

1. Ensure VSCode is installed.

2. Ensure Xcode command line tools are installed by running the following in terminal:
xcode-select --install

3. Ensure homebrew is installed. To install run the following command in terminal:
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

4. Install CMake in your terminal using homebrew:
brew install cmake

5. Install openCV in your teminal using homebrew:
brew install opencv


## To Build and Run:

1. Open the root directory of your project in VSCode

2. Install the following VSCode extensions:
    - C/C++
    - CMake
    - CMake Tools

3. Open the Command Palette (Cmd+Shift+P) and run CMake: Configure. Ensure Clang is selected as the kit.

4. Run CMake: Build from the Command Palette. Note that you can also build the project by
   navigating to the project's build folder in your terminal and run make

5. To run the sample code, find the port in which the Aurora is connected.
   This is usually in /dev/ at the root of your Mac.
   It should look similar to /dev/cu.usbserial-1430

6. Run the code with the following command from the build folder:
    ./depth_rendering <port_name> --run

    For example:
    ./depth_rendering /dev/cu.usbserial-1430 --run


## Troubleshooting:
The openCV path might differ between systems, which will require a change to CMakeLists.txt
To find the opencv path, run the following command:
    brew info opencv

This command will give you detailed information about the OpenCV installation,
including the path, which will look like:
    /usr/local/Cellar/opencv/4.6.0_1

Paste this path in line 11 of the CmakeLists.txt file in the home directory of the project