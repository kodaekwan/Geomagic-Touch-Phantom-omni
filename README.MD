## Test Environment
# [OS]
Windows11 - Ubuntu 20.04

# [WSL2] -> .wslconfig
networkingMode=mirrored

## Setup
# openhaptics SDK
link = https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US

# Driver 2016 version
"geomagic_touch_device_driver_2016.1-1-amd64.zip" 

## Install for CPP
$ sudo apt install g++
$ sudo apt install freeglut3-dev
$ sudo apt install libncurses5-dev
$ sudo apt install zlib1g-dev
$ sudo apt install libncurses5

## Install for python
pip install sysv-ipc
pip install numpy

## CMake
$ mkdir your_dir
$ cd your_dir
$ git clone URL
$ cmake CmakeLists.txt
$ cd build
$ make

## Run
# CPP
$ ./your_dir/build/omni

# Python
$ python3 omni_demo.py

## References
This project is based on:
- [phantom_omni by danepowell](https://github.com/danepowell/phantom_omni) - under the MIT License

Modifications include:
- Removed ROS dependency
- Integrated shared memory access
- Adapted for standalone C++ use


