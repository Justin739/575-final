//
// Created by justin on 4/20/18.
//

#ifndef INC_575_FINAL_PROJECT_CAPTURER_H
#define INC_575_FINAL_PROJECT_CAPTURER_H

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <iomanip>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "utils.h"

class Capturer {
    bool running;
    std::string inputDevice;
    std::string outputVideo;
    std::string outputCSV;

public:
    Capturer(std::string camera, std::string videoName, std::string csvName);
    void startCapture();
    void stopCapture();
};


#endif //INC_575_FINAL_PROJECT_CAPTURER_H
