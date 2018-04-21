//
// Created by justin on 4/21/18.
//

#ifndef INC_575_FINAL_PROJECT_PROCESSOR_H
#define INC_575_FINAL_PROJECT_PROCESSOR_H

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <alpr.h>
#include <algorithm>

struct frameData {
    int frameNum;
    double frameTime;
};

struct coord {
    double lat;
    double lon;
    double timestamp;
    int valid;
};

class Processor {
    std::string inputVideoFile;
    std::string outputVideoFile;
    std::string frameTimesFile;
    std::string gpsFile;

public:
    Processor(std::string inputVideo, std::string outputVideo, std::string frames, std::string gps);
    void processData();

private:
    struct frameData getFrameData(std::string line);
    struct coord getGPSData(std::string line);
};


#endif //INC_575_FINAL_PROJECT_PROCESSOR_H
