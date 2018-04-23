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
#include "utils.h"

class Processor {
    std::string inputVideoFile;
    std::string outputVideoFile;
    std::string frameTimesFile;
    std::string gpsFile;
    struct coord spots[44];

public:
    Processor(std::string inputVideo, std::string outputVideo, std::string frames, std::string gps);
    void processData();

private:
    struct frameData getFrameData(std::string line);
    struct coord getGPSData(std::string line);
    struct distanceReading distance(cv::Point topleft, cv::Point topright, cv::Point bottomleft, cv::Point bottomright);
    struct plateResult getBestPlate(std::vector<alpr::AlprPlateResult> plateReadings);
    double coord_course(struct coord origin, struct coord destination);
    struct coord coord_dist_radial(struct coord origin, double distance, double radial);
    double coord_distance(struct coord origin, struct coord destination);
    struct coord coord_interpolate(struct coord prev, struct coord next, double timenow);
    struct coord getCarPosition(std::vector<struct positionReading> positionResults, cv::Rect roiRect, double frameWidthX);
};


#endif //INC_575_FINAL_PROJECT_PROCESSOR_H
