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


#define deg_to_rad(deg) (deg * M_PI / 180.0)
#define rad_to_deg(rad) (rad * 180.0 / M_PI)

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

struct plateResult {
    double confidence;
    std::string plateText;
};

struct positionReading {
    double latitude;
    double longitude;
    double time;
    cv::Point center;
    double distance;
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
    double distance(cv::Point topleft, cv::Point topright, cv::Point bottomleft, cv::Point bottomright);
    struct plateResult getBestPlate(std::vector<alpr::AlprPlateResult> plateReadings);
    double coord_course(struct coord origin, struct coord destination);
    struct coord coord_dist_radial(struct coord origin, double distance, double radial);
    double coord_distance(struct coord origin, struct coord destination);
    struct coord coord_interpolate(struct coord prev, struct coord next, double timenow);
};


#endif //INC_575_FINAL_PROJECT_PROCESSOR_H
