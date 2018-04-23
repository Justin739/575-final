//
// Created by justin on 4/22/18.
//

#ifndef INC_575_FINAL_PROJECT_UTILS_H
#define INC_575_FINAL_PROJECT_UTILS_H

#include <opencv2/core/types.hpp>

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

struct distanceReading {
    double distance;
    double sizeRatio;
    double height;
    double width;
};

struct positionReading {
    double latitude;
    double longitude;
    double time;
    cv::Point center;
    struct distanceReading distance;
    double course;
};

#endif //INC_575_FINAL_PROJECT_UTILS_H
