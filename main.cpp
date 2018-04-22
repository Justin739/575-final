#include "Capturer.h"
#include "Processor.h"

/*
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <alpr.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <thread>         // std::thread
#include <stack>

#include <sys/time.h>

using namespace cv;
using namespace alpr;

void process_frames(std::queue<struct capturedFrame>* capturedFrames, bool* running);

void process_frames(std::queue<struct capturedFrame>* capturedFrames, bool* running) {
    // Initialize the library using United States style license plates.
    // You can use other countries/regions as well (for example: "eu", "au", or "kr")
    Alpr openalpr("us", "/usr/share/openalpr/config/openalpr.defaults.conf");

    // Optionally specify the top N possible plates to return (with confidences).  Default is 10
    openalpr.setTopN(5);

    // Optionally, provide the library with a region for pattern matching.  This improves accuracy by
    // comparing the plate text with the regional pattern.
    openalpr.setDefaultRegion("ia");

    // Make sure the library loaded before continuing.
    // For example, it could fail if the config/runtime_data is not found
    if (!openalpr.isLoaded()) {
        std::cerr << "Error loading OpenALPR" << std::endl;
        return;
    }

    // Region of interest to improve performance
    std::vector<AlprRegionOfInterest> roi;

    // Center the region of interest and save it into a rectangle object
    // TODO: Need to read the first frame before being able to initialize ROI
    while (capturedFrames->empty()) {
        // Wait for first frame to be taken - break if webcam stops capturing
        if (!(*running)) {
            return;
        }
    }

    // Save the region to the stack
    struct capturedFrame firstFrame = capturedFrames->front();
    Rect rect(0, 0, firstFrame.frame.cols, firstFrame.frame.rows);
    roi.emplace_back(AlprRegionOfInterest(rect.x, rect.y, rect.width, rect.height));
    capturedFrames->pop();

    int foundPlateCounter = 0;
    int noPlateCounter = 0;
    std::vector<AlprPlateResult> plateReadings;

    while (*running || !capturedFrames->empty()) {
        if (!capturedFrames->empty()) {
            // TODO: Debug mode - need to remove
            if (!(*running)) {
                std::cout << capturedFrames->size() << std::endl;
            }

            struct capturedFrame currFrame = capturedFrames->front();
            Mat frame = currFrame.frame;
            AlprResults frameResults = openalpr.recognize(frame.data, 3, frame.cols, frame.rows, roi);

            for (int i = 0; i < frameResults.plates.size(); i++) {
                AlprPlateResult currPlate = frameResults.plates[i];
                Point topleft(currPlate.plate_points[0].x, currPlate.plate_points[0].y);
                Point topright(currPlate.plate_points[1].x, currPlate.plate_points[1].y);
                Point bottomright(currPlate.plate_points[2].x, currPlate.plate_points[2].y);
                Point bottomleft(currPlate.plate_points[3].x, currPlate.plate_points[3].y);
                Scalar red(0, 0, 255);

                line(frame, topleft, topright, red);
                line(frame, topright, bottomright, red);
                line(frame, bottomright, bottomleft, red);
                line(frame, bottomleft, topleft, red);

                int xCenter = (currPlate.plate_points[0].x + currPlate.plate_points[1].x + currPlate.plate_points[2].x +
                               currPlate.plate_points[3].x) / 4;
                int yCenter = (currPlate.plate_points[0].y + currPlate.plate_points[1].y + currPlate.plate_points[2].y +
                               currPlate.plate_points[3].y) / 4;

                std::string licensePlateText = currPlate.bestPlate.characters;
                float confidenceLevel = currPlate.bestPlate.overall_confidence;
                std::vector<AlprPlate> topPlates = currPlate.topNPlates;
                std::cout << i << " " << licensePlateText << " " << confidenceLevel << std::endl;

                if (currPlate.bestPlate.overall_confidence > 85) {
                    plateReadings.push_back(currPlate);
                }
            }

            if (frameResults.plates.size() > 0) {
                foundPlateCounter++;
                noPlateCounter = 0;
            } else if (foundPlateCounter > 0) {
                noPlateCounter++;

                if (noPlateCounter >= 5) {
                    std::cout << "MIDDLE OF CARS" << std::endl;
                    // <Process the last car here>
                    std::vector<char> indivChars[plateReadings.size()];
                    int licensePlateLen = 0;
                    std::map<int, int> maxLicenseLen;

                    for (int i = 0; i != plateReadings.size(); i++) {
                        std::string plateChars = plateReadings[i].bestPlate.characters;

                        if (maxLicenseLen.find((int) plateChars.length()) != maxLicenseLen.end()) {
                            maxLicenseLen[(int) plateChars.length()]++;
                        } else {
                            maxLicenseLen.insert(std::make_pair((int) plateChars.length(), 1));
                        }

                        for (int j = 0; j < plateChars.length(); j++) {
                            //std::cout << plateChars[j] << std::endl;
                            indivChars[i].push_back(plateChars[j]);
                        }
                    }

                    std::map<int, int>::iterator iter;
                    int maxFreq = 0;

                    for (iter = maxLicenseLen.begin(); iter != maxLicenseLen.end(); iter++) {
                        if (iter->second > maxFreq) {
                            maxFreq = iter->second;
                            licensePlateLen = iter->first;
                        }
                    }

                    int histogram[128] = {};
                    char plateString[licensePlateLen] = {};
                    double confidence = 0;

                    for (int i = 0; i < licensePlateLen; i++) {
                        for (int j = 0; j < plateReadings.size(); j++) {
                            if (i < indivChars[j].size()) {
                                histogram[indivChars[j][i]]++;
                            }
                        }

                        auto mostLikelyChar = (char) std::distance(histogram, std::max_element(histogram, histogram +
                                                                                                          sizeof(histogram) /
                                                                                                          sizeof(char)));
                        confidence += histogram[mostLikelyChar] / (double) plateReadings.size();
                        plateString[i] = mostLikelyChar;
                        std::fill_n(histogram, sizeof(histogram) / sizeof(char), 0);
                    }

                    confidence = confidence / licensePlateLen;
                    std::cout << "Confidence: " << confidence << std::endl;
                    std::cout << "Final Result: ";
                    for (int i = 0; i < licensePlateLen; i++) {
                        std::cout << plateString[i];
                    }

                    std::cout << "" << std::endl;
                    foundPlateCounter = 0;
                    noPlateCounter = 0;
                    plateReadings.clear();
                }
            }

            capturedFrames->pop();
        }
    }
}
*/
void monitorStop(Capturer* cap) {
    // Open a window to detect key presses
    cv::namedWindow("Stopper Window", cv::WINDOW_AUTOSIZE);

    if (cv::waitKey(0) == 27) {
        std::cout << "--- Stopping capture ---" << std::endl;
        cap->stopCapture();
    }
}

int main(int argc, char* argv[]) {

    Capturer cap("/dev/video1", "recorded_video.avi", "frame_times.csv");
    std::thread stopper_thread(monitorStop, &cap);

    std::cout << "--- Starting capture ---" << std::endl;
    cap.startCapture();
    stopper_thread.join();

    // Start processing

    Processor proc("recorded_video.avi", "processed_video.avi", "frame_times.csv", "raw_log.txt");
    proc.processData();

    return 0;
}