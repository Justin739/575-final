#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <alpr.h>
#include "C920Camera.h"

using namespace cv;
using namespace alpr;

int main(int argc, char* argv[]) {

    // Load input video
    //  If your video is in a different source folder than your code,
    //  make sure you specify the directory correctly!
    //VideoCapture input_cap("high_res.avi");

    v4l2::C920Camera camera;
    camera.Open("/dev/video1");
    //VideoCapture input_cap("/dev/video1");

    // Check validity of target file
    if(!camera.IsOpen()) {
        std::cout << "Input video not found." << std::endl;
        return -1;
    }


    // Setup web cam
    int focus = 0;
    //int gain = 255;
    camera.ChangeCaptureSize(v4l2::CAPTURE_SIZE_1920x1080);
    camera.SetFocus(focus);
    //camera.SetBacklightCompensation(gain);
    //camera.SetBrightness(gain);
    //camera.SetContrast(gain);
    //camera.SetSaturation(gain);
    //camera.SetSharpness(gain);

    //camera.SetGain(gain);

    // Initialize the library using United States style license plates.
    // You can use other countries/regions as well (for example: "eu", "au", or "kr")
    Alpr openalpr("us", "/usr/share/openalpr/config/openalpr.defaults.conf");

    // Optionally specify the top N possible plates to return (with confidences).  Default is 10
    openalpr.setTopN(5);

    // Optionally, provide the library with a region for pattern matching.  This improves accuracy by
    // comparing the plate text with the regional pattern.
    //openalpr.setDefaultRegion("md");

    // Make sure the library loaded before continuing.
    // For example, it could fail if the config/runtime_data is not found
    if (openalpr.isLoaded() == false) {
        std::cerr << "Error loading OpenALPR" << std::endl;
        return 1;
    }

    // Loop to read from input one frame at a time, write text on frame, and
    // copy to output video
    Mat frame;
    //namedWindow("test", 1);

    std::vector<AlprRegionOfInterest> roi;

    if (camera.GrabFrame()) {
        camera.RetrieveMat(frame);
    }

    std::ofstream csvLog;
    csvLog.open("output.csv");

    int width = 500;
    int height = 400;
    Rect rect(frame.cols/2-width/2, frame.rows-height - 200, width, height);
    roi.push_back(AlprRegionOfInterest(rect.x, rect.y, rect.width, rect.height));
    //Size scaledSize(640, 360);
    long frameCounter = 0;
    int foundPlateCounter = 0;
    int noPlateCounter = 0;
    std::vector<AlprPlateResult> plateReadings;

    while(camera.GrabFrame() && camera.RetrieveMat(frame)) {
        frameCounter++;

        if (frameCounter < 0) {
            continue;
        }

        //resize(frame, frame, scaledSize, 0, 0, INTER_NEAREST);
        AlprResults frameResults = openalpr.recognize(frame.data, 3, frame.cols, frame.rows, roi);

        for (int i = 0; i < frameResults.plates.size(); i++)
        {
            AlprPlateResult currPlate = frameResults.plates[i];
            Point topleft(currPlate.plate_points[0].x, currPlate.plate_points[0].y);
            Point topright(currPlate.plate_points[1].x, currPlate.plate_points[1].y);
            Point bottomright(currPlate.plate_points[2].x, currPlate.plate_points[2].y);
            Point bottomleft(currPlate.plate_points[3].x, currPlate.plate_points[3].y);
            Scalar red(0,0,255);

            //rectangle(frame, Rect(x_p, y_p, width_p, height_p), Scalar(0,0,255));

            line(frame, topleft, topright, red);
            line(frame, topright, bottomright, red);
            line(frame, bottomright, bottomleft, red);
            line(frame, bottomleft, topleft, red);

            std::string licensePlateText = currPlate.bestPlate.characters;
            float confidenceLevel = currPlate.bestPlate.overall_confidence;
            std::vector<AlprPlate> topPlates = currPlate.topNPlates;
            std::cout << i << " " << licensePlateText << " "<< confidenceLevel << std::endl;
            plateReadings.push_back(currPlate);
        }

        if (frameResults.plates.size() > 0) {
            foundPlateCounter++;
            noPlateCounter = 0;
        } else if (foundPlateCounter > 0) {
            noPlateCounter++;

            if (noPlateCounter >= 5) {
                //std::cout << "MIDDLE OF CARS" << std::endl;
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

                char histogram[128] = {};
                char plateString[licensePlateLen] = {};

                for (int i = 0; i < licensePlateLen; i++) {
                    for (int j = 0; j < plateReadings.size(); j++) {
                        if (i < indivChars[j].size()) {
                            histogram[indivChars[j][i]]++;
                        }
                    }

                    char mostLikelyChar = (char) std::distance(histogram, std::max_element(histogram, histogram + sizeof(histogram) / sizeof(char)));
                    plateString[i] = mostLikelyChar;
                    std::fill_n(histogram, sizeof(histogram) / sizeof(char), 0);
                }

                for (int i = 0; i < licensePlateLen; i++) {
                    std::cout << plateString[i];
                    csvLog << plateString[i];
                }

                std::cout << "" << std::endl;
                csvLog << "\n";

                foundPlateCounter = 0;
                noPlateCounter = 0;
                plateReadings.clear();
            }
        }

        rectangle(frame, rect, Scalar(255,255,0));
        imshow("test", frame);

        char keyPressed = waitKey(30);

        if (keyPressed == 27) {
            break;
        } else if (keyPressed == 32) {
            waitKey(0);
        }
    }


    // free the capture objects from memory
    camera.Close();
    csvLog.close();
    return 0;
}