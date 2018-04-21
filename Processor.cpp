//
// Created by justin on 4/21/18.
//

#include "Processor.h"

Processor::Processor(std::string inputVideo, std::string outputVideo, std::string frames, std::string gps) {
    inputVideoFile = inputVideo;
    outputVideoFile = outputVideo;
    frameTimesFile = frames;
    gpsFile = gps;
}

void Processor::processData() {
    // Initialize the library using United States style license plates.
    // You can use other countries/regions as well (for example: "eu", "au", or "kr")
    alpr::Alpr openalpr("us", "/usr/share/openalpr/config/openalpr.defaults.conf");

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
    std::vector<alpr::AlprRegionOfInterest> roi;

    cv::VideoCapture inputCapture(inputVideoFile);
    cv::VideoWriter outputWriter(outputVideoFile,
                             inputCapture.get(CV_CAP_PROP_FOURCC),
                             inputCapture.get(CV_CAP_PROP_FPS),
                             cv::Size(inputCapture.get(CV_CAP_PROP_FRAME_WIDTH),
                                  inputCapture.get(CV_CAP_PROP_FRAME_HEIGHT)));

    if (!inputCapture.isOpened()) {
        std::cout << "Failed to open video." << std::endl;
        return;
    }

    if (!outputWriter.isOpened()) {
        std::cout << "Could not create output file." << std::endl;
        return;
    }

    std::ifstream frameFileStream(frameTimesFile);
    std::string frameLine;

    if (!frameFileStream) {
        std::cout << "Could not open frame time file." << std::endl;
        return;
    }

    std::ifstream gpsFileStream(gpsFile);
    std::string gpsLine;

    if (!gpsFileStream) {
        std::cout << "Could not open raw GPS data file." << std::endl;
        return;
    }

    cv::Mat frame;

    cv::namedWindow("Processed Video", 1);

    struct coord prevGPSData;
    struct coord nextGPSData;

    // Get the first two lines in the GPS file
    std::getline(gpsFileStream, gpsLine);
    prevGPSData = getGPSData(gpsLine);
    std::getline(gpsFileStream, gpsLine);
    nextGPSData = getGPSData(gpsLine);

    inputCapture.read(frame);

    int width = 500;
    int height = 400;
    cv::Rect rect((frame.cols / 2) - (width / 2), frame.rows - height - 200, width, height);
    roi.push_back(alpr::AlprRegionOfInterest(rect.x, rect.y, rect.width, rect.height));

    int foundPlateCounter = 0;
    int noPlateCounter = 0;
    std::vector<alpr::AlprPlateResult> plateReadings;

    while (inputCapture.read(frame)) {
        if (frameFileStream.is_open()) {
            std::cout << "File stream is open" << std::endl;
        } else {
            std::cout << "File NOT open" << std::endl;
        }
        std::getline(frameFileStream, frameLine);
        struct frameData currFrameData = getFrameData(frameLine);

        if (currFrameData.frameTime > nextGPSData.timestamp) {
            prevGPSData = nextGPSData;
            std::getline(gpsFileStream, gpsLine);
            nextGPSData = getGPSData(gpsLine);
        }

        std::cout << "Frame Number: " << std::setprecision(20) << currFrameData.frameNum << std::endl;
        std::cout << "Frame Time: " << std::setprecision(20) << currFrameData.frameTime << std::endl;
        //std::cout << "Prev GPS Time: " << std::setprecision(20) << prevGPSData.timestamp << std::endl;
        //std::cout << "Next GPS Time: " << std::setprecision(20) << nextGPSData.timestamp << std::endl;

        alpr::AlprResults frameResults = openalpr.recognize(frame.data, 3, frame.cols, frame.rows, roi);


        for (int i = 0; i < frameResults.plates.size(); i++) {
            alpr::AlprPlateResult currPlate = frameResults.plates[i];
            cv::Point topleft(currPlate.plate_points[0].x, currPlate.plate_points[0].y);
            cv::Point topright(currPlate.plate_points[1].x, currPlate.plate_points[1].y);
            cv::Point bottomright(currPlate.plate_points[2].x, currPlate.plate_points[2].y);
            cv::Point bottomleft(currPlate.plate_points[3].x, currPlate.plate_points[3].y);
            cv::Scalar red(0, 0, 255);

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
            std::vector<alpr::AlprPlate> topPlates = currPlate.topNPlates;
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

                for (int i = 0; i < plateReadings.size(); i++) {
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

                    int maxFreq = 0;
                    char mostLikelyChar = 0;

                    for (char histoIter = 0; histoIter < (sizeof(histogram) / sizeof(char)); histoIter++) {
                        if (histogram[histoIter] > maxFreq) {
                            maxFreq = histogram[histoIter];
                            mostLikelyChar = histoIter;
                        }
                    }

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

        // Draw the ROI
        rectangle(frame, rect, cv::Scalar(255,255,0));

        // Show the final processed frame
        cv::imshow("Processed Video", frame);

        // Check for user input to stop or pause the processing
        int keyPressed = cv::waitKey(1);

        if (keyPressed == 27) {
            break;
        } else if (keyPressed == 32) {
            cv::waitKey(0);
        }

        outputWriter.write(frame);
    }

    inputCapture.release();
    outputWriter.release();
}

struct frameData Processor::getFrameData(std::string line) {
    struct frameData lineData;
    std::stringstream lineStream(line);
    std::string token;
    int i = 0;

    while (std::getline(lineStream, token, ',')) {
        switch (i) {
            case 0:
                lineData.frameNum = std::stoi(token);
                break;
            case 1:
                lineData.frameTime = std::stod(token);
                break;
            default:
                std::cout << "File error" << std::endl;
        }

        i++;
    }

    return lineData;
}

struct coord Processor::getGPSData(std::string line) {
    struct coord lineData;
    lineData.valid = 1;
    std::stringstream lineStream(line);
    std::string token;
    int i = 0;

    while (std::getline(lineStream, token, ',')) {
        switch (i) {
            case 0:
                lineData.timestamp = std::stod(token);
                break;
            case 1:
                lineData.lat = std::stod(token);
                break;
            case 2:
                lineData.lon = std::stod(token);
                break;
            default:
                std::cout << "File error" << std::endl;
        }

        i++;
    }

    return lineData;
}