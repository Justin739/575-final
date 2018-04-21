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

    while (inputCapture.read(frame)) {
        cv::imshow("Processed Video", frame);
        std::getline(frameFileStream, frameLine);
        struct frameData currFrameData = getFrameData(frameLine);

        if (currFrameData.frameTime > nextGPSData.timestamp) {
            prevGPSData = nextGPSData;
            std::getline(gpsFileStream, gpsLine);
            nextGPSData = getGPSData(gpsLine);
        }

        std::cout << "Frame Number: " << std::setprecision(20) << currFrameData.frameNum << std::endl;
        std::cout << "Frame Time: " << std::setprecision(20) << currFrameData.frameTime << std::endl;
        std::cout << "Prev GPS Time: " << std::setprecision(20) << prevGPSData.timestamp << std::endl;
        std::cout << "Next GPS Time: " << std::setprecision(20) << nextGPSData.timestamp << std::endl;




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