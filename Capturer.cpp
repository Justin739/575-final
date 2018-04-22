//
// Created by justin on 4/20/18.
//

#include "Capturer.h"

Capturer::Capturer(std::string camera, std::string videoName, std::string csvName) {
    inputDevice = camera;
    outputVideo = videoName;
    outputCSV = csvName;
}

void Capturer::startCapture() {
    // Open up the webcam to start capturing frames
    cv::VideoCapture input_cap(inputDevice);

    // Ensure the webcam was found and opened
    if(!input_cap.isOpened()) {
        std::cout << "Input video not found." << std::endl;
        return;
    }

    // Force the capture resolution to be 1920 x 1080 @ 30fps
    input_cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    input_cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    input_cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    input_cap.set(CV_CAP_PROP_AUTOFOCUS, 0);

    // Open the output capture
    cv::VideoWriter output_cap(outputVideo,
                           input_cap.get(CV_CAP_PROP_FOURCC),
                           input_cap.get(CV_CAP_PROP_FPS),
                           cv::Size(input_cap.get(CV_CAP_PROP_FRAME_WIDTH),
                                input_cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

    // Matrix to store the frame contents temporarily
    cv::Mat frame;

    // Counts the current frame being shown
    int frameCounter = 1;

    std::ofstream csvLog;
    csvLog.open(outputCSV);

    // Holds the current time
    timeval tv;

    std::cout << "--- Capture started ---" << std::endl;
    running = true;

    // Continuously save frames until the webcam has closed
    while (input_cap.read(frame) && running) {
        output_cap.write(frame);
        gettimeofday(&tv, nullptr);
        double currTime = tv.tv_sec + (tv.tv_usec / 1000000.0);
        csvLog << frameCounter << "," << std::setprecision(20) << currTime << "\n";
        frameCounter++;
    }

    std::cout << "--- Capture stopped ---" << std::endl;
    input_cap.release();
    output_cap.release();
    csvLog.close();
}

void Capturer::stopCapture() {
    running = false;
}