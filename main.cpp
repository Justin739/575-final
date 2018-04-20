#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <alpr.h>
#include "C920Camera.h"
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

struct capturedFrame {
    Mat frame;
    double latitude;
    double longitude;
    int frameCounter;
};

void process_frames(std::queue<struct capturedFrame>* capturedFrames, bool* running);
void gps_updater(double* latitude, double* longitude, bool* running);

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

void gps_updater(double* latitude, double* longitude, bool* running) {
    char serial_path[120];
    strcpy(serial_path, "/dev/ttyACM0");

    int ser;
    ser = open(serial_path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (ser == -1) {
        fprintf(stderr, "opening serial port %s failed: %s\n", serial_path, strerror(errno));
        printf("hint: is the power on and the usb plugged in?\n");
        exit(1);
    }

    //fcntl(ser, F_SETFL, 0);

    struct termios options;
    tcgetattr(ser, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CRTSCTS;
    tcsetattr(ser, TCSAFLUSH, &options);

    char rx_buffer[4096];
    int rx_buffer_bytes;
    char line_buffer[1024];

    while(*running)
    {
        usleep(100000);

        ioctl(ser, FIONREAD, &rx_buffer_bytes);
        read(ser, rx_buffer, rx_buffer_bytes);
        char c;
        int start = 0;
        int end = 0;
        int i;
        for (i = 0; i < rx_buffer_bytes; i++)
        {
            c = rx_buffer[i];
            if (c == '\n')
            {
                end = i;
                int j;
                for (j = start; j < end; j++);
                memcpy(line_buffer, &rx_buffer[start], end - start);
                line_buffer[end-start-1] = '\0';
                start = end + 1;

                //printf("%s\n", line_buffer);

                // process line buffer
                if (strncmp(line_buffer, "$GPGLL", 6) == 0) {
                    // Position data: position fix, time of position fix, and status
                    // $GPGLL,4202.38085,N,09338.50822,W,004402.00,A,D*72
                    // 0 	Message ID $GPGLL
                    // 1 	Latitude in dd mm,mmmm format (0-7 decimal places)
                    // 2 	Direction of latitude N: North S: South
                    // 3 	Longitude in ddd mm,mmmm format (0-7 decimal places)
                    // 4 	Direction of longitude E: East W: West
                    // 5 	UTC of position in hhmmss.ss format
                    // 6 	Fixed text "A" shows that data is valid
                    // 7 	The checksum data, always begins with *
                    char extractor[16];
                    extractor[0] = line_buffer[7];
                    extractor[1] = line_buffer[8];
                    extractor[2] = '\0';
                    int lat_dec = atoi(extractor);

                    extractor[0] = line_buffer[9];
                    extractor[1] = line_buffer[10];
                    extractor[2] = line_buffer[11];
                    extractor[3] = line_buffer[12];
                    extractor[4] = line_buffer[13];
                    extractor[5] = line_buffer[14];
                    extractor[6] = line_buffer[15];
                    extractor[7] = line_buffer[16];
                    extractor[8] = '\0';

                    double lat_min = atof(extractor);

                    double lat = lat_dec + (lat_min / 60.0);
                    if (line_buffer[18] != 'N')
                    {
                        lat *= -1.0;
                    }

                    extractor[0] = line_buffer[20];
                    extractor[1] = line_buffer[21];
                    extractor[2] = line_buffer[22];
                    extractor[3] = '\0';
                    int lon_dec = atoi(extractor);

                    extractor[0] = line_buffer[23];
                    extractor[1] = line_buffer[24];
                    extractor[2] = line_buffer[25];
                    extractor[3] = line_buffer[26];
                    extractor[4] = line_buffer[27];
                    extractor[5] = line_buffer[28];
                    extractor[6] = line_buffer[29];
                    extractor[7] = line_buffer[30];
                    extractor[8] = '\0';

                    double lon_min = atof(extractor);

                    double lon = lon_dec + (lon_min / 60.0);
                    if (line_buffer[32] != 'E')
                    {
                        lon *= -1.0;
                    }

                    // hhmmss.ss
                    extractor[0] = line_buffer[34];
                    extractor[1] = line_buffer[35];
                    extractor[2] = '\0';
                    int hrs = atoi(extractor);

                    extractor[0] = line_buffer[36];
                    extractor[1] = line_buffer[37];
                    extractor[2] = '\0';
                    int min = atoi(extractor);

                    extractor[0] = line_buffer[38];
                    extractor[1] = line_buffer[39];
                    extractor[2] = line_buffer[40];
                    extractor[3] = line_buffer[41];
                    extractor[4] = line_buffer[42];
                    extractor[5] = '\0';
                    double sec = atof(extractor);

                    double time = sec + min * 60.0 + hrs * 3600.0;

                    *latitude = lat;
                    *longitude = lon;
                    //printf("%f:\t%f,%f\n", time, lat, lon);
                    //printf("%s\n", line_buffer);
                }
            }
        }
    }
}

int main(int argc, char* argv[]) {
    double latitude = 0;
    double longitude = 0;
    bool running = true;
    std::queue<struct capturedFrame> capturedFrames;

    // Start the thread to update GPS lat/long whenever the GPS gets an update
    //std::thread gps_thread(gps_updater, &latitude, &longitude, &running);

    // Open up the webcam to start capturing frames
    VideoCapture input_cap("/dev/video1");

    // Ensure the webcam was found and opened
    if(!input_cap.isOpened()) {
        std::cout << "Input video not found." << std::endl;
        return 0;
    }

    // Force the capture resolution to be 1920 x 1080 @ 30fps
    input_cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    input_cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    input_cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

    // Open the output capture
    VideoWriter output_cap("recorded_video.avi",
                           input_cap.get(CV_CAP_PROP_FOURCC),
                           input_cap.get(CV_CAP_PROP_FPS),
                           Size(input_cap.get(CV_CAP_PROP_FRAME_WIDTH),
                                input_cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

    // Matrix to store the frame contents temporarily
    Mat frame;

    // Counts the current frame being shown
    int frameCounter = 1;

    std::ofstream csvLog;
    csvLog.open("frame_times.csv");

    // Open a window to detect key presses
    namedWindow("Display window", WINDOW_AUTOSIZE);

    // Holds the current time
    timeval tv;

    // Continuously save frames until the webcam has closed
    while (input_cap.read(frame) && running) {
        output_cap.write(frame);
        gettimeofday(&tv, NULL);
        double currTime = tv.tv_sec + (tv.tv_usec / 1000000.0);
        csvLog << frameCounter << "," << std::setprecision(20) << currTime << "\n";

        if (waitKey(1) == 27) {
            running = false;
        }

        frameCounter++;
    }

    input_cap.release();
    output_cap.release();
    csvLog.close();
    //gps_thread.join();
    return 0;
}