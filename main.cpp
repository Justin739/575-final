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

using namespace cv;
using namespace alpr;

void gps_updater(double *latitude, double *longitude) {
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

    while(1)
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


    double latitude = 0;
    double longitude = 0;

    // Start the thread to update GPS lat/long
    std::thread gps_thread(gps_updater, &latitude, &longitude);


    // Setup web cam
    int focus = 0;
    //camera.ChangeCaptureSize(v4l2::CAPTURE_SIZE_1920x1080);
    camera.ChangeCaptureSize(v4l2::CAPTURE_SIZE_1280x720);
    camera.SetFocus(focus);

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

    VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),30, Size(1280,720));

    while(camera.GrabFrame() && camera.RetrieveMat(frame)) {
        frameCounter++;

        video.write(frame);

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

            if (currPlate.bestPlate.overall_confidence > 85) {
                plateReadings.push_back(currPlate);
            }

            ////////////////////////////// GPS//////////////////////////////////////////

            //printf("Latitude: %.10f, Longitude: %.10f\n", latitude, longitude);
            //std::cout << "Latitude: " << latitude << ", Longitude: " << longitude << std::endl;

            ///////////////////////////// END OF GPS ///////////////////////////////////
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
                double confidence = 0;

                for (int i = 0; i < licensePlateLen; i++) {
                    for (int j = 0; j < plateReadings.size(); j++) {
                        if (i < indivChars[j].size()) {
                            histogram[indivChars[j][i]]++;
                        }
                    }

                    char mostLikelyChar = (char) std::distance(histogram, std::max_element(histogram, histogram + sizeof(histogram) / sizeof(char)));

                    confidence += histogram[mostLikelyChar] / (double) plateReadings.size();

                    plateString[i] = mostLikelyChar;
                    std::fill_n(histogram, sizeof(histogram) / sizeof(char), 0);
                }

                confidence = confidence / licensePlateLen;

                std::cout << "Confidence: " << confidence << std::endl;
                std::cout << "Final Result: ";
                for (int i = 0; i < licensePlateLen; i++) {
                    std::cout << plateString[i];
                    //csvLog << plateString[i];
                }

                std::cout << "" << std::endl;
                //csvLog << "\n";

                foundPlateCounter = 0;
                noPlateCounter = 0;
                plateReadings.clear();
            }
        }

        csvLog << std::setprecision(10) << frameCounter;
        csvLog << ",";
        csvLog << std::setprecision(10) << latitude;
        csvLog << ",";
        csvLog << std::setprecision(10) << longitude;
        csvLog << "\n";

        //std::cout << frameCounter << "," << latitude << "," << longitude << "\n" << std::endl;

        rectangle(frame, rect, Scalar(255,255,0));
        //imshow("test", frame);

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
    video.release();
    return 0;
}