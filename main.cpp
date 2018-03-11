#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <alpr.h>

using namespace cv;
using namespace alpr;

int main(int argc, char* argv[]) {

    // Load input video
    //  If your video is in a different source folder than your code,
    //  make sure you specify the directory correctly!
    VideoCapture input_cap("high_res.avi");

    // Check validity of target file
    if(!input_cap.isOpened()) {
        std::cout << "Input video not found." << std::endl;
        return -1;
    }


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
    namedWindow("test", 1);

    std::vector<AlprRegionOfInterest> roi;
    input_cap.read(frame);

    int width = 250;
    int height =600;
    Rect rect(frame.cols/2-width/2, frame.rows-height, width, height);
    roi.push_back(AlprRegionOfInterest(rect.x, rect.y, rect.width, rect.height));
    //Size scaledSize(640, 360);
    int frameCounter = 0;
    int foundPlateCounter = 0;
    int noPlateCounter = 0;

    while(input_cap.read(frame)) {
        frameCounter++;

        if (frameCounter < 1500) {
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
        }

        if (frameResults.plates.size() > 0) {
            foundPlateCounter++;
            noPlateCounter = 0;
        } else if (foundPlateCounter > 0) {
            noPlateCounter++;

            if (noPlateCounter >= 5) {
                std::cout << "MIDDLE OF CARS" << std::endl;
                foundPlateCounter = 0;
                noPlateCounter = 0;
            }
        }

        rectangle(frame, rect, Scalar(255,255,0));
        putText(frame, "Hello World!",
                Point(0, 50),
                FONT_HERSHEY_PLAIN,
                1.0,
                Scalar(255, 255, 255));

        imshow("test", frame);

        char keyPressed = waitKey(30);

        if (keyPressed == 27) {
            break;
        } else if (keyPressed == 32) {
            waitKey(0);
        }
    }


    // free the capture objects from memory
    input_cap.release();
    return 0;
}