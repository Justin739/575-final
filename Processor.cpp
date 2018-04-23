//
// Created by justin on 4/21/18.
//

#include "Processor.h"

Processor::Processor(std::string inputVideo, std::string outputVideo, std::string frames, std::string gps) {
    inputVideoFile = inputVideo;
    outputVideoFile = outputVideo;
    frameTimesFile = frames;
    gpsFile = gps;

    srand(time(NULL));

// 0 - 9

    for(int i = 0; i < 44; i++)
    {
        spots[i].valid = 0;
    }

    spots[0].lat = 42.026294;
    spots[0].lon = -93.653913;

    spots[1].lat = 42.026294;
    spots[1].lon = -93.653880;


    spots[2].lat = 42.026294;
    spots[2].lon = -93.653847;


    spots[3].lat = 42.026294;
    spots[3].lon = -93.653814;


    spots[4].lat = 42.026294;
    spots[4].lon = -93.653780;


    spots[5].lat = 42.026294;
    spots[5].lon = -93.653745;


    //
    spots[6].lat = 42.026294;
    spots[6].lon = -93.653681;


    spots[7].lat = 42.026294;
    spots[7].lon = -93.653645;


    spots[8].lat = 42.026294;
    spots[8].lon = -93.653612;


    spots[9].lat = 42.026294;
    spots[9].lon = -93.653581;


    // 10 - 19


    spots[10].lat = 42.026294;
    spots[10].lon = -93.653551;


    spots[11].lat = 42.026294;
    spots[11].lon = -93.653515;


    spots[12].lat = 42.026294;
    spots[12].lon = -93.653480;


    spots[13].lat = 42.026294;
    spots[13].lon = -93.653449;


    spots[14].lat = 42.026294;
    spots[14].lon = -93.653416;


    spots[15].lat = 42.026294;
    spots[15].lon = -93.653381;


    spots[16].lat = 42.026294;
    spots[16].lon = -93.653350;


    spots[17].lat = 42.026294;
    spots[17].lon = -93.653319;


    spots[18].lat = 42.026294;
    spots[18].lon = -93.653284;


    spots[19].lat = 42.026294;
    spots[19].lon = -93.653219;


    // 20-29


    spots[20].lat = 42.026294;
    spots[20].lon = -93.653186;


    spots[21].lat = 42.026294;
    spots[21].lon = -93.653152;


    spots[22].lat = 42.026294;
    spots[22].lon = -93.653119;


    spots[23].lat = 42.026294;
    spots[23].lon = -93.653084;


    spots[24].lat = 42.026294;
    spots[24].lon = -93.653054;


    spots[25].lat = 42.026294;
    spots[25].lon = -93.653020;


    spots[26].lat = 42.026294;
    spots[26].lon = -93.652987;


    spots[27].lat = 42.026294;
    spots[27].lon = -93.652955;


    spots[28].lat = 42.026294;
    spots[28].lon = -93.652921;


    spots[29].lat = 42.026294;
    spots[29].lon = -93.652888;


    // 30-39
    spots[30].lat = 42.026294;
    spots[30].lon = -93.652853;


    spots[31].lat = 42.026294;
    spots[31].lon = -93.652786;


    spots[32].lat = 42.026294;
    spots[32].lon = -93.652753;


    spots[33].lat = 42.026294;
    spots[33].lon = -93.652721;


    spots[34].lat = 42.026294;
    spots[34].lon = -93.652686;


    spots[35].lat = 42.026294;
    spots[35].lon = -93.652655;


    spots[36].lat = 42.026294;
    spots[36].lon = -93.652622;


    spots[37].lat = 42.026294;
    spots[37].lon = -93.652588;


    spots[38].lat = 42.026294;
    spots[38].lon = -93.652553;


    spots[39].lat = 42.026294;
    spots[39].lon = -93.652522;


    // 40-43
    spots[40].lat = 42.026294;
    spots[40].lon = -93.652489;


    spots[41].lat = 42.026294;
    spots[41].lon = -93.652456;


    spots[42].lat = 42.026294;
    spots[42].lon = -93.652423;


    spots[43].lat = 42.026294;
    spots[43].lon = -93.652388;
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
    int height = 700;
    cv::Rect roiRect((frame.cols / 2) - (width / 2), frame.rows - height, width, height);
    roi.push_back(alpr::AlprRegionOfInterest(roiRect.x, roiRect.y, roiRect.width, roiRect.height));

    int foundPlateCounter = 0;
    int noPlateCounter = 0;
    std::vector<alpr::AlprPlateResult> plateReadings;
    std::vector<struct positionReading> positionReadings;
    int currFrame = 1;
    std::string currText;
    std::string finalPlateResult;
    cv::Scalar finalPlateColor;
    double finalPlateConfidence = 0;
    int plateHoldDuration = 0;
    double finalCarLatResult = 0;
    double finalCarLonResult = 0;

    double mapLatOrigin_1 = 42.026269;
    double mapLonOrigin_1 = -93.652374;

    double mapLatOrigin_2 = 42.026280;
    double mapLonOrigin_2 = -93.654172;

    double scalingFactor = 500.0 / (mapLonOrigin_2 - mapLonOrigin_1);
    std::vector<cv::Point> drivenPath;

    while (inputCapture.read(frame)) {
        if (currFrame < 300) {
            currFrame++;
            std::getline(frameFileStream, frameLine);
            continue;
        }

        // Place black frames to hold frame information
        cv::rectangle(frame, cv::Point(10, 10), cv::Point(300, 125), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::rectangle(frame, cv::Point(10, 10), cv::Point(300, 125), cv::Scalar(255, 255, 255), 2);
        cv::rectangle(frame, cv::Point((1920 - 10), 10), cv::Point((1920 - 300), 125), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::rectangle(frame, cv::Point((1920 - 10), 10), cv::Point((1920 - 300), 125), cv::Scalar(255, 255, 255), 2);

        std::getline(frameFileStream, frameLine);
        struct frameData currFrameData = getFrameData(frameLine);

        if (currFrameData.frameNum != currFrame) {
            // Something went wrong, keep reading till frames align
            std::cout << "Error: Frames don't align" << std::endl;
            continue;
        }

        while (currFrameData.frameTime > nextGPSData.timestamp) {
            prevGPSData = nextGPSData;
            std::getline(gpsFileStream, gpsLine);
            nextGPSData = getGPSData(gpsLine);
        }

        struct coord currentPos = coord_interpolate(prevGPSData, nextGPSData, currFrameData.frameTime);
        alpr::AlprResults frameResults = openalpr.recognize(frame.data, 3, frame.cols, frame.rows, roi);

        cv::rectangle(frame, cv::Point((1920/2 - 500/2-25), 10), cv::Point(1920/2+500/2+25, 180), cv::Scalar(0, 0, 0), CV_FILLED);
        cv::rectangle(frame, cv::Point((1920/2 - 500/2-25), 10), cv::Point(1920/2+500/2+25, 180), cv::Scalar(255, 255, 255), 2);


        int offsetMapX =  (1920 / 2 + 270-25);
        int offsetMapY = (500 / 2) - 100;

        int spotsIter;
        for (spotsIter = 0; spotsIter < 44; spotsIter++)
        {
            if (spots[spotsIter].valid == 0) {
                cv::circle(frame, cv::Point(-(spots[spotsIter].lon - mapLonOrigin_1) * scalingFactor + offsetMapX,
                                            (spots[spotsIter].lat - mapLatOrigin_1) * scalingFactor + offsetMapY), 3,
                           cv::Scalar(255, 0, 0), 1);

            }
            else if (spots[spotsIter].valid == 1)
            {
                cv::circle(frame, cv::Point(-(spots[spotsIter].lon - mapLonOrigin_1) * scalingFactor + offsetMapX,
                                            (spots[spotsIter].lat - mapLatOrigin_1) * scalingFactor + offsetMapY), 3,
                           cv::Scalar(0, 255, 0), CV_FILLED);
            }
            else {
                cv::circle(frame, cv::Point(-(spots[spotsIter].lon - mapLonOrigin_1) * scalingFactor + offsetMapX,
                                            (spots[spotsIter].lat - mapLatOrigin_1) * scalingFactor + offsetMapY), 3,
                           cv::Scalar(0, 0, 255), CV_FILLED);
            }

        }


        double xMap = -(currentPos.lon - mapLonOrigin_1) * scalingFactor + offsetMapX;
        double yMap = (currentPos.lat - mapLatOrigin_1) * scalingFactor + offsetMapY;
        std::cout << xMap << "," << yMap << std::endl;
        drivenPath.push_back(cv::Point((int) xMap, (int) yMap));
        cv::Point lastPoint = drivenPath[0];

        for (cv::Point newPoint : drivenPath) {
            line(frame, lastPoint, newPoint, cv::Scalar(0, 0, 255), 2);
            lastPoint = newPoint;
        }

        cv::circle(frame, lastPoint, 2, cv::Scalar(255, 255, 255), CV_FILLED);

        for (int i = 0; i < frameResults.plates.size(); i++) {
            alpr::AlprPlateResult currPlate = frameResults.plates[i];
            cv::Point topleft(currPlate.plate_points[0].x, currPlate.plate_points[0].y);
            cv::Point topright(currPlate.plate_points[1].x, currPlate.plate_points[1].y);
            cv::Point bottomright(currPlate.plate_points[2].x, currPlate.plate_points[2].y);
            cv::Point bottomleft(currPlate.plate_points[3].x, currPlate.plate_points[3].y);
            cv::Scalar red(0, 0, 255);

            line(frame, topleft, topright, red, 2);
            line(frame, topright, bottomright, red, 2);
            line(frame, bottomright, bottomleft, red, 2);
            line(frame, bottomleft, topleft, red, 2);

            int xCenter = (currPlate.plate_points[0].x + currPlate.plate_points[1].x + currPlate.plate_points[2].x +
                           currPlate.plate_points[3].x) / 4;
            int yCenter = (currPlate.plate_points[0].y + currPlate.plate_points[1].y + currPlate.plate_points[2].y +
                           currPlate.plate_points[3].y) / 4;

            line(frame, cv::Point(roiRect.x + (roiRect.width / 2), 1080), cv::Point(xCenter, currPlate.plate_points[2].y), red, 2);

            struct distanceReading plateDistance = distance(topleft, topright, bottomleft, bottomright);

            //std::cout << topleft.x << ", " << topleft.y << ", " << topright.x << ", " << topright.y << ", " << bottomleft.x << ", " << bottomleft.y << ", " << bottomright.x << ", " << bottomright.y << std::endl;
            //std::cout << "License Plate Distance: " << std::setprecision(20) << plateDistance.distance << std::endl;

            std::string licensePlateText = currPlate.bestPlate.characters;
            float confidenceLevel = currPlate.bestPlate.overall_confidence;
            std::vector<alpr::AlprPlate> topPlates = currPlate.topNPlates;
            //std::cout << i << " " << licensePlateText << " " << confidenceLevel << std::endl;

            currText = "License Plate: " + licensePlateText;
            cv::putText(frame, currText, cv::Point((1920 - 280), 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            currText = "Confidence: " + std::to_string(confidenceLevel) + "%";
            plateHoldDuration = 0;

            if (confidenceLevel >= 90) {
                // green
                cv::putText(frame, currText, cv::Point((1920 - 280), 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            } else if (confidenceLevel >= 80) {
                // yellow
                cv::putText(frame, currText, cv::Point((1920 - 280), 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
            } else {
                // red
                cv::putText(frame, currText, cv::Point((1920 - 280), 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
            }

            plateReadings.push_back(currPlate);
            struct positionReading currPosition;
            currPosition.distance = plateDistance;
            currPosition.longitude = currentPos.lon;
            currPosition.latitude = currentPos.lat;
            currPosition.time = currentPos.timestamp;
            currPosition.course = coord_course(prevGPSData, nextGPSData);
            cv::Point centerPos;
            centerPos.x = xCenter;
            centerPos.y = yCenter;
            currPosition.center = centerPos;
            positionReadings.push_back(currPosition);
        }

        // Check if a license plate was just found
        if (plateHoldDuration > 0) {
            currText = "License Plate: " + finalPlateResult;
            cv::putText(frame, currText, cv::Point((1920 - 280), 40), cv::FONT_HERSHEY_PLAIN, 1, finalPlateColor);
            currText = "Confidence: " + std::to_string(finalPlateConfidence) + "%";
            cv::putText(frame, currText, cv::Point((1920 - 280), 60), cv::FONT_HERSHEY_PLAIN, 1, finalPlateColor);
            currText = "Car Latitude: " + std::to_string(finalCarLatResult);
            cv::putText(frame, currText, cv::Point((1920 - 280), 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            currText = "Car Longitude: " + std::to_string(finalCarLonResult);
            cv::putText(frame, currText, cv::Point((1920 - 280), 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
            plateHoldDuration--;
        }

        // Draw the ROI
        rectangle(frame, roiRect , cv::Scalar(255,255,0));

        // Place frame information on the top left
        currText = "Frame #: " + std::to_string(currFrame);
        cv::putText(frame, currText, cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
        currText = "Time: " + std::to_string(currFrameData.frameTime);
        cv::putText(frame, currText, cv::Point(20, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
        currText = "Latitude: " + std::to_string(currentPos.lat);
        cv::putText(frame, currText, cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
        currText = "Longitude: " + std::to_string(currentPos.lon);
        cv::putText(frame, currText, cv::Point(20, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

        if (frameResults.plates.size() > 0) {
            foundPlateCounter++;
            noPlateCounter = 0;
        } else if (foundPlateCounter > 0) {
            noPlateCounter++;

            if (noPlateCounter >= 5) {
                struct plateResult avgPlateResult = getBestPlate(plateReadings);
                struct coord carPosition = getCarPosition(positionReadings, roiRect, 1920);
                currText = "Car Latitude: " + std::to_string(carPosition.lat);
                cv::putText(frame, currText, cv::Point((1920 - 280), 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
                currText = "Car Longitude: " + std::to_string(carPosition.lon);
                cv::putText(frame, currText, cv::Point((1920 - 280), 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
                finalCarLatResult = carPosition.lat;
                finalCarLonResult = carPosition.lon;


                int i;
                double min_dist = 1000000000.0;
                int index_dist = 0;
                for (i =0; i < 44; i++){
                    double dist = coord_distance(carPosition, spots[i]);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        index_dist = i;
                    }
                }

                // TODO: MAke sure to check against local database!

                spots[index_dist].valid = rand() % 2 + 1;



                // TODO: Position confidence


                std::cout << "Confidence: " << avgPlateResult.confidence << std::endl;
                std::cout << "Final Result: " << avgPlateResult.plateText << std::endl;
                foundPlateCounter = 0;
                noPlateCounter = 0;
                plateReadings.clear();
                positionReadings.clear();

                finalPlateResult = avgPlateResult.plateText;

                // Show the final plate result (green/yellow/red)
                currText = "License Plate: " + avgPlateResult.plateText;
                cv::putText(frame, currText, cv::Point((1920 - 280), 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
                currText = "Confidence: " + std::to_string(avgPlateResult.confidence) + "%";
                finalPlateConfidence = avgPlateResult.confidence;

                if (avgPlateResult.confidence >= 90) {
                    // green
                    finalPlateColor = cv::Scalar(0, 255, 0);
                } else if (avgPlateResult.confidence >= 80) {
                    // yellow
                    finalPlateColor = cv::Scalar(0, 255, 255);
                } else {
                    // red
                    finalPlateColor = cv::Scalar(0, 0, 255);
                }

                cv::putText(frame, currText, cv::Point((1920 - 280), 60), cv::FONT_HERSHEY_PLAIN, 1, finalPlateColor);
                plateHoldDuration = 25;
            }
        }

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
        currFrame++;
    }

    inputCapture.release();
    outputWriter.release();
}

struct coord Processor::getCarPosition(std::vector<struct positionReading> positionResults, cv::Rect roiRect, double frameWidthX) {
    struct coord carPosition;
    double longitudeAvg = 0;
    double latitudeAvg = 0;
    double scannedTimeStamp = 0;

    for (struct positionReading platePos : positionResults) {
        double zDist = platePos.distance.distance;
        double xDist = ((platePos.center.x - (frameWidthX / 2)) * 0.3048) / platePos.distance.width;
        double theta = atan(xDist / zDist) * (180.0 / M_PI);
        //std::cout << theta << std::endl;
        double distToPlate = cos(deg_to_rad(theta)) * zDist;
        struct coord origin;
        origin.lat = platePos.latitude;
        origin.lon = platePos.longitude;
        origin.timestamp = platePos.time;
        origin.valid = 1;
        double radial = platePos.course + 90 + theta;

        if (theta < 1 && theta > -1) {
            scannedTimeStamp = platePos.time;
        }

        struct coord currPlatePos = coord_dist_radial(origin, distToPlate, radial);
        radial -= theta;
        struct coord currCarPosition = coord_dist_radial(currPlatePos, 2.0, radial);
        //std::cout << "Latitude: " << std::setprecision(15) << currCarPosition.lat << ", Longitude: " << currCarPosition.lon << std::endl;

        // Average latitude
        latitudeAvg += currCarPosition.lat;

        // Average longitude
        longitudeAvg += currCarPosition.lon;
    }

    latitudeAvg = latitudeAvg / (double) positionResults.size();
    longitudeAvg = longitudeAvg / (double) positionResults.size();
    carPosition.lon = longitudeAvg;
    carPosition.lat = latitudeAvg;
    carPosition.valid = 1;
    carPosition.timestamp = scannedTimeStamp;
    return carPosition;
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

struct distanceReading Processor::distance(cv::Point topleft, cv::Point topright, cv::Point bottomleft,
                                        cv::Point bottomright) {
    double distance = 0;
    double height=((bottomleft.y-topleft.y)+(bottomright.y-topright.y)) / 2.0;
    double width=((topright.x-topleft.x)+(bottomright.x-bottomleft.x)) / 2.0;
    double diagonal=sqrt(height*height+width*width);
    double plateRatio=width / height;
    struct distanceReading plateReading;
    plateReading.sizeRatio = plateRatio;
    plateReading.distance = 446.909*pow(1/diagonal,0.99108);
    plateReading.height = height;
    plateReading.width = width;
    return plateReading;
}

struct plateResult Processor::getBestPlate(std::vector<alpr::AlprPlateResult> plateReadings) {
    struct plateResult bestPlate;
    int maxLength = 0;
    float indivPercSum = 0;

    // find the max length & individual percentages as a whole (given from ALPR)
    for (alpr::AlprPlateResult indivPlateResult : plateReadings) {
        std::string plateString = indivPlateResult.bestPlate.characters;
        indivPercSum += indivPlateResult.bestPlate.overall_confidence;

        if (maxLength < plateString.size()) {
            maxLength = (int) plateString.size();
        }
    }

    // allocate array for length frequency (length + 1 because need to account for no length)
    std::vector<int> lengthFrequency(maxLength + 1, 0);


    // calculate length frequency from each reading
    for (alpr::AlprPlateResult indivPlateResult : plateReadings) {
        std::string plateString = indivPlateResult.bestPlate.characters;
        lengthFrequency[plateString.size()]++;
    }


    // get index of the most frequent length (index of the maximum element)
    int likelyLength = (int) std::distance(std::begin(lengthFrequency), std::max_element(std::begin(lengthFrequency), std::end(lengthFrequency)));

    // string to hold most probable plate string
    std::string result;


    // confidence measure
    double confidence = 0.0;


    // create a histogram of letter frequencies in a column
    for (int i = 0; i< likelyLength; i++)
    {
        // create zeroed letter frequency counter
        std::vector<int> columnLetterFrequency(std::vector<int>(256, 0));


        // iterate through nth letter of the plate to calculate frequency
        int currentIndex = 0;
        for (alpr::AlprPlateResult indivPlateResult : plateReadings) {
            std::string plateString = indivPlateResult.bestPlate.characters;

            // only read character if it exists in the string (some plates are shorter)
            if (currentIndex <= plateString.size() - 1)
            {
                // update frequency result
                columnLetterFrequency[plateString[i]]++;
            }
        }

        // figure out most likely character (maximum value of the array)
        int likelyChar = (int) std::distance(std::begin(columnLetterFrequency), std::max_element(std::begin(columnLetterFrequency), std::end(columnLetterFrequency)));

        // calculate individual letter confidence factor to the confidence sum
        confidence += (double) columnLetterFrequency[likelyChar] / (double) plateReadings.size();

        // add most likely character into the final string
        result.push_back((char) likelyChar);
    }

    // average the confidence sum of all the individual letters
    confidence = confidence / (double) likelyLength;
    confidence = confidence * (indivPercSum / (double) plateReadings.size());
    bestPlate.confidence = confidence;
    bestPlate.plateText = result;
    return bestPlate;
}

struct coord Processor::coord_interpolate(struct coord prev, struct coord next, double timenow)
{
    /* create coordinate to hold the result */
    struct coord interpolated;

    double distance = coord_distance(prev, next);
    double course = coord_course(prev, next);
    double speed = distance / (next.timestamp - prev.timestamp);

    double time_new = timenow - prev.timestamp;
    double distance_new = speed * time_new;

    interpolated = coord_dist_radial(prev, distance_new, course);
    interpolated.timestamp = timenow;
    interpolated.valid = 2;

    /* return result */
    return interpolated;
}

double Processor::coord_distance(struct coord origin, struct coord destination)
{
    /* convert coordinates to radians */
    origin.lat=deg_to_rad(origin.lat);
    origin.lon=deg_to_rad(origin.lon);
    destination.lat=deg_to_rad(destination.lat);
    destination.lon=deg_to_rad(destination.lon);

    /* formula from http://www.edwilliams.org/avform.htm#Dist */
    double distance = 2*asin(sqrt(pow(sin((origin.lat-destination.lat)/2.0),2.0)+cos(origin.lat)*cos(destination.lat)*pow(sin((origin.lon-destination.lon)/2.0),2.0)));

    /* convert radial distance to meters (1 deg = 60 nmi, 1 nmi = 1852 m) */
    distance = rad_to_deg(distance) * 60.0 * 1852.0;

    /* return result */
    return distance;
}


struct coord Processor::coord_dist_radial(struct coord origin, double distance, double radial)
{
    /* convert parameters to radians */
    origin.lat = deg_to_rad(origin.lat);
    origin.lon = deg_to_rad(origin.lon);

    /* invert radial to use positive clockwise angle and convert to radians */
    radial = deg_to_rad(-radial);

    /* convert distance to angular distance (1 deg = 60 nmi, 1 m = 0.000539957 nmi) */
    distance = deg_to_rad(distance * 0.000539957 / 60.0);

    /* create struct for result coordinate */
    struct coord result;

    /* formulas from http://www.edwilliams.org/avform.htm#LL */
    result.lat = asin(sin(origin.lat)*cos(distance)+cos(origin.lat)*sin(distance)*cos(radial));
    result.lon = fmod(origin.lon-atan2(sin(radial)*sin(distance)*cos(origin.lat),cos(distance)-sin(origin.lat)*sin(result.lat))+M_PI,2.0*M_PI)-M_PI;

    /* convert result coordinate to degrees */
    result.lat = rad_to_deg(result.lat);
    result.lon = rad_to_deg(result.lon);

    /* return result pointer */
    return result;
}


double Processor::coord_course(struct coord origin, struct coord destination)
{
    /* convert coordinates to radians */
    origin.lat=deg_to_rad(origin.lat);
    origin.lon=deg_to_rad(origin.lon);
    destination.lat=deg_to_rad(destination.lat);
    destination.lon=deg_to_rad(destination.lon);

    /* formula from http://www.edwilliams.org/avform.htm#Crs */
    double course = fmod(atan2(sin(origin.lon-destination.lon)*cos(destination.lat),cos(origin.lat)*sin(destination.lat)-sin(origin.lat)*cos(destination.lat)*cos(origin.lon-destination.lon)),2*M_PI);

    /* convert course to degrees and add offset for standard course range*/
    course = -rad_to_deg(course);

    // flip negative values around
    if (course < 0.0)
        course = course + 360.0;

    /* return result */
    return course;
}