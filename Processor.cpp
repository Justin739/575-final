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
    int height = 600;
    cv::Rect rect((frame.cols / 2) - (width / 2), frame.rows - height - 100, width, height);
    roi.push_back(alpr::AlprRegionOfInterest(rect.x, rect.y, rect.width, rect.height));

    int foundPlateCounter = 0;
    int noPlateCounter = 0;
    std::vector<alpr::AlprPlateResult> plateReadings;
    std::vector<struct positionReading> positionReadings;
    int currFrame = 1;

    while (inputCapture.read(frame)) {
        std::getline(frameFileStream, frameLine);
        struct frameData currFrameData = getFrameData(frameLine);

        if (currFrameData.frameNum != currFrame) {
            // Something went wrong, keep reading till frames align
            std::cout << "Error: Frames don't align" << std::endl;
            continue;
        }

        if (currFrameData.frameTime > nextGPSData.timestamp) {
            prevGPSData = nextGPSData;
            std::getline(gpsFileStream, gpsLine);
            nextGPSData = getGPSData(gpsLine);
        }

        struct coord currentPos = coord_interpolate(prevGPSData, nextGPSData, currFrameData.frameTime);
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

            double plateDistance = distance(topleft, topright, bottomleft, bottomright);

            //std::cout << topleft.x << ", " << topleft.y << ", " << topright.x << ", " << topright.y << ", " << bottomleft.x << ", " << bottomleft.y << ", " << bottomright.x << ", " << bottomright.y << std::endl;
            //std::cout << "License Plate Distance: " << std::setprecision(20) << plateDistance << std::endl;

            std::string licensePlateText = currPlate.bestPlate.characters;
            float confidenceLevel = currPlate.bestPlate.overall_confidence;
            std::vector<alpr::AlprPlate> topPlates = currPlate.topNPlates;
            std::cout << i << " " << licensePlateText << " " << confidenceLevel << std::endl;

            if (currPlate.bestPlate.overall_confidence > 85) {
                plateReadings.push_back(currPlate);
            }

            struct positionReading currPosition;
            currPosition.distance = plateDistance;
            currPosition.longitude = currentPos.lon;
            currPosition.latitude = currentPos.lat;
            currPosition.time = currentPos.timestamp;
            cv::Point centerPos;
            centerPos.x = xCenter;
            centerPos.y = yCenter;
            currPosition.center = centerPos;
            positionReadings.push_back(currPosition);
        }


        if (frameResults.plates.size() > 0) {
            foundPlateCounter++;
            noPlateCounter = 0;
        } else if (foundPlateCounter > 0) {
            noPlateCounter++;

            if (noPlateCounter >= 5) {
                std::cout << "MIDDLE OF CARS" << std::endl;
                struct plateResult avgPlateResult = getBestPlate(plateReadings);
                std::cout << "Confidence: " << avgPlateResult.confidence << std::endl;
                std::cout << "Final Result: " << avgPlateResult.plateText << std::endl;
                foundPlateCounter = 0;
                noPlateCounter = 0;
                plateReadings.clear();
                positionReadings.clear();
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
        currFrame++;
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

double Processor::distance(cv::Point topleft, cv::Point topright, cv::Point bottomleft,
                                        cv::Point bottomright) {
    double distance = 0;
    double height=((bottomleft.y-topleft.y)+(bottomright.y-topright.y)) / 2;
    double width=((topright.x-topleft.x)+(bottomright.x-bottomleft.x)) / 2;
    double diagonal=sqrt(height*height+width*width);
    //ideal ratio=2 length/width
    int plateRatio=height/width;
    double error=0.5;

    if (plateRatio-2<error){
        //diagonal vs distance calculation
        distance=3509.92*pow((1/diagonal),1.30719);
        //width vs distance calculation
        //distance=3057.56*pow((1/width),1.30719);
        //height vs distance calculation
        //distance=1209.22*pow((1/height),1.31062);
        //height*width vs distance calculation
        //distance=1923.99*pow((1/(width*height)),0.65445);
    }

    return distance;
}

struct plateResult Processor::getBestPlate(std::vector<alpr::AlprPlateResult> plateReadings) {
    struct plateResult bestPlate;
    int maxLength = 0;

    // find the max length
    for (alpr::AlprPlateResult indivPlateResult : plateReadings) {
        std::string plateString = indivPlateResult.bestPlate.characters;

        if (maxLength < plateString.size()) {
            maxLength = (int) plateString.size();
        }
    }

    // allocate array for lenght frequency (lenght + 1 because need to account for no lenght)
    std::vector<int> lengthFrequency(maxLength + 1, 0);


    // calculate lenght frequency from each reading
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