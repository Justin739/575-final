#include "Capturer.h"
#include "Processor.h"

void monitorStop(Capturer* cap) {
    // Open a window to detect key presses
    cv::namedWindow("Stopper Window", cv::WINDOW_AUTOSIZE);

    if (cv::waitKey(0) == 27) {
        std::cout << "--- Stopping capture ---" << std::endl;
        cap->stopCapture();
    }
}

int main(int argc, char* argv[]) {
    /*
    // Start capturing
    Capturer cap("/dev/video1", "recorded_video.avi", "frame_times.csv");
    std::thread stopper_thread(monitorStop, &cap);

    std::cout << "--- Starting capture ---" << std::endl;
    cap.startCapture();
    stopper_thread.join();
    */

    // Start processing
    Processor proc("recorded_video.avi", "processed_video.avi", "frame_times.csv", "raw_log.txt");
    proc.processData();

    return 0;
}