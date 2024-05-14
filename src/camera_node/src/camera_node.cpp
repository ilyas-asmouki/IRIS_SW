#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // Open the camera
    cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // Attempt to set camera parameters
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    camera.set(cv::CAP_PROP_FPS, 30);

    std::cout << "Press 'c' to capture an image or 'q' to quit." << std::endl;

    cv::Mat frame;
    char key;
    while (true) {
        camera >> frame;
        if (frame.empty()) {
            std::cerr << "ERROR: Blank frame grabbed" << std::endl;
            continue;
        }

        // Display the frame (optional)
        cv::imshow("Camera", frame);

        key = cv::waitKey(1);
        if (key == 'c') {
            // Save the captured frame
            cv::imwrite("photo.jpg", frame);
            std::cout << "Photo saved as photo.jpg" << std::endl;
        } else if (key == 'q') {
            break;
        }
    }

    camera.release();
    cv::destroyAllWindows();
    return 0;
}
