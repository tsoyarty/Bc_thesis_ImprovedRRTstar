#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main() {
    // Assuming you are generating 100 frames
    for (int i = 0; i < 100; ++i) {
        // Create an example image (replace this with your actual rendering code)
        cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::putText(image, "Frame " + std::to_string(i), cv::Point(50, 250), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);

        // Save the image
        std::string filename = "frames/frame_" + std::to_string(i) + ".png";
        cv::imwrite(filename, image);
        std::cout << "Saved " << filename << std::endl;
    }
    return 0;
}
