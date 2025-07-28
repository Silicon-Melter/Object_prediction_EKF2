#include <opencv2/opencv.hpp>
using namespace cv;

int main() {
    // Create a green image
    Mat img(480, 640, CV_8UC3, Scalar(0, 255, 0));

    putText(img, "Hello OpenCV on macOS!", Point(50, 240), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);

    imshow("Test Window", img);
    waitKey(0);
    return 0;
}
