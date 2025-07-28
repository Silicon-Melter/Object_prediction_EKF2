#include <opencv2/opencv.hpp>
using namespace cv;

int main() {
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        printf("❌ Cannot open camera\n");
        return -1;
    }

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        imshow("Webcam Feed", frame);
        if (waitKey(1) == 27) break; // ESC to exit
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
