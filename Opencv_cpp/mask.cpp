#include <opencv2/opencv.hpp>
using namespace cv;

int main() {
    VideoCapture cap(1);
    if (!cap.isOpened()) {
        printf("❌ Cannot open camera\n");
        return -1;
    }

    while (true) {
        Mat frame, hsv, mask1, mask2, mask, result;
        cap >> frame;
        if (frame.empty()) break;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        Scalar lower_red1(0, 100, 100);
        Scalar upper_red1(10, 255, 255);

        Scalar lower_red2(160, 100, 100);
        Scalar upper_red2(179, 255, 255);
        inRange(hsv, lower_red1, upper_red1, mask1);
        inRange(hsv, lower_red2, upper_red2, mask2);
        mask = mask1 | mask2;
        bitwise_and(frame, frame, result, mask);
        imshow("Original", frame);
        imshow("Red Mask", mask);
        imshow("Red Objects", result);

        if (waitKey(1) == 27) break; // Press ESC to exit
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
