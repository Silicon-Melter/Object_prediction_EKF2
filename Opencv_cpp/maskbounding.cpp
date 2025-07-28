#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        printf("❌ Cannot open camera\n");
        return -1;
    }

    // Kalman Filter: 4 state variables (x, y, vx, vy), 2 measurements (x, y)
    KalmanFilter KF(4, 2, 0);
    KF.transitionMatrix = (Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(0.1));
    KF.statePost = (Mat_<float>(4, 1) << 0, 0, 0, 0);

    Mat frame;
    Point2f lastPrediction(0, 0);
    Mat measurement = Mat::zeros(2, 1, CV_32F);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // Convert to HSV
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // Threshold for red
        Mat mask1, mask2, mask;
        inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(160, 100, 100), Scalar(179, 255, 255), mask2);
        mask = mask1 | mask2;

        // Clean up noise
        morphologyEx(mask, mask, MORPH_OPEN, Mat());
        morphologyEx(mask, mask, MORPH_DILATE, Mat());

        // Find contours
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        Point2f measuredCenter;
        bool found = false;

        for (const auto& contour : contours) {
            if (contourArea(contour) > 500) {
                Rect bbox = boundingRect(contour);
                measuredCenter = Point2f(bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);
                found = true;
                rectangle(frame, bbox, Scalar(255, 0, 0), 2);
                break; // Use only largest contour
            }
        }

        // Kalman prediction step
        Mat prediction = KF.predict();
        Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));

        // If measurement found, correct filter
        if (found) {
            measurement.at<float>(0) = measuredCenter.x;
            measurement.at<float>(1) = measuredCenter.y;
            KF.correct(measurement);
        }

        // Draw results
        if (found)
            circle(frame, measuredCenter, 6, Scalar(0, 0, 255), -1); // red = measured
        circle(frame, predictPt, 6, Scalar(0, 255, 0), -1);          // green = predicted

        // Display
        imshow("Red Object Tracker", frame);
        if (waitKey(1) == 27) break; // ESC
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
