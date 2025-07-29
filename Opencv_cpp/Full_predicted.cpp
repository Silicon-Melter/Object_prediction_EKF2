#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap(0); // Open webcam
    if (!cap.isOpened()) {
        cout << "❌ Cannot open camera" << endl;
        return -1;
    }

    // === 6D Kalman Filter: [x, y, vx, vy, ax, ay] ===
    const int stateSize = 6;
    const int measSize = 2;
    KalmanFilter KF(stateSize, measSize, 0, CV_32F);

    float dt = 1.0f; // assume 1 frame per step (tune this with actual FPS if needed)

    // Transition matrix A (includes constant acceleration model)
    KF.transitionMatrix = (Mat_<float>(stateSize, stateSize) <<
        1, 0, dt,  0, 0.5f*dt*dt, 0,
        0, 1,  0, dt, 0, 0.5f*dt*dt,
        0, 0,  1,  0, dt, 0,
        0, 0,  0,  1, 0, dt,
        0, 0,  0,  0, 1, 0,
        0, 0,  0,  0, 0, 1
    );

    // Measurement matrix H
    KF.measurementMatrix = Mat::zeros(measSize, stateSize, CV_32F);
    KF.measurementMatrix.at<float>(0, 0) = 1.0f; // x
    KF.measurementMatrix.at<float>(1, 1) = 1.0f; // y

    setIdentity(KF.processNoiseCov, Scalar::all(1e-2));     // More trust in motion model
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1)); // Measurement noise (mask jitter)
    setIdentity(KF.errorCovPost, Scalar::all(0.1));
    KF.statePost = Mat::zeros(stateSize, 1, CV_32F);

    Mat measurement = Mat::zeros(measSize, 1, CV_32F);

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Convert to HSV and mask red
        Mat hsv, mask1, mask2, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(160, 100, 100), Scalar(179, 255, 255), mask2);
        mask = mask1 | mask2;

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
                rectangle(frame, bbox, Scalar(255, 0, 0), 2);
                found = true;
                break;
            }
        }

        // Kalman prediction
        Mat prediction = KF.predict();
        Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1));

        // Update with measurement
        if (found) {
            measurement.at<float>(0) = measuredCenter.x;
            measurement.at<float>(1) = measuredCenter.y;
            KF.correct(measurement);
        }

        // Draw current
        if (found)
            circle(frame, measuredCenter, 5, Scalar(0, 0, 255), -1); // Red = actual
        circle(frame, predictPt, 5, Scalar(0, 255, 0), -1);          // Green = smoothed

        // === Predict full trajectory ===
        KalmanFilter KF_clone = KF; // Copy current state
        Point2f lastPt = predictPt;
        Size predictedBoxSize = Size(40,40);
        if(found){
            Rect bbox = boundingRect(contours[0]);
            predictedBoxSize = bbox.size();
        }
        for (int i = 0; i < 10; ++i) {
            Mat future = KF_clone.predict();
            Point2f pt(future.at<float>(0), future.at<float>(1));
            Point topLeft(pt.x - predictedBoxSize.width / 2.0, pt.y - predictedBoxSize.height / 2.0);
            Rect predictedBox(topLeft, predictedBoxSize);
            rectangle(frame, predictedBox, Scalar(255, 255, 0), 1); // Cyan box
            lastPt = pt;
        }

        // Draw final predicted impact point
        circle(frame, lastPt, 6, Scalar(0, 255, 255), FILLED);
        putText(frame, format("Predicted impact: (%.0f, %.0f)", lastPt.x, lastPt.y),
                Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);

        imshow("Soccer Trajectory Prediction", frame);
        if (waitKey(1) == 27) break; // ESC
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
