
// use if opencv was compiled? FIXME
//#include <cv.h>
//#include <highgui.h>
//#include <imgproc.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//#define DEBUG

#include "serial.h"

#include <stdint.h>
#include <string>
#include <iostream>
#include <exception>
#include <stdio.h>
#include <unistd.h>

using std::cout;
using std::endl;
using cv::Mat;


#define DOWNSCALE 1.3 // was 1.5
#define TINY_DOWNSCALE 3
#define NETLIGHTMODE false


void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double scale, const cv::Scalar& color) {
    for (int y = 0; y < cflowmap.rows; y += step)
        for (int x = 0; x < cflowmap.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflowmap, cv::Point(x, y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
        }
}


int main( int argc, char** argv ) {

    int serial = 0;

    const char* port = "/dev/ttyUSB0";
    if (!NETLIGHTMODE) {
        serial = lin_serial_connect(port, B9600, 0, 0);
       //serial = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC); // serial to the servos.
        if (serial < 0) {
            std::cout << "Eroor!" << std::endl;
        }
        //set_interface_attribs (serial, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        //set_blocking (serial, 0);                // set no blocking
    } else {
        lin_serial_connect(port);
    }
    uint16_t res[16];

    Mat trollFace = cv::imread("trollface.png", -1);

    Mat frame, miniFrame, previousFrame, flow;
    Mat tinyFrame, tinyPreviousFrame;

    cv::VideoCapture cap(CV_CAP_ANY); // open the default camera
    if (!cap.isOpened()) {
        cout << "unable to open camera 0" << endl;
        cap = cv::VideoCapture(1); // open the default camera
        if (!cap.isOpened()) {
            cout << "unable to open camera 1" << endl;
            return -1;
        }
    }

    cap >> frame;
    cv::cvtColor(frame, frame, CV_BGR2GRAY);

    cv::CascadeClassifier haar_cascade;
    haar_cascade.load("haarcascade_frontalface_default.xml");
    std::vector<cv::Rect> faces;

    float aspectRatio = 1.0 * static_cast<float>(frame.cols) / static_cast<float>(frame.rows);
    int frameRateVal = 100;
    int resizeVal = 0; //frame.rows - 16;
    int blurVal = 0;
    int cannyVal = 0;
    int thresholdVal = 0;
    int motionVal = 0;
    int motionVal2 = 0;
    int facesVal = 0;

    cv::namedWindow("playground", CV_WINDOW_NORMAL);
    cv::namedWindow("sliders", CV_WINDOW_NORMAL);
    cv::createTrackbar("frame rate", "sliders", &frameRateVal, 100);
    cv::createTrackbar("resize", "sliders", &resizeVal, frame.rows - 16);
    cv::createTrackbar("gaussian blur", "sliders", &blurVal, 50);
    cv::createTrackbar("edge detection", "sliders", &cannyVal, 150);
    cv::createTrackbar("adaptive threshold", "sliders", &thresholdVal, 50);
    cv::createTrackbar("motion sensitivity", "sliders", &motionVal, 50); // TODO make sensitivity thres on vectors.
    cv::createTrackbar("camera sensitivity", "sliders", &motionVal2, 50); // TODO make sensitivity thres on vectors.
    cv::createTrackbar("face detection", "sliders", &facesVal, 3); // TODO make sensitivity thres on vectors.

    while (true) {
        cv::Mat processed;
        previousFrame = frame;
        cap >> frame; // get a new frame from camera
        //cv::cvtColor(frame, frame, CV_BGR2GRAY);
        cv::cvtColor(frame, frame, CV_BGR2BGRA);
        cv::resize(frame, miniFrame, cv::Size(frame.cols/DOWNSCALE, frame.rows/DOWNSCALE));
        processed = frame;

        if (resizeVal != 0) {
            if (resizeVal < 16) {
                resizeVal = 16;
            }
            std::cout << "aspectRatio " << aspectRatio << "  resizeVal: " << resizeVal << " resizeVal*aspectRatio" << resizeVal*aspectRatio << std::endl;
            cv::resize(processed, processed, cv::Size((resizeVal+16)*(aspectRatio), resizeVal+16));
        }

        if (blurVal != 0) {
            cv::GaussianBlur(processed, processed, cv::Size(7,7), blurVal/10., blurVal/10.);
        }

        if (cannyVal != 0) {
            cv::Canny(processed, processed, 0, cannyVal, 3);
        }
        if (thresholdVal !=0) {
            cv::adaptiveThreshold(processed, processed, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 19, thresholdVal);
            cv::bitwise_not(processed, processed);
        }
        if (motionVal != 0 && frame.size() == previousFrame.size()) {
            if (frame.elemSize() != 1) {
                cv::cvtColor(frame, frame, CV_BGR2GRAY);
            }
            if (previousFrame.channels() != 1) {
                cv::cvtColor(previousFrame, previousFrame, CV_BGR2GRAY);
            }
            cv::calcOpticalFlowFarneback(frame, previousFrame, flow, 0.5, 3, 5, 1, 5, 1.2, 0);
            flow = cv::abs(flow);
            cv::threshold(flow, flow, motionVal, 0, cv::THRESH_TOZERO);  // FIXME
            cv::Scalar res = cv::sum(flow);
            std::cout << "flow total : " << res.val[0] + res.val[1] << std::endl;
            cv::cvtColor(processed, processed, CV_GRAY2RGB);
            drawOptFlowMap(flow, processed, 6, 1.5, CV_RGB(0, 255, 0));
        }
/*        if (!NETLIGHTMODE && frame.size() == previousFrame.size()) {
            cv::resize(frame, tinyFrame, cv::Size(frame.cols / TINY_DOWNSCALE, frame.rows / TINY_DOWNSCALE));
            cv::resize(previousFrame, tinyPreviousFrame, cv::Size(frame.cols / TINY_DOWNSCALE, frame.rows / TINY_DOWNSCALE));
            cv::calcOpticalFlowFarneback(tinyFrame, tinyPreviousFrame, flow, 0.5, 3, 5, 1, 5, 1.2, 0);
            cv::threshold(flow, flow, motionVal2, 0, cv::THRESH_TOZERO);  // FIXME  makes next process bug.
            cv::Scalar res = cv::sum(flow);
            if (res[0] > 0 && res[0] > 2500) {
                write(serial, "s\n", 2);
            } else if (res[0] > 2500) {
                write(serial, "w\n", 2);
            }
            if (res[1] > 0 && res[0] > 2500) {
                write(serial, "a\n", 2);
            } else if (res[1] > 2500) {
                write(serial, "d\n", 2);
            }
            std::cout << "scalar " << res[0] << " " << res[1] << std::endl;
        }*/

        /*
         * initialize background
         * get next frame, analyze spacial variation
         * get active areas, find potential heads
         * if heads present, continuously track
         * act accordingly
         */
        if (facesVal != 0) {
            if (frame.elemSize() == 1) { // switch to color.
                cv::cvtColor(frame, frame, CV_GRAY2BGR);
            }
            haar_cascade.detectMultiScale(miniFrame, faces);
            Mat resizedTroll, roi;
            cv::Rect headSize;
            for (std::vector<cv::Rect>::const_iterator it = faces.begin(); it != faces.end(); ++it) {
                headSize = cv::Rect(it->x*DOWNSCALE, it->y*DOWNSCALE, it->width*DOWNSCALE, it->height*DOWNSCALE);
                if (facesVal == 1) {
                    rectangle(processed, headSize, CV_RGB(0, 255,0), 2);
                } else if (facesVal ==2) {
                    roi = processed.colRange(it->x * DOWNSCALE, (it->x + it->width) * DOWNSCALE).rowRange(it->y, (it->y + it->height) * DOWNSCALE);
                    cv::resize(trollFace, resizedTroll, roi.size());
                    std::vector<cv::Mat> alphaMask;
                    cv::split(resizedTroll, alphaMask);
                    resizedTroll.copyTo(roi, alphaMask[3]);
                } else if (facesVal ==3) {
                    // put a fuuuuuu
                }
            }
        }
        // TODO background adaptive filter
        //

        // TODO: train a face live?
        // Methodo: get face. get images from face. rotate and crop. re-train the algo.

        cv::imshow("playground", processed);

        if(cv::waitKey(100 - frameRateVal+1) >= 0) {
        }
    }
}


