#include <cv.h>
#include <highgui.h>
#include <string>
#include <iostream>
//#include <imgproc.hpp>
#include <exception>

//#include <stdint.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#define DEBUG

using std::cout;
using std::endl;
using cv::Mat;

int32_t getMotionRatio(cv::Mat image, cv::Mat previous, cv::Mat& flow) {
#ifdef DEBUG
	Utils::clockStart();
#endif
	try {
		cv::calcOpticalFlowFarneback(previous, image, flow, 0.5, 3, 3, 1, 5, 1.2, 0);
	} catch(cv::Exception e) {
		cout << e.what() << endl;
	}
	//uint32_t timing = Utils::clockEnd();
	//DLOG(INFO) << "farnBack timing: " << timing;
	flow = cv::abs(flow);
	cv::Scalar res = cv::sum(flow);
#ifdef DEBUG
	cv::Mat imageControl;
	cv::cvtColor(image, imageControl, CV_GRAY2RGB);
	cv::drawOptFlowMap(_flow, imageControl, _controlImageInterval, 1.5, CV_RGB(0, 255, 0));
	_manager->generateDebugImage(imageControl, QString("Motion Vectors feedback, motion = %1").arg(res.val[0] + res.val[1]).toStdString(), typeid(*this).name());
#endif
	return res.val[0] + res.val[1];
}


void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step, double scale, const cv::Scalar& color) {
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step) {
			const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
			cv::line(cflowmap, cv::Point(x, y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
		}
}


// get next image from video, 24hz
// get motion ratio
// compare motion ratio
// 	if > threshold then 
// 		get foreground, modify and display on screen
// 	if not, then slirgly update background levels.
// detect faces, do stuff on it.

int main( int argc, char** argv ) {
    bool canny = false;
    bool motion = true;

    int cannyValue = 30;

	cv::namedWindow("playground", CV_WINDOW_NORMAL);
    cv::createTrackbar("canny", "playground", &cannyValue, 70);

	cv::VideoCapture cap(CV_CAP_ANY); // open the default camera
	if (!cap.isOpened()) {
		cout << "FAIL!" << endl;
		return -1;
	}

    Mat frame, previousFrame, flow;
    cap >> frame;
    cv::cvtColor(frame, frame, CV_BGR2GRAY);
	while (true) {
        cv::Mat processed;
        previousFrame = frame;
		cap >> frame; // get a new frame from camera
        cv::cvtColor(frame, frame, CV_BGR2GRAY);

        if (canny) {
            cv::GaussianBlur(frame, frame, cv::Size(7,7), 1.5, 1.5);
            cv::Canny(frame, processed, 0, cannyValue, 3);
        } else if (motion) {
            cv::calcOpticalFlowFarneback(frame, previousFrame, flow, 0.5, 3, 3, 1, 5, 1.2, 0);
            flow = cv::abs(flow);
            cv::Scalar res = cv::sum(flow);
            std::cout << "flow total : " << res.val[0] + res.val[1] << std::endl;
            cv::cvtColor(frame, processed, CV_GRAY2RGB);
            drawOptFlowMap(flow, processed, 4, 1.5, CV_RGB(0, 255, 0));
        }


        //cv::resize(frame, frame, cv::Size(16,16));
		cv::imshow("playground", processed);
		if(cv::waitKey(30) >= 0) break;
	}
}


