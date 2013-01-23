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

int32_t getMotionRatio2(cv::Mat image, cv::Mat previous, cv::Mat flow) {
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
	int threshold = 17;
	cv::namedWindow("playground", CV_WINDOW_NORMAL);
	cv::createTrackbar("Adaptive threshold", "playground", &threshold, 40);

	
	cv::Mat image = cv::imread("DSC_0015.JPG");
	cv::cvtColor(image, image, CV_RGB2GRAY);
	cv::Mat copy = image.clone();

	cv::VideoCapture cap(0); // open the default camera
	if (!cap.isOpened()) {
		return -1;
	}

	Mat edges;
	cv::namedWindow("edges",1);
	while (true) {
		Mat frame;
		cap >> frame; // get a new frame from camera
		cv::cvtColor(frame, edges, CV_BGR2GRAY);
		cv::GaussianBlur(edges, edges, cv::Size(7,7), 1.5, 1.5);
		cv::Canny(edges, edges, 0, 30, 3);
		cv::imshow("playground", edges);
		if(cv::waitKey(30) >= 0) break;
	}

	/*
	for (int i = 0; i < 70; i++) {
		//void adaptiveThreshold(const Mat& src, Mat& dst, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double C)
		cv::adaptiveThreshold(image, copy, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 3, i);
		cv::imshow("playground", copy);
		cv::waitKey(0);
	}*/
}


