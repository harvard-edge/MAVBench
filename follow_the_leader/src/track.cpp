#include "track.h"

KCFtracker::KCFtracker(const bounding_box& bb, cv::Mat frame) : tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB)
{
	tracker.init(cv::Rect(bb.x, bb.y, bb.w, bb.h), frame);
	peak = 0;
}

bounding_box KCFtracker::track(cv::Mat frame) {
	bounding_box result;
	double old_peak = peak;
	cv::Rect rect = tracker.update(frame, &peak);

	result.x = rect.x;
	result.y = rect.y;
	result.w = rect.width;
	result.h = rect.height;
	result.conf = peak != 0 ? peak/old_peak : 1.0;

	return result;
}
/*
TLDtracker::TLDtracker(const bounding_box& bb, cv::Mat frame)
{
	cv::Mat grey;

#if CV_MAJOR_VERSION==3
	cv::cvtColor(frame, grey, COLOR_BGR2GRAY);
#else
	cv::cvtColor(frame, grey, CV_BGR2GRAY);
#endif //CV_MAJOR_VERSION==3

	tracker.detectorCascade->imgWidth = grey.cols;
	tracker.detectorCascade->imgHeight = grey.rows;
	tracker.detectorCascade->imgWidthStep = grey.step;

	cv::Rect rect(bb.x, bb.y, bb.w, bb.h);
	tracker.selectObject(grey, &rect);
}

bounding_box TLDtracker::track(cv::Mat frame) {
	bounding_box result = {-1, -1, -1, -1};

	tracker.processImage(frame);

	if(tracker.currBB != 0) {
		cv::Rect rect = *(tracker.currBB);
		result.x = rect.x;
		result.y = rect.y;
		result.w = rect.width;
		result.h = rect.height;
		//result.conf = tracker.currConf;
		result.conf = 1.0;
	}

	return result;
}
*/
