#ifndef OBJDETECT_H
#define OBJDETECT_H
#include <opencv2/opencv.hpp>
#include "bounding_box.h"
#define GPU 1
#define CPU 0
#if GPU==1
#if CV_MAJOR_VERSION==3
#include <opencv2/cudaobjdetect.hpp>
#else
#include <opencv2/gpu/gpu.hpp>
#endif
#endif //GPU==1

namespace darknet {
extern "C" {
#if GPU==0

#undef GPU
#undef CUDNN
#include "darknet/src/network.h"
#include "darknet/src/region_layer.h"
#include "darknet/src/cost_layer.h"
#include "darknet/src/utils.h"
#include "darknet/src/parser.h"
#include "darknet/src/box.h"
#include "darknet/src/demo.h"
#include "darknet/src/option_list.h"
#include "darknet/src/blas.h"
image ipl_to_image(IplImage* src);
#define GPU 1
#define CUDNN 0

#else

#if CUDNN==0
#undef CUDNN
#undef __cplusplus
#include "darknet/src/network.h"
#include "darknet/src/region_layer.h"
#include "darknet/src/cost_layer.h"
#include "darknet/src/utils.h"
#include "darknet/src/parser.h"
#include "darknet/src/box.h"
#include "darknet/src/demo.h"
#include "darknet/src/option_list.h"
#include "darknet/src/blas.h"
image ipl_to_image(IplImage* src);
void cuda_set_device(int n);
#define __cplusplus 201103
#define CUDNN 0

#else

#undef __cplusplus
#include "darknet/src/network.h"
#include "darknet/src/region_layer.h"
#include "darknet/src/cost_layer.h"
#include "darknet/src/utils.h"
#include "darknet/src/parser.h"
#include "darknet/src/box.h"
#include "darknet/src/demo.h"
#include "darknet/src/option_list.h"
#include "darknet/src/blas.h"
image ipl_to_image(IplImage* src);
void cuda_set_device(int n);
#define __cplusplus 201103

#endif // CUDNN==0

#endif //GPU==1
}
}



class HOGDetector{
public:
	HOGDetector();

	bounding_box detect_person(cv::Mat frame);
	bounding_box detect_person_cpu(cv::Mat frame);
#if GPU==1
	bounding_box detect_person_gpu(cv::Mat frame);
#endif //GPU==1

private:
	cv::HOGDescriptor hog_cpu;

#if GPU==1
#if CV_MAJOR_VERSION==3
	cv::Ptr<cv::cuda::HOG> hog_gpu;
#else
	cv::gpu::HOGDescriptor hog_gpu;
#endif //CV_MAJOR_VERSION==3
#endif //GPU==1
};

class HaarDetector{
public:
	HaarDetector();

	bounding_box detect_person(cv::Mat frame);
	bounding_box detect_person_cpu(cv::Mat frame);
#if GPU==1
	bounding_box detect_person_gpu(cv::Mat frame);
#endif //GPU==1

private:
	cv::CascadeClassifier haar_cpu;

#if GPU==1
#if CV_MAJOR_VERSION==3
	cv::Ptr<cv::cuda::CascadeClassifier> haar_gpu;
#else
	cv::gpu::CascadeClassifier_GPU haar_gpu;
#endif // CV_MAJOR_VERSION==3
#endif // GPU==1
};


class YOLODetector{
public:
	YOLODetector();
	bounding_box detect_person(cv::Mat frame);

private:
	bounding_box best_detection(const darknet::image& im, int num, float thresh, darknet::box *boxes, float **probs, char **names, int classes);

	darknet::list *options;
	char **names;
	darknet::network net;
};

class FRCNNDetector{
public:
	FRCNNDetector();
	bounding_box detect_person(cv::Mat frame);
};



#endif
