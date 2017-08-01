#include "objdetect.h"
#include <vector>
#include <opencv2/objdetect/objdetect.hpp>

#define haar_scaleFactor 1.05
#define haar_minNeighbors 3

#ifndef OPENCV_FOLDER
#define OPENCV_FOLDER /home/ubuntu/Downloads/opencv-2.4.13
#endif


// HOG Detector
HOGDetector::HOGDetector()
{
	hog_cpu.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

#if GPU==1
#if CV_MAJOR_VERSION==3
	if(cv::cuda::getCudaEnabledDeviceCount() == 0)
		return;
	cv::cuda::setDevice(0);

    hog_gpu = cv::cuda::HOG::create();
	hog_gpu->setSVMDetector(hog_gpu->getDefaultPeopleDetector());
#else
    hog_gpu.setSVMDetector(cv::gpu::HOGDescriptor::getDefaultPeopleDetector());
#endif //CV_MAJOR_VERSION==3

    // Warm up GPUS
    for (int i = 0; i < 3; i++) {
        cv::Mat blank_frame(360, 360, CV_8UC3);
        //detect_person_gpu(blank_frame);
    }
#endif //GPU==1
}

bounding_box HOGDetector::detect_person(cv::Mat frame)
{
#if GPU==1
	return detect_person_gpu(frame);
#else
	return detect_person_cpu(frame);
#endif
}

bounding_box HOGDetector::detect_person_cpu(cv::Mat frame)
{
	bounding_box result = {-1.0, -1.0, -1.0, -1.0, -1.0};
	std::vector<cv::Rect> targets;
	std::vector<double> confidences;

	// Convert to grayscale
	cv::Mat gray;
#if CV_MAJOR_VERSION==3
	cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
#else
	cv::cvtColor(frame, gray, CV_RGB2GRAY);
#endif

	hog_cpu.detectMultiScale(gray, targets, confidences);

	if (targets.size() > 0) {
		int max_conf = 0;
		int max_conf_id = 0;

		for (int i = 0; i < targets.size(); i++) {
			if (confidences[i] >= max_conf) {
				max_conf = confidences[i];
				max_conf_id = i;
			}
		}

		result.x = targets[max_conf_id].x;
		result.y = targets[max_conf_id].y;
		result.w = targets[max_conf_id].width;
		result.h = targets[max_conf_id].height;
		result.conf = 1.0;//confidences[max_conf];
	}

	return result;
}

#if GPU==1
bounding_box HOGDetector::detect_person_gpu(cv::Mat frame)
{
	bounding_box result = {-1.0, -1.0, -1.0, -1.0, -1.0};
	std::vector<cv::Rect> targets;

	// Convert to grayscale
	cv::Mat gray;
#if CV_MAJOR_VERSION==3
	cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
#else
	cv::cvtColor(frame, gray, CV_RGB2GRAY);
#endif

#if CV_MAJOR_VERSION==3
	std::vector<double> confidences;
	cv::cuda::GpuMat image_gpu(gray);

	hog_gpu->detectMultiScale(image_gpu, targets, &confidences);

	if (targets.size() > 0) {
		int max_conf = 0;
		int max_conf_id = 0;

		for (int i = 0; i < targets.size(); i++) {
			if (confidences[i] >= max_conf) {
				max_conf = confidences[i];
				max_conf_id = i;
			}
		}

		result.x = targets[max_conf_id].x;
		result.y = targets[max_conf_id].y;
		result.w = targets[max_conf_id].width;
		result.h = targets[max_conf_id].height;
		result.conf = confidences[max_conf];
	}

#else
	cv::gpu::GpuMat gpu_img;

	gpu_img.upload(gray);

	hog_gpu.detectMultiScale(gpu_img, targets);
	
	// Unfortunately, OpenCV 2 doesn't give confidence scores for HOG GPU, so we just return the first person we see
	if (targets.size() > 0) {
		result.x = targets[0].x;
		result.y = targets[0].y;
		result.w = targets[0].width;
		result.h = targets[0].height;
		result.conf = 1.0;
	}
#endif

	return result;
}
#endif


// HaarDetector
HaarDetector::HaarDetector()
{
	haar_cpu.load("OPENCV_FOLDER/data/haarcascades/haarcascade_fullbody.xml");

#if GPU==1
#if CV_MAJOR_VERSION==3
	if(cv::cuda::getCudaEnabledDeviceCount() == 0)
		return -1;
	cv::cuda::setDevice(0);

    haar_gpu = cv::cuda::CascadeClassifier::create("OPENCV_FOLDER/data/haarcascades_cuda/haarcascade_fullbody.xml");
    haar_gpu->setScaleFactor(haar_scaleFactor);
    haar_gpu->setMinNeighbors(haar_minNeighbors);
#else
    haar_gpu.load("OPENCV_FOLDER/data/haarcascades_GPU/haarcascade_fullbody.xml");
    haar_gpu.findLargestObject = false;
#endif //CV_MAJOR_VERSION==3

    // Warm up GPUS
    for (int i = 0; i < 3; i++) {
        cv::Mat blank_frame(360, 360, CV_8UC1);
        //detect_person_gpu(blank_frame);
    }
#endif //GPU==1
}

bounding_box HaarDetector::detect_person(cv::Mat frame)
{
#if GPU==1
	return detect_person_gpu(frame);
#else
	return detect_person_cpu(frame);
#endif
}

bounding_box HaarDetector::detect_person_cpu(cv::Mat frame)
{
	bounding_box result = {-1.0, -1.0, -1.0, -1.0, -1.0};
	std::vector<cv::Rect> targets;

	haar_cpu.detectMultiScale(frame, targets, haar_scaleFactor, haar_minNeighbors);

	// Unfortunately, OpenCV doesn't have a documented way of getting confidence values for HaarCascades
	if (targets.size() > 0) {
		result.x = targets[0].x;
		result.y = targets[0].y;
		result.w = targets[0].width;
		result.h = targets[0].height;
		result.conf = 1.0;
	}

	return result;
}

#if GPU==1
bounding_box HaarDetector::detect_person_gpu(cv::Mat frame)
{
	bounding_box result = {-1.0, -1.0, -1.0, -1.0, -1.0};

	// Convert to grayscale
	cv::Mat gray;
#if CV_MAJOR_VERSION==3
	cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
#else
	cv::cvtColor(frame, gray, CV_RGB2GRAY);
#endif

#if CV_MAJOR_VERSION==3
	cv::cuda::GpuMat objbuf;
	std::vector<cv::Rect> targets;

	cv::cuda::GpuMat image_gpu(gray);

	haar_gpu->detectMultiScale(image_gpu, objbuf);
	haar_gpu->convert(objbuf, targets);

	if (targets.size() > 0) {
		// Unfortunately, OpenCV doesn't have a documented way of getting confidence values for HaarCascades
		result.x = targets[0].x;
		result.y = targets[0].y;
		result.w = targets[0].width;
		result.h = targets[0].height;
		result.conf = 1.0;
	}error: (-215) img.type() == CV_8UC1 || img.type() == CV_8UC4 in function detectMultiScale
#else
	int detections_num;
	cv::Mat faces_downloaded, frameDisp;
	cv::gpu::GpuMat frame_gpu, facesBuf_gpu;
	cv::Rect* targets;

	frame_gpu.upload(gray);

	detections_num = haar_gpu.detectMultiScale(frame_gpu, facesBuf_gpu, haar_scaleFactor, haar_minNeighbors);
	facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
	targets = faces_downloaded.ptr<cv::Rect>();

	if (detections_num > 0) {
		// Unfortunately, OpenCV doesn't have a documented way of getting confidence values for HaarCascades
        cv::Point pt = targets[0].tl();
        cv::Size sz = targets[0].size();

        result.x = pt.x;
        result.y = pt.y;
        result.w = sz.width;
        result.h = sz.height;
    }
#endif

	return result;
}
#endif //GPU==1


// YOLODetector
char datacfg[] = "darknet/cfg/coco.data";
char cfgfile[] = "darknet/cfg/yolo.cfg";
char weightfile[] = "/media/ubuntu/0403-0201/yolo.weights";
float thresh = 0.24;
float hier_thresh = 0.5;

YOLODetector::YOLODetector()
{
#if GPU==1
    darknet::cuda_set_device(0);
#endif

    options = darknet::read_data_cfg(datacfg);
    names = darknet::get_labels("darknet/data/coco.names");

    net = darknet::parse_network_cfg(cfgfile);
    darknet::load_weights(&net, weightfile);
    darknet::set_batch_network(&net, 1);
    srand(2222222);

#if GPU==1
    // Warm up GPUS
    for (int i = 0; i < 3; i++) {
        cv::Mat blank_frame(360, 360, CV_8UC3);
        //detect_person(blank_frame);
    }
#endif
}

bounding_box YOLODetector::detect_person(cv::Mat frame)
{
	bounding_box result = {-1, -1, -1, -1};
	
	const float nms=.4;

    //image im = load_image_color(input,0,0);
	IplImage ipl = frame;
	darknet::image im = darknet::ipl_to_image(&ipl);
	darknet::rgbgr_image(im);
	// darknet::image resized = resize_image(im, w, h);
	// darknet::free_image(im);
 	// im = resized;

    darknet::image sized = letterbox_image(im, net.w, net.h);
    darknet::layer l = net.layers[net.n-1];

    darknet::box *boxes = (darknet::box*) calloc(l.w*l.h*l.n, sizeof(darknet::box));
    float **probs = (float**) calloc(l.w*l.h*l.n, sizeof(float *));
    for(int j = 0; j < l.w*l.h*l.n; ++j)
    	probs[j] = (float*) calloc(l.classes + 1, sizeof(float *));

    float *X = sized.data;
    darknet::network_predict(net, X);
    darknet::get_region_boxes(l, im.w, im.h, net.w, net.h, thresh, probs, boxes, 0, 0, hier_thresh, 1);

    if (nms)
    	darknet::do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);

    //draw_detections(im, l.w*l.h*l.n, thresh, boxes, probs, names, alphabet, l.classes);
    result = best_detection(im, l.w*l.h*l.n, thresh, boxes, probs, names, l.classes);

    darknet::free_image(im);
    darknet::free_image(sized);
    free(boxes);
    darknet::free_ptrs((void **)probs, l.w*l.h*l.n);

    return result;
}

bounding_box YOLODetector::best_detection(const darknet::image& im, int num, float thresh, darknet::box *boxes, float **probs, char **names, int classes)
{
	bounding_box result = {-1, -1, -1, -1};
	float max_prob = -1;

    for (int i = 0; i < num; i++) {
        int _class = darknet::max_index(probs[i], classes);
        float prob = probs[i][_class];

        if(prob > thresh && prob > max_prob && strcmp("person", names[_class]) == 0){
    		prob = max_prob;

            darknet::box b = boxes[i];

            int left  = (b.x-b.w/2.)*im.w;
            int right = (b.x+b.w/2.)*im.w;
            int top   = (b.y-b.h/2.)*im.h;
            int bot   = (b.y+b.h/2.)*im.h;

            if(left < 0)
            	left = 0;
            if(right > im.w-1)
            	right = im.w-1;
            if(top < 0)
            	top = 0;
            if(bot > im.h-1)
            	bot = im.h-1;

            result.x = left;
            result.y = top;
            result.w = right - left;
            result.h = bot - top;
            result.conf = 1.0;
        }
    }

    return result;
}
