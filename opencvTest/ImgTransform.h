#pragma once
#include "SharedHead.h"
using namespace cv;
class CimgTransform
{
public:
	static void calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR);
	//static void quantizedOrientations(const Mat& src, Mat& magnitude, Mat& angle, float threshold);
	static void hysteresisGradient(Mat& magnitude, Mat& quantized_angle, Mat& angle, float threshold);
};
