// targetTracking.cpp : 定义控制台应用程序的入口点。
//
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>

#include <stdio.h>
#include <thread>
#include <windows.h>
#include <iostream>

using namespace cv;
using namespace std;


static inline Point calcPoint(Point2f center, double R, double angle)
{
	return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}


/**
* @brief 对输入图像进行细化,骨骼化
* @param src为输入图像,用cvThreshold函数处理过的8位灰度图像格式，元素中只有0与1,1代表有元素，0代表为空白
* @param maxIterations限制迭代次数，如果不进行限制，默认为-1，代表不限制迭代次数，直到获得最终结果
* @return 为对src细化后的输出图像,格式与src格式相同，元素中只有0与1,1代表有元素，0代表为空白
*/
cv::Mat thinImage(const cv::Mat & src, const int maxIterations = -1)
{
	assert(src.type() == CV_8UC1);
	cv::Mat dst;
	int width = src.cols;
	int height = src.rows;
	src.copyTo(dst);
	int count = 0;  //记录迭代次数    
	while (true)
	{
		count++;
		if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达    
			break;
		std::vector<uchar *> mFlag; //用于标记需要删除的点    
		//对点标记    
		for (int i = 0; i < height; ++i)
		{
			uchar * p = dst.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				//如果满足四个条件，进行标记    
				//  p9 p2 p3    
				//  p8 p1 p4    
				//  p7 p6 p5    
				uchar p1 = p[j];
				if (p1 != 1) continue;
				uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
				uchar p8 = (j == 0) ? 0 : *(p + j - 1);
				uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
				uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
				uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
				uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
				uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
				uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					int ap = 0;
					if (p2 == 0 && p3 == 1) ++ap;
					if (p3 == 0 && p4 == 1) ++ap;
					if (p4 == 0 && p5 == 1) ++ap;
					if (p5 == 0 && p6 == 1) ++ap;
					if (p6 == 0 && p7 == 1) ++ap;
					if (p7 == 0 && p8 == 1) ++ap;
					if (p8 == 0 && p9 == 1) ++ap;
					if (p9 == 0 && p2 == 1) ++ap;

					if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
					{
						//标记    
						mFlag.push_back(p + j);
					}
				}
			}
		}

		//将标记的点删除    
		for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			**i = 0;
		}

		//直到没有点满足，算法结束    
		if (mFlag.empty())
		{
			break;
		}
		else
		{
			mFlag.clear();//将mFlag清空    
		}

		//对点标记    
		for (int i = 0; i < height; ++i)
		{
			uchar * p = dst.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				//如果满足四个条件，进行标记    
				//  p9 p2 p3    
				//  p8 p1 p4    
				//  p7 p6 p5    
				uchar p1 = p[j];
				if (p1 != 1) continue;
				uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
				uchar p8 = (j == 0) ? 0 : *(p + j - 1);
				uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
				uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
				uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
				uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
				uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
				uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					int ap = 0;
					if (p2 == 0 && p3 == 1) ++ap;
					if (p3 == 0 && p4 == 1) ++ap;
					if (p4 == 0 && p5 == 1) ++ap;
					if (p5 == 0 && p6 == 1) ++ap;
					if (p6 == 0 && p7 == 1) ++ap;
					if (p7 == 0 && p8 == 1) ++ap;
					if (p8 == 0 && p9 == 1) ++ap;
					if (p9 == 0 && p2 == 1) ++ap;

					if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
					{
						//标记    
						mFlag.push_back(p + j);
					}
				}
			}
		}

		//将标记的点删除    
		for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			**i = 0;
		}

		//直到没有点满足，算法结束    
		if (mFlag.empty())
		{
			break;
		}
		else
		{
			mFlag.clear();//将mFlag清空    
		}
	}
	return dst;
}

void chao_thinimage(Mat &srcimage)//单通道、二值化后的图像  
{
	vector<Point> deletelist1;
	int Zhangmude[9];
	int nl = srcimage.rows;
	int nc = srcimage.cols;
	while (true)
	{
		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();

		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();
	}
}

//亲测，两种效果都不佳


void selectScatteredFeatures(const std::vector<Point>& candidates,
	std::vector<Point>& features, size_t num_features,float distance)
{
	//基本思想：让备选点分别和选中点中的所有点计算距离，如果都满足距离条件，则归入选中点，一直循环，直到选中点数量满足要求为止
	features.clear();
	float distance_sq = distance * distance;
	int i = 0;
	while (features.size() < num_features)
	{
		Point c = candidates[i];

		// Add if sufficient distance away from any previously chosen feature
		bool keep = true;
		for (int j = 0; (j < (int)features.size()) && keep; ++j)
		{
			Point f = features[j];
			keep = (c.x - f.x)*(c.x - f.x) + (c.y - f.y)*(c.y - f.y) >= distance_sq;
		}
		if (keep)
			features.push_back(c); 
		if (++i == (int)candidates.size())
		{
			// Start back at beginning, and relax required distance
			i = 0;
			distance -= 1.0f;
			distance_sq = distance * distance;
		}
	}
}

static void help()
{
	printf("\nExample of c calls to OpenCV's Kalman filter.\n"
		"   Tracking of rotating point.\n"
		"   Rotation speed is constant.\n"
		"   Both state and measurements vectors are 1D (a point angle),\n"
		"   Measurement is the real point angle + gaussian noise.\n"
		"   The real and the estimated points are connected with yellow line segment,\n"
		"   the real and the measured points are connected with red line segment.\n"
		"   (if Kalman filter works correctly,\n"
		"    the yellow segment should be shorter than the red one).\n"
		"\n"
		"   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
		"   Pressing ESC will stop the program.\n"
		);
}

//全局变量
static Mat img(800, 800, CV_8UC3);
//定义滤波器，状态量有5个：xy方向位置、xy方向速度及圆半径。(todo::再加xy的加速度)；观测量有3个：xy方向位置和圆半径
static KalmanFilter KF(5, 3, 0, CV_32FC1);
double old_time = 0;
int r = 3;//圆特征半径
int px = 0; int py = 0; //鼠标位置
int dir = 2;
void on_Mouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_MOUSEMOVE)
	{
		px = x;
		py = y;
	}	
}
int main(int, char**)
{
	//自己编写测试代码：实现在鼠标的位置处画圆特征，圆特征的大小随时间不断地增大或者缩小（增大到最大值后缩小，缩小到最小值后增大，循坏反复，
	//利用opencv的kalman滤波器对特征的中心坐标及圆的大小进行实时预测，并和测量值进行对比。结果在屏幕中打印出来，绿色圆圈为观测值，红色圆圈
	//为预测值）
#if 1
	//just for fun
	img = cv::imread("C:/Users/lenovo/Desktop/test2.jpg");
	//cv::threshold(img, img, 100, 255, ThresholdTypes::THRESH_BINARY);
	cv::cvtColor(img,img,COLOR_BGR2GRAY);
	cv::Canny(img, img, 20, 100);
	vector<Point> pnts;
	vector<Point> scatter_pnts;
	for (int i = 0; i < img.rows; i++)
	{
		uchar *pt = img.ptr<uchar>(i);
		for (int j = 0; j < img.cols; j++)
		{
			if (pt[j] == 255)
			{
				pnts.push_back(Point(i, j));
			}
		}
	}
	selectScatteredFeatures(pnts, scatter_pnts, 950,10);
	img = cv::Mat(img.rows,img.cols,CV_8UC3);
	


	cv::namedWindow("kalman");
	cv::imshow("kalman", img);
	waitKey(5000);


#if 0
	for (int i = 0; i < scatter_pnts.size(); i++)
	{
		px = scatter_pnts[i].y; py = scatter_pnts[i].x;
		if (px == 0 || py == 0)
		{
			continue;
		}

		//绘制观测值（红色）
		//img.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255);
		cv::circle(img, Point(px, py), 2, cv::Scalar(0, 0, 255), 2);
		cv::circle(img, Point(px, py), r, cv::Scalar(0, 0, 255), 2);

		//绘制预测值（绿色）
		cv::Mat_<float> predictState(KF.predict());
		//img.at<cv::Vec3b>(predictState(1, 0), predictState(0, 0)) = cv::Vec3b(0, 255, 0);
		cv::circle(img, Point(predictState(0, 0), predictState(1, 0)), 2, cv::Scalar(0, 255, 0));
		cv::circle(img, Point(predictState(0, 0), predictState(1, 0)), r, cv::Scalar(0, 255, 0));

		cv::imshow("kalman", img);
		waitKey(10);
	}
#endif
#if 1
	//设置观测矩阵,只对x y的坐标及圆半径进行观测
	KF.measurementMatrix.at<float>(0, 0) = 1;
	KF.measurementMatrix.at<float>(1, 1) = 1;
	KF.measurementMatrix.at<float>(2, 4) = 1;
	//todo::opencv源码中将系统的不确定性矩阵设置成对角单位阵，如果没有不确定性，是否可以设置为0矩阵
	//cv::setMouseCallback("kalman", on_Mouse,0);

	for (int i = 0; i < scatter_pnts.size(); i++)
	{
		px = scatter_pnts[i].y; py = scatter_pnts[i].x;
		if (px == 0 || py == 0)
		{
			//cv::imshow("kalman", img);
			//waitKey(30);
			continue;
		}
		double _curtime = cv::getTickCount() / cv::getTickFrequency();
		double dt = _curtime - old_time;
		old_time = _curtime;

		//绘制观测值（红色）
		//img.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255);
		cv::circle(img, Point(px, py), 2, cv::Scalar(0, 0, 255), 2);
		cv::circle(img, Point(px, py), r, cv::Scalar(0, 0, 255), 2);

		//绘制预测值（绿色）
		cv::Mat_<float> predictState(KF.predict());
		//img.at<cv::Vec3b>(predictState(1, 0), predictState(0, 0)) = cv::Vec3b(0, 255, 0);
		cv::circle(img, Point(predictState(0, 0), predictState(1, 0)), 2, cv::Scalar(0, 255, 0));
		cv::circle(img, Point(predictState(0, 0), predictState(1, 0)), r, cv::Scalar(0, 255, 0));

		cv::imshow("kalman", img);

		//对状态进行更新
		float *_da = KF.statePost.ptr<float>(0, 0);
		_da[2] = (px - _da[0]) / dt;
		_da[3] = (py - _da[1]) / dt;
		_da[0] = px;
		_da[1] = py;
		_da[4] = r;

		if (r > 8) dir = -2;
		if (r < 3) dir = 2;
		r += dir;
		if (i == scatter_pnts.size() - 1)
		{
			waitKey();
		}
		waitKey(1);
	}
#endif
#endif
	//opencv 中sample代码
#if 0
	help();
	Mat img(500, 500, CV_8UC3);
	KalmanFilter KF(2, 1, 0);
	Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
	Mat processNoise(2, 1, CV_32F);
	Mat measurement = Mat::zeros(1, 1, CV_32F);
	char code = (char)-1;

	for (;;)
	{
		randn(state, Scalar::all(0), Scalar::all(0.1));
		KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
		setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
		setIdentity(KF.errorCovPost, Scalar::all(1));

		randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

		for (;;)
		{
			Point2f center(img.cols*0.5f, img.rows*0.5f);
			float R = img.cols / 3.f;
			double stateAngle = state.at<float>(0);
			Point statePt = calcPoint(center, R, stateAngle);

			Mat prediction = KF.predict();
			double predictAngle = prediction.at<float>(0);
			Point predictPt = calcPoint(center, R, predictAngle);

			randn(measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));

			// generate measurement
			measurement += KF.measurementMatrix*state;

			double measAngle = measurement.at<float>(0);
			Point measPt = calcPoint(center, R, measAngle);

			// plot points
#define drawCross( center, color, d )                                        \
	line(img, Point(center.x - d, center.y - d), \
	Point(center.x + d, center.y + d), color, 1, LINE_AA, 0); \
	line(img, Point(center.x + d, center.y - d), \
	Point(center.x - d, center.y + d), color, 1, LINE_AA, 0)

			img = Scalar::all(0);
			drawCross(statePt, Scalar(255, 255, 255), 3);
			drawCross(measPt, Scalar(0, 0, 255), 3);
			drawCross(predictPt, Scalar(0, 255, 0), 3);
			line(img, statePt, measPt, Scalar(0, 0, 255), 3, LINE_AA, 0);
			line(img, statePt, predictPt, Scalar(0, 255, 255), 3, LINE_AA, 0);

			if (theRNG().uniform(0, 4) != 0)
				KF.correct(measurement);

			randn(processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
			state = KF.transitionMatrix*state + processNoise;

			imshow("Kalman", img);
			code = (char)waitKey(100);

			if (code > 0)
				break;
		}
		if (code == 27 || code == 'q' || code == 'Q')
			break;
	}
#endif

	return 0;
}

