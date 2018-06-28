// opencvtest.cpp : 定义控制台应用程序的入口点。
//
#include <math.h>
#include "stdafx.h"
//***opencv header
#include "SharedHead.h"
#include "XMLReader.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include "opencv.hpp"//hpp file equal the .h + .cpp, the fun decalare and implement in the same file.
#include "cvtool.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/flann.hpp"
#include "opencv2/stitching.hpp"
//linemod 模块
#include "linemod.hpp"
#include "rgbd.hpp"

//***
#include "XMLWriter.h"
#include "XMLReader.h"
#include "CoreAlgorithm.h"
#include "ImgTransform.h"

//***opencv CUDA module header
#include "opencv2/cudawarping.hpp"
#include "opencv2/cudastereo.hpp"
//***cuda header

using namespace cv;
using namespace cv::cuda;
using namespace cv::linemod;
using namespace cv::detail;

//Author: samylee
//Contact email: ahuljx@126.com
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <stdio.h>  


Mat org, dst, img, tmp;

//鼠标响应
void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	static Point pre_pt = (-1, -1);
	static Point cur_pt = (-1, -1);
	double imshowScaler = *(double*)ustc;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		org.copyTo(img);
		pre_pt = Point(x, y);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//摁下左键，flags为1 
	{
		img.copyTo(tmp);
		cur_pt = Point(x, y);
		rectangle(tmp, pre_pt, cur_pt, Scalar(255, 0, 0, 0), 5, 8, 0);
		imshow("img", tmp);
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		org.copyTo(img);
		cur_pt = Point(x, y) ;
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 5, 8, 0);
		imshow("img", img);
		img.copyTo(tmp);
		int width = abs(pre_pt.x - cur_pt.x);
		int height = abs(pre_pt.y - cur_pt.y);
		if (width == 0 || height == 0)
		{
			return;
		}
		dst = org(Rect(min(cur_pt.x, pre_pt.x), min(cur_pt.y , pre_pt.y), width, height));
		namedWindow("dst");
		imshow("dst", dst);
	}
}

void fadfasdf()
{
	cout << __FUNCTION__ << endl;
}

//函数声明
void myStereoRectify(InputArray _cameraMatrix1, InputArray _distCoeffs1,
	InputArray _cameraMatrix2, InputArray _distCoeffs2,
	Size imageSize, InputArray _Rmat, InputArray _Tmat,
	OutputArray _Rmat1, OutputArray _Rmat2,
	OutputArray _Pmat1, OutputArray _Pmat2,
	OutputArray _Qmat, int flags,
	double alpha, Size newImageSize,
	Rect* validPixROI1, Rect* validPixROI2);
void myCanny(InputArray _src, OutputArray _dst,
	double low_thresh, double high_thresh,
	int aperture_size, bool L2gradient);


#if 1
int _tmain(int argc, _TCHAR* argv[])
{

	//测试图形ROI
#if 0
	Mat src = imread("./10.bmp");
	CvBox2D box;
	box.angle = 30;
	box.center = CvPoint2D32f(100, 100);
	box.size = Size(100, 100);

	Mat src_roi;
	Mat mask = Mat::zeros(src.size(), CV_8UC1);


#endif


	//测试将图像投影到球面  初次尝试失败
#if 0
	Mat src_img = imread("./data file/measureImgL13.bmp");
	Mat dst_img;
	CamPara campara;
	Mat intrinsic, discoeffs;
	XMLReader::readCamPara("./data file/CamParaDstL.xml", campara);
	cvtool::cvtCamPara(campara, intrinsic, discoeffs);
	intrinsic.convertTo(intrinsic, CV_32FC1);
	discoeffs.convertTo(discoeffs, CV_32FC1);
	Ptr<WarperCreator> warper_creator = makePtr<cv::SphericalWarperGpu>();
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(3));
	warper->warp(src_img, intrinsic, Mat::eye(3, 3, CV_32FC1), INTER_LINEAR, BORDER_CONSTANT, dst_img);
	int i = 0;
#endif

	//vector <Point2i> p,dst;
	//p.push_back(Point2i(21, 21));
	//Mat m = getRotationMatrix2D(Point2i(0, 0), 45, 1);
	//transform(p, dst, m);
	//int i = 0;

	cout << "<<<<<<<<<<<<<<<<<<<<2D物体识别模块>>>>>>>>>>>>>>>>>>>>>" << endl;
#pragma region 物体识别2d
// 物体识别2d
#if 1
	//制作模板
	double imshowScaler = 0.2;
	std::vector< Ptr<Modality> > modalities;
	modalities.push_back(makePtr<ColorGradient>());
	int t_inPyramid[] = {6,6,3};
	vector <int> T_pyramid(t_inPyramid,t_inPyramid+3);
	cv::Ptr<cv::linemod::Detector> detector = makePtr<Detector>(modalities, T_pyramid);
	//detector = cv::linemod::getDefaultLINEMOD();//linemod-3D
	//detector = cv::linemod::getDefaultLINE();//linemod-2D
	cout << "金字塔的层数：" << detector->pyramidLevels() << endl;
	org = imread("./data file/test img/image_02.bmp", IMREAD_COLOR);
	namedWindow("img", CV_WINDOW_KEEPRATIO);
	resizeWindow("img", org.size().width * imshowScaler, org.size().height * imshowScaler);
	setMouseCallback("img", on_mouse, &imshowScaler);
	imshow("img", org);
	waitKey(0);
	vector <Mat> sources;
	sources.push_back(dst);
	string class_id = "class0";
	int templateID = detector->addTemplate(sources, class_id, Mat());

	//提取模板的轮廓
	for (int i = 11; i < 47; i++)
	{
		stringstream ddft;
		ddft << "./data file/test img/part/" << i << ".bmp";
		org = imread(ddft.str(), IMREAD_COLOR);
		//匹配

		clock_t t1 = clock();
		sources.clear();
		sources.push_back(org);
		int matching_threshold = 60;
		std::vector<cv::linemod::Match> matches;
		std::vector<cv::String> class_ids;
		std::vector<cv::Mat> quantized_images;
		detector->match(sources, (float)matching_threshold, matches, class_ids); //这里的sources就是搜索图像，里面存有深度图像和彩色图像
		if (matches.size() <= 0)
		{
			cout << "识别失败！" << endl;
			continue;
		}
		clock_t t2 = clock();
		cout << "耗时：" << t2 - t1 << endl;
		for (int j = 0; j < matches.size(); j ++)
		{
			
			vector<Template> matchTemplate = detector->getTemplates("class0", matches[j].template_id);
			for (int k = 0; k < matchTemplate[0].features.size(); k++)
			{
				Point pt = cvPoint(matchTemplate[0].features[k].x + matches[j].x, matchTemplate[0].features[k].y + matches[j].y);
				circle(org, pt, 4, Scalar(0, 0, 255), -1);
			}
		}
		imshow("img", org);
		waitKey(0);
	}
#if 1

#endif

#endif 
#pragma endregion 物体识别2d
	cout << "<<<<<<<<<<<<<<<<<<<<2D物体识别模块>>>>>>>>>>>>>>>>>>>>>" << endl;
#pragma region Mat数据基本操作
	//对3通道的mat数据读取元素时，如果不用向量类型，则输出的数据不是当前点的某一通道的数据，具体这个数据是怎么得到的还不知道，应该是按一定的公式解算的。
#if 0
	Mat dd = imread("./data file/change principal test/calib image/L/1.bmp");
	for (int i = 0; i < 100; i++)
	{
		cout << i <<"点"<<endl;
		cout << dd.at<uchar>(0,i)<<endl;
		cout << dd.at<Vec3b>(0,i)[0]<<endl;
	}
#endif
	//mat赋值
#if 0
	//***point3f 和 mat之间可相互赋值
	Point3f pnt(1, 2, 3);
	Mat pntm(pnt);
	cout << pntm.t() << endl;
	Mat te = pntm.t();
	Point3f ppt(te);
	cout << ppt << endl;

	Mat A = Mat::ones(5, 5, CV_64FC1);
	cout << A.step<<endl;
	A = Mat::ones(9, 9, CV_64FC1);
	cout << A.step << endl;
	A = Mat::ones(5, 9, CV_64FC1);
	cout << A.step << endl;
	A = Mat::ones(9, 5, CV_64FC1);
	cout << A.step << endl;
	A = Mat::ones(9, 5, CV_64FC1);
	cout << A.step1(0) << endl;
	CvMat df;

#endif

	//mat 元素操作
#if 0
	Mat mat3 = Mat::zeros(3, 3, CV_8UC3);
	uchar *pnt = mat3.data;	
	pnt[0] = 14;
	pnt[1] = 15;
	pnt[3] = 17;
	for (int i = 0; i < mat3.rows; i++)
	{
		uchar *pr = mat3.ptr<uchar>(i);
		for (int j = 0; j < mat3.cols*mat3.channels(); j++)
		{
			pr[j] = j;
		}
	}
	cout << mat3 << endl;
#endif

#pragma endregion 图像数据基本操作
#pragma region 图像变换
	//测试通过已知的相机外参将世界坐标平面转化成平行视图  ?????
#if 0
	CamPara camP;
	vector<Mat> parallelImgR;
	Mat cameraMatrix, distCoeffs;
	Mat srcImg = imread("./data file/change principal test/calib image/L/1.bmp", IMREAD_GRAYSCALE);
	Mat resultImgMat = Mat(2 * srcImg.rows, 2 * srcImg.cols, srcImg.type());
	XMLReader::readCamPara("./data file/change principal test/calib image/L/camDataDist.xml", camP);
	cvtool::cvtCamPara(camP, cameraMatrix, distCoeffs);
	CoreAlgorithm::calculateParallelViewR(camP.imgRTVec, parallelImgR);
	CoreAlgorithm::undistortImg(srcImg, resultImgMat, cameraMatrix, distCoeffs, parallelImgR[0]);
	int i = 0;
#endif
#pragma endregion 图像变换
#pragma region opencv 图像处理
	//分离彩色图片通道，查看各通道对颜色的响应
#if 0
	Mat img = imread("./data file/laser.jpg");
	Mat chan[3];
	clock_t tt1 = clock();
	split(img, chan);
	clock_t tt2 = clock();
	cout << "处理时间" << tt2 - tt1 << endl;
	cv::namedWindow("ch1", CV_WINDOW_KEEPRATIO);
	cv::namedWindow("ch2", CV_WINDOW_KEEPRATIO);
	cv::namedWindow("ch3", CV_WINDOW_KEEPRATIO);
	cv::resizeWindow("ch1", img.size().width / 2, img.size().height / 2);
	cv::resizeWindow("ch2", img.size().width / 2, img.size().height / 2);
	cv::resizeWindow("ch3", img.size().width / 2, img.size().height / 2);
	cv::imshow("ch1",chan[0]);
	cv::imshow("ch2",chan[1]);
	cv::imshow("ch3",chan[2]);
	waitKey(0);
	imwrite("./data file/ch3.jpg",chan[2]);
	cvtColor(img, img, CV_RGB2GRAY);
	imwrite("./data file/gray.jpg", img);
#endif

	//sobel test
#if 0
	clock_t t1 = clock();
	for (long i = 0; i < 460800000; i++)
	{
		int ik = ((54 * 839 + 15)+(54 * 839 + 15)+(54 * 839 + 15))/10;
	}
	clock_t t2 = clock();
	cout << t2 - t1 << endl;
	Mat img = imread("./data file/test img/rim.png", IMREAD_GRAYSCALE);
	Mat dx, dy, dxy;
	clock_t t3 = clock();
	Sobel(img, dx, CV_16S, 1, 0);
	Sobel(img, dy, CV_16S, 0, 1);
	clock_t t4 = clock();
	cout << t4 - t3 << endl;
	Sobel(img, dxy, CV_16S, 1, 1);
	cout << dx.at<short>(10, 14) << endl;
	cout << dxy.at<short>(10, 14) << endl;
	cout << sqrt(pow(dx.at<short>(10, 14), 2) + pow(dy.at<short>(10, 14), 2))<<endl;
	Mat rdx, rdy;
	Scharr(img, rdx, CV_16S, 1, 0);
	Scharr(img, rdy, CV_16S, 0, 1);
	cout << rdx.at<short>(10, 14) << endl;
	cout << rdy.at<short>(10, 14) << endl;
#endif

	//canny测试
#if 0
	Mat img = imread("./data file/test img/image_02.bmp", IMREAD_GRAYSCALE);
	Mat dst;
	myCanny(img, dst, 20, 60, 3, true);
#endif

#if 0
	vector<TagPoint3f> pns;
	XMLReader::readLightPenDotPara("C:/Users/lenovo/Desktop/lightPenFeaturesData.xml", pns);
	CamPara cam_para;
	XMLReader::readCamPara("./data file/stereo vision/CalibrateData/CamParaL.xml", cam_para);
	Mat initrisic;
	Mat distCoeffs;
	cvtool::cvtCamPara(cam_para, initrisic, distCoeffs);
#endif


	//实验undistortPoints函数
#if 0
	vector<Point2f> pnt;
	vector <Point2f> srcpnt;
	pnt.push_back(Point2f(1242, 1032));
	undistortPoints(pnt, srcpnt, initrisic, distCoeffs); //srcpnt 的得到是
	int i = 0;
#endif

	//图像的模板操作
#if 0
	Sobel()

#endif

	//用surf或者sift特征描述子进行模板匹配
#if 0
	Mat scrImg = imread("../datafile/test img/surf test/image1.jpg",IMREAD_GRAYSCALE);
	Mat temImg = imread("../datafile/test img/surf test/template.jpg", IMREAD_GRAYSCALE);
	std::vector<cv::KeyPoint> keypoints;
	
	cv::SurfFeatureDetector surf(2500);
	surf.detect(image, keypoints);
	cv::drawKeypoints(image, keypoints, image, cv::Scalar::all(255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	namedWindow("image1");
#endif


	//测试灰度梯度计算
#if 0
	Mat img1 = cv::imread("../datafile/test img/rim.png", IMREAD_GRAYSCALE);
	//namedWindow("imshow", CV_WINDOW_AUTOSIZE);

	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	Sobel(img1, grad_x, 1, 1, 0, 3, 1, 0, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
#endif

#if 0
	//计算基础矩阵
	//方法一：通过相机的相互位姿关系计算
#endif
#if 0

#endif
	//test opencv rodrigues
#if 0
	Mat a = (Mat_<double>(3, 3) << 0.9890, -0.0132, -0.1471,
		0.0253, 0.9964, 0.0807,
		0.1455, -0.0835, 0.9858);
	cout << a << endl;
	Mat b;
	cv::Rodrigues(a, b);
	cout << b << endl;
#endif
#if 0 //匹配精度精化
	img = imread("D:\\vs13 program\\Test project\\datafile\\椭圆1.jpg", IMREAD_GRAYSCALE);
	Mat showimg;
	int i = 5;
	vector<vector <Point>> contours;
	vector<cv::Vec4i> hierarchy;
	Canny(img, outimg, 1000, 1000 * 3, 5, false);
	findContours(outimg,
		contours,					//所获得的轮廓点
		hierarchy,
		CV_RETR_LIST,				//获取轮廓的方法
		CV_CHAIN_APPROX_NONE);		//轮廓近似的方法
	for (int idx = 0; idx < contours.size(); idx++)
	{
		if (contours[idx].size() < 50)
		{
			continue;
		}
		//对轮廓进行K次迭代
		srand((int)time(NULL));
		for (int K = 0; K < 10; K++)
		{
			//随机获取轮廓中的五个点拟合椭圆
			vector<Point> pnt5;
			vector <int> index;
			stringstream str;
			int num = 15;
			showimg = imread("D:\\vs13 program\\Test project\\datafile\\椭圆1.jpg");
			Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
			ProduceRandNum(contours[idx].size() / 4, contours[idx].size() / 1.5, num, index); //发现随机取5个点进行拟合时效果很差
			for (int i = 0; i < num; i++)
			{
				Scalar _color((rand() & 255), (rand() & 255), (rand() & 255));
				circle(showimg, contours[idx][index[i]], 2, _color, 6);
				pnt5.push_back(contours[idx][index[i]]);
			}
			str << "D:\\vs13 program\\Test project\\datafile\\result\\img_" << K << "pnt.jpg";
			imwrite(str.str(), showimg);
			RotatedRect _rotatebox;
			_rotatebox = fitEllipse(pnt5);
			ellipse(showimg, _rotatebox, color, 3);
			//hyper方法
			Vector2d pos[15];
			Vector6d paras;
			for (int j = 0; j < pnt5.size(); j++)
			{
				pos[j][0] = pnt5[j].x;
				pos[j][1] = pnt5[j].y;
			}
			hyper_renormalization(pos, 15, paras);
			cout << "paras = " << paras << endl;
			str.str("");
			str << "D:\\vs13 program\\Test project\\datafile\\result\\img_" << K << ".jpg";
			imwrite(str.str(), showimg);
			for (int j = 0; j < num; j++)
			{
				if (onEllipse(_rotatebox, pnt5[j]))
					cout << "ok" << endl;
				else
				{
					cout << "NG" << endl;
				}
			}
			//对判断局外点是否在椭圆上

		}

	}
	Mat imge_contour = imread("D:\\vs13 program\\Test project\\datafile\\recImgL1.bmp");
	int idx = 0;
	for (; idx >= 0; idx = hierarchy[idx][0])
	{
		RotatedRect _rotatebox;
		if (contours[idx].size() < 10)
		{
			continue;
		}
		_rotatebox = fitEllipse(contours[idx]);
		if ((_rotatebox.size.height / _rotatebox.size.width)>2)
		{
			continue;
		}
		if (_rotatebox.size.height < 50)
		{
			continue;
		}
		Scalar color((rand() & 255), (rand() & 255), (rand() & 255));//表示颜色
		//ellipse(imge_contour, _rotatebox, color, 2);
		drawContours(imge_contour, contours,
			idx, // draw all contours
			color, // in white
			1); // with a thickness of 2
	}
	//equalizeHist(img, img);
	namedWindow("canny", WINDOW_NORMAL);
	resizeWindow("canny", img.cols / 4, img.rows / 4);
	imshow("canny", img);
	createTrackbar("trackBar", "canny", &i, 1500, on_trackbar);
	//adaptiveThreshold(img, outimg, 255, cv::AdaptiveThresholdTypes::ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, 5);
	waitKey();
#endif

#if 0 测试视差
	Mat img = imread("D:/vs13 program/RobotCatch3D/RobotCatch3D/disparity.bmp", CV_LOAD_IMAGE_ANYDEPTH);
	int i = 0;
#endif
	//测试自适应阈值
#if 0
	Mat scr = imread("../image/objImage_0.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	//高斯模糊
	Mat dst, dst1, dst2, dst3;
	Size ksize;
	ksize.height = 21;
	ksize.width = 21;
	GaussianBlur(scr, dst, ksize, 2);
	GaussianBlur(scr, dst1, ksize, 4);
	GaussianBlur(scr, dst2, ksize, 8);
	GaussianBlur(scr, dst3, ksize, 16);
	//gaussianblur ksize取大值时，xsigma也需要取大值，模糊化才明显。这很好理解，高斯分布中当x在3*sigma后的y值很小，反映在图像中即离中心3*sigma的像素权值很小，影响系数很小。
	adaptiveThreshold(scr, dst, 255, CV_ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 501, 5);
	//剔除二值化后的图像噪点
	//方法一：中值滤波
	Mat blurdst;
	medianBlur(dst, blurdst, 25);
	namedWindow("showimage");
	imshow("showimage", dst);

#endif
#if 0//opencv 中的特征匹配
	SiftDescriptorExtractor descript;

#endif

#pragma endregion opencv 图像处理
#pragma region opencv立体视觉模块测试
	//***立体视觉的图像矫正过程
#if 0
	DWORD t1 = GetTickCount();
	CamPara Lcam, Rcam;
	RT relativeRT;
	Size imgSize;
	Mat LInPara, RInPara, Ldifc, Rdifc;
	Mat  Rl, Rr, Pl, Pr, Q;
	XMLReader::ReadStereoPara("../datafile/stereo vision/CalibrateData/StereoPara.xml", Lcam, Rcam, relativeRT, imgSize);
	cvtool::cvtCamPara(Lcam, LInPara, Ldifc);
	cvtool::cvtCamPara(Rcam, RInPara, Rdifc);
	//step 1:进行立体矫正，获取矫正矩阵，已经对对图像的畸变进行了修正
	cv::stereoRectify(LInPara, Ldifc, RInPara, Rdifc, imgSize, relativeRT.R, relativeRT.T, Rl, Rr, Pl, Pr, Q, 0, 1, imgSize);
	//step 2:计算原始图像和矫正图像的映射表
	Mat mapLx, mapLy, mapRx, mapRy;//映射表
	cv::initUndistortRectifyMap(LInPara, Ldifc, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
	cv::initUndistortRectifyMap(RInPara, Rdifc, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);
	//step 3:根据映射表矫正图像
	Mat imgL = imread("../datafile/stereo vision/L/measureImageL0.bmp");
	Mat imgR = imread("../datafile/stereo vision/R/measureImgR0.bmp");
	cv::remap(imgL, imgL, mapLx, mapLy, cv::INTER_LINEAR);
	cv::remap(imgR, imgR, mapRx, mapRy, cv::INTER_LINEAR);
	DWORD t2 = GetTickCount();
	std::cout << "computer time in CPU = " << t2 - t1 << endl;
	//完成
#endif
	//测试通过remap进行图片矫正和用undistor的区别
#if 1

#endif

	//***从图像坐标系投影到世界坐标系
#pragma endregion opencv立体视觉模块测试
#pragma region opencv CUDA速度测试

	//***第一个CUDA例子:测试remap在gpu中计算的速度
#if 0
	DWORD t1 = GetTickCount();
	CamPara Lcam, Rcam;
	RT relativeRT;
	Size imgSize;
	Mat LInPara, RInPara, Ldifc, Rdifc;
	Mat  Rl, Rr, Pl, Pr, Q;
	XMLReader::ReadStereoPara("./data file/stereo vision/CalibrateData/StereoPara.xml", Lcam, Rcam, relativeRT, imgSize);
	cvtool::cvtCamPara(Lcam, LInPara, Ldifc);
	cvtool::cvtCamPara(Rcam, RInPara, Rdifc);
	//step 1:进行立体矫正，获取矫正矩阵，已经对对图像的畸变进行了修正
	cv::stereoRectify(LInPara, Ldifc, RInPara, Rdifc, imgSize, relativeRT.R, relativeRT.T, Rl, Rr, Pl, Pr, Q, 0, 1, imgSize);
	//step 2:计算原始图像和矫正图像的映射表
	Mat mapLx, mapLy, mapRx, mapRy;//映射表
	cv::initUndistortRectifyMap(LInPara, Ldifc, Rl, Pl, imgSize, CV_32FC1, mapLx, mapLy);
	cv::initUndistortRectifyMap(RInPara, Rdifc, Rr, Pr, imgSize, CV_32FC1, mapRx, mapRy);
	
	//step 3:根据映射表矫正图像(在gpu中计算)
	int num_dev = getCudaEnabledDeviceCount();
	if (num_dev <= 0)
	{
		std::cout << "没有可用GPU设备！" << endl;
		return 0;
	}
	int enable_device_id = -1;
	for (int i = 0; i < num_dev; i++)
	{
		DeviceInfo dev_info(i);
		if (dev_info.isCompatible())
		{
			enable_device_id = i;
		}
	}
	if (enable_device_id < 0)
	{
		std::cerr << "GPU module isn't built for GPU" << std::endl;
		return -1;
	}
	setDevice(enable_device_id);
	Mat imgL = imread("../datafile/stereo vision/L/measureImageL0.bmp");
	Mat imgR = imread("../datafile/stereo vision/R/measureImgR0.bmp");
	GpuMat imgL_gpu(imgL);
	GpuMat imgR_gpu(imgR);
	GpuMat mapLx_gpu(mapLx);
	GpuMat mapLy_gpu(mapLy);
	GpuMat mapRx_gpu(mapRx);
	GpuMat mapRy_gpu(mapRy);

	cv::Rect ROI(0, 0, 500, 500);
	GpuMat temp(imgL_gpu, ROI);
	cv::cuda::remap(imgL_gpu, imgL_gpu, mapLx_gpu, mapLy_gpu, cv::INTER_LINEAR);
	cv::cuda::remap(imgL_gpu, imgL_gpu, mapRx_gpu, mapRy_gpu, cv::INTER_LINEAR);
	imgL_gpu.download(imgL);
	imgR_gpu.download(imgR);
	DWORD t2 = GetTickCount();
	cout << "computer time in GPU = " << t2 - t1 << endl;
	//测试结果：在GPU中的计算速度大概是CPU中的两倍，包括图片上载和下载时间
#endif

#if 0
	Mat df = (cv::Mat_<int>(1,3) << 0, -1, 0);
	Mat idf = -df;
	cout << idf << endl;
	Vec3f vvv(1, 0, 0);
	Vec3f dfd = -vvv;
	cout << dfd << endl;
	Vec3f vv(0, 0, 1);
	cout << vvv.cross(vv);
	df(Range(0, 1), Range(0, 3)) = -df;
	cout << df << endl;
#endif

#if 0
	Mat_<double> iR = Mat::eye(4, 4, CV_64FC1);
	double *ir = &iR(0, 0);
	cout << ir[1] << *(ir + 5) << endl;
	bool flagd = 0 & CV_CALIB_ZERO_DISPARITY;
#endif

#if 0
	double _om[2][2] = { {2,2}, {2,2} };
	CvMat om;
	Mat omm = Mat(2, 2, CV_64FC1, _om);
	om = cvMat(2, 2, CV_64FC1, _om); //这时候改变数组，cvmat中的数据也会跟着改变，这是一种比较好的对cvmat的复制方式，对于mat同样适用。
	_om[1][1] = 3;
	cout << om.data.db << endl;
#endif

	//cvGetRows, cvConvertScale meanStdDev;  mat to arry;
#if 0
	vector<Point2f> pnts;
	for (int i = 0; i < 10; i++)
	{
		pnts.push_back(Point2f(i, 2));
	}
	double _om[3][2] = { { 2, 2 }, { 2, 2 }, { 2, 3 } };
	double _omm[3][2] = { { 2, 2 }, { 2, 2 }, { 2, 3 } };
	CvMat sr = cvMat(3, 2, CV_64FC1, _om), ds;

	cvGetRows(&sr, &ds, 1, 2); //用于从sr获取n行的数据给ds，注：CvMat是共享指针。
	_om[1][1] = 255;
	CvMat dst = cvMat(3, 2, CV_64FC1, _omm);
	cvConvertScale(&sr, &dst, 2, 1); //对sr*scale +offset 赋值给dst, 这是一个赋值操作，dst只要不是指向同一个数据就不会互相影响
	_om[1][1] = 0;
	Mat m,d;
	Mat ssr = Mat(3, 2, CV_64FC3, _om);
	meanStdDev(pnts, m, d); //可以处理vector
	cout << "finish" << endl;
#endif
	
	//std::copy;  back_inserter();  norm(); 
#if 0
	vector<Point2f> pnts,pnt1s;
	for (int i = 0; i < 3; i++)
	{
		pnts.push_back(Point2f(0, 2));
		pnt1s.push_back(Point2f(1, 2));
	}
	double dfs = norm(pnts, pnt1s, CV_L2); //如果输入的src为vectors时，分别对vector中每个元素进行norm()，然后对求得的结果再norm()
	vector <Point2f> copnts;
	copnts.push_back(Point2f(0, 0));
	std::copy(pnts.begin(), pnts.end(), back_inserter(copnts)); 
	int i = 0;
#endif

	//***测试改变相机的主点位置对双目标和最终重建的影响
#if 0
	string datafilePath = "./data file/change principal test/cam params/";
	//读取相机参数
	CamPara readCamParaL, readCamParaR;
	string camParaPathL = datafilePath + "LeftcamDataDist.xml";
	string camParaPathR = datafilePath + "RightcamDataDist.xml";
	XMLReader::readCamPara(camParaPathL, readCamParaL);
	XMLReader::readCamPara(camParaPathR, readCamParaR);

	//读取角点数据
	CalibrationData camCornersL;
	CalibrationData camCornersR;
	string camCornersPathL = datafilePath + "LeftCornerData.xml";
	string camCornersPathR = datafilePath + "RightCornerData.xml";
	XMLReader::readCornerData(camCornersPathL, camCornersL);
	XMLReader::readCornerData(camCornersPathR, camCornersR);

	//从角点对象读取图像尺寸
	Size imgSize;	//相对位姿和图像尺寸
	imgSize.width = camCornersL.imgWidth;
	imgSize.height = camCornersL.imgHeight;

	//进行数据转换
	Mat intrinsicParaL, intrinsicParaR, distCoeffsL, distCoeffsR;
	cvtool::cvtCamPara(readCamParaL, intrinsicParaL, distCoeffsL);
	cvtool::cvtCamPara(readCamParaR, intrinsicParaR, distCoeffsR);

	//进行双目标定
	Mat R, T, E, F;//标定得到的参数R,T为两相机相对位姿
	double time1 = static_cast<double>(cv::getTickCount());
	double rms = stereoCalibrate(camCornersL.plane3dPntsVec, camCornersL.plane2dPntsVec, camCornersR.plane2dPntsVec,
		intrinsicParaL, distCoeffsL, intrinsicParaR, distCoeffsR,
		Size(camCornersL.imgWidth, camCornersL.imgHeight), R, T, E, F,
		CV_CALIB_FIX_INTRINSIC +
		/*CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_FIX_FOCAL_LENGTH +
		CV_CALIB_FIX_PRINCIPAL_POINT +
		CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH*/
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));//TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6)

	//保存双目标定结果，给矫正时候使用
	RT relativeRT;
	Rodrigues(R, relativeRT.R);
	relativeRT.T = T;

	XMLWriter::WriteStereoPara(datafilePath + "StereoPara.xml", readCamParaL, readCamParaR, relativeRT, imgSize);

	//***立体矫正(用比原图像更大尺寸的矫正图像进行校正)
	Mat  Rl, Rr, Pl, Pr, Q;
	//step 1:进行立体矫正，获取矫正矩阵，已经对对图像的畸变进行了修正
	cv::Size dstSize(imgSize.width,imgSize.height); //矫正后的图片尺寸
	//intrinsicParaL.at<double>(0, 2) *= 2;
	//intrinsicParaR.at<double>(0, 2) *= 2;
	myStereoRectify(intrinsicParaL, distCoeffsL, intrinsicParaR, distCoeffsR, imgSize, relativeRT.R, relativeRT.T, Rl, Rr, Pl, Pr, Q, 0, -1, dstSize,0,0);  //p1 和 pr结果为负值？？？
	//step 2:计算原始图像和矫正图像的映射表
	Mat mapLx, mapLy, mapRx, mapRy;//映射表
	//Pl.at <double>(0, 2) = imgSize.width/2;
	//Pr.at <double>(0, 2) = imgSize.width/2;
	initUndistortRectifyMap(intrinsicParaL, distCoeffsL, Rl, Pl, dstSize, CV_32FC1, mapLx, mapLy);
	initUndistortRectifyMap(intrinsicParaR, distCoeffsR, Rr, Pr, dstSize, CV_32FC1, mapRx, mapRy);
	//step 3:矫正图片
	for (int i = 0; i < 4; i++)
	{
		stringstream _str;
		Mat dstImgL, dstImgR;
		//_str << "./data file/change principal test/calib image/L/" << i+1 << ".bmp";
		_str << "./data file/change principal test/laser image/L/" << "objImage_" << i << ".bmp";
		Mat srcimgL = imread(_str.str(), IMREAD_GRAYSCALE);
		_str.str("");
		//_str << "./data file/change principal test/calib image/R/" << i+1 << ".bmp";
		_str << "./data file/change principal test/laser image/R/" << "objImage_" << i << ".bmp";
		Mat srcimgR = imread(_str.str(), IMREAD_GRAYSCALE);
		cv::remap(srcimgL, dstImgL, mapLx, mapLy, cv::INTER_LINEAR);
		cv::remap(srcimgR, dstImgR, mapRx, mapRy, cv::INTER_LINEAR);
		int fdf = 0;
	}
#endif

#pragma endregion opencv CUDA速度测试
                                 	return 0;
}
#endif


//测试opencv中的立体矫正 但相机的角度变大时，会出现矫正图像一片空白
#if 0
static void
icvGetRectangles(const CvMat* cameraMatrix, const CvMat* distCoeffs,
const CvMat* R, const CvMat* newCameraMatrix, CvSize imgSize,
cv::Rect_<float>& inner, cv::Rect_<float>& outer)
{
	const int N = 9;
	int x, y, k;
	cv::Ptr<CvMat> _pts(cvCreateMat(1, N*N, CV_32FC2));
	CvPoint2D32f* pts = (CvPoint2D32f*)(_pts->data.ptr);

	for (y = k = 0; y < N; y++)
	for (x = 0; x < N; x++)
		pts[k++] = cvPoint2D32f((float)x*imgSize.width / (N - 1),
		(float)y*imgSize.height / (N - 1));

	cvUndistortPoints(_pts, _pts, cameraMatrix, distCoeffs, R, newCameraMatrix);

	float iX0 = -FLT_MAX, iX1 = FLT_MAX, iY0 = -FLT_MAX, iY1 = FLT_MAX;
	float oX0 = FLT_MAX, oX1 = -FLT_MAX, oY0 = FLT_MAX, oY1 = -FLT_MAX;
	// find the inscribed rectangle.
	// the code will likely not work with extreme rotation matrices (R) (>45%)
	for (y = k = 0; y < N; y++)
	for (x = 0; x < N; x++)
	{
		CvPoint2D32f p = pts[k++];
		oX0 = MIN(oX0, p.x);
		oX1 = MAX(oX1, p.x);
		oY0 = MIN(oY0, p.y);
		oY1 = MAX(oY1, p.y);

		if (x == 0)
			iX0 = MAX(iX0, p.x);
		if (x == N - 1)
			iX1 = MIN(iX1, p.x);
		if (y == 0)
			iY0 = MAX(iY0, p.y);
		if (y == N - 1)
			iY1 = MIN(iY1, p.y);
	}
	inner = cv::Rect_<float>(iX0, iY0, iX1 - iX0, iY1 - iY0);
	outer = cv::Rect_<float>(oX0, oY0, oX1 - oX0, oY1 - oY0);
}

void mycvStereoRectify(const CvMat* _cameraMatrix1, const CvMat* _cameraMatrix2,
	const CvMat* _distCoeffs1, const CvMat* _distCoeffs2,
	CvSize imageSize, const CvMat* matR, const CvMat* matT,
	CvMat* _R1, CvMat* _R2, CvMat* _P1, CvMat* _P2,
	CvMat* matQ, int flags, double alpha, CvSize newImgSize,
	CvRect* roi1, CvRect* roi2)
{
	double _om[3], _t[3], _uu[3] = { 0, 0, 0 }, _r_r[3][3], _pp[3][4];
	double _ww[3], _wr[3][3], _z[3] = { 0, 0, 0 }, _ri[3][3];
	cv::Rect_<float> inner1, inner2, outer1, outer2;

	CvMat om = cvMat(3, 1, CV_64F, _om);
	CvMat t = cvMat(3, 1, CV_64F, _t);
	CvMat uu = cvMat(3, 1, CV_64F, _uu);
	CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
	CvMat pp = cvMat(3, 4, CV_64F, _pp);
	CvMat ww = cvMat(3, 1, CV_64F, _ww); // temps
	CvMat wR = cvMat(3, 3, CV_64F, _wr);
	CvMat Z = cvMat(3, 1, CV_64F, _z);
	CvMat Ri = cvMat(3, 3, CV_64F, _ri);
	double nx = imageSize.width, ny = imageSize.height;
	int i, k;

	if (matR->rows == 3 && matR->cols == 3)
		cvRodrigues2(matR, &om);          // get vector rotation
	else
		cvConvert(matR, &om); // it's already a rotation vector
	cvConvertScale(&om, &om, -0.5); // get average rotation
	cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging
	cvMatMul(&r_r, matT, &t);

	int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
	double c = _t[idx], nt = cvNorm(&t, 0, CV_L2);
	_uu[idx] = c > 0 ? 1 : -1;

	// calculate global Z rotation
	cvCrossProduct(&t, &uu, &ww);
	double nw = cvNorm(&ww, 0, CV_L2);
	if (nw > 0.0)
		cvConvertScale(&ww, &ww, acos(fabs(c) / nt) / nw);
	cvRodrigues2(&ww, &wR);

	// apply to both views
	cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
	cvConvert(&Ri, _R1);
	cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
	cvConvert(&Ri, _R2);
	cvMatMul(&Ri, matT, &t);

	// calculate projection/camera matrices
	// these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
	double fc_new = DBL_MAX;
	CvPoint2D64f cc_new[2] = { { 0, 0 }, { 0, 0 } };

	for (k = 0; k < 2; k++) {
		const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
		const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
		double dk1 = Dk ? cvmGet(Dk, 0, 0) : 0;
		double fc = cvmGet(A, idx ^ 1, idx ^ 1);
		if (dk1 < 0) {
			fc *= 1 + dk1*(nx*nx + ny*ny) / (4 * fc*fc);
		}
		fc_new = MIN(fc_new, fc);
	}

	for (k = 0; k < 2; k++)
	{
		const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
		const CvMat* Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
		CvPoint2D32f _pts[4];
		CvPoint3D32f _pts_3[4];
		CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
		CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);

		for (i = 0; i < 4; i++)
		{
			int j = (i < 2) ? 0 : 1;
			_pts[i].x = (float)((i % 2)*(nx - 1));
			_pts[i].y = (float)(j*(ny - 1));
		}
		cvUndistortPoints(&pts, &pts, A, Dk, 0, 0);
		cvConvertPointsHomogeneous(&pts, &pts_3);

		//Change camera matrix to have cc=[0,0] and fc = fc_new
		double _a_tmp[3][3];
		CvMat A_tmp = cvMat(3, 3, CV_64F, _a_tmp);
		_a_tmp[0][0] = fc_new;
		_a_tmp[1][1] = fc_new;
		_a_tmp[0][2] = 0.0;
		_a_tmp[1][2] = 0.0;
		cvProjectPoints2(&pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts);
		CvScalar avg = cvAvg(&pts);
		cc_new[k].x = (nx - 1) / 2 - avg.val[0];
		cc_new[k].y = (ny - 1) / 2 - avg.val[1];
	}

	// vertical focal length must be the same for both images to keep the epipolar constraint
	// (for horizontal epipolar lines -- TBD: check for vertical epipolar lines)
	// use fy for fx also, for simplicity

	// For simplicity, set the principal points for both cameras to be the average
	// of the two principal points (either one of or both x- and y- coordinates)
	if (flags & CV_CALIB_ZERO_DISPARITY)
	{
		cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
		cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
	}
	else if (idx == 0) // horizontal stereo
		cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
	else // vertical stereo
		cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

	cvZero(&pp);
	_pp[0][0] = _pp[1][1] = fc_new;
	_pp[0][2] = cc_new[0].x;
	_pp[1][2] = cc_new[0].y;
	_pp[2][2] = 1;
	cvConvert(&pp, _P1);

	_pp[0][2] = cc_new[1].x;
	_pp[1][2] = cc_new[1].y;
	_pp[idx][3] = _t[idx] * fc_new; // baseline * focal length
	cvConvert(&pp, _P2);

	alpha = MIN(alpha, 1.);

	icvGetRectangles(_cameraMatrix1, _distCoeffs1, _R1, _P1, imageSize, inner1, outer1);
	icvGetRectangles(_cameraMatrix2, _distCoeffs2, _R2, _P2, imageSize, inner2, outer2);

	{
		newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
		double cx1_0 = cc_new[0].x;
		double cy1_0 = cc_new[0].y;
		double cx2_0 = cc_new[1].x;
		double cy2_0 = cc_new[1].y;
		//按图像的缩放比例，缩放主点的位置
		double cx1 = newImgSize.width*cx1_0 / imageSize.width;
		double cy1 = newImgSize.height*cy1_0 / imageSize.height;
		double cx2 = newImgSize.width*cx2_0 / imageSize.width;
		double cy2 = newImgSize.height*cy2_0 / imageSize.height;
		double s = 1.;

		if (alpha >= 0)  //焦距长度在此处发生了变化
		{
			double s0 = (std::max)((std::max)((std::max)((double)cx1 / (cx1_0 - inner1.x), (double)cy1 / (cy1_0 - inner1.y)),
				(double)(newImgSize.width - cx1) / (inner1.x + inner1.width - cx1_0)),
				(double)(newImgSize.height - cy1) / (inner1.y + inner1.height - cy1_0));
			s0 = (std::max)((std::max)((std::max)((std::max)((double)cx2 / (cx2_0 - inner2.x), (double)cy2 / (cy2_0 - inner2.y)),
				(double)(newImgSize.width - cx2) / (inner2.x + inner2.width - cx2_0)),
				(double)(newImgSize.height - cy2) / (inner2.y + inner2.height - cy2_0)),
				s0);

			double s1 = (std::min)((std::min)((std::min)((double)cx1 / (cx1_0 - outer1.x), (double)cy1 / (cy1_0 - outer1.y)),
				(double)(newImgSize.width - cx1) / (outer1.x + outer1.width - cx1_0)),
				(double)(newImgSize.height - cy1) / (outer1.y + outer1.height - cy1_0));
			s1 = (std::min)((std::min)((std::min)((std::min)((double)cx2 / (cx2_0 - outer2.x), (double)cy2 / (cy2_0 - outer2.y)),
				(double)(newImgSize.width - cx2) / (outer2.x + outer2.width - cx2_0)),
				(double)(newImgSize.height - cy2) / (outer2.y + outer2.height - cy2_0)),
				s1);

			s = s0*(1 - alpha) + s1*alpha;
		}

		fc_new *= s;
		cc_new[0] = cvPoint2D64f(cx1, cy1);
		cc_new[1] = cvPoint2D64f(cx2, cy2);

		cvmSet(_P1, 0, 0, fc_new);
		cvmSet(_P1, 1, 1, fc_new);
		cvmSet(_P1, 0, 2, cx1);
		cvmSet(_P1, 1, 2, cy1);

		cvmSet(_P2, 0, 0, fc_new);
		cvmSet(_P2, 1, 1, fc_new);
		cvmSet(_P2, 0, 2, cx2);
		cvmSet(_P2, 1, 2, cy2);
		cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));

		if (roi1)
		{
			*roi1 = cv::Rect(cvCeil((inner1.x - cx1_0)*s + cx1),
				cvCeil((inner1.y - cy1_0)*s + cy1),
				cvFloor(inner1.width*s), cvFloor(inner1.height*s))
				& cv::Rect(0, 0, newImgSize.width, newImgSize.height);
		}

		if (roi2)
		{
			*roi2 = cv::Rect(cvCeil((inner2.x - cx2_0)*s + cx2),
				cvCeil((inner2.y - cy2_0)*s + cy2),
				cvFloor(inner2.width*s), cvFloor(inner2.height*s))
				& cv::Rect(0, 0, newImgSize.width, newImgSize.height);
		}
	}

	if (matQ)
	{
		double q[] =
		{
			1, 0, 0, -cc_new[0].x,
			0, 1, 0, -cc_new[0].y,
			0, 0, 0, fc_new,
			0, 0, -1. / _t[idx],
			(idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y) / _t[idx]
		};
		CvMat Q = cvMat(4, 4, CV_64F, q);
		cvConvert(&Q, matQ);
	}
}

void myStereoRectify(InputArray _cameraMatrix1, InputArray _distCoeffs1,
	InputArray _cameraMatrix2, InputArray _distCoeffs2,
	Size imageSize, InputArray _Rmat, InputArray _Tmat,
	OutputArray _Rmat1, OutputArray _Rmat2,
	OutputArray _Pmat1, OutputArray _Pmat2,
	OutputArray _Qmat, int flags,
	double alpha, Size newImageSize,
	Rect* validPixROI1, Rect* validPixROI2)
{
	Mat cameraMatrix1 = _cameraMatrix1.getMat(), cameraMatrix2 = _cameraMatrix2.getMat();
	Mat distCoeffs1 = _distCoeffs1.getMat(), distCoeffs2 = _distCoeffs2.getMat();
	Mat Rmat = _Rmat.getMat(), Tmat = _Tmat.getMat();
	CvMat c_cameraMatrix1 = cameraMatrix1;
	CvMat c_cameraMatrix2 = cameraMatrix2;
	CvMat c_distCoeffs1 = distCoeffs1;
	CvMat c_distCoeffs2 = distCoeffs2;
	CvMat c_R = Rmat, c_T = Tmat;

	int rtype = CV_64F;
	_Rmat1.create(3, 3, rtype);
	_Rmat2.create(3, 3, rtype);
	_Pmat1.create(3, 4, rtype);
	_Pmat2.create(3, 4, rtype);
	CvMat c_R1 = _Rmat1.getMat(), c_R2 = _Rmat2.getMat(), c_P1 = _Pmat1.getMat(), c_P2 = _Pmat2.getMat();
	CvMat c_Q, *p_Q = 0;

	if (_Qmat.needed())
	{
		_Qmat.create(4, 4, rtype);
		p_Q = &(c_Q = _Qmat.getMat());
	}

	mycvStereoRectify(&c_cameraMatrix1, &c_cameraMatrix2, &c_distCoeffs1, &c_distCoeffs2,
		imageSize, &c_R, &c_T, &c_R1, &c_R2, &c_P1, &c_P2, p_Q, flags, alpha,
		newImageSize, (CvRect*)validPixROI1, (CvRect*)validPixROI2);
}

#endif

//opencv Canny源码
#if 1
void myCanny(InputArray _src, OutputArray _dst,
	double low_thresh, double high_thresh,
	int aperture_size, bool L2gradient)
{
	Mat src = _src.getMat();           //输入图像，必须为单通道灰度图  
	CV_Assert(src.depth() == CV_8U); // 8位无符号  

	_dst.create(src.size(), CV_8U);    //根据src的大小构造目标矩阵dst  
	Mat dst = _dst.getMat();           //输出图像，为单通道黑白图  


	// low_thresh 表示低阈值， high_thresh表示高阈值  
	// aperture_size 表示算子大小，默认为3  
	// L2gradient计算梯度幅值的标识，默认为false  

	// 如果L2gradient为false 并且 apeture_size的值为-1（-1的二进制标识为：1111 1111）  
	// L2gradient为false 则计算sobel导数时，用G = |Gx|+|Gy|  
	// L2gradient为true  则计算sobel导数时，用G = Math.sqrt((Gx)^2 + (Gy)^2) 根号下 开平方  

	if (!L2gradient && (aperture_size & CV_CANNY_L2_GRADIENT) == CV_CANNY_L2_GRADIENT)
	{
		// CV_CANNY_L2_GRADIENT 宏定义其值为： Value = (1<<31) 1左移31位  即2147483648  
		//backward compatibility  

		// ~标识按位取反  
		aperture_size &= ~CV_CANNY_L2_GRADIENT;//相当于取绝对值  
		L2gradient = true;
	}


	// 判别条件1：aperture_size是奇数  
	// 判别条件2: aperture_size的范围应当是[3,7], 默认值3   
	if ((aperture_size & 1) == 0 || (aperture_size != -1 && (aperture_size < 3 || aperture_size > 7)))
		CV_Error(CV_StsBadFlag, "");  // 报错  

	if (low_thresh > high_thresh)           // 如果低阈值 > 高阈值  
		std::swap(low_thresh, high_thresh); // 则交换低阈值和高阈值  

#ifdef HAVE_TEGRA_OPTIMIZATION  
	if (tegra::canny(src, dst, low_thresh, high_thresh, aperture_size, L2gradient))
		return;
#endif  

#ifdef USE_IPP_CANNY  
	if (aperture_size == 3 && !L2gradient &&
		ippCanny(src, dst, (float)low_thresh, (float)high_thresh))
		return;
#endif  

	const int cn = src.channels();           // cn为输入图像的通道数  
	Mat dx(src.rows, src.cols, CV_16SC(cn)); // 存储 x方向 方向导数的矩阵，CV_16SC(cn)：16位有符号cn通道  
	Mat dy(src.rows, src.cols, CV_16SC(cn)); // 存储 y方向 方向导数的矩阵 ......  

	/*Sobel参数说明：(参考cvSobel)
	cvSobel(
	const  CvArr* src,                // 输入图像
	CvArr*        dst,                // 输入图像
	int           xorder，            // x方向求导的阶数
	int           yorder，         // y方向求导的阶数
	int           aperture_size = 3   // 滤波器的宽和高 必须是奇数
	);
	*/

	// BORDER_REPLICATE 表示当卷积点在图像的边界时，原始图像边缘的像素会被复制，并用复制的像素扩展原始图的尺寸  
	// 计算x方向的sobel方向导数，计算结果存在dx中  
	Sobel(src, dx, CV_16S, 1, 0, aperture_size, 1, 0, cv::BORDER_REPLICATE); 
	// 计算y方向的sobel方向导数，计算结果存在dy中  
	Sobel(src, dy, CV_16S, 0, 1, aperture_size, 1, 0, cv::BORDER_REPLICATE);

	//L2gradient为true时， 表示需要根号下开平方运算，阈值也需要平方  
	if (L2gradient)
	{
		low_thresh = (std::min)(32767.0, low_thresh);
		high_thresh = (std::min)(32767.0, high_thresh);

		if (low_thresh > 0) low_thresh *= low_thresh;    //低阈值平方运算  
		if (high_thresh > 0) high_thresh *= high_thresh; //高阈值平方运算  
	}

	int low = cvFloor(low_thresh);   // cvFloor返回不大于参数的最大整数值, 相当于取整  
	int high = cvFloor(high_thresh);

	// ptrdiff_t 是C/C++标准库中定义的一个数据类型，signed类型，通常用于存储两个指针的差（距离），可以是负数  
	// mapstep 用于存放  
	ptrdiff_t mapstep = src.cols + 2; // +2 表示左右各扩展一条边  

	// AutoBuffer<uchar> 会自动分配一定大小的内存，并且指定内存中的数据类型是uchar  
	// 列数 +2 表示图像左右各自扩展一条边 （用于复制边缘像素，扩大原始图像）  
	// 行数 +2 表示图像上下各自扩展一条边  
	AutoBuffer<uchar> buffer((src.cols + 2)*(src.rows + 2) + cn * mapstep * 3 * sizeof(int));

	int* mag_buf[3];  //定义一个大小为3的int型指针数组，  
	mag_buf[0] = (int*)(uchar*)buffer;
	mag_buf[1] = mag_buf[0] + mapstep*cn;
	mag_buf[2] = mag_buf[1] + mapstep*cn;
	memset(mag_buf[0], 0, /* cn* */mapstep*sizeof(int));

	uchar* map = (uchar*)(mag_buf[2] + mapstep*cn);
	memset(map, 1, mapstep);
	memset(map + mapstep*(src.rows + 1), 1, mapstep);
	int maxsize = (std::max)(1 << 10, src.cols * src.rows / 10); // 2的10次幂 1024  
	std::vector<uchar*> stack(maxsize); // 定义指针类型向量，用于存地址  
	uchar **stack_top = &stack[0];      // 栈顶指针（指向指针的指针），指向stack[0], stack[0]也是一个指针  
	uchar **stack_bottom = &stack[0];   // 栈底指针 ，初始时 栈底指针 == 栈顶指针  


	// 梯度的方向被近似到四个角度之一 (0, 45, 90, 135 四选一)  
	/* sector numbers
	(Top-Left Origin)

	1   2   3
	*  *  *
	* * *
	0*******0
	* * *
	*  *  *
	3   2   1
	*/


	// define 定义函数块  
	// CANNY_PUSH(d) 是入栈函数， 参数d表示地址指针，让该指针指向的内容为2（int型强制转换成uchar型），并入栈，栈顶指针+1  
	// 2表示 像素属于某条边缘 可以看下方的注释  
	// CANNY_POP(d) 是出栈函数， 栈顶指针-1，然后将-1后的栈顶指针指向的值，赋给d  
#define CANNY_PUSH(d)    *(d) = uchar(2), *stack_top++ = (d)  
#define CANNY_POP(d)     (d) = *--stack_top  

	// calculate magnitude and angle of gradient, perform non-maxima suppression.  
	// fill the map with one of the following values:  
	// 0 - the pixel might belong to an edge 可能属于边缘  
	// 1 - the pixel can not belong to an edge 不属于边缘  
	// 2 - the pixel does belong to an edge 一定属于边缘  

	// for内进行非极大值抑制 + 滞后阈值处理  
	for (int i = 0; i <= src.rows; i++) // i 表示第i行  
	{

		// i == 0 时，_norm 指向 mag_buf[1]  
		// i > 0 时， _norm 指向 mag_buf[2]  
		// +1 表示跳过每行的第一个元素，因为是后扩展的边，不可能是边缘  
		int* _norm = mag_buf[(i > 0) + 1] + 1;

		if (i < src.rows)
		{
			short* _dx = dx.ptr<short>(i); // _dx指向dx矩阵的第i行  
			short* _dy = dy.ptr<short>(i); // _dy指向dy矩阵的第i行  

			if (!L2gradient) // 如果 L2gradient为false  
			{
				for (int j = 0; j < src.cols*cn; j++) // 对第i行里的每一个值都进行计算  
					_norm[j] = std::abs(int(_dx[j])) + std::abs(int(_dy[j])); // 用||+||计算  
			}
			else
			{
				for (int j = 0; j < src.cols*cn; j++)
					//用平方计算,当 L2gradient为 true时，高低阈值都被平方了，所以此处_norm[j]无需开平方  
					_norm[j] = int(_dx[j])*_dx[j] + int(_dy[j])*_dy[j]; //  
			}

			if (cn > 1) // 如果不是单通道  
			{
				for (int j = 0, jn = 0; j < src.cols; ++j, jn += cn)
				{
					int maxIdx = jn;
					for (int k = 1; k < cn; ++k)
					if (_norm[jn + k] > _norm[maxIdx]) maxIdx = jn + k;
					_norm[j] = _norm[maxIdx];
					_dx[j] = _dx[maxIdx];
					_dy[j] = _dy[maxIdx];
				}
			}
			_norm[-1] = _norm[src.cols] = 0; // 最后一列和第一列的梯度幅值设置为0  
		}
		// 当i == src.rows （最后一行）时，申请空间并且每个空间的值初始化为0, 存储在mag_buf[2]中  
		else
			memset(_norm - 1, 0, /* cn* */mapstep*sizeof(int));

		// at the very beginning we do not have a complete ring  
		// buffer of 3 magnitude rows for non-maxima suppression  
		if (i == 0)
			continue;

		uchar* _map = map + mapstep*i + 1; // _map 指向第 i+1 行，+1表示跳过该行第一个元素  
		_map[-1] = _map[src.cols] = 1; // 第一列和最后一列不是边缘，所以设置为1  

		int* _mag = mag_buf[1] + 1; // take the central row 中间那一行  
		ptrdiff_t magstep1 = mag_buf[2] - mag_buf[1];
		ptrdiff_t magstep2 = mag_buf[0] - mag_buf[1];

		const short* _x = dx.ptr<short>(i - 1);
		const short* _y = dy.ptr<short>(i - 1);

		// 如果栈的大小不够，则重新为栈分配内存（相当于扩大容量）  
		if ((stack_top - stack_bottom) + src.cols > maxsize)
		{
			int sz = (int)(stack_top - stack_bottom);
			maxsize = maxsize * 3 / 2;
			stack.resize(maxsize);
			stack_bottom = &stack[0];
			stack_top = stack_bottom + sz;
		}

		int prev_flag = 0; //前一个像素点 0：非边缘点 ；1：边缘点  
		for (int j = 0; j < src.cols; j++) // 第 j 列  
		{
#define CANNY_SHIFT 15  
			// tan22.5  
			const int TG22 = (int)(0.4142135623730950488016887242097*(1 << CANNY_SHIFT) + 0.5);

			int m = _mag[j];

			if (m > low) // 如果大于低阈值  
			{
				int xs = _x[j];    // dx中 第i-1行 第j列  
				int ys = _y[j];    // dy中 第i-1行 第j列  
				int x = std::abs(xs);
				int y = std::abs(ys) << CANNY_SHIFT;

				int tg22x = x * TG22;

				if (y < tg22x) //角度小于22.5 用区间表示：[0, 22.5)  
				{
					// 与左右两点的梯度幅值比较，如果比左右都大  
					//（此时当前点是左右邻域内的极大值），则 goto __ocv_canny_push 执行入栈操作  
					if (m > _mag[j - 1] && m >= _mag[j + 1]) goto __ocv_canny_push;
				}
				else //角度大于22.5  
				{
					int tg67x = tg22x + (x << (CANNY_SHIFT + 1));
					if (y > tg67x) //(67.5, 90)  
					{
						//与上下两点的梯度幅值比较，如果比上下都大  
						//（此时当前点是左右邻域内的极大值），则 goto __ocv_canny_push 执行入栈操作  
						if (m > _mag[j + magstep2] && m >= _mag[j + magstep1]) goto __ocv_canny_push;
					}
					else //[22.5, 67.5]  
					{
						// ^ 按位异或 如果xs与ys异号 则取-1 否则取1  
						int s = (xs ^ ys) < 0 ? -1 : 1;
						//比较对角线邻域  
						if (m > _mag[j + magstep2 - s] && m > _mag[j + magstep1 + s]) goto __ocv_canny_push;
					}
				}
			}

			//比当前的梯度幅值低阈值还低，直接被确定为非边缘  
			prev_flag = 0;
			_map[j] = uchar(1); // 1 表示不属于边缘  

			continue;
		__ocv_canny_push:
			// 前一个点不是边缘点 并且 当前点的幅值大于高阈值（大于高阈值被视为边缘像素） 并且 正上方的点不是边缘点  
			if (!prev_flag && m > high && _map[j - mapstep] != 2)
			{
				//将当前点的地址入栈，入栈前，会将该点地址指向的值设置为2（查看上面的宏定义函数块里）  
				CANNY_PUSH(_map + j);
				prev_flag = 1;
			}
			else
				_map[j] = 0;
		}

		// scroll the ring buffer  
		// 交换指针指向的位置，向上覆盖，把mag_[1]的内容覆盖到mag_buf[0]上  
		// 把mag_[2]的内容覆盖到mag_buf[1]上  
		// 最后 让mag_buf[2]指向_mag指向的那一行  
		_mag = mag_buf[0];
		mag_buf[0] = mag_buf[1];
		mag_buf[1] = mag_buf[2];
		mag_buf[2] = _mag;
	}


	// now track the edges (hysteresis thresholding)  
	// 通过上面的for循环，确定了各个邻域内的极大值点为边缘点（标记为2）  
	// 现在，在这些边缘点的8邻域内（上下左右+4个对角）,将可能的边缘点（标记为0）确定为边缘  
	while (stack_top > stack_bottom)
	{
		uchar* m;
		if ((stack_top - stack_bottom) + 8 > maxsize)
		{
			int sz = (int)(stack_top - stack_bottom);
			maxsize = maxsize * 3 / 2;
			stack.resize(maxsize);
			stack_bottom = &stack[0];
			stack_top = stack_bottom + sz;
		}

		CANNY_POP(m); // 出栈  

		if (!m[-1])         CANNY_PUSH(m - 1);
		if (!m[1])          CANNY_PUSH(m + 1);
		if (!m[-mapstep - 1]) CANNY_PUSH(m - mapstep - 1);
		if (!m[-mapstep])   CANNY_PUSH(m - mapstep);
		if (!m[-mapstep + 1]) CANNY_PUSH(m - mapstep + 1);
		if (!m[mapstep - 1])  CANNY_PUSH(m + mapstep - 1);
		if (!m[mapstep])    CANNY_PUSH(m + mapstep);
		if (!m[mapstep + 1])  CANNY_PUSH(m + mapstep + 1);
	}

	// the final pass, form the final image  
	// 生成边缘图  
	const uchar* pmap = map + mapstep + 1;
	uchar* pdst = dst.ptr();
	for (int i = 0; i < src.rows; i++, pmap += mapstep, pdst += dst.step)
	{
		for (int j = 0; j < src.cols; j++)
			pdst[j] = (uchar)-(pmap[j] >> 1);
	}
}
#endif