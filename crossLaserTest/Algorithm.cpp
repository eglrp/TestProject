#include "Algorithm.h"

bool CAlgorithm::FitPlane(vector<Point3f>srcPnts, double *PlaneParam)
{
	Mat coefficient_matrix = Mat::zeros(3, 3, CV_32F);
	Mat variable_matrix = Mat::zeros(3, 1, CV_32F);
	Mat equation_right = Mat::zeros(3, 1, CV_32F);
	Plane model;
	for (unsigned int i = 0; i<srcPnts.size(); i++)
	{
		coefficient_matrix.at<float>(0, 0) += pow(srcPnts[i].x, 2);
		coefficient_matrix.at<float>(0, 1) += srcPnts[i].x*srcPnts[i].y;
		coefficient_matrix.at<float>(0, 2) += srcPnts[i].x;
		coefficient_matrix.at<float>(1, 1) += pow(srcPnts[i].y, 2);
		coefficient_matrix.at<float>(1, 2) += srcPnts[i].y;
		equation_right.at<float>(0, 0) += srcPnts[i].x*srcPnts[i].z;
		equation_right.at<float>(1, 0) += srcPnts[i].y*srcPnts[i].z;
		equation_right.at<float>(2, 0) += srcPnts[i].z;

	}
	coefficient_matrix.at<float>(1, 0) = coefficient_matrix.at<float>(0, 1);
	coefficient_matrix.at<float>(2, 0) = coefficient_matrix.at<float>(0, 2);
	coefficient_matrix.at<float>(2, 1) = coefficient_matrix.at<float>(1, 2);
	coefficient_matrix.at<float>(2, 2) = float(srcPnts.size());
	if (!solve(coefficient_matrix, equation_right, variable_matrix, cv::DECOMP_CHOLESKY))//高斯消元法?此处系数矩阵为对称矩阵
	{
		return false;
	}
	//方向向量单位化
	float dist = sqrt(pow(variable_matrix.at<float>(0, 0), 2) + pow(variable_matrix.at<float>(1, 0), 2) + float(1.0));
	if (dist == 0)
	{
		model.normal.x = 1;
		return false;
	}
	else
	{
		model.normal.x = variable_matrix.at<float>(0, 0) / dist;
		model.normal.y = variable_matrix.at<float>(1, 0) / dist;
		model.normal.z = float(-1.0) / dist;
		/*		model.normal.x=-variable_matrix.at<float>(0,0)/dist;
		model.normal.y=-variable_matrix.at<float>(1,0)/dist;
		model.normal.z=-float(-1.0)/dist; */
	}
	model.orignal.x = 0;
	model.orignal.y = 0;
	model.orignal.z = variable_matrix.at<float>(2, 0);
	//if (sizeof(PlaneParam) < 4*sizeof(double))
	//{
	//	return false;
	//}	
	PlaneParam[0] = model.normal.x;
	PlaneParam[1] = model.normal.y;
	PlaneParam[2] = model.normal.z;
	PlaneParam[3] = -(PlaneParam[2] * model.orignal.z);

#ifdef DEBUG
	//统计点到平面的距离
	vector<double> dis, meanDis,stdDis;
	double nor = sqrt(PlaneParam[0] * PlaneParam[0] + PlaneParam[1] * PlaneParam[1] + PlaneParam[2] * PlaneParam[2]);
	for (int i = 0; i < srcPnts.size(); i++)
	{
		double temp = (PlaneParam[0] * srcPnts[i].x + PlaneParam[1] * srcPnts[i].y + PlaneParam[2] * srcPnts[i].z + PlaneParam[3]) / nor;
		 dis.push_back(temp);
	}
	cv::meanStdDev(dis, meanDis, stdDis);
	//平面拟合结果显示

	cout << "------------------平面拟合结果-------------------" << endl;
	cout << "parameter vector: [" << PlaneParam[0] << " " << PlaneParam[1] << " " << PlaneParam[2] <<" "<<PlaneParam[3]<<"]"<< endl;
	cout << "mean = " << meanDis[0] << endl;
	cout << "std = " << stdDis[0] << endl;
	cout << "up max = " << *max_element(dis.begin(), dis.end()) << endl;
	cout << "low max = " << *min_element(dis.begin(), dis.end()) << endl;
	cout << "------------------------------------------------" << endl;
#endif // DEBUG
	return true;
}


double CAlgorithm::projectPointToImg(const vector<Point3f> &srcPnts, const vector<Point2f> &srcImgPnts, const Mat& intrinsicPara, const Mat& distCoeffs, const Mat &r, const Mat &t, vector<Point2f> &dstPnt)
{
	//投影
	if (srcPnts.size()== 0)
	{
		return -1;
	}
	cv::projectPoints(srcPnts, r, t, intrinsicPara, distCoeffs, dstPnt);
	//计算反向投影误差
	if (srcImgPnts.size() != srcPnts.size())
	{
		return -1;
	}
	double errMean = 0,errStd = 0;
	for (int i = 0; i < srcImgPnts.size(); i++)
	{
		errMean += sqrt(pow(srcImgPnts[i].x - dstPnt[i].x, 2) + pow(srcImgPnts[i].y - dstPnt[i].y, 2));
	}
	errMean /= srcImgPnts.size();
	for (int i = 0; i < srcImgPnts.size(); i++)
	{
		errStd += (sqrt(pow(srcImgPnts[i].x - dstPnt[i].x, 2) + pow(srcImgPnts[i].y - dstPnt[i].y, 2)) - errMean);
	}
	errStd /= srcImgPnts.size();
	errStd = sqrt(errStd);
	return errStd;
}

bool CAlgorithm::GaussKNew(Mat& gaussK, int Krow, int Kcol, float Ksig)
//计算高斯核 
//Krow:高斯核的行, Kcol:高斯核的列, Krow & Kcol 必须为奇数, Ksig:高斯核的标准差，对于条纹中心提取，使用5x5，标准差=1
{
	if (Krow % 2 == 0 || Kcol % 2 == 0)
	{
		cout << "The size of gaussK is not odd." << endl;
		return false;
	}
	gaussK.create(Krow, Kcol, CV_32FC1);
	for (int i = -Krow / 2; i <= Krow / 2; i++)
	{
		for (int j = -Kcol / 2; j <= Kcol / 2; j++)
		{
			gaussK.at<float>(i + Krow / 2, j + Kcol / 2) = 1 / (2 * PI*Ksig)*exp(-(i*i + j*j) / (2.0*Ksig*Ksig));
		}
	}
	return true;
}

bool CAlgorithm::getRowMaxLocMid(uchar* rowPtr, int colStart, int colEnd, int& maxValue, int& colMaxMid, const bool& MaxMidType)
//查找最大值；MaxMidType=1:定位中间；MaxMidType=0：定位第一个最大值
{
	maxValue = rowPtr[colStart];
	colMaxMid = colStart;
	if (MaxMidType == 0)
	{
		for (int col = colStart + 1; col < colEnd; ++col)
		{
			if (rowPtr[col]>maxValue)
			{
				maxValue = rowPtr[col];
				colMaxMid = col;
			}
		}
	}
	else if (MaxMidType == 1)
	{
		int colMax = colStart;
		for (int col = colStart + 1; col < colEnd; ++col)
		{
			if (rowPtr[col]>maxValue)
			{
				maxValue = rowPtr[col];
				colMax = col;
				colMaxMid = colMax;
			}
			else if (rowPtr[col] == maxValue)
			{
				//原来的处理方法：
				//colMaxMid = (colMax + col) / 2;
				//zzl modify 判断中间点的亮度是否也是较亮点，如果不是则舍弃，取原来的最大点
				int colMaxMidtep = 0;
				colMaxMidtep = (colMax + col) / 2;
				if (abs(rowPtr[colMaxMidtep] - maxValue) < 20)
				{
					colMaxMid = colMaxMidtep;
				}
			}
		}
	}
	return true;
}

bool CAlgorithm::getMultMax(uchar* rowPtr, int colStart, int colEnd, int windowwidth, int findNum, vector <uchar>& maxValue, vector <int>& colMax)
{
	int halfwin = ceil(windowwidth / 2.0);
	for (int i = 0; i < findNum; i++)
	{
		uchar _max = 100;
		int _maxPos = 0;
		for (int j = colStart + 1; j < colEnd; j++)
		{
			if (rowPtr[j] > _max)
			{
				bool _flag = false;
				//遍历已经提取的最大值点，找到离他最近的最大值点，从这两点的中间点处开始往外遍历一个windowwidth宽度，如果出现背景亮度，则认为该最大值是
				//新条纹上的最大值，如果没有，则为同一条纹上的最大值。
				for (int k = 0; k < colMax.size(); k++)
				{
					//int mid = (j + colMax[k]) / 2;
					//int low = min(mid - halfwin, min(j, colMax[k]));
					//int up = min(mid + halfwin, max(j, colMax[k]));
					//for (int n = low; n < up; n++)
					//{
					//	if (rowPtr[n] < 50){
					//		_flag = true;
					//		break;
					//	}
					//	_flag = false;
					//}
					if (abs(j - colMax[k]) < windowwidth){
						_flag = true;
						break;
					}

				}
				if (!_flag)
				{
					_max = rowPtr[j];
					_maxPos = j;
				}
			}
		}
		maxValue.push_back(_max);
		colMax.push_back(_maxPos);
	}
	return true;
}

bool CAlgorithm::getRowMaxLocMid(Mat& rowMat, int colStart, int colEnd, int& maxValue, int& colMaxMid, const bool& MaxMidType)
//查找最大值；MaxMidType=1:定位中间；MaxMidType=0：定位第一个最大值
{
	//只能用于行遍历，当进行列遍历时，会因内存不连续出现问题
#if 1
	if (!rowMat.isContinuous())
	{
		return false;
	}
	uchar* rowPtr = rowMat.ptr<uchar>(0);
	maxValue = rowPtr[colStart];
	colMaxMid = colStart;
	if (MaxMidType == 0)
	{
		for (int col = colStart + 1; col < colEnd; ++col)
		{
			if (rowPtr[col]>maxValue)
			{
				maxValue = rowPtr[col];
				colMaxMid = col;
			}
		}
	}
	else if (MaxMidType == 1)
	{
		int colMax = colStart;
		for (int col = colStart + 1; col < colEnd; ++col)
		{
			if (rowPtr[col]>maxValue)
			{
				maxValue = rowPtr[col];
				colMax = col;
				colMaxMid = colMax;
			}
			else if (rowPtr[col] == maxValue)
			{
				//原来的处理方法：
				//colMaxMid = (colMax + col) / 2;
				//zzl modify 判断中间点的亮度是否也是较亮点，如果不是则舍弃，取原来的最大点
				int colMaxMidtep = 0;
				colMaxMidtep = (colMax + col) / 2;
				if (abs(rowPtr[colMaxMidtep] - maxValue) < 20)
				{
					colMaxMid = colMaxMidtep;
				}
				//end zzl modify  
			}
		}
	}
#endif
	//add by zzl 修复以上问题
#if 0
	if (rowMat.rows != 1 && rowMat.cols !=1)
	{
		cout << "Row image error." << endl;
		return false;
	}
	if ()
	{
	}
	maxValue = rowMat.at<uchar>(colStart);
	colMaxMid = colStart;
	if (MaxMidType == 0)
	{
		for (int col = colStart + 1; col < colEnd; ++col)
		{
			if (rowPtr[col]>maxValue)
			{
				maxValue = rowPtr[col];
				colMaxMid = col;
			}
		}
	}
	else if (MaxMidType == 1)
	{
		int colMax = colStart;
		for (int col = colStart + 1; col < colEnd; ++col)
		{
			if (rowPtr[col]>maxValue)
			{
				maxValue = rowPtr[col];
				colMax = col;
				colMaxMid = colMax;
			}
			else if (rowPtr[col] == maxValue)
			{
				//原来的处理方法：
				//colMaxMid = (colMax + col) / 2;
				//zzl modify 判断中间点的亮度是否也是较亮点，如果不是则舍弃，取原来的最大点
				int colMaxMidtep = 0;
				colMaxMidtep = (colMax + col) / 2;
				if (abs(rowPtr[colMaxMidtep] - maxValue) < 20)
				{
					colMaxMid = colMaxMidtep;
				}
				//end zzl modify  
			}
		}
	}
#endif

	return true;
}

bool CAlgorithm::getColMaxLocMid(Mat& colMat, int rowStart, int rowEnd, int& maxValue, int& rowMaxMid, const bool& MaxMidType)
//查找最大值；MaxMidType=1:定位中间；MaxMidType=0：定位第一个最大值
{
	if (colMat.cols != 1)
	{
		cout << "Row image error." << endl;
		return false;
	}
	maxValue = colMat.at<uchar>(rowStart,0);
	rowMaxMid = rowStart;
	if (MaxMidType == 0)
	{
		for (int row = rowStart + 1; row < rowEnd; ++row)
		{
			if (colMat.at<uchar>(row, 0)>maxValue)
			{
				maxValue = colMat.at<uchar>(row, 0);
				rowMaxMid = row;
			}
		}
	}
	else if (MaxMidType == 1)
	{
		int rowMax = rowStart;
		for (int row = rowStart + 1; row < rowEnd; ++row)
		{
			if (colMat.at<uchar>(row, 0)>maxValue)
			{
				maxValue = colMat.at<uchar>(row, 0);
				rowMax = row;
				rowMaxMid = rowMax;
			}
			else if (colMat.at<uchar>(row, 0) == maxValue)
			{
				//原来的处理方法：
				//colMaxMid = (colMax + col) / 2;
				//zzl modify 判断中间点的亮度是否也是较亮点，如果不是则舍弃，取原来的最大点
				int colMaxMidtep = 0;
				colMaxMidtep = (rowMax + row) / 2;
				if (abs(colMat.at<uchar>(colMaxMidtep, 0) - maxValue) < 20)
				{
					rowMaxMid = colMaxMidtep;
				}
				//end zzl modify  
			}
		}
	}
	return true;
}

bool CAlgorithm::PartGaussianBlur(Mat& srcImg, Mat& srcCpy, int rowCurrent, int colStart, int colEnd, Mat& gaussK)
{
	if (gaussK.rows != 5 || gaussK.cols != 5)
	{
		cout << "GaussBlur Kernal'size error." << endl;
		return false;
	}
	if (srcImg.empty() || srcCpy.empty())
	{
		cout << "GaussBlur Image is empty." << endl;
		return false;
	}
	for (int col = colStart; col < colEnd; ++col)
	{
		float numTem = 0;
		for (int rowTem = 0; rowTem < 5; ++rowTem)
		{
			for (int colTem = 0; colTem < 5; ++colTem)
			{
				numTem = numTem + srcCpy.at<uchar>(rowCurrent - 2 + rowTem, col - 2 + colTem)*gaussK.at<float>(rowTem, colTem);
			}
		}
		//Mat srcKernal = srcImg(cv::Range(rowCurrent - 2, rowCurrent + 3), cv::Range(col - 2, col + 3));
		//srcKernal.convertTo(srcKernal, CV_32FC1);
		//Mat kernalSum = srcKernal.mul(gaussK);
		//cv::Scalar sTem = sum(kernalSum);
		srcImg.at<uchar>(rowCurrent, col) = numTem;
	}
	return true;
}

bool CAlgorithm::PartGaussianBlur_col(Mat& srcImg, Mat& srcCpy, int curCol, int rowStart, int rowEnd, Mat& gaussK)
{
	if (gaussK.rows != 5 || gaussK.cols != 5)
	{
		cout << "GaussBlur Kernal'size error." << endl;
		return false;
	}
	if (srcImg.empty() || srcCpy.empty())
	{
		cout << "GaussBlur Image is empty." << endl;
		return false;
	}
	for (int row = rowStart; row < rowEnd; ++row)
	{
		float numTem = 0;
		for (int rowTem = 0; rowTem < 5; ++rowTem)
		{
			for (int colTem = 0; colTem < 5; ++colTem)
			{
				numTem = numTem + srcCpy.at<uchar>(row - 2 + rowTem, curCol - 2 + colTem)*gaussK.at<float>(rowTem, colTem);
			}
		}
		srcImg.at<uchar>(row, curCol) = numTem;
	}
	return true;
}

bool CAlgorithm::FitPlane2(vector<Point3f>srcPnts, double *PlaneParam)
{
	Mat pntTomean = Mat::zeros(srcPnts.size(),3,CV_64FC1);
	Scalar _mean = cv::mean(srcPnts);
	for (int i = 0; i < srcPnts.size(); i++)
	{
		pntTomean.at<double>(i, 0) = srcPnts[i].x - _mean[0];
		pntTomean.at<double>(i, 1) = srcPnts[i].y - _mean[1];
		pntTomean.at<double>(i, 2) = srcPnts[i].z - _mean[2];
	}
	Mat u, v, w;
	cv::SVDecomp(pntTomean, u, v, w);
	double a = w.at<double>(0, 2);
	double b = w.at<double>(1, 2);
	double c = w.at<double>(2, 2);
	double l1 = sqrt(a*a+b*b+c*c);
	double d = -cv::Vec3d(a,b,c).dot(cv::Vec3d(_mean[0],_mean[1],_mean[2]));

	PlaneParam[0] = a / l1;
	PlaneParam[1] = b / l1;
	PlaneParam[2] = c / l1;
	PlaneParam[3] = d / l1;
	//平面拟合结果显示
#ifdef DEBUG
	cout << "------------------平面拟合结果-------------------" << endl;
	cout << "parameter vector: [" << PlaneParam[0] << " " << PlaneParam[1] << " " << PlaneParam[2] << " " << PlaneParam[3] << "]" << endl;
	//cout << "mean = " << meanDis[0] << endl;
	//cout << "std = " << stdDis[0] << endl;
	//cout << "up max = " << *max_element(dis.begin(), dis.end()) << endl;
	//cout << "low max = " << *min_element(dis.begin(), dis.end()) << endl;
	cout << "------------------------------------------------" << endl;
#endif // DEBUG
	return true;
}

bool CAlgorithm::FitCircle3D(vector<Point3f>srcPnts, circle3d &circle3dParam)
{
	//***拟合平面
	double planeParams[4];
	FitPlane(srcPnts, planeParams);
	Mat planeParams_ori = Mat(4, 1, CV_64FC1 ,planeParams);
	//***求球的特殊解
	Mat A = Mat::ones(srcPnts.size(), 4, CV_64FC1);
	Mat B = Mat::zeros(srcPnts.size(),1,CV_64FC1);
	for (int i = 0; i < srcPnts.size();i++)
	{
		A.at<double>(i, 0) = srcPnts[i].x;
		A.at<double>(i, 1) = srcPnts[i].y;
		A.at<double>(i, 2) = srcPnts[i].z;
		B.at<double>(i, 0) = pow(cv::norm(srcPnts[i]), 2);
	}
	Mat temp = A.t()*A;
	invert(temp, temp, cv::DECOMP_SVD); //求伪逆
	Mat ballParams = temp*A.t()*B;
	//***求平面上的球心点
	Mat planeParams_zero = (Mat_<double>(1, 4) << planeParams[0], planeParams[1], planeParams[2], 0);
	planeParams_zero = planeParams_zero / 2;
	temp = -(planeParams_zero * ballParams + planeParams[3]) / (planeParams_zero*planeParams_ori);
	double lamda = temp.at<double>(0,0);
	Mat center_temp = ballParams + lamda*planeParams_ori;
	//***输出结果
	circle3dParam.center = Point3d(Mat(center_temp.rowRange(0, 3)*0.5));
	circle3dParam.normal = cv::Vec3d(planeParams_ori.rowRange(0, 3));
	circle3dParam.normal = normalize(circle3dParam.normal);
	circle3dParam.radius = sqrt(center_temp.at<double>(3, 0) + pow(norm(circle3dParam.center), 2));
	cout << "-------------空间圆拟合结果-------------" << endl;
	cout << "circle center: " << circle3dParam.center << endl;
	cout << "circle normal: " << circle3dParam.normal << endl;
	cout << "circle raius: " << circle3dParam.radius << endl;
	cout << "--------------------------------------" << endl;
	return true;
}

bool CAlgorithm::LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)
{
	//将图像转化成灰度图
	if (src.channels() != 1){
		cvtColor(src, src, CV_RGB2GRAY);
	}

	//截图原图像的roi部分作为中心提取的新图像
	if (ROI != cv::Rect(0, 0, 0, 0)){
		src = src(ROI);
	}
	//进行高斯滤波，去除噪声---为了提高效率，只对条纹中心区域进行滤波
	//GaussianBlur(src, src, Size(5, 5), 0, 0);
	//先定义原图像的备份
	Mat srcCpy = src.clone();

	//定义是否是新条纹标识
	bool isNear = 0;
	//是否是条纹上第一个点标志
	bool isFirst = 1;

	//求该行图像中亮度最大值所在位置Maxc
	Point data;
	Point2f center;
	int maxVal;
	Point maxLoc;

	int trapeHeiTem = 0;
	int colRangeHalf = 0;
	//max函数中mask的起始列和终止列
	int colStart = 0;
	int colEnd = 0;
	int colMax;

	//threTem用于设定首点和新条纹的阈值
	int threTem;
	//设定条纹中心第一个点的列，以此确定条纹的骨架区域
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//有效条纹中心提取的光条宽度均值和标准差
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//窗口宽度的一半
	int width = (windowwidth - 1) / 2;

	//由于使用高斯滤波时没有对边界进行插值，所以行的开始值和结束值必须缩减
	for (int i = 2; i < src.rows - 2; i++)
	{
		Mat src1 = src.row(i);
		maxVal = 0;
		if (isNear)
		{
			int centerOldCol = LaserCenter.size() - 1;
			colRangeHalf = (5 * windowwidth + 4 * trapeHeiTem - 1) / 2;
			if (LaserCenter[centerOldCol].x - colRangeHalf<0)
			{
				colStart = 0;
			}
			else
			{
				colStart = int(LaserCenter[centerOldCol].x - colRangeHalf);
			}
			if (LaserCenter[centerOldCol].x + colRangeHalf>src.cols - 1)
			{
				colEnd = src.cols - 1;
			}
			else
			{
				colEnd = int(LaserCenter[centerOldCol].x + colRangeHalf);
			}
			//mask
			//Mat colMask(1, src.cols, CV_8UC1, cv::Scalar(0));
			//for (int colTem = colStart; colTem < colEnd; ++colTem)
			//{
			//	colMask.at<uchar>(0, colTem) = 1;
			//}
			//该minMaxLoc函数只能够找到最大值左边界的点及其位置，所以下面查找左右边界的方法有问题
			//getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
			getRowMaxLocMid(src1, colStart, colEnd, maxVal, colMax, 1);
			//minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, colMask); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
			data.y = i;
			data.x = colMax;

			//进行越界检查
			if (data.x<windowwidth / 2 + 2 || data.x> src1.cols - 1 - windowwidth / 2 - 2)
			{
				if (trapeHeiTem >= searchWinHei)
				{
					trapeHeiTem = 0;
					isNear = 0;
				}
				else
				{
					++trapeHeiTem;
				}
				continue;
			}
			//确定前景亮度I1、背景亮度I2、条纹宽度w
			double I1 = 0, I2 = 0, w = 0;
			double m1 = 0, m2 = 0, m3 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				m1 += pow(src.at<uchar>(data.y, data.x - width + j), 1);
				m2 += pow(src.at<uchar>(data.y, data.x - width + j), 2);
				m3 += pow(src.at<uchar>(data.y, data.x - width + j), 3);
			}
			m1 = m1 / windowwidth;
			m2 = m2 / windowwidth;
			m3 = m3 / windowwidth;

			double a = 0, b = 0, c = 0;
			a = m2 - m1*m1;
			b = m1*m2 - m3;
			c = m1*m3 - m2*m2;

			I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
			I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
			w = windowwidth*((m1 - I1) / (I2 - I1));

			//Step 3： I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
			//if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold)
			if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold)
			{
				if (trapeHeiTem >= searchWinHei)
				{
					trapeHeiTem = 0;
					isNear = 0;
				}
				else
				{
					++trapeHeiTem;
				}
				continue;
			}
			//比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
			//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
			if (windowwidth < w)
			{
				if (trapeHeiTem >= searchWinHei)
				{
					trapeHeiTem = 0;
					isNear = 0;
				}
				else
				{
					++trapeHeiTem;
				}
				continue;
			}
			//////////////////////////////////
			////////////////zwy///////////////
#ifdef DEBUGzwy
			if (w > 6)
			{
				//PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);
				PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);

				int colLeft, colLeftTmp = 0;
				int length = 0, lengthmax, FirstTimeFlag = 0, LeftFlag = 0;
				uchar* src1Ptr = src1.ptr<uchar>(0);
				for (int i = 0; i < src1.cols; i++)
				{
					if (src1Ptr[i] >= 240)
					{
						length++;
						if (LeftFlag == 0)
						{
							colLeftTmp = i;
							LeftFlag = 1;
						}
					}
					else
					{
						if (FirstTimeFlag == 0)
						{
							lengthmax = length;
							colLeft = colLeftTmp;
							length = 0;
							FirstTimeFlag = 1;
						}
						else
						{
							if (lengthmax < length)
							{
								lengthmax = length;
								colLeft = colLeftTmp;
								length = 0;
							}
						}
						LeftFlag = 0;
					}
				}
				if (lengthmax < length)
					lengthmax = length;
				center.x = colLeft + lengthmax / 2.0;
				center.y = data.y;
				LaserCenter.push_back(center);
				continue;
			}
#endif
			//////////////////////////////////
			////////////////zwy///////////////

			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>(data.y, data.x - width + j);
				if (PixelValue>I1 + Threshold)  //改动
				{
					sum1 += (PixelValue - I1)*(data.x - width + j);
					sum2 += PixelValue - I1;
				}
			}
			if (sum1 != 0)
			{
				center.y = data.y;
				center.x = sum1 / sum2;
				LaserCenter.push_back(center);
				stripeWidth.push_back(w);
				trapeHeiTem = 0;
			}

		}
		else
		{
			//加入CenterRange操作，限制只在起始点列的centerRange范围内查找
			if (isFirst)
			{
				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
				data.y = i;
				data.x = maxLoc.x;
				//对于起始点前景和背景阈值使用的大一点，以保证激光条纹的骨架是绝对正确的
				threTem = 30;
			}
			else
			{
				//getRowMaxLoc(src1, rangeColStart, rangeColEnd, maxVal, colMax);
				getRowMaxLocMid(src1, rangeColStart, rangeColEnd, maxVal, colMax, 1);

				data.y = i;
				data.x = colMax;
				threTem = Threshold;
			}

			//进行越界检查
			if (data.x<windowwidth / 2 + 2 || data.x> src1.cols - 1 - windowwidth / 2 - 2)
			{
				continue;
			}
			//确定前景亮度I1、背景亮度I2、条纹宽度w
			double I1 = 0, I2 = 0, w = 0;

			double m1 = 0, m2 = 0, m3 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				m1 += pow(src.at<uchar>(data.y, data.x - width + j), 1);
				m2 += pow(src.at<uchar>(data.y, data.x - width + j), 2);
				m3 += pow(src.at<uchar>(data.y, data.x - width + j), 3);
			}
			m1 = m1 / windowwidth;
			m2 = m2 / windowwidth;
			m3 = m3 / windowwidth;

			double a = 0, b = 0, c = 0;
			a = m2 - m1*m1;
			b = m1*m2 - m3;
			c = m1*m3 - m2*m2;

			I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
			I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
			w = windowwidth*((m1 - I1) / (I2 - I1));

			if (a <0 || b > 0 || c < 0 || I2 - I1 <= threTem)
			{
				continue;
			}


			//比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
			//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
			if (windowwidth < w)
			{
				continue;
			}
			//////////////////////////////////
			////////////////zwy///////////////
#ifdef DEBUGzwy
			if (w > 6)
			{
				PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);
				//PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);

				int colLeft, colLeftTmp = 0;
				int length = 0, lengthmax, FirstTimeFlag = 0, LeftFlag = 0;
				uchar* src1Ptr = src1.ptr<uchar>(0);
				for (int i = 0; i < src1.cols; i++)
				{
					if (src1Ptr[i] >= 240)
					{
						length++;
						if (LeftFlag == 0)
						{
							colLeftTmp = i;
							LeftFlag = 1;
						}
					}
					else
					{
						if (FirstTimeFlag == 0)
						{
							lengthmax = length;
							colLeft = colLeftTmp;
							length = 0;
							FirstTimeFlag = 1;
						}
						else
						{
							if (lengthmax < length)
							{
								lengthmax = length;
								colLeft = colLeftTmp;
								length = 0;
							}
						}
						LeftFlag = 0;
					}
				}
				if (lengthmax < length)
					lengthmax = length;
				center.x = colLeft + lengthmax / 2.0;
				center.y = data.y;
				LaserCenter.push_back(center);
				if (LaserCenter.size() == 1)
				{
					firstCenterCol = LaserCenter[0].x;
					isFirst = 0;

					if (firstCenterCol - centerRange < 0)
					{
						rangeColStart = 0;
					}
					else
					{
						rangeColStart = firstCenterCol - centerRange;
					}
					if (firstCenterCol + centerRange > src1.cols - 1)
					{
						rangeColEnd = src1.cols - 1;
					}
					else
					{
						rangeColEnd = firstCenterCol + centerRange;
					}
				}
				isNear = 1;
				continue;
			}
#endif
			//////////////////////////////////
			////////////////zwy///////////////

			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//重心法求亚像素条纹中心，并统计重心计算的像素个数n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>(data.y, data.x - ((windowwidth - 1) / 2) + j);
				if (PixelValue>I1 + Threshold)
				{
					sum1 += (PixelValue - I1)*(data.x - width + j);
					sum2 += PixelValue - I1;
				}
			}
			if (sum1 != 0)
			{
				center.y = data.y;
				center.x = sum1 / sum2;
				LaserCenter.push_back(center);
				stripeWidth.push_back(w);
				if (LaserCenter.size() == 1)
				{
					firstCenterCol = LaserCenter[0].x;
					isFirst = 0;

					if (firstCenterCol - centerRange < 0)
					{
						rangeColStart = 0;
					}
					else
					{
						rangeColStart = firstCenterCol - centerRange;
					}
					if (firstCenterCol + centerRange > src1.cols - 1)
					{
						rangeColEnd = src1.cols - 1;
					}
					else
					{
						rangeColEnd = firstCenterCol + centerRange;
					}
				}
				isNear = 1;

			}
		}

	}
	//zzl modify 进行过亮点剔除
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0]; 
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //只对宽度变化较大的条纹进行粗大剔除
	{
		for (int i = 0; i < stripeWidth.size(); i++)
		{
			if (stripeWidth[i] > meanWidth + 1.5*stdWidth)
			{
				continue;
			}
			temCenters.push_back(LaserCenter[i]);
		}
		LaserCenter = temCenters;
	}
#endif

#ifdef _DEBUG
	cvtColor(srcCpy, srcCpy, CV_GRAY2RGB);
	for (int i = 0; i < LaserCenter.size(); i++)
	{
		srcCpy.at<cv::Vec3b>(LaserCenter[i].y, LaserCenter[i].x) = cv::Vec3b(0, 0, 255);
	}
#endif // _DEBUG

	//将roi图像条纹中心坐标转换成原图像中的坐标
	for (int num = 0; num < LaserCenter.size(); ++num)
	{
		LaserCenter[num].x = LaserCenter[num].x + ROI.tl().x;
		LaserCenter[num].y = LaserCenter[num].y + ROI.tl().y;
	}

	return 1;
}

bool CAlgorithm::LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType)
{
	cv::Rect roi;
	roi.x = 0; roi.y = 0;
	roi.width = src.cols; roi.height = src.rows; //待确认
	LaserCenterDetect_row(src, Threshold, windowwidth, centerRange, LaserCenter, stripeMeanWidth, stripeStdWidth, searchWinHei, gaussK, MaxMidType, roi);
	return true;
}

bool CAlgorithm::LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)
{
	//将图像转化成灰度图
	if (src.channels() != 1){
		cvtColor(src,src,CV_RGB2GRAY);
	}

	//截图原图像的roi部分作为中心提取的新图像
	if (ROI != cv::Rect(0,0,0,0)){
		src = src(ROI);
	}
	//先定义原图像的备份
	Mat srcCpy = src.clone();

	//定义是否是新条纹标识
	bool isNear = 0;
	//是否是条纹上第一个点标志
	bool isFirst = 1;

	//求该行图像中亮度最大值所在位置Maxc
	Point data;
	Point2f center;
	int maxVal;
	Point maxLoc;

	int trapeHeiTem = 0;
	int colRangeHalf = 0;
	//max函数中mask的起始列和终止列
	int colStart = 0;
	int colEnd = 0;
	int colMax;

	//threTem用于设定首点和新条纹的阈值
	int threTem;
	//设定条纹中心第一个点的列，以此确定条纹的骨架区域
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//有效条纹中心提取的光条宽度均值和标准差
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//窗口宽度的一半
	int width = (windowwidth - 1) / 2;
	//test
	clock_t ta1 = 0;

	//由于使用高斯滤波时没有对边界进行插值，所以行的开始值和结束值必须缩减
	for (int i = 2; i < src.cols - 2; i++)
	{
		Mat src1 = src.col(i);
		maxVal = 0;
		if (isNear)
		{
			int centerOldCol = LaserCenter.size() - 1;
			colRangeHalf = (5 * windowwidth + 4 * trapeHeiTem - 1) / 2;
			if (LaserCenter[centerOldCol].y - colRangeHalf<0)
			{
				colStart = 0;
			}
			else
			{
				colStart = int(LaserCenter[centerOldCol].y - colRangeHalf);
			}
			if (LaserCenter[centerOldCol].y + colRangeHalf>src.rows - 1)
			{
				colEnd = src.rows - 1;
			}
			else
			{
				colEnd = int(LaserCenter[centerOldCol].y + colRangeHalf);
			}
			//mask
			//Mat colMask(1, src.cols, CV_8UC1, cv::Scalar(0));
			//for (int colTem = colStart; colTem < colEnd; ++colTem)
			//{
			//	colMask.at<uchar>(0, colTem) = 1;
			//}
			//该minMaxLoc函数只能够找到最大值左边界的点及其位置，所以下面查找左右边界的方法有问题
			//getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
	/*		clock_t t1 = clock();*/
			getColMaxLocMid(src1, colStart, colEnd, maxVal, colMax, 1);
			//clock_t t2 = clock();
			//ta1 += t2 - t1;
			//minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, colMask); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
			data.y = i;
			data.x = colMax;

			//进行越界检查
			if (data.x<windowwidth / 2 + 2 || data.x> src1.rows - 1 - windowwidth / 2 - 2)
			{
				if (trapeHeiTem >= searchWinHei)
				{
					trapeHeiTem = 0;
					isNear = 0;
				}
				else
				{
					++trapeHeiTem;
				}
				continue;
			}
			//确定前景亮度I1、背景亮度I2、条纹宽度w
			double I1 = 0, I2 = 0, w = 0;
			double m1 = 0, m2 = 0, m3 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				m1 += pow(src.at<uchar>(data.x - width + j,data.y), 1);
				m2 += pow(src.at<uchar>(data.x - width + j,data.y), 2);
				m3 += pow(src.at<uchar>(data.x - width + j,data.y), 3);
			}
			m1 = m1 / windowwidth;
			m2 = m2 / windowwidth;
			m3 = m3 / windowwidth;

			double a = 0, b = 0, c = 0;
			a = m2 - m1*m1;
			b = m1*m2 - m3;
			c = m1*m3 - m2*m2;

			I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
			I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
			w = windowwidth*((m1 - I1) / (I2 - I1));

			//Step 3： I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
			//if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold)
			if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold)
			{
				if (trapeHeiTem >= searchWinHei)
				{
					trapeHeiTem = 0;
					isNear = 0;
				}
				else
				{
					++trapeHeiTem;
				}
				continue;
			}
			//比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
			//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
			if (windowwidth < w)
			{
				if (trapeHeiTem >= searchWinHei)
				{
					trapeHeiTem = 0;
					isNear = 0;
				}
				else
				{
					++trapeHeiTem;
				}
				continue;
			}

			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur_col(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>( data.x - width + j, data.y); //col
				if (PixelValue>I1 + Threshold)  //改动
				{
					sum1 += (PixelValue - I1)*(data.x - width + j);
					sum2 += PixelValue - I1;
				}
			}
			if (sum1 != 0)
			{
				center.x = data.y;
				center.y = sum1 / sum2;
				LaserCenter.push_back(center);
				stripeWidth.push_back(w);
				trapeHeiTem = 0;
			}

		}
		else
		{
			//加入CenterRange操作，限制只在起始点列的centerRange范围内查找
			if (isFirst)
			{
				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
				data.y = i;
				data.x = maxLoc.y;
				//对于起始点前景和背景阈值使用的大一点，以保证激光条纹的骨架是绝对正确的
				threTem = 30;
			}
			else
			{
				//getRowMaxLoc(src1, rangeColStart, rangeColEnd, maxVal, colMax);
				/*			clock_t t3;
							getColMaxLocMid(src1, rangeColStart, rangeColEnd, maxVal, colMax, 1);
							clock_t t4;
							ta1 += t4 - t3;
							data.y = i;
							data.x = colMax;*/

				////删除centerRange限制 替换上面代码
				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
				data.y = i;
				data.x = maxLoc.y;
				threTem = Threshold;
			}

			//进行越界检查
			if (data.x<windowwidth / 2 + 2 || data.x> src1.rows - 1 - windowwidth / 2 - 2)
			{
				continue;
			}
			//确定前景亮度I1、背景亮度I2、条纹宽度w
			double I1 = 0, I2 = 0, w = 0;

			double m1 = 0, m2 = 0, m3 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				m1 += pow(src.at<uchar>(data.x - width + j, data.y), 1); //col 修改处
				m2 += pow(src.at<uchar>(data.x - width + j, data.y), 2);
				m3 += pow(src.at<uchar>(data.x - width + j, data.y), 3);
			}
			m1 = m1 / windowwidth;
			m2 = m2 / windowwidth;
			m3 = m3 / windowwidth;

			double a = 0, b = 0, c = 0;
			a = m2 - m1*m1;
			b = m1*m2 - m3;
			c = m1*m3 - m2*m2;

			I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
			I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
			w = windowwidth*((m1 - I1) / (I2 - I1));

			if (a <0 || b > 0 || c < 0 || I2 - I1 <= threTem)
			{
				continue;
			}


			//比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
			//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
			if (windowwidth < w)
			{
				continue;
			}

			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur_col(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//重心法求亚像素条纹中心，并统计重心计算的像素个数n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>(data.x - ((windowwidth - 1) / 2) + j, data.y);
				if (PixelValue>I1 + Threshold)
				{
					sum1 += (PixelValue - I1)*(data.x - width + j);
					sum2 += PixelValue - I1;
				}
			}
			if (sum1 != 0)
			{
				center.x = data.y;
				center.y = sum1 / sum2;
				LaserCenter.push_back(center);
				stripeWidth.push_back(w);
				if (LaserCenter.size() == 1)
				{
					firstCenterCol = LaserCenter[0].y; //col
					isFirst = 0;
					//删除centerRange限制
					/*if (firstCenterCol - centerRange < 0)
					{
					rangeColStart = 0;
					}
					else
					{
					rangeColStart = firstCenterCol - centerRange;
					}
					if (firstCenterCol + centerRange > src1.rows - 1)
					{
					rangeColEnd = src1.rows - 1;
					}
					else
					{
					rangeColEnd = firstCenterCol + centerRange;
					}*/
				}
				isNear = 1;

			}
		}

	}
	//zzl modify 进行过亮点剔除
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0];
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //只对宽度变化较大的条纹进行粗大剔除
	{
		for (int i = 0; i < stripeWidth.size(); i++)
		{
			if (stripeWidth[i] > meanWidth + 1.5*stdWidth)
			{
				continue;
			}
			temCenters.push_back(LaserCenter[i]);
		}
		LaserCenter = temCenters;
	}
#endif

#ifdef _DEBUG
	cvtColor(srcCpy, srcCpy, CV_GRAY2RGB);
	for (int i = 0; i < LaserCenter.size(); i++)
	{
		srcCpy.at<cv::Vec3b>(LaserCenter[i].y, LaserCenter[i].x) = cv::Vec3b(0, 0, 255);
	}
#endif // _DEBUG

	//将roi图像条纹中心坐标转换成原图像中的坐标
	for (int num = 0; num < LaserCenter.size(); ++num)
	{
		LaserCenter[num].x = LaserCenter[num].x + ROI.tl().x;
		LaserCenter[num].y = LaserCenter[num].y + ROI.tl().y;
	}
	cout << "取得最大像素耗时：" << ta1 << endl;
	return 1;
}

bool CAlgorithm::LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange, 
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType)
{
	cv::Rect roi;
	roi.x = 0; roi.y = 0;
	roi.width = src.cols; roi.height = src.rows; //待确认
	LaserCenterDetect(src, Threshold, windowwidth, centerRange, LaserCenter,stripeMeanWidth, stripeStdWidth, searchWinHei, gaussK, MaxMidType, roi);
	return true;
}

bool CAlgorithm::rigidTransformPointCould(const Mat& rigidTransformRT, vector<Point3f> &srcPnts, vector<Point3f> &dstPnts)
{
	vector<Point3f>temp;
	Mat matpnt = Mat::ones(4, 1, CV_64FC1);
	for (int i = 0; i < srcPnts.size(); i++)
	{
		temp.push_back(Point3f());
		matpnt.at<double>(0, 0) = srcPnts[i].x;
		matpnt.at<double>(1, 0) = srcPnts[i].y;
		matpnt.at<double>(2, 0) = srcPnts[i].z;
		Mat pnt = rigidTransformRT*matpnt;
		temp[i].x = pnt.at<double>(0, 0);
		temp[i].y = pnt.at<double>(1, 0);
		temp[i].z = pnt.at<double>(2, 0);
	}
	dstPnts = temp;
	return true;
}

void CAlgorithm::rigidTransformFrom3Pnt(const vector<Point3f>pntsInRef, const vector<Point3f> pntsInLocal, Mat &rt)
{
	if (pntsInLocal.size() != 3 || pntsInRef.size() != 3)
	{
		return;
	}
	//检查是否共线
	Mat rt1, rt2;
	frameFrom3Pnts(pntsInRef, rt1);
	frameFrom3Pnts(pntsInLocal, rt2);
	cv::invert(rt2, rt2);
	rt = rt1*rt2;
	return;
}

void CAlgorithm::frameFrom3Pnts(const vector<Point3f>pnts, Mat &rt)
{
	cv::Vec3d tem = cv::Vec3d(pnts[2].x - pnts[0].x, pnts[2].y - pnts[0].y, pnts[2].z - pnts[0].z);
	tem = normalize(tem);
	cv::Vec3d x = cv::Vec3d(pnts[1].x - pnts[0].x, pnts[1].y - pnts[0].y, pnts[1].z - pnts[0].z);
	//cv::Vec3d x = cv::Vec3d(pnts[0].x - pnts[1].x, pnts[0].y - pnts[1].y, pnts[0].z - pnts[1].z);
	x = cv::normalize(x);
	cv::Vec3d z = cv::normalize(x.cross(tem)); //两个单位阵叉乘一定是单位阵吗？？
	cv::Vec3d y = cv::normalize(z.cross(x));
	rt = Mat::eye(4, 4, CV_64FC1);
	cv::Mat(x).copyTo(rt.col(0).rowRange(0, 3));
	cv::Mat(y).copyTo(rt.col(1).rowRange(0, 3));
	cv::Mat(z).copyTo(rt.col(2).rowRange(0, 3));
	rt.at<double>(0, 3) = pnts[0].x;
	rt.at<double>(1, 3) = pnts[0].y;
	rt.at<double>(2, 3) = pnts[0].z;
	return;
}

Mat CAlgorithm::rotz(double rotAngle, bool radFlag)
{
	Mat _rt = Mat::eye(4, 4, CV_64FC1);
	if (!radFlag)
	{
		rotAngle = rotAngle *PI / 180;
	}
	_rt.at<double>(0,0) = cos(rotAngle);
	_rt.at<double>(0,1) = -sin(rotAngle);
	_rt.at<double>(1,0) = sin(rotAngle);
	_rt.at<double>(1,1) = cos(rotAngle);
	return _rt;
}

//算法备份
#if bak
bool CAlgorithm::CrossLaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange, vector<laserCenter> &LaserCenters, double &stripeMeanWidth, double &stripeStdWidth, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)

{
	//将图像转化成灰度图
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//截图原图像的roi部分作为中心提取的新图像
	src = src(ROI);
	//进行高斯滤波，去除噪声---为了提高效率，只对条纹中心区域进行滤波
	//GaussianBlur(src, src, Size(5, 5), 0, 0);
	//先定义原图像的备份
	Mat srcCpy = src.clone();

	//定义是否是新条纹标识
	bool isNear = 0;

	//求该行图像中亮度最大值所在位置Maxc
	Point data;
	int maxVal;
	Point maxLoc;

	int colRangeHalf = 0;
	//max函数中mask的起始列和终止列
	int colStart = 0;
	int colEnd = 0;

	//threTem用于设定首点和新条纹的阈值
	int threTem;
	//设定条纹中心第一个点的列，以此确定条纹的骨架区域
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//有效条纹中心提取的光条宽度均值和标准差
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//窗口宽度的一半
	int width = (windowwidth - 1) / 2;
	vector <int> oldCenters;	//上一次的重心位置，支持多个重心
	oldCenters.reserve(20);		//预先开辟20个元素空间。
	int stripNum = 2;  //条纹数量

	//由于使用高斯滤波时没有对边界进行插值，所以行的开始值和结束值必须缩减
	for (int i = 2; i < src.rows - 2; i++)
	{
		//Mat src1 = src.row(i);
		uchar *src1 = src.ptr<uchar>(i);
		int cols = src.cols;
		laserCenter  center; //每一次遍历提取的重心坐标
		maxVal = 0;
		bool hadCenter = false;
		if (isNear)
		{
			colRangeHalf = (3 * windowwidth) / 2;
			for (int j = 0; j < oldCenters.size(); j++)
			{
				vector <uchar> _maxva;
				vector <int> _maxPos;
				colStart = oldCenters[j] - colRangeHalf < 0 ? 0 : int(oldCenters[j] - colRangeHalf);
				colEnd = oldCenters[j] + colRangeHalf > cols - 1 ? colEnd = src.cols - 1 : int(oldCenters[j] + colRangeHalf);
				getMultMax(src1, colStart, colEnd, windowwidth, 1, _maxva, _maxPos);
				data.y = i;
				data.x = _maxPos[0];

				//进行越界检查
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2){
					isNear = false;
					continue;
				}

				//确定前景亮度I1、背景亮度I2、条纹宽度w
				double I1 = 0, I2 = 0, w = 0;
				double m1 = 0, m2 = 0, m3 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					m1 += pow(src1[data.x - width + k], 1);
					m2 += pow(src1[data.x - width + k], 2);
					m3 += pow(src1[data.x - width + k], 3);
				}
				m1 = m1 / windowwidth;
				m2 = m2 / windowwidth;
				m3 = m3 / windowwidth;

				double a = 0, b = 0, c = 0;
				a = m2 - m1*m1;
				b = m1*m2 - m3;
				c = m1*m3 - m2*m2;

				I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
				I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
				w = windowwidth*((m1 - I1) / (I2 - I1));

				//I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
				//比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
				//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//对当前行窗口区域中的像素进行高斯滤波
				PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//重心法求亚像素条纹中心，并统计重心计算的像素个数n1
				float PixelValue = 0;
				float sum1 = 0;
				float sum2 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					PixelValue = src.at<uchar>(i, data.x - width + k);
					if (PixelValue>I1 + Threshold)  
					{
						sum1 += (PixelValue - I1)*(data.x - width + k);
						sum2 += PixelValue - I1;
					}
				}
				if (sum1 != 0)
				{
					center.index = i;
					float y = sum1 / sum2;
					center.pt.push_back(y);
					stripeWidth.push_back(w);
					hadCenter = true;
				}
			}
			if (center.pt.size() == stripNum)
			{
				//如果条纹离得比较近，则认为只有一个条纹 
				vector <float>  diss;
				int _num = center.pt.size();
				for (int n = 0; n < _num - 1; n++)
				{
					for (int nn = n + 1; nn < _num; nn++)
					{
						diss.push_back(fabs(center.pt[n] - center.pt[nn]));
					}
				}
				for (int n = 0; n < diss.size(); n++)
				{
					if (diss[n] < 3*windowwidth)
					{
						isNear = false;
						continue;
					}
				}

				isNear = true;
				oldCenters.clear();
				for (int k = 0; k < stripNum; k++)
				{
					oldCenters.push_back(int(center.pt[k]));
				}
			}
		}
		else
		{
			vector <uchar> _maxva;
			vector <int> _maxPos;
			getMultMax(src1, 0, cols - 1,windowwidth, stripNum, _maxva, _maxPos);
			for (int j = 0; j < stripNum; j++)
			{
				data.y = i;
				data.x = _maxPos[j];

				//进行越界检查
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2)
				{
					isNear = false;
					continue;
				}

				//确定前景亮度I1、背景亮度I2、条纹宽度w
				double I1 = 0, I2 = 0, w = 0;
				double m1 = 0, m2 = 0, m3 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					m1 += pow(src1[data.x - width + k], 1);
					m2 += pow(src1[data.x - width + k], 2);
					m3 += pow(src1[data.x - width + k], 3);
				}
				m1 = m1 / windowwidth;
				m2 = m2 / windowwidth;
				m3 = m3 / windowwidth;

				double a = 0, b = 0, c = 0;
				a = m2 - m1*m1;
				b = m1*m2 - m3;
				c = m1*m3 - m2*m2;

				I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
				I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
				w = windowwidth*((m1 - I1) / (I2 - I1));

				//I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
				//比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
				//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//对当前行窗口区域中的像素进行高斯滤波
				PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//重心法求亚像素条纹中心，并统计重心计算的像素个数n1
				float PixelValue = 0;
				float sum1 = 0;
				float sum2 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					PixelValue = src.at<uchar>(i, data.x - width + k);
					if (PixelValue>I1 + Threshold)  //改动
					{
						sum1 += (PixelValue - I1)*(data.x - width + k);
						sum2 += PixelValue - I1;
					}
				}
				if (sum1 != 0)
				{
					center.index = i;
					center.pt.push_back(sum1 / sum2);
					stripeWidth.push_back(w);
					hadCenter = true;
				}
			}
			if (center.pt.size() == stripNum)
			{
				//如果条纹离得比较近，则认为只有一个条纹 
				vector <float>  diss;
				int _num = center.pt.size();
				for (int n = 0; n < _num - 1; n++)
				{
					for (int nn = n + 1; nn < _num; nn++)
					{
						diss.push_back(fabs(center.pt[n] - center.pt[nn]));
					}
				}
				for (int n = 0; n < diss.size(); n++)
				{
					if (diss[n] < 3 * windowwidth)
					{
						isNear = false;
						continue;
					}
				}

				isNear = true;
				oldCenters.clear();
				for (int k = 0; k < stripNum; k++)
				{
					oldCenters.push_back(int(center.pt[k]));
				}
			}
		}
		if (hadCenter)
			LaserCenters.push_back(center);
	}

	//进行条纹点分类
	//vector<Point> pnts;
	//for (int i = 0; i < LaserCenters.size(); i++)
	//{
	//	for (int j = 0; j < LaserCenters[i].pt.size(); j++)
	//	{
	//		pnts.push_back(LaserCenters[i].pt[j]);
	//	}
	//}

	//zzl modify 进行过亮点剔除
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0];
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //只对宽度变化较大的条纹进行粗大剔除
	{
		for (int i = 0; i < stripeWidth.size(); i++)
		{
			if (stripeWidth[i] > meanWidth + 1.5*stdWidth)
			{
				continue;
			}
			temCenters.push_back(LaserCenter[i]);
		}
		LaserCenter = temCenters;
	}
#endif
	//将roi图像条纹中心坐标转换成原图像中的坐标
	for (int num = 0; num < LaserCenters.size(); ++num)
	{
		for (int k = 0; k < LaserCenters[num].pt.size(); k++){
			LaserCenters[num].pt[k] = LaserCenters[num].pt[k] + ROI.tl().x;
		}
		LaserCenters[num].index = LaserCenters[num].index + ROI.tl().y;
	}
	return 1;
}
#endif
bool CAlgorithm::CaculateHMatric(const double *planeParams, const CamPara &camParam, cv::Mat &camFrameInLaserFrame, cv::Mat &Hmat)
{
	//step1 由光刀平面构建激光坐标系（参考坐标系为相机坐标系）
	cv::Vec3d oriPnt;//相机光心在激光平面的投影
	double t = -planeParams[3] / (pow(planeParams[0], 2) + pow(planeParams[1], 2) + pow(planeParams[2], 2));
	oriPnt[0] = t*planeParams[0];
	oriPnt[1] = t*planeParams[1];
	oriPnt[2] = t*planeParams[2];
	cv::Vec3d v_x, v_y, v_z;
	cv::normalize(oriPnt, v_y);
	v_y = -v_y;
	cv:normalize(v_y.cross(cv::Vec3d(0, 0, 1)), v_x);
	v_z = v_x.cross(v_y);
	Mat rt = Mat::eye(4, 4, CV_64FC1);
	rt.at<double>(0, 0) = v_x[0]; rt.at<double>(1, 0) = v_x[1]; rt.at<double>(2, 0) = v_x[2];
	rt.at<double>(0, 1) = v_y[0]; rt.at<double>(1, 1) = v_y[1]; rt.at<double>(2, 1) = v_y[2];
	rt.at<double>(0, 2) = v_z[0]; rt.at<double>(1, 2) = v_z[1]; rt.at<double>(2, 2) = v_z[2];
	rt.at<double>(0, 3) = oriPnt[0]; rt.at<double>(1, 3) = oriPnt[1]; rt.at<double>(2, 3) = oriPnt[2];
	cv::invert(rt, camFrameInLaserFrame);
	vector<Mat> _temp;
	_temp.push_back(camFrameInLaserFrame);
	//step2 构建像素坐标和光刀平面上点的物理坐标系的单应矩阵（参考坐标系：光刀坐标系）
	Mat intrinsicPara;
	Mat distCoeffs;
	cvtool::cvtCamPara(camParam, intrinsicPara, distCoeffs);
	Mat Htemp;
	Mat temp = Mat::zeros(3, 4, CV_64FC1);
	intrinsicPara.copyTo(temp.rowRange(0, 3).colRange(0, 3));
	Htemp = temp*rt;
	Hmat = Mat::zeros(3, 3, CV_64FC1);
	vector<Mat> H;
	Htemp.col(0).copyTo(Hmat.col(0));
	Htemp.col(2).copyTo(Hmat.col(1));
	Htemp.col(3).copyTo(Hmat.col(2));
	//test 
	//H.push_back(Hmat);
	//XMLWriter::WriteRT33(filePath + "/test.xml", H);
	//H.clear();
	//vector <Mat> tep1;
	//XMLReader::ReadRT33(filePath + "/test.xml", tep1);
	//cout << "求逆之前，保存之后再读取 = " << tep1[0] << endl;
	//Mat temdf; invert(tep1[0], temdf);
	//cout << "求逆之前，保存之后再读取,再求逆 = " << temdf << endl;
	//cout << "求逆之前，保存之前 = " << Hmat << endl;


	//cv::invert(Hmat, Hmat);//求逆以后，数值过小，保存后再读出数据有偏差
	//step3 以xml格式文件保存单应矩阵Hmat
	cv::invert(Hmat, Hmat);
	return true;
}

//bool CAlgorithm::getWTT(uchar *src, cv::Point pos, int windowwidth, double &I1, double &I2, double &W)
//{
//	double I1 = 0, I2 = 0, w = 0;
//	double m1 = 0, m2 = 0, m3 = 0;
//	int width = (windowwidth - 1) / 2;
//	uchar *src1 = src;
//	for (int j = 0; j < windowwidth; j++)
//	{
//		m1 += pow(src1[pos.x - width + j], 1);
//		m2 += pow(src1[pos.x - width + j], 2);
//		m3 += pow(src1[pos.x - width + j], 3);
//	}
//	m1 = m1 / windowwidth;
//	m2 = m2 / windowwidth;
//	m3 = m3 / windowwidth;
//
//	double a = 0, b = 0, c = 0;
//	a = m2 - m1*m1;
//	b = m1*m2 - m3;
//	c = m1*m3 - m2*m2;
//
//	I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
//	I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
//	w = windowwidth*((m1 - I1) / (I2 - I1));
//}

//旧版光条中心提取算法
#if 0
bool CSingleCamLaserSys::LaserCenterDetector(const Mat& srcimage, const cv::Rect mask, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter)
{
	//将图像转化成灰度图
	Mat src;
	if (srcimage.channels() == 1)
	{
		src = srcimage;
	}
	else
	{
		cv::cvtColor(srcimage, src, CV_BGR2GRAY);
	}
	src = Mat(src, mask);
	GaussianBlur(src, src, Size(5, 5), 0, 0);     //高斯滤波
	//Step 1：求该行图像中亮度最大值所在位置Maxc
	Point2f data;
	vector<Point2f> Maxc;
	Point2f center;
	for (int i = 0; i < src.rows; i++)
	{
		Mat src1 = src.row(i).clone();
		//cout << "src1 = " << endl << " " << src1 << endl << endl;
		double maxVal = 0;
		Point maxLoc;
		cv::minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, Mat()); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
		//data.x=i;
		//data.y=maxLoc.x;
		//Maxc.push_back(data);
		int GrayValue = src.at<uchar>(i, maxLoc.x);
		int k1 = 1;
		int k11 = 0;
		for (k1; k1 < src1.cols - maxLoc.x; k1++)
		{
			int GrayValue1 = src.at<uchar>(i, maxLoc.x + k1);
			if (GrayValue1 == GrayValue)
			{
				k11 = k1;
			}
		}
		int k2 = 1;
		int k22 = 0;
		for (k2; k2 < maxLoc.x; k2++)
		{
			int GrayValue2 = src.at<uchar>(i, maxLoc.x - k2);
			if (GrayValue2 == GrayValue)
			{
				k22 = k2;
			}
		}
		data.x = i;
		data.y = (maxLoc.x + k11 + maxLoc.x - k22) / 2;

		////ZLL 越界检查本应该检查的是x，但是这里周涛把data的x，y写反了。应该是x行，y列
		if (data.y<windowwidth / 2 || data.y> src1.cols - 1 - windowwidth / 2)
		{
			continue;
		}
		////// zhangxu added  end
		//Step 2：确定前景亮度I1、背景亮度I2、条纹宽度w
		//Step 2.1：
		double I1 = 0, I2 = 0, w = 0;
		int width = (windowwidth - 1) / 2;
		double m1 = 0, m2 = 0, m3 = 0;
		for (int j = 0; j < windowwidth; j++)
		{
			m1 += pow(src.at<uchar>(data.x, data.y - width + j), 1);
			m2 += pow(src.at<uchar>(data.x, data.y - width + j), 2);
			m3 += pow(src.at<uchar>(data.x, data.y - width + j), 3);
		}
		m1 = m1 / windowwidth;
		m2 = m2 / windowwidth;
		m3 = m3 / windowwidth;
		//Step 2.2：
		double a = 0, b = 0, c = 0;
		a = m2 - m1*m1;
		b = m1*m2 - m3;
		c = m1*m3 - m2*m2;

		I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
		I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
		w = windowwidth*((m1 - I1) / (I2 - I1));
		//// test
		//cout << i << endl;
		////////////// end test
		if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold) //Step 3： I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
		{
			Maxc.push_back(data);
		}
		else
		{
			continue;
		}
		//Step 5:  比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
		int n1 = windowwidth;
		if (n1 < w)
		{
			continue;
		}
		//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
		float PixelValue = 0;
		float sum1 = 0;
		float sum2 = 0;
		for (int j = 0; j < n1; j++)
		{
			PixelValue = src.at<uchar>(data.x, data.y - ((n1 - 1) / 2) + j);
			if (PixelValue>I1 + Threshold)  //改动
			{
				sum1 += PixelValue*(data.y - ((n1 - 1) / 2) + j);
				sum2 += PixelValue/* - I1*/;
			}
		}
		if (sum1 != 0)
		{
			///center.y = (int)sum1 / sum2;
			///center.x = data.x;
			////// modified by zhangxu
			center.y = data.x + mask.y;
			center.x = sum1 / sum2 + mask.x;
			///// end
			LaserCenter.push_back(center);
		}
	}
	return true;
}

bool CSingleCamLaserSys::LaserCenterDetector(const Mat& srcimage, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter)
{
	//将图像转化成灰度图
	Mat src;
	if (srcimage.channels() == 1)
	{
		src = srcimage;
	}
	else
	{
		cv::cvtColor(srcimage, src, CV_BGR2GRAY);
	}
	GaussianBlur(src, src, Size(5, 5), 0, 0);     //高斯滤波
	//Step 1：求该行图像中亮度最大值所在位置Maxc
	Point2f data;
	vector<Point2f> Maxc;
	Point2f center;
	for (int i = 0; i < src.rows; i++)
	{
		Mat src1 = src.row(i).clone();
		//cout << "src1 = " << endl << " " << src1 << endl << endl;
		double maxVal = 0;
		Point maxLoc;
		minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, Mat()); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
		//data.x=i;
		//data.y=maxLoc.x;
		//Maxc.push_back(data);
		int GrayValue = src.at<uchar>(i, maxLoc.x);
		int k1 = 1;
		int k11 = 0;
		for (k1; k1 < src1.cols - maxLoc.x; k1++)
		{
			int GrayValue1 = src.at<uchar>(i, maxLoc.x + k1);
			if (GrayValue1 == GrayValue)
			{
				k11 = k1;
			}
		}
		int k2 = 1;
		int k22 = 0;
		for (k2; k2 < maxLoc.x; k2++)
		{
			int GrayValue2 = src.at<uchar>(i, maxLoc.x - k2);
			if (GrayValue2 == GrayValue)
			{
				k22 = k2;
			}
		}
		data.x = i;
		data.y = (maxLoc.x + k11 + maxLoc.x - k22) / 2;

		////ZLL 越界检查本应该检查的是x，但是这里周涛把data的x，y写反了。应该是x行，y列
		if (data.y<windowwidth / 2 || data.y> src1.cols - 1 - windowwidth / 2)
		{
			continue;
		}
		////// zhangxu added  end
		//Step 2：确定前景亮度I1、背景亮度I2、条纹宽度w
		//Step 2.1：
		double I1 = 0, I2 = 0, w = 0;
		int width = (windowwidth - 1) / 2;
		double m1 = 0, m2 = 0, m3 = 0;
		for (int j = 0; j < windowwidth; j++)
		{
			m1 += pow(src.at<uchar>(data.x, data.y - width + j), 1);
			m2 += pow(src.at<uchar>(data.x, data.y - width + j), 2);
			m3 += pow(src.at<uchar>(data.x, data.y - width + j), 3);
		}
		m1 = m1 / windowwidth;
		m2 = m2 / windowwidth;
		m3 = m3 / windowwidth;
		//Step 2.2：
		double a = 0, b = 0, c = 0;
		a = m2 - m1*m1;
		b = m1*m2 - m3;
		c = m1*m3 - m2*m2;

		I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
		I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
		w = windowwidth*((m1 - I1) / (I2 - I1));
		//// test
		//cout << i << endl;
		////////////// end test
		if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold) //Step 3： I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
		{
			Maxc.push_back(data);
		}
		else
		{
			continue;
		}
		//Step 5:  比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
		int n1 = windowwidth;
		if (n1 < w)
		{
			continue;
		}
		//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
		float PixelValue = 0;
		float sum1 = 0;
		float sum2 = 0;
		for (int j = 0; j < n1; j++)
		{
			PixelValue = src.at<uchar>(data.x, data.y - ((n1 - 1) / 2) + j);
			if (PixelValue>I1 + Threshold)  //改动
			{
				sum1 += PixelValue*(data.y - ((n1 - 1) / 2) + j);
				sum2 += PixelValue/* - I1*/;
			}
		}
		if (sum1 != 0)
		{
			///center.y = (int)sum1 / sum2;
			///center.x = data.x;
			////// modified by zhangxu
			center.y = data.x;
			center.x = sum1 / sum2;
			///// end
			LaserCenter.push_back(center);
		}
	}
	return true;
}
#endif



