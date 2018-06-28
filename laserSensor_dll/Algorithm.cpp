#include "Algorithm.h"
#include ".\splineFit_src\interpolation.h"
using namespace alglib;

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
	if (!solve(coefficient_matrix, equation_right, variable_matrix, cv::DECOMP_CHOLESKY))//��˹��Ԫ��?�˴�ϵ������Ϊ�Գƾ���
	{
		return false;
	}
	//����������λ��
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
	//ͳ�Ƶ㵽ƽ��ľ���
	vector<double> dis, meanDis,stdDis;
	double nor = sqrt(PlaneParam[0] * PlaneParam[0] + PlaneParam[1] * PlaneParam[1] + PlaneParam[2] * PlaneParam[2]);
	for (int i = 0; i < srcPnts.size(); i++)
	{
		double temp = (PlaneParam[0] * srcPnts[i].x + PlaneParam[1] * srcPnts[i].y + PlaneParam[2] * srcPnts[i].z + PlaneParam[3]) / nor;
		 dis.push_back(temp);
	}
	cv::meanStdDev(dis, meanDis, stdDis);
	//ƽ����Ͻ����ʾ

	cout << "------------------ƽ����Ͻ��-------------------" << endl;
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
	//ͶӰ
	if (srcPnts.size()== 0)
	{
		return -1;
	}
	cv::projectPoints(srcPnts, r, t, intrinsicPara, distCoeffs, dstPnt);
	//���㷴��ͶӰ���
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
//�����˹�� 
//Krow:��˹�˵���, Kcol:��˹�˵���, Krow & Kcol ����Ϊ����, Ksig:��˹�˵ı�׼���������������ȡ��ʹ��5x5����׼��=1
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
//�������ֵ��MaxMidType=1:��λ�м䣻MaxMidType=0����λ��һ�����ֵ
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
				//ԭ���Ĵ�������
				//colMaxMid = (colMax + col) / 2;
				//zzl modify �ж��м��������Ƿ�Ҳ�ǽ����㣬���������������ȡԭ��������
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
				//�����Ѿ���ȡ�����ֵ�㣬�ҵ�������������ֵ�㣬����������м�㴦��ʼ�������һ��windowwidth��ȣ�������ֱ������ȣ�����Ϊ�����ֵ��
				//�������ϵ����ֵ�����û�У���Ϊͬһ�����ϵ����ֵ��
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
//�������ֵ��MaxMidType=1:��λ�м䣻MaxMidType=0����λ��һ�����ֵ
{
	//ֻ�������б������������б���ʱ�������ڴ治������������
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
				//ԭ���Ĵ�������
				//colMaxMid = (colMax + col) / 2;
				//zzl modify �ж��м��������Ƿ�Ҳ�ǽ����㣬���������������ȡԭ��������
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
	//add by zzl �޸���������
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
				//ԭ���Ĵ�������
				//colMaxMid = (colMax + col) / 2;
				//zzl modify �ж��м��������Ƿ�Ҳ�ǽ����㣬���������������ȡԭ��������
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
//�������ֵ��MaxMidType=1:��λ�м䣻MaxMidType=0����λ��һ�����ֵ
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
				//ԭ���Ĵ�������
				//colMaxMid = (colMax + col) / 2;
				//zzl modify �ж��м��������Ƿ�Ҳ�ǽ����㣬���������������ȡԭ��������
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

bool CAlgorithm::estimatePNP(vector<Point2f>imgPnts, const Mat &intrinsic, const Mat& distCoeffs, const Mat &r, const Mat &t,
	const Point2f &disXY, const int &ringRow, const int &ringCol,  double &errMean, double &errStd)
{
	vector<Point3f> Pnt3D;
	vector <double> dis;
	CoreAlgorithm::project2CalibrationBoard(imgPnts, Pnt3D, intrinsic, distCoeffs, r, t);
	//����1���������ֵ��ʵ��ֵ�Ĳ�� ���Աȷ������ֱ����㣨i,j���͵㣨i+1,j��,�㣨i,j���͵㣨i,j+1���ľ��룬���׼�����ʵ�ʾ���֮�
#if 0
	for (int i = 0; i < Pnt3D.size(); i++)
	{
		if ((i + 1) % ringCol != 0)
		{
			double _dis = sqrt(pow(Pnt3D[i].x - Pnt3D[i + 1].x, 2) + pow(Pnt3D[i].y - Pnt3D[i + 1].y, 2) + pow(Pnt3D[i].z - Pnt3D[i + 1].z, 2));
			dis.push_back(_dis);
		}
	}
	vector <double> rdis(dis.size(), disXY.x);
	cv::subtract(dis, rdis, dis);
	vector <double> me, st;
	cv::meanStdDev(dis, me, st);
	errMean = me[0]; errStd = st[0];
#endif
	//����2��ȡÿһ�У��У���������һ�����һ�������룬���Խ����ϵĵ�������
	//����β����
	int num = Pnt3D.size();
	for (int i = 0; i < ringCol; i++)
	{
		double _dis = sqrt(pow(Pnt3D[i].x - Pnt3D[num - ringCol + i].x, 2) + pow(Pnt3D[i].y - Pnt3D[num - ringCol + i].y, 2) + pow(Pnt3D[i].z - Pnt3D[num - ringCol + i].z, 2));
		_dis -= disXY.y*(ringRow - 1);
		dis.push_back(_dis);
	}
	//����β����
	for (int i = 0; i < (ringRow - 1)*ringCol; i += ringCol)
	{
		double _dis = sqrt(pow(Pnt3D[i].x - Pnt3D[i + ringCol - 1].x, 2) + pow(Pnt3D[i].y - Pnt3D[i + ringCol - 1].y, 2) + pow(Pnt3D[i].z - Pnt3D[i + ringCol - 1].z, 2));
		_dis -= disXY.x*(ringCol - 1);
		dis.push_back(_dis);
	}
	//��һ������һ��
	dis.push_back(sqrt(pow(Pnt3D[0].x - Pnt3D[num - 1].x, 2) + pow(Pnt3D[0].y - Pnt3D[num - 1].y, 2) + pow(Pnt3D[0].z - Pnt3D[num - 1].z, 2)) - 
		sqrt(pow((ringCol - 1)*disXY.x, 2) + pow((ringRow - 1)*disXY.y, 2)));
	vector <double> me, st;
	cv::meanStdDev(dis, me, st);
	errMean = me[0]; errStd = st[0];
	cout << "----------------��ͶӰ���-----------------" << endl;
	cout << "����ֵ�� " << errMean << "��׼� " << errStd << endl;
	
	for (int i = 0; i < dis.size(); i++)
	{
		cout << dis[i] << " ";
	}
	cout << endl;
	cout << "------------------------------------------" << endl;
	return true;
}

bool CAlgorithm::estimateStereoVision(const vector<Point2f> &LimgPnts, const vector<Point2f> &RimgPnts, const CamPara &LcamParam, const CamPara &RcamParam,
	const RT &relativeRT, const Point2f &disXY, const int &ringRow, const int &ringCol, double &errMean, double &errStd)
{
	if (LimgPnts.size() != RimgPnts.size())
	{
		return false;
	}
	double *r, *t;
	if (relativeRT.R.isContinuous() && relativeRT.T.isContinuous())
	{
		r = (double *)relativeRT.R.data;
		t = (double *)relativeRT.T.data;
	}
	else
	{
		return false;
	}
	//double r[3] = { relativeRT.R.at<double>(0, 0), relativeRT.R.at<double>(0, 1), relativeRT.R.at<double>(0, 2) };
	//double t[3] = { relativeRT.T.at<double>(0, 0), relativeRT.T.at<double>(0, 1), relativeRT.T.at<double>(0, 2) };
	vector<Point3f> pnts;
	vector<double> dis;
	CoreAlgorithm::Cal3dPoint(LimgPnts, LcamParam, RimgPnts, RcamParam, r, t, pnts);
	for (int i = 0; i < pnts.size(); i++)
	{
		if ((i + 1) % ringCol != 0)
		{
			double _dis = sqrt(pow(pnts[i].x - pnts[i + 1].x, 2) + pow(pnts[i].y - pnts[i + 1].y, 2) + pow(pnts[i].z - pnts[i + 1].z, 2));
			dis.push_back(_dis);
		}
	}
	vector <double> rdis(dis.size(), disXY.x);
	cv::subtract(dis, rdis, dis);
	vector <double> me, st;
	cv::meanStdDev(dis, me, st);
	errMean = me[0]; errStd = st[0];
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
	//ƽ����Ͻ����ʾ
#ifdef DEBUG
	cout << "------------------ƽ����Ͻ��-------------------" << endl;
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
	//***���ƽ��
	double planeParams[4];
	FitPlane(srcPnts, planeParams);
	Mat planeParams_ori = Mat(4, 1, CV_64FC1 ,planeParams);
	//***����������
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
	invert(temp, temp, cv::DECOMP_SVD); //��α��
	Mat ballParams = temp*A.t()*B;
	//***��ƽ���ϵ����ĵ�
	Mat planeParams_zero = (Mat_<double>(1, 4) << planeParams[0], planeParams[1], planeParams[2], 0);
	planeParams_zero = planeParams_zero / 2;
	temp = -(planeParams_zero * ballParams + planeParams[3]) / (planeParams_zero*planeParams_ori);
	double lamda = temp.at<double>(0,0);
	Mat center_temp = ballParams + lamda*planeParams_ori;
	//***������
	circle3dParam.center = Point3d(Mat(center_temp.rowRange(0, 3)*0.5));
	circle3dParam.normal = cv::Vec3d(planeParams_ori.rowRange(0, 3));
	circle3dParam.normal = normalize(circle3dParam.normal);
	circle3dParam.radius = sqrt(center_temp.at<double>(3, 0) + pow(norm(circle3dParam.center), 2));
	cout << "-------------�ռ�Բ��Ͻ��-------------" << endl;
	cout << "circle center: " << circle3dParam.center << endl;
	cout << "circle normal: " << circle3dParam.normal << endl;
	cout << "circle raius: " << circle3dParam.radius << endl;
	cout << "--------------------------------------" << endl;
	return true;
}

bool CAlgorithm::binocularCalibrate(CamPara &LcamParams, CamPara &RcamParams, CalibrationData &LcamData, CalibrationData &RcamData, RT &relativeRT)
{
	//��������ת��
	Mat intrinsicParaL, intrinsicParaR, distCoeffsL, distCoeffsR;
	cvtool::cvtCamPara(LcamParams, intrinsicParaL, distCoeffsL);
	cvtool::cvtCamPara(RcamParams, intrinsicParaR, distCoeffsR);

	//����˫Ŀ�궨
	Mat R, T, E, F;//�궨�õ��Ĳ���R,TΪ��������λ��
	double rms = stereoCalibrate(LcamData.plane3dPntsVec, LcamData.plane2dPntsVec, RcamData.plane2dPntsVec,
		intrinsicParaL, distCoeffsL, intrinsicParaR, distCoeffsR,
		Size(LcamData.imgWidth, RcamData.imgHeight), R, T, E, F,
		CV_CALIB_FIX_INTRINSIC +
		/*CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_FIX_FOCAL_LENGTH +
		CV_CALIB_FIX_PRINCIPAL_POINT +
		CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH*/
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));//TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6)

	//����˫Ŀ�궨���
	cv::Rodrigues(R, relativeRT.R);
	relativeRT.T = T;
	return true;
}

//ԭʼ����������ȡ�㷨����
//bool CAlgorithm::LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
//	vector<Point2f> &LaserCenter, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)
//{
//	//��ͼ��ת���ɻҶ�ͼ
//	if (src.channels() != 1)
//	{
//		cout << "Laser image channel erro." << endl;
//		return false;
//	}
//
//	//��ͼԭͼ���roi������Ϊ������ȡ����ͼ��
//	src = src(ROI);
//	//���и�˹�˲���ȥ������---Ϊ�����Ч�ʣ�ֻ������������������˲�
//	//GaussianBlur(src, src, Size(5, 5), 0, 0);
//	//�ȶ���ԭͼ��ı���
//	Mat srcCpy = src.clone();
//
//	//�����Ƿ��������Ʊ�ʶ
//	bool isNear = 0;
//	//�Ƿ��������ϵ�һ�����־
//	bool isFirst = 1;
//
//	//�����ͼ�����������ֵ����λ��Maxc
//	Point data;
//	Point2f center;
//	int maxVal;
//	Point maxLoc;
//
//	int trapeHeiTem = 0;
//	int colRangeHalf = 0;
//	//max������mask����ʼ�к���ֹ��
//	int colStart = 0;
//	int colEnd = 0;
//	int colMax;
//
//	//threTem�����趨�׵�������Ƶ���ֵ
//	int threTem;
//	//�趨�������ĵ�һ������У��Դ�ȷ�����ƵĹǼ�����
//	int firstCenterCol;
//	int rangeColStart, rangeColEnd;
//
//	//���ڿ�ȵ�һ��
//	int width = (windowwidth - 1) / 2;
//
//	//����ʹ�ø�˹�˲�ʱû�жԱ߽���в�ֵ�������еĿ�ʼֵ�ͽ���ֵ��������
//	for (int i = 2; i < src.rows - 2; i++)
//	{
//		Mat src1 = src.row(i);
//		maxVal = 0;
//		if (isNear)
//		{
//			int centerOldCol = LaserCenter.size() - 1;
//			colRangeHalf = (5 * windowwidth + 4 * trapeHeiTem - 1) / 2;
//			if (LaserCenter[centerOldCol].x - colRangeHalf<0)
//			{
//				colStart = 0;
//			}
//			else
//			{
//				colStart = int(LaserCenter[centerOldCol].x - colRangeHalf);
//			}
//			if (LaserCenter[centerOldCol].x + colRangeHalf>src.cols - 1)
//			{
//				colEnd = src.cols - 1;
//			}
//			else
//			{
//				colEnd = int(LaserCenter[centerOldCol].x + colRangeHalf);
//			}
//			//mask
//			//Mat colMask(1, src.cols, CV_8UC1, cv::Scalar(0));
//			//for (int colTem = colStart; colTem < colEnd; ++colTem)
//			//{
//			//	colMask.at<uchar>(0, colTem) = 1;
//			//}
//			//��minMaxLoc����ֻ�ܹ��ҵ����ֵ��߽�ĵ㼰��λ�ã���������������ұ߽�ķ���������
//			//getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
//			getRowMaxLocMid(src1, colStart, colEnd, maxVal, colMax, 1);
//			//minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, colMask); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
//			data.y = i;
//			data.x = colMax;
//
//			//����Խ����
//			if (data.x<windowwidth / 2 + 2 || data.x> src1.cols - 1 - windowwidth / 2 - 2)
//			{
//				if (trapeHeiTem >= searchWinHei)
//				{
//					trapeHeiTem = 0;
//					isNear = 0;
//				}
//				else
//				{
//					++trapeHeiTem;
//				}
//				continue;
//			}
//			//ȷ��ǰ������I1����������I2�����ƿ��w
//			double I1 = 0, I2 = 0, w = 0;
//			double m1 = 0, m2 = 0, m3 = 0;
//			for (int j = 0; j < windowwidth; j++)
//			{
//				m1 += pow(src.at<uchar>(data.y, data.x - width + j), 1);
//				m2 += pow(src.at<uchar>(data.y, data.x - width + j), 2);
//				m3 += pow(src.at<uchar>(data.y, data.x - width + j), 3);
//			}
//			m1 = m1 / windowwidth;
//			m2 = m2 / windowwidth;
//			m3 = m3 / windowwidth;
//
//			double a = 0, b = 0, c = 0;
//			a = m2 - m1*m1;
//			b = m1*m2 - m3;
//			c = m1*m3 - m2*m2;
//
//			I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
//			I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
//			w = windowwidth*((m1 - I1) / (I2 - I1));
//
//			//Step 3�� I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
//			//if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold)
//			if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold)
//			{
//				if (trapeHeiTem >= searchWinHei)
//				{
//					trapeHeiTem = 0;
//					isNear = 0;
//				}
//				else
//				{
//					++trapeHeiTem;
//				}
//				continue;
//			}
//			//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
//			//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
//			if (windowwidth < w)
//			{
//				if (trapeHeiTem >= searchWinHei)
//				{
//					trapeHeiTem = 0;
//					isNear = 0;
//				}
//				else
//				{
//					++trapeHeiTem;
//				}
//				continue;
//			}
//			//////////////////////////////////
//			////////////////zwy///////////////
//#ifdef DEBUGzwy
//			if (w > 6)
//			{
//				//PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);
//				PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);
//
//				int colLeft, colLeftTmp = 0;
//				int length = 0, lengthmax, FirstTimeFlag = 0, LeftFlag = 0;
//				uchar* src1Ptr = src1.ptr<uchar>(0);
//				for (int i = 0; i < src1.cols; i++)
//				{
//					if (src1Ptr[i] >= 240)
//					{
//						length++;
//						if (LeftFlag == 0)
//						{
//							colLeftTmp = i;
//							LeftFlag = 1;
//						}
//					}
//					else
//					{
//						if (FirstTimeFlag == 0)
//						{
//							lengthmax = length;
//							colLeft = colLeftTmp;
//							length = 0;
//							FirstTimeFlag = 1;
//						}
//						else
//						{
//							if (lengthmax < length)
//							{
//								lengthmax = length;
//								colLeft = colLeftTmp;
//								length = 0;
//							}
//						}
//						LeftFlag = 0;
//					}
//				}
//				if (lengthmax < length)
//					lengthmax = length;
//				center.x = colLeft + lengthmax / 2.0;
//				center.y = data.y;
//				LaserCenter.push_back(center);
//				continue;
//			}
//#endif
//			//////////////////////////////////
//			////////////////zwy///////////////
//
//			//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
//			PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);
//			//PartMeanBlur(src, i, data.x - width, data.x + width);
//
//			//Step 4�����ķ����������������ģ���ͳ�����ļ�������ظ���n1
//			float PixelValue = 0;
//			float sum1 = 0;
//			float sum2 = 0;
//			for (int j = 0; j < windowwidth; j++)
//			{
//				PixelValue = src.at<uchar>(data.y, data.x - width + j);
//				if (PixelValue>I1 + Threshold)  //�Ķ�
//				{
//					sum1 += (PixelValue - I1)*(data.x - width + j);
//					sum2 += PixelValue - I1;
//				}
//			}
//			if (sum1 != 0)
//			{
//				center.y = data.y;
//				center.x = sum1 / sum2;
//				LaserCenter.push_back(center);
//				trapeHeiTem = 0;
//			}
//
//		}
//		else
//		{
//
//			//����CenterRange����������ֻ����ʼ���е�centerRange��Χ�ڲ���
//			if (isFirst)
//			{
//				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
//				data.y = i;
//				data.x = maxLoc.x;
//				//������ʼ��ǰ���ͱ�����ֵʹ�õĴ�һ�㣬�Ա�֤�������ƵĹǼ��Ǿ�����ȷ��
//				threTem = 30;
//			}
//			else
//			{
//				//getRowMaxLoc(src1, rangeColStart, rangeColEnd, maxVal, colMax);
//				getRowMaxLocMid(src1, rangeColStart, rangeColEnd, maxVal, colMax, 1);
//
//				data.y = i;
//				data.x = colMax;
//				threTem = Threshold;
//			}
//
//			//����Խ����
//			if (data.x<windowwidth / 2 + 2 || data.x> src1.cols - 1 - windowwidth / 2 - 2)
//			{
//				continue;
//			}
//			//ȷ��ǰ������I1����������I2�����ƿ��w
//			double I1 = 0, I2 = 0, w = 0;
//
//			double m1 = 0, m2 = 0, m3 = 0;
//			for (int j = 0; j < windowwidth; j++)
//			{
//				m1 += pow(src.at<uchar>(data.y, data.x - width + j), 1);
//				m2 += pow(src.at<uchar>(data.y, data.x - width + j), 2);
//				m3 += pow(src.at<uchar>(data.y, data.x - width + j), 3);
//			}
//			m1 = m1 / windowwidth;
//			m2 = m2 / windowwidth;
//			m3 = m3 / windowwidth;
//
//			double a = 0, b = 0, c = 0;
//			a = m2 - m1*m1;
//			b = m1*m2 - m3;
//			c = m1*m3 - m2*m2;
//
//			I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
//			I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
//			w = windowwidth*((m1 - I1) / (I2 - I1));
//
//			if (a <0 || b > 0 || c < 0 || I2 - I1 <= threTem)
//			{
//				continue;
//			}
//
//
//			//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
//			//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
//			if (windowwidth < w)
//			{
//				continue;
//			}
//
//			//////////////////////////////////
//			////////////////zwy///////////////
//#ifdef DEBUGzwy
//			if (w > 6)
//			{
//				PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);
//				//PartGaussianBlur(src, srcCpy, i, 3, 1275, gaussK);
//
//				int colLeft, colLeftTmp = 0;
//				int length = 0, lengthmax, FirstTimeFlag = 0, LeftFlag = 0;
//				uchar* src1Ptr = src1.ptr<uchar>(0);
//				for (int i = 0; i < src1.cols; i++)
//				{
//					if (src1Ptr[i] >= 240)
//					{
//						length++;
//						if (LeftFlag == 0)
//						{
//							colLeftTmp = i;
//							LeftFlag = 1;
//						}
//					}
//					else
//					{
//						if (FirstTimeFlag == 0)
//						{
//							lengthmax = length;
//							colLeft = colLeftTmp;
//							length = 0;
//							FirstTimeFlag = 1;
//						}
//						else
//						{
//							if (lengthmax < length)
//							{
//								lengthmax = length;
//								colLeft = colLeftTmp;
//								length = 0;
//							}
//						}
//						LeftFlag = 0;
//					}
//				}
//				if (lengthmax < length)
//					lengthmax = length;
//				center.x = colLeft + lengthmax / 2.0;
//				center.y = data.y;
//				LaserCenter.push_back(center);
//				if (LaserCenter.size() == 1)
//				{
//					firstCenterCol = LaserCenter[0].x;
//					isFirst = 0;
//
//					if (firstCenterCol - centerRange < 0)
//					{
//						rangeColStart = 0;
//					}
//					else
//					{
//						rangeColStart = firstCenterCol - centerRange;
//					}
//					if (firstCenterCol + centerRange > src1.cols - 1)
//					{
//						rangeColEnd = src1.cols - 1;
//					}
//					else
//					{
//						rangeColEnd = firstCenterCol + centerRange;
//					}
//				}
//				isNear = 1;
//				continue;
//			}
//#endif
//			//////////////////////////////////
//			////////////////zwy///////////////
//
//			//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
//			PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);
//			//PartMeanBlur(src, i, data.x - width, data.x + width);
//
//			//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
//			float PixelValue = 0;
//			float sum1 = 0;
//			float sum2 = 0;
//			for (int j = 0; j < windowwidth; j++)
//			{
//				PixelValue = src.at<uchar>(data.y, data.x - ((windowwidth - 1) / 2) + j);
//				if (PixelValue>I1 + Threshold)
//				{
//					sum1 += (PixelValue - I1)*(data.x - width + j);
//					sum2 += PixelValue - I1;
//				}
//			}
//			if (sum1 != 0)
//			{
//				center.y = data.y;
//				center.x = sum1 / sum2;
//				LaserCenter.push_back(center);
//				if (LaserCenter.size() == 1)
//				{
//					firstCenterCol = LaserCenter[0].x;
//					isFirst = 0;
//
//					if (firstCenterCol - centerRange < 0)
//					{
//						rangeColStart = 0;
//					}
//					else
//					{
//						rangeColStart = firstCenterCol - centerRange;
//					}
//					if (firstCenterCol + centerRange > src1.cols - 1)
//					{
//						rangeColEnd = src1.cols - 1;
//					}
//					else
//					{
//						rangeColEnd = firstCenterCol + centerRange;
//					}
//				}
//				isNear = 1;
//
//			}
//		}
//
//	}
//	//��roiͼ��������������ת����ԭͼ���е�����
//	for (int num = 0; num < LaserCenter.size(); ++num)
//	{
//		LaserCenter[num].x = LaserCenter[num].x + ROI.tl().x;
//		LaserCenter[num].y = LaserCenter[num].y + ROI.tl().y;
//	}
//
//	return 1;
//}
bool CAlgorithm::LaserCenterDetect_row(Mat& src, const int Threshold, const int windowwidth, int centerRange,
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)
{
	//��ͼ��ת���ɻҶ�ͼ
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//��ͼԭͼ���roi������Ϊ������ȡ����ͼ��
	src = src(ROI);
	//���и�˹�˲���ȥ������---Ϊ�����Ч�ʣ�ֻ������������������˲�
	//GaussianBlur(src, src, Size(5, 5), 0, 0);
	//�ȶ���ԭͼ��ı���
	Mat srcCpy = src.clone();

	//�����Ƿ��������Ʊ�ʶ
	bool isNear = 0;
	//�Ƿ��������ϵ�һ�����־
	bool isFirst = 1;

	//�����ͼ�����������ֵ����λ��Maxc
	Point data;
	Point2f center;
	int maxVal;
	Point maxLoc;

	int trapeHeiTem = 0;
	int colRangeHalf = 0;
	//max������mask����ʼ�к���ֹ��
	int colStart = 0;
	int colEnd = 0;
	int colMax;

	//threTem�����趨�׵�������Ƶ���ֵ
	int threTem;
	//�趨�������ĵ�һ������У��Դ�ȷ�����ƵĹǼ�����
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//��Ч����������ȡ�Ĺ�����Ⱦ�ֵ�ͱ�׼��
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//���ڿ�ȵ�һ��
	int width = (windowwidth - 1) / 2;

	//����ʹ�ø�˹�˲�ʱû�жԱ߽���в�ֵ�������еĿ�ʼֵ�ͽ���ֵ��������
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
			//��minMaxLoc����ֻ�ܹ��ҵ����ֵ��߽�ĵ㼰��λ�ã���������������ұ߽�ķ���������
			//getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
			getRowMaxLocMid(src1, colStart, colEnd, maxVal, colMax, 1);
			//minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, colMask); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
			data.y = i;
			data.x = colMax;

			//����Խ����
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
			//ȷ��ǰ������I1����������I2�����ƿ��w
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

			//Step 3�� I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
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
			//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
			//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
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

			//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
			PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//Step 4�����ķ����������������ģ���ͳ�����ļ�������ظ���n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>(data.y, data.x - width + j);
				if (PixelValue>I1 + Threshold)  //�Ķ�
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
			//����CenterRange����������ֻ����ʼ���е�centerRange��Χ�ڲ���
			if (isFirst)
			{
				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
				data.y = i;
				data.x = maxLoc.y;
				//������ʼ��ǰ���ͱ�����ֵʹ�õĴ�һ�㣬�Ա�֤�������ƵĹǼ��Ǿ�����ȷ��
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

			//����Խ����
			if (data.x<windowwidth / 2 + 2 || data.x> src1.cols - 1 - windowwidth / 2 - 2)
			{
				continue;
			}
			//ȷ��ǰ������I1����������I2�����ƿ��w
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


			//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
			//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
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

			//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
			PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
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
	//zzl modify ���й������޳�
#if 1
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0]; 
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //ֻ�Կ�ȱ仯�ϴ�����ƽ��дִ��޳�
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

	//��roiͼ��������������ת����ԭͼ���е�����
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
	roi.width = src.cols; roi.height = src.rows; //��ȷ��
	LaserCenterDetect_row(src, Threshold, windowwidth, centerRange, LaserCenter, stripeMeanWidth, stripeStdWidth, searchWinHei, gaussK, MaxMidType, roi);
	return true;
}

bool CAlgorithm::LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange,
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)
{
	//��ͼ��ת���ɻҶ�ͼ
	if (src.channels() != 1){
		cvtColor(src,src,CV_RGB2GRAY);
	}

	//��ͼԭͼ���roi������Ϊ������ȡ����ͼ��
	if (ROI != cv::Rect(0,0,0,0)){
		src = src(ROI);
	}
	//�ȶ���ԭͼ��ı���
	Mat srcCpy = src.clone();

	//�����Ƿ��������Ʊ�ʶ
	bool isNear = 0;
	//�Ƿ��������ϵ�һ�����־
	bool isFirst = 1;

	//�����ͼ�����������ֵ����λ��Maxc
	Point data;
	Point2f center;
	int maxVal;
	Point maxLoc;

	int trapeHeiTem = 0;
	int colRangeHalf = 0;
	//max������mask����ʼ�к���ֹ��
	int colStart = 0;
	int colEnd = 0;
	int colMax;

	//threTem�����趨�׵�������Ƶ���ֵ
	int threTem;
	//�趨�������ĵ�һ������У��Դ�ȷ�����ƵĹǼ�����
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//��Ч����������ȡ�Ĺ�����Ⱦ�ֵ�ͱ�׼��
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//���ڿ�ȵ�һ��
	int width = (windowwidth - 1) / 2;
	//test
	clock_t ta1 = 0;

	//����ʹ�ø�˹�˲�ʱû�жԱ߽���в�ֵ�������еĿ�ʼֵ�ͽ���ֵ��������
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
			//��minMaxLoc����ֻ�ܹ��ҵ����ֵ��߽�ĵ㼰��λ�ã���������������ұ߽�ķ���������
			//getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
	/*		clock_t t1 = clock();*/
			getColMaxLocMid(src1, colStart, colEnd, maxVal, colMax, 1);
			//clock_t t2 = clock();
			//ta1 += t2 - t1;
			//minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, colMask); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
			data.y = i;
			data.x = colMax;

			//����Խ����
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
			//ȷ��ǰ������I1����������I2�����ƿ��w
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

			//Step 3�� I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
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
			//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
			//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
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

			//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
			PartGaussianBlur_col(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//Step 4�����ķ����������������ģ���ͳ�����ļ�������ظ���n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>( data.x - width + j, data.y); //col
				if (PixelValue>I1 + Threshold)  //�Ķ�
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
			//����CenterRange����������ֻ����ʼ���е�centerRange��Χ�ڲ���
			if (isFirst)
			{
				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
				data.y = i;
				data.x = maxLoc.y;
				//������ʼ��ǰ���ͱ�����ֵʹ�õĴ�һ�㣬�Ա�֤�������ƵĹǼ��Ǿ�����ȷ��
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

				////ɾ��centerRange���� �滻�������
				minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
				data.y = i;
				data.x = maxLoc.y;
				threTem = Threshold;
			}

			//����Խ����
			if (data.x<windowwidth / 2 + 2 || data.x> src1.rows - 1 - windowwidth / 2 - 2)
			{
				continue;
			}
			//ȷ��ǰ������I1����������I2�����ƿ��w
			double I1 = 0, I2 = 0, w = 0;

			double m1 = 0, m2 = 0, m3 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				m1 += pow(src.at<uchar>(data.x - width + j, data.y), 1); //col �޸Ĵ�
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


			//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
			//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
			if (windowwidth < w)
			{
				continue;
			}

			//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
			PartGaussianBlur_col(src, srcCpy, i, data.x - width, data.x + width, gaussK);
			//PartMeanBlur(src, i, data.x - width, data.x + width);

			//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
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
					//ɾ��centerRange����
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
	//zzl modify ���й������޳�
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0];
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //ֻ�Կ�ȱ仯�ϴ�����ƽ��дִ��޳�
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

	//��roiͼ��������������ת����ԭͼ���е�����
	for (int num = 0; num < LaserCenter.size(); ++num)
	{
		LaserCenter[num].x = LaserCenter[num].x + ROI.tl().x;
		LaserCenter[num].y = LaserCenter[num].y + ROI.tl().y;
	}
	cout << "ȡ��������غ�ʱ��" << ta1 << endl;
	return 1;
}

bool CAlgorithm::LaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange, 
	vector<Point2f> &LaserCenter, double &stripeMeanWidth, double &stripeStdWidth, int searchWinHei, Mat& gaussK, const bool& MaxMidType)
{
	cv::Rect roi;
	roi.x = 0; roi.y = 0;
	roi.width = src.cols; roi.height = src.rows; //��ȷ��
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
	//����Ƿ���
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
	cv::Vec3d z = cv::normalize(x.cross(tem)); //������λ����һ���ǵ�λ���𣿣�
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

//�㷨����
#if bak
bool CAlgorithm::CrossLaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange, vector<laserCenter> &LaserCenters, double &stripeMeanWidth, double &stripeStdWidth, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)

{
	//��ͼ��ת���ɻҶ�ͼ
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//��ͼԭͼ���roi������Ϊ������ȡ����ͼ��
	src = src(ROI);
	//���и�˹�˲���ȥ������---Ϊ�����Ч�ʣ�ֻ������������������˲�
	//GaussianBlur(src, src, Size(5, 5), 0, 0);
	//�ȶ���ԭͼ��ı���
	Mat srcCpy = src.clone();

	//�����Ƿ��������Ʊ�ʶ
	bool isNear = 0;

	//�����ͼ�����������ֵ����λ��Maxc
	Point data;
	int maxVal;
	Point maxLoc;

	int colRangeHalf = 0;
	//max������mask����ʼ�к���ֹ��
	int colStart = 0;
	int colEnd = 0;

	//threTem�����趨�׵�������Ƶ���ֵ
	int threTem;
	//�趨�������ĵ�һ������У��Դ�ȷ�����ƵĹǼ�����
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//��Ч����������ȡ�Ĺ�����Ⱦ�ֵ�ͱ�׼��
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//���ڿ�ȵ�һ��
	int width = (windowwidth - 1) / 2;
	vector <int> oldCenters;	//��һ�ε�����λ�ã�֧�ֶ������
	oldCenters.reserve(20);		//Ԥ�ȿ���20��Ԫ�ؿռ䡣
	int stripNum = 2;  //��������

	//����ʹ�ø�˹�˲�ʱû�жԱ߽���в�ֵ�������еĿ�ʼֵ�ͽ���ֵ��������
	for (int i = 2; i < src.rows - 2; i++)
	{
		//Mat src1 = src.row(i);
		uchar *src1 = src.ptr<uchar>(i);
		int cols = src.cols;
		laserCenter  center; //ÿһ�α�����ȡ����������
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

				//����Խ����
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2){
					isNear = false;
					continue;
				}

				//ȷ��ǰ������I1����������I2�����ƿ��w
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

				//I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
				//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
				//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
				PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
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
				//���������ñȽϽ�������Ϊֻ��һ������ 
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

				//����Խ����
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2)
				{
					isNear = false;
					continue;
				}

				//ȷ��ǰ������I1����������I2�����ƿ��w
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

				//I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
				//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
				//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
				PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
				float PixelValue = 0;
				float sum1 = 0;
				float sum2 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					PixelValue = src.at<uchar>(i, data.x - width + k);
					if (PixelValue>I1 + Threshold)  //�Ķ�
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
				//���������ñȽϽ�������Ϊֻ��һ������ 
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

	//�������Ƶ����
	//vector<Point> pnts;
	//for (int i = 0; i < LaserCenters.size(); i++)
	//{
	//	for (int j = 0; j < LaserCenters[i].pt.size(); j++)
	//	{
	//		pnts.push_back(LaserCenters[i].pt[j]);
	//	}
	//}

	//zzl modify ���й������޳�
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0];
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //ֻ�Կ�ȱ仯�ϴ�����ƽ��дִ��޳�
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
	//��roiͼ��������������ת����ԭͼ���е�����
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
bool CAlgorithm::CrossLaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, int centerRange, vector<laserCenter> &LaserCenters, double &stripeMeanWidth, double &stripeStdWidth, Mat& gaussK, const bool& MaxMidType, cv::Rect ROI)

{
	//��ͼ��ת���ɻҶ�ͼ
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//��ͼԭͼ���roi������Ϊ������ȡ����ͼ��
	src = src(ROI);
	//���и�˹�˲���ȥ������---Ϊ�����Ч�ʣ�ֻ������������������˲�
	//GaussianBlur(src, src, Size(5, 5), 0, 0);
	//�ȶ���ԭͼ��ı���
	Mat srcCpy = src.clone();

	//�����Ƿ��������Ʊ�ʶ
	bool isNear = 0;

	//�����ͼ�����������ֵ����λ��Maxc
	Point data;
	int maxVal;
	Point maxLoc;

	int colRangeHalf = 0;
	//max������mask����ʼ�к���ֹ��
	int colStart = 0;
	int colEnd = 0;

	//threTem�����趨�׵�������Ƶ���ֵ
	int threTem;
	//�趨�������ĵ�һ������У��Դ�ȷ�����ƵĹǼ�����
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//��Ч����������ȡ�Ĺ�����Ⱦ�ֵ�ͱ�׼��
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//���ڿ�ȵ�һ��
	int width = (windowwidth - 1) / 2;
	vector <int> oldCenters;	//��һ�ε�����λ�ã�֧�ֶ������
	oldCenters.reserve(20);		//Ԥ�ȿ���20��Ԫ�ؿռ䡣
	int stripNum = 2;  //��������

	//����ʹ�ø�˹�˲�ʱû�жԱ߽���в�ֵ�������еĿ�ʼֵ�ͽ���ֵ��������
	for (int i = 2; i < src.rows - 2; i++)
	{
		//Mat src1 = src.row(i);
		uchar *src1 = src.ptr<uchar>(i);
		int cols = src.cols;
		laserCenter  center; //ÿһ�α�����ȡ����������
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

				//����Խ����
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2){
					isNear = false;
					continue;
				}

				//ȷ��ǰ������I1����������I2�����ƿ��w
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

				//I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
				//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
				//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
				PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
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
				//���������ñȽϽ�������Ϊֻ��һ������ 
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
		else
		{
			vector <uchar> _maxva;
			vector <int> _maxPos;
			getMultMax(src1, 0, cols - 1, windowwidth, stripNum, _maxva, _maxPos);
			for (int j = 0; j < stripNum; j++)
			{
				data.y = i;
				data.x = _maxPos[j];

				//����Խ����
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2)
				{
					isNear = false;
					continue;
				}

				//ȷ��ǰ������I1����������I2�����ƿ��w
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

				//I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
				//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
				//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
				PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
				float PixelValue = 0;
				float sum1 = 0;
				float sum2 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					PixelValue = src.at<uchar>(i, data.x - width + k);
					if (PixelValue>I1 + Threshold)  //�Ķ�
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
				//���������ñȽϽ�������Ϊֻ��һ������ 
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

	//�������Ƶ����
	//vector<Point> pnts;
	//for (int i = 0; i < LaserCenters.size(); i++)
	//{
	//	for (int j = 0; j < LaserCenters[i].pt.size(); j++)
	//	{
	//		pnts.push_back(LaserCenters[i].pt[j]);
	//	}
	//}

	//zzl modify ���й������޳�
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0];
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //ֻ�Կ�ȱ仯�ϴ�����ƽ��дִ��޳�
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
	//��roiͼ��������������ת����ԭͼ���е�����
	for (int num = 0; num < LaserCenters.size(); ++num)
	{
		for (int k = 0; k < LaserCenters[num].pt.size(); k++){
			LaserCenters[num].pt[k] = LaserCenters[num].pt[k] + ROI.tl().x;
		}
		LaserCenters[num].index = LaserCenters[num].index + ROI.tl().y;
	}
	return 1;
}
#if 1
bool CAlgorithm::CrossLaserCenterDetect(Mat& src, const int Threshold, const int windowwidth, vector<Point2f> pnt1, vector<Point2f> pnt2, cv::Rect ROI)
{
	//��ͼ��ת���ɻҶ�ͼ
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//��ͼԭͼ���roi������Ϊ������ȡ����ͼ��
	src = src(ROI);
	//�ȶ���ԭͼ��ı���
	Mat srcCpy = src.clone();
	int stripNum = 2;  //��������
	//***����ͼ���һ�׵����Ͷ��׵���
	Mat img = src.clone();
	img.convertTo(img, CV_32FC1);
	GaussianBlur(img, img, Size(0, 0), 5, 5); //��˹�������w/sqrt(3)����������wΪ���ƵĿ��

	//һ��ƫ����
	Mat m1, m2;
	m1 = (Mat_<float>(1, 2) << 1, -1);  //xƫ��
	m2 = (Mat_<float>(2, 1) << 1, -1);  //yƫ��

	Mat dx, dy;
	filter2D(img, dx, CV_32FC1, m1);
	filter2D(img, dy, CV_32FC1, m2);

	//����ƫ����
	Mat m3, m4, m5;
	m3 = (Mat_<float>(1, 3) << 1, -2, 1);   //����xƫ��
	m4 = (Mat_<float>(3, 1) << 1, -2, 1);   //����yƫ��
	m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);   //����xyƫ��

	Mat dxx, dyy, dxy;
	filter2D(img, dxx, CV_32FC1, m3);
	filter2D(img, dyy, CV_32FC1, m4);
	filter2D(img, dxy, CV_32FC1, m5);

	//�������ĵ�map������ӳ�������ĵ�������ϵ������ֵ��洢�˹�������������LaserCenters�е����
	Mat map = cv::Mat::zeros(img.rows, img.cols, CV_16UC1);
	//Mat map = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

	//�����Ƿ��������Ʊ�ʶ
	bool isNear = 0;

	//��������
	vector<Point2f> LaserCenters;
	LaserCenters.reserve(max (src.rows,src.cols));
	//�������ĵķ���
	vector <Point2f> normals;
	normals.reserve(stripNum*src.rows); //todo::Ԥ����С�д�����

	//�����ͼ�����������ֵ����λ��Maxc
	Point data;
	int maxVal;
	Point maxLoc;

	int colRangeHalf = 0;
	//max������mask����ʼ�к���ֹ��
	int colStart = 0;
	int colEnd = 0;

	//threTem�����趨�׵�������Ƶ���ֵ
	int threTem;
	//�趨�������ĵ�һ������У��Դ�ȷ�����ƵĹǼ�����
	int firstCenterCol;
	int rangeColStart, rangeColEnd;

	//ÿһ�е��������Ƶ����ľ���:x������ǵ�ǰ�е�λ�ã�y��������Ĳ�ֵ
	vector <Point2f> centerdis;

	//��Ч����������ȡ�Ĺ�����Ⱦ�ֵ�ͱ�׼��
	double meanWidth = 0, stdWidth = 0;
	vector<double> stripeWidth;
	//���ڿ�ȵ�һ��
	int width = (windowwidth - 1) / 2;
	vector <int> oldCenters;	//��һ�ε�����λ�ã�֧�ֶ������
	oldCenters.reserve(stripNum);		

	//����ʹ�ø�˹�˲�ʱû�жԱ߽���в�ֵ�������еĿ�ʼֵ�ͽ���ֵ��������
	for (int i = 2; i < src.rows - 2; i++)
	{
		//Mat src1 = src.row(i);
		uchar *src1 = src.ptr<uchar>(i);
		int cols = src.cols;
		laserCenter  center; //ÿһ�α�����ȡ����������
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

				//����Խ����
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2){
					isNear = false;
					continue;
				}

				//ȷ��ǰ������I1����������I2�����ƿ��w
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

				//I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
				//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
				//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
				//PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
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
					LaserCenters.push_back(Point2f(i, y));
					//��map�б�ʶ�õ�
					map.at<unsigned short>(i, y) = LaserCenters.size() - 1;
					//map.at<uchar>(i, y) = 255;

					//��õ㴦�ķ��࣬�����ݷ����ж����ĸ�����
					Point pt = Point(i, y);
					Mat hessian(2, 2, CV_32FC1);
					hessian.at<float>(0, 0) = dxx.at<float>(pt.x, pt.y);
					hessian.at<float>(0, 1) = dxy.at<float>(pt.x, pt.y);
					hessian.at<float>(1, 0) = dxy.at<float>(pt.x, pt.y);
					hessian.at<float>(1, 1) = dyy.at<float>(pt.x, pt.y);

					Mat eValue;
					Mat eVectors;
					eigen(hessian, eValue, eVectors);

					double nx, ny;
					double fmaxD = 0;
					if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))  //������ֵ���ʱ��Ӧ����������
					{
						nx = eVectors.at<float>(0, 0);
						ny = eVectors.at<float>(0, 1);
						fmaxD = eValue.at<float>(0, 0);
					}
					else
					{
						nx = eVectors.at<float>(1, 0);
						ny = eVectors.at<float>(1, 1);
						fmaxD = eValue.at<float>(1, 0);
					}
					normals.push_back(Point2f(nx, ny));
				}
			}

			//����������֮��ľ���
			if (center.pt.size() == 1) centerdis.push_back(Point2f(i, 0));
			if (center.pt.size() == 2) centerdis.push_back(Point2f(i, abs(center.pt[0] - center.pt[1])));
			
			if (center.pt.size() == stripNum)
			{
				//���������ñȽϽ�������Ϊֻ��һ������ 
				vector <float>  diss;
				int _num = center.pt.size();
				for (int n = 0; n < _num - 1; n++)
				{
					for (int nn = n + 1; nn < _num; nn++)
					{
						diss.push_back(fabs(center.pt[n] - center.pt[nn]));
					}
				}
				bool flag = false;
				for (int n = 0; n < diss.size(); n++)
				{
					if (diss[n] < 3 * windowwidth)
					{
						isNear = false;
						flag = true;
						break;
					}
				}
				if (flag) continue;

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
			getMultMax(src1, 0, cols - 1, windowwidth, stripNum, _maxva, _maxPos);
			for (int j = 0; j < stripNum; j++)
			{
				data.y = i;
				data.x = _maxPos[j];

				//����Խ����
				if (data.x<windowwidth / 2 + 2 || data.x> cols - 1 - windowwidth / 2 - 2)
				{
					isNear = false;
					continue;
				}

				//ȷ��ǰ������I1����������I2�����ƿ��w
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

				//I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
				//�Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
				//�Ƚ����ƿ���Ƿ���ڴ��ڿ�ȣ������ڣ������ĵ���Ч
				if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold || windowwidth < w)
				{
					isNear = false;
					continue;
				}
				//�Ե�ǰ�д��������е����ؽ��и�˹�˲�
				//PartGaussianBlur(src, srcCpy, i, data.x - width, data.x + width, gaussK);

				//���ķ����������������ģ���ͳ�����ļ�������ظ���n1
				float PixelValue = 0;
				float sum1 = 0;
				float sum2 = 0;
				for (int k = 0; k < windowwidth; k++)
				{
					PixelValue = src.at<uchar>(i, data.x - width + k);
					if (PixelValue>I1 + Threshold)  //�Ķ�
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
					LaserCenters.push_back(Point2f(i, y));
					//��map�б�ʶ�õ�
					map.at<unsigned short>(i, y) = LaserCenters.size() - 1;
					//map.at<uchar>(i, y) = 255;

					//��õ㴦�ķ��࣬�����ݷ����ж����ĸ�����
					Point pt = Point(i, y);
					Mat hessian(2, 2, CV_32FC1);
					hessian.at<float>(0, 0) = dxx.at<float>(pt.x, pt.y);
					hessian.at<float>(0, 1) = dxy.at<float>(pt.x, pt.y);
					hessian.at<float>(1, 0) = dxy.at<float>(pt.x, pt.y);
					hessian.at<float>(1, 1) = dyy.at<float>(pt.x, pt.y);

					Mat eValue;
					Mat eVectors;
					eigen(hessian, eValue, eVectors);

					double nx, ny;
					double fmaxD = 0;
					if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))  //������ֵ���ʱ��Ӧ����������
					{
						nx = eVectors.at<float>(0, 0);
						ny = eVectors.at<float>(0, 1);
						fmaxD = eValue.at<float>(0, 0);
					}
					else
					{
						nx = eVectors.at<float>(1, 0);
						ny = eVectors.at<float>(1, 1);
						fmaxD = eValue.at<float>(1, 0);
					}
					normals.push_back(Point2f(nx, ny));
				}
			}
			//����������֮��ľ���
			if (center.pt.size() == 1) centerdis.push_back(Point2f(i, 0));
			if (center.pt.size() == 2) centerdis.push_back(Point2f(i, abs(center.pt[0] - center.pt[1])));

			if (center.pt.size() == stripNum)
			{
				//���������ñȽϽ�������Ϊֻ��һ������ 
				vector <float>  diss;
				int _num = center.pt.size();
				for (int n = 0; n < _num - 1; n++)
				{
					for (int nn = n + 1; nn < _num; nn++)
					{
						diss.push_back(fabs(center.pt[n] - center.pt[nn]));
					}
				}
				bool flag = false;
				for (int n = 0; n < diss.size(); n++)
				{
					if (diss[n] < 3 * windowwidth)
					{
						isNear = false;
						flag = true;
						break;
					}
				}
				if (flag) continue;

				isNear = true;
				oldCenters.clear();
				for (int k = 0; k < stripNum; k++)
				{
					oldCenters.push_back(int(center.pt[k]));
				}

			}
		}
	}

	//test 
	//cvtColor(srcCpy, srcCpy, CV_GRAY2RGB);
	//for (int i = 0; i < LaserCenters.size(); i++)
	//{
	//	srcCpy.at<cv::Vec3b>(LaserCenters[i].x, LaserCenters[i].y) = cv::Vec3b(0, 0, 255);
	//	cv::Point endpt = LaserCenters[i] + Point2f(normals[i].x * 10, normals[i].y * 10);
	//	//show.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
	//	cv::line(srcCpy, Point(LaserCenters[i].y, LaserCenters[i].x), Point(endpt.y,endpt.x), cv::Scalar(0, 255, 0));
	//}

	//***�������Ƶ����
	//map.convertTo(map, CV_8UC1);
	Point cross;
	classCrossPt(map, LaserCenters, normals,centerdis, 5 * PI / 180, 10, pnt1, pnt2,cross);

	//test
	cvtColor(srcCpy, srcCpy, CV_GRAY2RGB);
	circle(srcCpy, Point(cross.y,cross.x), 10, Scalar(255, 0, 255), 5);
	for (int i = 0; i < pnt1.size(); i++)
	{
		srcCpy.at<cv::Vec3b>(pnt1[i].x, pnt1[i].y) = cv::Vec3b(0, 0, 255);
	}
	for (int i = 0; i < pnt2.size(); i++)
	{
		srcCpy.at<cv::Vec3b>(pnt2[i].x, pnt2[i].y) = cv::Vec3b(0, 255, 0);
	}

	//zzl modify ���й������޳�
#if 0
	vector<double> tep1, tep2;
	vector <Point2f> temCenters;
	cv::meanStdDev(stripeWidth, tep1, tep2);
	stripeMeanWidth = meanWidth = tep1[0];
	stripeStdWidth = stdWidth = tep2[0];
	if (stdWidth > 3) //ֻ�Կ�ȱ仯�ϴ�����ƽ��дִ��޳�
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
	//��roiͼ��������������ת����ԭͼ���е�����
	for (int num = 0; num < pnt1.size(); ++num)
	{
		pnt1[num].x += ROI.tl().y;
		pnt1[num].y += ROI.tl().x;
	}
	for (int num = 0; num < pnt2.size(); ++num)
	{
		pnt2[num].x += ROI.tl().y;
		pnt2[num].y += ROI.tl().x;
	}
	return 1;
}

#endif

void CAlgorithm::getPointNormal(const Mat &src, const vector<Point> &pnts, vector<Point2f> &normals)
{
	Mat img;
	img = src.clone();
	img.convertTo(img, CV_32FC1);

	//һ��ƫ����
	Mat m1, m2;
	m1 = (Mat_<float>(1, 2) << 1, -1);  //xƫ��
	m2 = (Mat_<float>(2, 1) << 1, -1);  //yƫ��

	Mat dx, dy;
	filter2D(img, dx, CV_32FC1, m1);
	filter2D(img, dy, CV_32FC1, m2);

	//����ƫ����
	Mat m3, m4, m5;
	m3 = (Mat_<float>(1, 3) << 1, -2, 1);   //����xƫ��
	m4 = (Mat_<float>(3, 1) << 1, -2, 1);   //����yƫ��
	m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);   //����xyƫ��

	Mat dxx, dyy, dxy;
	filter2D(img, dxx, CV_32FC1, m3);
	filter2D(img, dyy, CV_32FC1, m4);
	filter2D(img, dxy, CV_32FC1, m5);
	
	//������Щ���hessian����
	for (int i = 0; i < pnts.size(); i++)
	{
		Point pt = pnts[i];
		Mat hessian(2, 2, CV_32FC1);
		hessian.at<float>(0, 0) = dxx.at<float>(pt.x, pt.y);
		hessian.at<float>(0, 1) = dxy.at<float>(pt.x, pt.y);
		hessian.at<float>(1, 0) = dxy.at<float>(pt.x, pt.y);
		hessian.at<float>(1, 1) = dyy.at<float>(pt.x, pt.y);

		Mat eValue;
		Mat eVectors;
		eigen(hessian, eValue, eVectors);

		double nx, ny;
		double fmaxD = 0;
		if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))  //������ֵ���ʱ��Ӧ����������
		{
			nx = eVectors.at<float>(0, 0);
			ny = eVectors.at<float>(0, 1);
			fmaxD = eValue.at<float>(0, 0);
		}
		else
		{
			nx = eVectors.at<float>(1, 0);
			ny = eVectors.at<float>(1, 1);
			fmaxD = eValue.at<float>(1, 0);
		}
		normals.push_back(cv::Point2f(nx, ny));
	}
	return;
}

void CAlgorithm::getPointNormal(const Mat &src, const vector<Point2f> &pnts, vector<Point2f> &normals)
{
	vector <Point> _pnts;
	_pnts.reserve(2000);
	for (int i = 0; i < pnts.size(); i++)
	{
		_pnts.push_back(Point(pnts[i].x, pnts[i].y));
	}
	getPointNormal(src, _pnts, normals);
}

void CAlgorithm::smoothPoints(vector<Point2f> srcPnts, vector<Point2f> &smoothPnts, int fitMethod, float sampleDis)
{
	real_1d_array x, y;
	x.setlength(srcPnts.size());
	y.setlength(srcPnts.size());
	smoothPnts.clear();
	for (int i = 0; i < srcPnts.size(); i++)
	{
		x[i] = srcPnts[i].x;
		y[i] = srcPnts[i].y;
	}
	//todo::ֱ�����
	if (fitMethod == 1)
	{
		
	}
	//AKima spline��Ϸ������ŵ㣺�������Ƚ��ȶ���http://www.alglib.net/interpolation/spline3.php
	if (fitMethod == 2)
	{
		spline1dinterpolant c;
		spline1dbuildakima(x, y, c);
		int gap = srcPnts.back().x - srcPnts[0].x;
		for (int i = 0; i <= gap; i++)
		{
			float x = srcPnts[0].x + i;
			float y = spline1dcalc(c, x);
			smoothPnts.push_back(Point2f(x, y));
		}
	}
}

//��ʮ�ּ������ƽ��з���
void CAlgorithm::classCrossPt(const Mat &src, const vector <laserCenter> &LaserCenters, vector<Point2f> &pnts1, vector<Point2f> &pnts2)
{
	//�Ե��Ƶ�б��Ӧ�þ����㷨�����ƽ��з���
	//step 1:��ȡ�����������ĵķ��ࣨhession���󷽷����㷨�ࣩ
	vector <Point2f> srcpnts;
	vector <Point2f> normals;
	srcpnts.reserve(2500);
	for (int i = 0; i < LaserCenters.size(); i++){
		for (int j = 0; j < LaserCenters[i].pt.size(); j++){
			srcpnts.push_back(Point2f(LaserCenters[i].index, LaserCenters[i].pt[j]));
		}
	}
	getPointNormal(src, srcpnts, normals);

	//for test 
	Mat show_img = src.clone();
	cvtColor(show_img, show_img, CV_GRAY2BGR);

	//step 2:���ݷ�����о���
	for (int i = 0; i < normals.size(); i++)
	{
		Point2f nl = normals[i];
		if (nl.x * nl.y < 0)
		{
			pnts1.push_back(srcpnts[i]);
		}
		else
		{
			pnts2.push_back(srcpnts[i]);
		}
		//test:��ͼ������ʾ����
		Point pt(srcpnts[i].x, srcpnts[i].y);
		cv::Point endpt = pt + Point(nl.x * 10, nl.y * 10);
		cv::line(show_img, Point(pt.y, pt.x), Point(endpt.y, endpt.x), cv::Scalar(0, 255, 0));
	}

	//test: �ӷ���ֲ�ͼ��������û���Եľ������
	//ofstream fi("./normal.txt", ios::trunc);
	//for (int i = 0; i < normals.size(); i++ ){
	//	fi << normals[i].x << " "<<normals[i].y<<endl;
	//}
	//fi.close();
	cv::namedWindow("normal");
	//cv::imshow("normal", show_img);
	//cv::waitKey(0);
}

bool CAlgorithm::classCrossPt(Mat &map, const vector <Point2f> &LaserCenters, const vector <Point2f> &normals, const vector<Point2f> &centerDis,
	const float angleThreasold, const int Radius, vector<Point2f> &pnts1, vector<Point2f> &pnts2, Point &cross)
{
	vector <bool> flags(LaserCenters.size(), false); //��ʶ�㼯�е��Ƿ��Ѿ����࣬false��ʾδ����
	vector <vector <int>>  ptCluster; //�༯��
	if (Radius <= 0) return false;
	int kthreasold = Radius; //������ֵ
	float cosThreasold = cosf(angleThreasold); //todo::�ԽǶ�ֵ��������

	//***�ҳ������Ĵ���λ�� (�ҵò�׼���д�����)
	/*int scop = 50;
	int step = 2;
	int crossPos = 0;
	int disnum = centerDis.size();
	for (int i = 0; i < disnum; i++){
		int dNum = 0, upNum = 0;
		int dtol = 0, uptol = 0;
		if (!centerDis[i].y){
			int up = i - scop, down = i + scop;
			if (up < 0 || down > disnum - 1) continue;
			for (int j = up; j < i-step; j +=step){
				if (centerDis[j + step].y < centerDis[j].y)
					dNum++;
				dtol++;
			}
			for (int j = i; j < down - step; j += step){
				if (centerDis[j].y < centerDis[j + step].y)
					upNum++;
				uptol++;
			}
			if (dNum / (float)dtol > 0.7 && upNum / uptol > 0.7)
			{
				cross.x = centerDis[i].x;
				break;
			}
		}
	}

	for (int i = 0; i < map.cols; i++){
		if (map.at<unsigned short>(cross.x, i) != 0)
		{
			cross.y = i;
			break;
		}
	}
*/
	//***�������ľ���
	for (int i = 0; i < LaserCenters.size(); i++)
	{
		if (flags[i]) continue;
		vector <int>  _cluster;
		_cluster.push_back(i);
		flags[i] = true;
		queue <int> seeds;
		seeds.push(i);
		while (!seeds.empty())
		{
			int sid = seeds.back();
			seeds.pop();
			//������ֵ�ڲ������ĵ�
			int cr = (int)LaserCenters[sid].x;
			int cl = (int)LaserCenters[sid].y;
			map.at <unsigned short>(cr, cl) = 0;

			int rup = cr + kthreasold, rdown = cr - kthreasold;
			int cup = cl + kthreasold, cdown = cl - kthreasold;
			for (int j = rdown; j <= rup; j++)
			{
				for (int k = cdown; k <= cup; k++)
				{
					if (j < 0 || k < 0 || j > map.rows-1 || k > map.cols-1) 
						continue;
					UINT _id = map.at<unsigned short>(j, k);
					if (_id == 0)
						continue;
					double _cos = normals[sid].dot(normals[_id]);
					if (abs(_cos) > cosThreasold)
					{
						_cluster.push_back(_id);
						flags[_id] = true;
						map.at<unsigned short>(j, k) = 0;
						//_cluster.push_back(LaserCenters[_id]); //todo::�Ƿ�ֻҪ������Χ�ھ��ж�Ϊ���ڵ�
						seeds.push(_id);
					}
				}
			}
		}
		ptCluster.push_back(_cluster);
	}

	//test 

	//***����ϲ�����������������
	int minCluster = 10;
	for (int i = 0; i < ptCluster.size(); i++){
		if (ptCluster[i].size() < minCluster) continue;
		float sumx = 0;
		int _size = ptCluster[i].size();
		for (int j = 0; j < _size; j++){
			sumx += normals[ptCluster[i][j]].x*normals[ptCluster[i][j]].y; //todo::ȷ��normal�Ƿ��һ��
		}
		if (sumx > 0){
			for (int j = 0; j < _size; j++)
				pnts1.push_back(LaserCenters[ptCluster[i][j]]);
		}
		else{
			for (int j = 0; j < _size; j++)
				pnts2.push_back(LaserCenters[ptCluster[i][j]]);
		}	
	}
	return true;
}

bool CAlgorithm::CaculateHMatric(const double *planeParams, const CamPara &camParam, cv::Mat &camFrameInLaserFrame, cv::Mat &Hmat)
{
	//step1 �ɹ⵶ƽ�湹����������ϵ���ο�����ϵΪ�������ϵ��
	cv::Vec3d oriPnt;//��������ڼ���ƽ���ͶӰ
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
	//step2 ������������͹⵶ƽ���ϵ����������ϵ�ĵ�Ӧ���󣨲ο�����ϵ���⵶����ϵ��
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
	//cout << "����֮ǰ������֮���ٶ�ȡ = " << tep1[0] << endl;
	//Mat temdf; invert(tep1[0], temdf);
	//cout << "����֮ǰ������֮���ٶ�ȡ,������ = " << temdf << endl;
	//cout << "����֮ǰ������֮ǰ = " << Hmat << endl;


	//cv::invert(Hmat, Hmat);//�����Ժ���ֵ��С��������ٶ���������ƫ��
	//step3 ��xml��ʽ�ļ����浥Ӧ����Hmat
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

//�ɰ����������ȡ�㷨
#if 0
bool CSingleCamLaserSys::LaserCenterDetector(const Mat& srcimage, const cv::Rect mask, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter)
{
	//��ͼ��ת���ɻҶ�ͼ
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
	GaussianBlur(src, src, Size(5, 5), 0, 0);     //��˹�˲�
	//Step 1�������ͼ�����������ֵ����λ��Maxc
	Point2f data;
	vector<Point2f> Maxc;
	Point2f center;
	for (int i = 0; i < src.rows; i++)
	{
		Mat src1 = src.row(i).clone();
		//cout << "src1 = " << endl << " " << src1 << endl << endl;
		double maxVal = 0;
		Point maxLoc;
		cv::minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, Mat()); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
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

		////ZLL Խ���鱾Ӧ�ü�����x�������������ΰ�data��x��yд���ˡ�Ӧ����x�У�y��
		if (data.y<windowwidth / 2 || data.y> src1.cols - 1 - windowwidth / 2)
		{
			continue;
		}
		////// zhangxu added  end
		//Step 2��ȷ��ǰ������I1����������I2�����ƿ��w
		//Step 2.1��
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
		//Step 2.2��
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
		if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold) //Step 3�� I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
		{
			Maxc.push_back(data);
		}
		else
		{
			continue;
		}
		//Step 5:  �Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
		int n1 = windowwidth;
		if (n1 < w)
		{
			continue;
		}
		//Step 4�����ķ����������������ģ���ͳ�����ļ�������ظ���n1
		float PixelValue = 0;
		float sum1 = 0;
		float sum2 = 0;
		for (int j = 0; j < n1; j++)
		{
			PixelValue = src.at<uchar>(data.x, data.y - ((n1 - 1) / 2) + j);
			if (PixelValue>I1 + Threshold)  //�Ķ�
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
	//��ͼ��ת���ɻҶ�ͼ
	Mat src;
	if (srcimage.channels() == 1)
	{
		src = srcimage;
	}
	else
	{
		cv::cvtColor(srcimage, src, CV_BGR2GRAY);
	}
	GaussianBlur(src, src, Size(5, 5), 0, 0);     //��˹�˲�
	//Step 1�������ͼ�����������ֵ����λ��Maxc
	Point2f data;
	vector<Point2f> Maxc;
	Point2f center;
	for (int i = 0; i < src.rows; i++)
	{
		Mat src1 = src.row(i).clone();
		//cout << "src1 = " << endl << " " << src1 << endl << endl;
		double maxVal = 0;
		Point maxLoc;
		minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, Mat()); //����MinMaxLoc ����Ԫ���е���С����ֵ�Լ����ǵ�λ��
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

		////ZLL Խ���鱾Ӧ�ü�����x�������������ΰ�data��x��yд���ˡ�Ӧ����x�У�y��
		if (data.y<windowwidth / 2 || data.y> src1.cols - 1 - windowwidth / 2)
		{
			continue;
		}
		////// zhangxu added  end
		//Step 2��ȷ��ǰ������I1����������I2�����ƿ��w
		//Step 2.1��
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
		//Step 2.2��
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
		if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold) //Step 3�� I1-I2 ����ֵT�Ƚϣ���С�ڣ�������޼�����������
		{
			Maxc.push_back(data);
		}
		else
		{
			continue;
		}
		//Step 5:  �Ƚ����ļ�������ظ���n1 ��w����n1���ڵ���w����Ϊ��Ч����ֵp
		int n1 = windowwidth;
		if (n1 < w)
		{
			continue;
		}
		//Step 4�����ķ����������������ģ���ͳ�����ļ�������ظ���n1
		float PixelValue = 0;
		float sum1 = 0;
		float sum2 = 0;
		for (int j = 0; j < n1; j++)
		{
			PixelValue = src.at<uchar>(data.x, data.y - ((n1 - 1) / 2) + j);
			if (PixelValue>I1 + Threshold)  //�Ķ�
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



