#include "stdafx.h"
#include "ImgTransform.h"


void CimgTransform::calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR)
{
	for (int i = 0; i < rt_vec.size(); i++)
	{
		Mat R = rt_vec[i].R;
		Mat T = rt_vec[i].T;
		Mat _R;
		Rodrigues(R, _R);
		Mat H0(3, 3, CV_64F);
		_R.copyTo(H0);
		T.col(0).copyTo(H0.col(2));
		///// just for test  观察T的值
		float t1, t2, t3;
		t1 = T.at<double>(0, 0);
		t2 = T.at<double>(1, 0);
		t3 = T.at<double>(2, 0);
		//// end the test

		Mat H1 = Mat::zeros(3, 3, CV_64F);
		T.col(0).copyTo(H1.col(2));
		///////  以下的标定板坐标系是  x向下，y向右，z标定板向前，和matlabtool box camera calibration一样
		//H1.at<double>(0, 1) = 1;
		//H1.at<double>(1, 0) = 1;
		///////  以下的标定板坐标系是  x向右，y向下，z标定板向后，
		H1.at<double>(0, 0) = 1;
		H1.at<double>(1, 1) = 1;
		///// end


		Mat H0_invert;
		invert(H0, H0_invert);

		_R = H1*H0_invert;

		parallelR.push_back(_R);
	}
}


// 该函数没有做论文中提出的将领域内的梯度方向集成到一个像素中的操作
//void CimgTransform::quantizedOrientations(const Mat& src, Mat& magnitude,
//	Mat& angle, float threshold)
//{
//	magnitude.create(src.size(), CV_32F);
//
//	// Allocate temporary buffers
//	Size size = src.size();
//	Mat sobel_3dx; // per-channel horizontal derivative
//	Mat sobel_3dy; // per-channel vertical derivative
//	Mat sobel_dx(size, CV_32F);      // maximum horizontal derivative
//	Mat sobel_dy(size, CV_32F);      // maximum vertical derivative
//	Mat sobel_ag;  // final gradient orientation (unquantized)
//	Mat smoothed;
//
//	// Compute horizontal and vertical image derivatives on all color channels separately
//	static const int KERNEL_SIZE = 7;
//	// For some reason cvSmooth/cv::GaussianBlur, cvSobel/cv::Sobel have different defaults for border handling...
//	GaussianBlur(src, smoothed, Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0, BORDER_REPLICATE);
//	Sobel(smoothed, sobel_3dx, CV_16S, 1, 0, 3, 1.0, 0.0, BORDER_REPLICATE);
//	Sobel(smoothed, sobel_3dy, CV_16S, 0, 1, 3, 1.0, 0.0, BORDER_REPLICATE);
//
//	short * ptrx = (short *)sobel_3dx.data;
//	short * ptry = (short *)sobel_3dy.data;
//	float * ptr0x = (float *)sobel_dx.data;
//	float * ptr0y = (float *)sobel_dy.data;
//	float * ptrmg = (float *)magnitude.data;
//
//	const int length1 = static_cast<const int>(sobel_3dx.step1());
//	const int length2 = static_cast<const int>(sobel_3dy.step1());
//	const int length3 = static_cast<const int>(sobel_dx.step1());
//	const int length4 = static_cast<const int>(sobel_dy.step1());
//	const int length5 = static_cast<const int>(magnitude.step1());
//	const int length0 = sobel_3dy.cols * 3;
//
//	for (int r = 0; r < sobel_3dy.rows; ++r)
//	{
//		int ind = 0;
//
//		for (int i = 0; i < length0; i += 3)
//		{
//			// Use the gradient orientation of the channel whose magnitude is largest
//			int mag1 = ptrx[i + 0] * ptrx[i + 0] + ptry[i + 0] * ptry[i + 0];
//			int mag2 = ptrx[i + 1] * ptrx[i + 1] + ptry[i + 1] * ptry[i + 1];
//			int mag3 = ptrx[i + 2] * ptrx[i + 2] + ptry[i + 2] * ptry[i + 2];
//
//			if (mag1 >= mag2 && mag1 >= mag3)
//			{
//				ptr0x[ind] = ptrx[i];
//				ptr0y[ind] = ptry[i];
//				ptrmg[ind] = (float)mag1;
//			}
//			else if (mag2 >= mag1 && mag2 >= mag3)
//			{
//				ptr0x[ind] = ptrx[i + 1];
//				ptr0y[ind] = ptry[i + 1];
//				ptrmg[ind] = (float)mag2;
//			}
//			else
//			{
//				ptr0x[ind] = ptrx[i + 2];
//				ptr0y[ind] = ptry[i + 2];
//				ptrmg[ind] = (float)mag3;
//			}
//			++ind;
//		}
//		ptrx += length1;
//		ptry += length2;
//		ptr0x += length3;
//		ptr0y += length4;
//		ptrmg += length5;
//	}
//
//	// Calculate the final gradient orientations
//	phase(sobel_dx, sobel_dy, sobel_ag, true); //计算梯度方向
//	hysteresisGradient(magnitude, angle, sobel_ag, threshold * threshold);
//}

void CimgTransform::hysteresisGradient(Mat& magnitude, Mat& quantized_angle,
	Mat& angle, float threshold)
{
	// Quantize 360 degree range of orientations into 16 buckets
	// Note that [0, 11.25), [348.75, 360) both get mapped in the end to label 0,
	// for stability of horizontal and vertical features.
	Mat_<unsigned char> quantized_unfiltered;
	angle.convertTo(quantized_unfiltered, CV_8U, 16.0 / 360.0); //将角度信息从360缩减到16度，值得学习

	// Zero out top and bottom rows
	/// @todo is this necessary, or even correct?
	memset(quantized_unfiltered.ptr(), 0, quantized_unfiltered.cols);
	memset(quantized_unfiltered.ptr(quantized_unfiltered.rows - 1), 0, quantized_unfiltered.cols);
	// Zero out first and last columns
	for (int r = 0; r < quantized_unfiltered.rows; ++r)
	{
		quantized_unfiltered(r, 0) = 0;
		quantized_unfiltered(r, quantized_unfiltered.cols - 1) = 0;
	}

	// Mask 16 buckets into 8 quantized orientations
	for (int r = 1; r < angle.rows - 1; ++r)
	{
		uchar* quant_r = quantized_unfiltered.ptr<uchar>(r);
		for (int c = 1; c < angle.cols - 1; ++c)
		{
			quant_r[c] &= 7; //屏蔽掉第4位，去掉了方向信息（这里的方向信息就是论文中提到的direction而非orientation）
		}
	}
	//***获取3领域内的梯度信息，只取领域内某个梯度方向的数量超过一定阈值的像素点
	// Filter the raw quantized image. Only accept pixels where the magnitude is above some
	// threshold, and there is local agreement on the quantization.
	quantized_angle = Mat::zeros(angle.size(), CV_8U);
	for (int r = 1; r < angle.rows - 1; ++r)
	{
		float* mag_r = magnitude.ptr<float>(r);

		for (int c = 1; c < angle.cols - 1; ++c)
		{
			if (mag_r[c] > threshold)
			{
				// Compute histogram of quantized bins in 3x3 patch around pixel
				int histogram[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

				uchar* patch3x3_row = &quantized_unfiltered(r - 1, c - 1);
				histogram[patch3x3_row[0]]++;
				histogram[patch3x3_row[1]]++;
				histogram[patch3x3_row[2]]++;

				patch3x3_row += quantized_unfiltered.step1();
				histogram[patch3x3_row[0]]++;
				histogram[patch3x3_row[1]]++;
				histogram[patch3x3_row[2]]++;

				patch3x3_row += quantized_unfiltered.step1();
				histogram[patch3x3_row[0]]++;
				histogram[patch3x3_row[1]]++;
				histogram[patch3x3_row[2]]++;

				// Find bin with the most votes from the patch
				int max_votes = 0;
				int index = -1;
				for (int i = 0; i < 8; ++i)
				{
					if (max_votes < histogram[i])
					{
						index = i;
						max_votes = histogram[i];
					}
				}

				// Only accept the quantization if majority of pixels in the patch agree
				static const int NEIGHBOR_THRESHOLD = 5;
				if (max_votes >= NEIGHBOR_THRESHOLD)
					quantized_angle.at<uchar>(r, c) = uchar(1 << index);
			}
		}
	}
}
