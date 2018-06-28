//#include"stdafx.h"
#include "corealgorithm.h"

//using namespace cv;
#define NO_OBJECT 0
//#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ELEM(img, r, c) (CV_IMAGE_ELEM(img, unsigned char, r, c))
#define ONETWO(L, r, c, col) (L[(r) * (col) + c])

void CoreAlgorithm::ZhangCalibrationMethod_CPP( const CalibrationData& cData,CamPara& campara )
{
    if(cData.frameNumList.size()==0)
        return;

    int ImgHeight = cData.imgHeight;
    int ImgWidth = cData.imgWidth;
    int imgAmount = cData.frameNumList.size();
    vector< vector<Point3f> > object_points = cData.plane3dPntsVec;
    vector< vector<Point2f> > image_points = cData.plane2dPntsVec;

    cv::Size imageSize(ImgWidth,ImgHeight);

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat cam_intr_para(3, 3, CV_64FC1);
    cv::Mat distCoeffs(1, 4, CV_64FC1);

    //cv::calibrateCamera(object_points,image_points,imageSize,cam_intr_para,distCoeffs,rvecs,tvecs);
    float rms_error = cv::calibrateCamera(object_points,image_points,imageSize,cam_intr_para,distCoeffs,rvecs,tvecs);
	//cout <<"畸变:" <<distCoeffs.col(4) << endl;
    cout<<endl<<"rms_error = "<<rms_error<<endl<<endl;

    //相机参数及图片外参数赋值
    for (int i=0; i < 3; i++)
    {
        for (int j=0; j < 3; j++)
        {
            campara.CameraIntrinsic[i][j] = cam_intr_para.at<double>(i,j);
        }
    }
    for (int i = 0; i < 4; i++)
    {
        campara.DistortionCoeffs[i] = distCoeffs.at<double>(0,i);
    }
    campara.imgRTVec.clear();
    for(int i=0;i<rvecs.size();i++)
    {
        RT rt;
        rt.R = rvecs[i];
        rt.T = tvecs[i];
        campara.imgRTVec.push_back(rt);
    }

    vector< vector<Point2f> > error_image_points;
    Mat outpara_covariance(6,6,CV_64FC1);
    Mat inpara_covariance(8,8,CV_64FC1);
    Mat inpara_outpara_covariance(8,6,CV_64FC1);
    //covariance的前8个位置用来存放内参数协方差
    Mat covariance = Mat::zeros(8+6*imgAmount,8+6*imgAmount,CV_64FC1);

    for( int i=0; i<imgAmount;i++ )
    {
        int cornerNum = image_points[i].size();
        vector<Point2f> perframe_imagepnt;
        vector<Point3f> perframe_objectpnt = object_points[i];
        Mat rot_vector_perframe(3,1,CV_64FC1);
        Mat tra_vector_perframe(3,1,CV_64FC1);
        tra_vector_perframe = campara.imgRTVec[i].T;
        rot_vector_perframe = campara.imgRTVec[i].R;

        Mat dpoutpara_perframe(2*cornerNum,6,CV_64FC1);
        Mat dpinpara_perframe(2*cornerNum,8,CV_64FC1);

        //计算重投影误差和导数？
        //矩阵大小 2Nx(10+<numDistCoeffs>)，这里numDistCoeffs=4
        //所以大小为 2N x 14，按照drot dt df dc ddist的顺序进行排列
        //在c接口中，雅克比矩阵被放在几个分开的部分中
        Mat jacobian;
        projectPoints(perframe_objectpnt
                      ,rot_vector_perframe,tra_vector_perframe
                      ,cam_intr_para,distCoeffs,
                      perframe_imagepnt,jacobian);
        //将重投影得到的计算角点保存起来
        error_image_points.push_back(perframe_imagepnt);

        //将drot dt放到doutpara中
        for( int j=0;j<6;j++ )
        {
            jacobian.col(j).copyTo(dpoutpara_perframe.col(j));
        }
        //将df dc ddist合并到dpinpara中
        for( int j=0;j<8;j++ )
        {
            jacobian.col(j+6).copyTo(dpinpara_perframe.col(j));
        }

        //求协方差矩阵
        // outpara_covariance[6X6] = dpoutpara_perframe(2N*6  转置)＊dpoutpara_perframe(2N*6)
        mulTransposed(dpoutpara_perframe, outpara_covariance, 1);
        // inpara_covariance[8X8] = dpinpara_perframe(2N*8  转置)＊dpinpara_perframe(2N*8)
        mulTransposed(dpinpara_perframe, inpara_covariance, 1);
        // inpara_outpara_covariance[8X6] = dpinpara_perframe[2NX8 转置]＊dpoutpara_perframe(2N*6)
        gemm(dpinpara_perframe,dpoutpara_perframe,1,0,0,inpara_outpara_covariance,CV_GEMM_A_T);

        //更新本帧的协方差矩阵
        for( int row=0;row<8;row++ )
        {
            for( int col=0;col<8;col++ )
            {
                //内参数的协方差不断叠加
                covariance.at<double>(row,col) += inpara_covariance.at<double>(row,col);
            }
        }
        for (int row = 0; row < 8; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                //填充inpara_outpara_covariance转置，前8个位置除外
                covariance.at<double>( row , col+8+i*6) = inpara_outpara_covariance.at<double>(row,col);
                covariance.at<double>( col+8+i*6, row) = inpara_outpara_covariance.at<double>(row,col);
            }
        }
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                //在剩下的位置填充outpara_covariance
                covariance.at<double>( row+8+i*6 , col+8+i*6 ) = outpara_covariance.at<double>(row,col);
            }
        }
    }
    //完成for循环以后，将初始角点和计算角点相减，获得差值
    vector<Point2f> subDst;
    vector<double>	absErr;
    int totalPoints=0;
    double totalError=0;
    for( int i=0;i<image_points.size();i++ )
    {
        vector<Point2f> temp;
        subtract(error_image_points[i],image_points[i],temp);
        //test by zhaorui
        //        for(int z=0;z<temp.size();z++)
        //        {
        //            cout<<temp[z].x<<" "<<temp[z].y<<endl;
        //        }
        //end test
        copy(temp.begin(),temp.end(),back_inserter(subDst));
        double err;
        int n = image_points[i].size();
        err = norm(error_image_points[i],image_points[i],CV_L2);
        absErr.push_back((float)sqrt(err*err/n));
        totalError += err*err;
        totalPoints += n;
    }
    // totalError /= totalPoints;
    totalError = sqrt(totalError/totalPoints);

    Scalar xmean,ymean;
    Scalar xstd_dev,ystd_dev;
    Scalar errormean,errorstd_dev;
#ifdef DEBUG
    //计算所有点的误差平均值和标准差
    for (int i = 0; i < subDst.size(); i++)
    {
        //qDebug() << subDst[i].x << " " <<subDst[i].y;
        //std::cerr << subDst[i].x << " " <<subDst[i].y;
    }
#endif // DEBUG
    meanStdDev(subDst,errormean,errorstd_dev);

    campara.ReprojectionError[0] = errorstd_dev[0];
    campara.ReprojectionError[1] = errorstd_dev[1];
    campara.totalReproNormErr = totalError;
    campara.reprojectNormErr = absErr;
    //计算标定参数误差，3倍标准差，这里是怎么算的？
    vector<double> para_error;
    Mat inver_converiance(covariance.rows,covariance.cols,CV_64FC1);
    invert(covariance,inver_converiance);
    Mat diag_covariance(inver_converiance.rows,1,CV_64FC1);
    diag_covariance = inver_converiance.diag(0);//取主对角线

    for( int row=0;row<diag_covariance.rows;row++ )
    {
        //为什么只取val[0]???
        para_error.push_back( 3*errorstd_dev.val[0]*sqrt(abs(diag_covariance.at<double>(row,0))));

    }
    campara.fcError[0]=para_error[0];
    campara.fcError[1]=para_error[1];
    campara.ccError[0]=para_error[2];
    campara.ccError[1]=para_error[3];
    for(int i=0;i<4;i++)
        campara.kcError[i]=para_error[i+4];
}

bool CoreAlgorithm::isBelong2Vec( vector<int> vec,int index )
{
    for(int i=0;i<vec.size();i++)
    {
        if( index == vec[i])
            return true;
    }
    return false;
}

void CoreAlgorithm::stereoCalibration(const CamPara& camPara1,const CamPara& camPara2,
                                      const CalibrationData& cData1,const CalibrationData& cData2,RT& rt)
{
    Mat intrinsicPara_left(3,3,CV_64FC1);
    Mat intrinsicPara_right(3,3,CV_64FC1);
    Mat distCoeffs_left(1,4,CV_64FC1);
    Mat distCoeffs_right(1,4,CV_64FC1);

    for( int i=0; i<3;i++ )
    {
        for( int j=0;j<3;j++ )
        {
            intrinsicPara_left.at<double>(i,j) = camPara1.CameraIntrinsic[i][j];
            intrinsicPara_right.at<double>(i,j) = camPara2.CameraIntrinsic[i][j];
        }
    }
    for( int i=0; i<4;i++ )
    {
        distCoeffs_left.at<double>(0,i) =  camPara1.DistortionCoeffs[i];
        distCoeffs_right.at<double>(0,i) = camPara2.DistortionCoeffs[i];
    }

    Mat R,T,E,F,R_vec;
    //    //opencv2.44
    //    stereoCalibrate(cData1.plane3dPntsVec,cData1.plane2dPntsVec,cData2.plane2dPntsVec,
    //        intrinsicPara_left,distCoeffs_left,	intrinsicPara_right,distCoeffs_right,
    //        Size(cData1.imgWidth,cData1.imgHeight),R,T,E,F,
    //        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
    //        CV_CALIB_FIX_INTRINSIC +
    //        CV_CALIB_FIX_ASPECT_RATIO +
    //        CV_CALIB_FIX_FOCAL_LENGTH +
    //        CV_CALIB_FIX_PRINCIPAL_POINT +
    //        CV_CALIB_ZERO_TANGENT_DIST +
    //        CV_CALIB_SAME_FOCAL_LENGTH*/
    //        CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    //opencv3.0
    stereoCalibrate(cData1.plane3dPntsVec,cData1.plane2dPntsVec,cData2.plane2dPntsVec,
                    intrinsicPara_left,distCoeffs_left,	intrinsicPara_right,distCoeffs_right,
                    Size(cData1.imgWidth,cData1.imgHeight),R,T,E,F,cv::CALIB_FIX_INTRINSIC +
					cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

    Rodrigues(R,R_vec);

    R_vec.copyTo(rt.R);
    T.copyTo(rt.T);
}



void CoreAlgorithm::removeRepeatPoints( vector<Point2f>& points )
{
    vector<Point2f> returnVec;
    for(int i=0;i<points.size();i++)
    {
        for (int j=i+1;j<points.size();j++)
        {
            if( (points[i].x-points[j].x)<1 && (points[i].y-points[j].y)<1 )
            {
                returnVec.push_back(Point2f((points[i].x+points[j].x)/2,(points[i].y+points[j].y)/2));
            }
        }
    }
    points = returnVec;
}


void CoreAlgorithm::detectLightSpot_LED( const Mat& imgMat,vector<Point2f>& centerPnt,vector<float>& longaxisRadius)
{
    float width_height_ratio=4;
    Mat img_threshold;
    //STEP-1:二值化
    //blur(imgMat,img_threshold,Size(8,8),Point(-1,-1));
    double minvalue,maxvalue;
    cv::minMaxLoc(imgMat,&minvalue,&maxvalue);
    //step-3 高置信梯度区域的选择
    //step-3-1 针对求出的梯度矩阵进行二值化
    int thresholdvalue = (int)(minvalue+maxvalue)/2;
	threshold(imgMat, img_threshold, thresholdvalue, 255, cv::THRESH_BINARY);
    //namedWindow("window",2);
    //imshow("window",img_threshold);
    //waitKey();
    //在处理佳能相机里当感光度不同时，设置的阈值也不同。ISO1600为50合适。
    ///// 自适应二值化
    /* int block_size = 145;
     cv::Scalar meanvalue = cv::mean(img_threshold);
     int C_threshold = int(-1*(abs(2*meanvalue.val[0])+13));
     adaptiveThreshold(img_threshold,img_threshold,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,block_size,C_threshold);*/
    //CV_ADAPTIVE_THRESH_MEAN_C:使用3x3方格中像素的平均值减去5来作为自适应的阈值
    //对白色区域膨胀再腐蚀
	int close_type = cv::MORPH_ELLIPSE;
    int dilate_size = 1;
    Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
    //dilate(img_threshold, img_threshold, element,Point(-1, -1),1);//闭运算
    /// test

    ///// end test
    erode(img_threshold, img_threshold, element,Point(-1, -1),1);
    dilate(img_threshold, img_threshold, element,Point(-1, -1),1);//开运算
    //STEP-2：寻找轮廓
    vector<vector<cv::Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_threshold,contours,hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    //STEP-3:椭圆拟合,找中心,并进行筛选
    RotatedRect rotatedBox;
    //vector<RotatedRect> rotatedBoxVec;
    //vector<RotatedRect> temp_rotatedBoxVec;
    for( vector<vector<Point>>::iterator itr=contours.begin();itr!=contours.end();++itr)
    {
        static int n=0;
        //根据圆特征的封闭性进行滤波操作
        int distanceToHead = abs(itr->at(itr->size()-1).x+itr->at(itr->size()-1).y-itr->at(0).x-itr->at(0).y);
        if(itr->size()<10||itr->size()>500||distanceToHead>4)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值
            //if(itr->size()<10||itr->size()>500)//此处需要根据工作最远离和工作最近距离以及圆斑真实大小确定其阈值

            continue;

        try
        {
            rotatedBox = fitEllipse((*itr));
        }
        catch (...)//三个点表示任何异常
        {
            continue;
        }
        //temp_rotatedBoxVec.push_back(rotatedBox);
        float height = rotatedBox.size.height;
        float width = rotatedBox.size.width;
        //根据轮廓长度判断其是否为椭圆
        double Ellipselength,ratiolengthsize;
        double PI = 3.1415926;
        if(height > width)
        {
            Ellipselength =PI*width+2*(height-width);
        }
        else
        {
            Ellipselength =PI*height+2*(width-height);
        }
        ratiolengthsize = Ellipselength/itr->size();
        n++;
        //如果光斑点的形状不规则的话，这个比例可能要大一点
        if( (height > width ? (height/width<width_height_ratio) : (width/height<width_height_ratio))&&(0.9<ratiolengthsize)&&(ratiolengthsize<1.3)&&height>5&&width>5)
        {
            //rotatedBoxVec.push_back(rotatedBox);
            centerPnt.push_back(rotatedBox.center);
            if(height > width)
            {
                longaxisRadius.push_back(height/2);
            }
            else
            {
                longaxisRadius.push_back(width/2);
            }
        }

    }
    return;
    //STEP-4 对检测出的中心点进行过滤和剔除


}

bool CoreAlgorithm::Cal3dPointStrLight(const vector<Point2f>& pointCam,const vector<float>& proImg_x,const CamPara& camParaLeft, const CamPara& camParapro,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d)
{
    point3d.clear();
    //// 正则化　摄像机图像坐标
    vector<Point2f> NormPixCam;
    double CAMfc[2], CAMcc[2], CAMkc[5], CAMalfpha_c;
    CAMfc[0] = camParaLeft.CameraIntrinsic[0][0];
    CAMfc[1] = camParaLeft.CameraIntrinsic[1][1];
    CAMcc[0] = camParaLeft.CameraIntrinsic[0][2];
    CAMcc[1] = camParaLeft.CameraIntrinsic[1][2];
    CAMalfpha_c = camParaLeft.CameraIntrinsic[0][1];
    for(int i=0;i<4;i++)
    {
        CAMkc[i] = camParaLeft.DistortionCoeffs[i];
    }
    CAMkc[4] = 0;
    normalizPixel(pointCam,NormPixCam,CAMfc, CAMcc, CAMkc, CAMalfpha_c);

    Mat Kp(3,3,CV_32FC1);
    Kp.at<float>(0,0) = camParapro.CameraIntrinsic[0][0];	//// 注意赋值的时候 at 坐标先y，后x
    Kp.at<float>(1,0) = camParapro.CameraIntrinsic[1][0];
    Kp.at<float>(2,0) = camParapro.CameraIntrinsic[2][0];
    Kp.at<float>(0,1) = camParapro.CameraIntrinsic[0][1];
    Kp.at<float>(1,1) = camParapro.CameraIntrinsic[1][1];
    Kp.at<float>(2,1) = camParapro.CameraIntrinsic[2][1];
    Kp.at<float>(0,2) = camParapro.CameraIntrinsic[0][2];
    Kp.at<float>(1,2) = camParapro.CameraIntrinsic[1][2];
    Kp.at<float>(2,2) = camParapro.CameraIntrinsic[2][2];

    //获得Mat类型的R、T
    Mat R(3,3,CV_32FC1);
    Mat Rvec(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
    {
        Rvec.at<float>(i,0) = rotVector[i];
    }
    Rodrigues(Rvec,R);

    Mat T(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
    {
        T.at<float>(i,0) = traVector[i];
    }

    Mat P(3,3,CV_32FC1);
    P = Kp*R;

    Mat Kt(3,1,CV_32FC1);
    //// 赋值平移向量
    for(int i=0;i<3;i++)
    {
        Kt.at<float>(i,0) = traVector[i];
    }
    Kt = Kp*Kt; /// 构造出P矩阵的第四列


    ///// 计算三维坐标
    for (unsigned int i=0;i<NormPixCam.size();i++)
    {
        /// 给定系数矩阵，保存在Kp　　３Ｘ３;给定等式右边向量　保存在Rvec　３Ｘ１
        float temp = proImg_x[i] * Kt.at<float>(2,0)-Kt.at<float>(0,0);

        Kp.at<float>(0,0) = 1; Kp.at<float>(0,1) = 0; Kp.at<float>(0,2) = -NormPixCam[i].x;
        Kp.at<float>(1,0) = 0; Kp.at<float>(1,1) = 1; Kp.at<float>(1,2) = -NormPixCam[i].y;

        Kp.at<float>(2,0) = (P.at<float>(0,0) - proImg_x[i] * P.at<float>(2,0))/temp;
        Kp.at<float>(2,1) = (P.at<float>(0,1) - proImg_x[i] * P.at<float>(2,1))/temp;
        Kp.at<float>(2,2) = (P.at<float>(0,2) - proImg_x[i] * P.at<float>(2,2))/temp;

        Rvec.at<float>(0,0) = 0;
        Rvec.at<float>(1,0) = 0;
        Rvec.at<float>(2,0) = 1;

        Mat inverA(3,3,CV_32FC1);
        Mat point(3,1,CV_32FC1);

        invert(Kp,inverA,cv::DECOMP_SVD);//求逆矩阵

        point = inverA*Rvec;
        Point3f pnt;
        if (point.at<float>(2,0)<0)
        {
            pnt.x = -point.at<float>(0,0);
            pnt.y = -point.at<float>(1,0);
            pnt.z = -point.at<float>(2,0);
        }
        else
        {
            pnt.x = point.at<float>(0,0);
            pnt.y = point.at<float>(1,0);
            pnt.z = point.at<float>(2,0);
        }
        point3d.push_back(pnt);
    }
    return true;
}

bool CoreAlgorithm::Cal3dPoint( const vector<Point2f> pointLeft
                                ,const CamPara& camParaLeft
                                ,const vector<Point2f> pointRight
                                ,const CamPara& camParaRight
                                ,const double rotVector[3]
,const double traVector[3]
,vector<Point3f>& point3d )
{
    if (pointLeft.size() != pointRight.size())
        return false;

    point3d.clear();
    vector<Point2f> NormPixLeft;
    vector<Point2f> NormPixRight;

    double Rfc[2], Rcc[2], Rkc[5], Ralfpha_c=0;
    double Lfc[2], Lcc[2], Lkc[5], Lalfpha_c=0;
    double L2RRotVector[3];
    double L2RTraVector[3];

    for(int i=0;i<3;i++)
    {
        L2RRotVector[i] = rotVector[i];
        L2RTraVector[i] = traVector[i];
    }

    Rfc[0] = camParaRight.CameraIntrinsic[0][0];
    Rfc[1] = camParaRight.CameraIntrinsic[1][1];
    Rcc[0] = camParaRight.CameraIntrinsic[0][2];
    Rcc[1] = camParaRight.CameraIntrinsic[1][2];
    for(int i=0;i<4;i++)
    {
        Rkc[i] = camParaRight.DistortionCoeffs[i];
    }
    Rkc[4] = 0;

    Lfc[0] = camParaLeft.CameraIntrinsic[0][0];
    Lfc[1] = camParaLeft.CameraIntrinsic[1][1];
    Lcc[0] = camParaLeft.CameraIntrinsic[0][2];
    Lcc[1] = camParaLeft.CameraIntrinsic[1][2];
    for(int i=0;i<4;i++)
    {
        Lkc[i] = camParaLeft.DistortionCoeffs[i];
    }
    Lkc[4] = 0;

    //// 正则化　摄像机图像坐标
    normalizPixel(pointLeft,NormPixLeft,Lfc, Lcc, Lkc, Lalfpha_c);
    //// 正则化　投影机图像坐标
    normalizPixel(pointRight,NormPixRight,Rfc, Rcc, Rkc, Ralfpha_c);

    Mat Kp(3,3,CV_32FC1);

    //获得Mat类型的R、T
    Mat R(3,3,CV_32FC1);
    Mat Rvec(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
    {
        Rvec.at<float>(i,0) = rotVector[i];
    }
    Rodrigues(Rvec,R);

    Mat T(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
    {
        T.at<float>(i,0) = traVector[i];
    }

    //计算三维坐标,使用right相机的x坐标和y坐标
    for (int i=0;i<NormPixLeft.size();i++)
    {
        //给定系数矩阵，保存在Kp　３Ｘ３
        float temp = NormPixRight[i].x * T.at<float>(2,0) - T.at<float>(0,0);//这个值是什么含义？

        Kp.at<float>(0,0) = 1; Kp.at<float>(0,1) = 0; Kp.at<float>(0,2) = -NormPixLeft[i].x;
        Kp.at<float>(1,0) = 0; Kp.at<float>(1,1) = 1; Kp.at<float>(1,2) = -NormPixLeft[i].y;

        Kp.at<float>(2,0) = (R.at<float>(0,0) - NormPixRight[i].x * R.at<float>(2,0))/temp;
        Kp.at<float>(2,1) = (R.at<float>(0,1) - NormPixRight[i].x * R.at<float>(2,1))/temp;
        Kp.at<float>(2,2) = (R.at<float>(0,2) - NormPixRight[i].x * R.at<float>(2,2))/temp;

        Rvec.at<float>(2,0) = 1;

        Mat inverA(3,3,CV_32FC1);
        Mat point(3,1,CV_32FC1);

        invert(Kp,inverA);//求逆矩阵

        inverA.col(2).copyTo(point.col(0));
        Point3f pnt;
        if (point.at<float>(2,0)<0)
        {
            pnt.x = -point.at<float>(0,0);
            pnt.y = -point.at<float>(1,0);
            pnt.z = -point.at<float>(2,0);
        }
        else
        {
            pnt.x = point.at<float>(0,0);
            pnt.y = point.at<float>(1,0);
            pnt.z = point.at<float>(2,0);
        }
        point3d.push_back(pnt);
    }
    return true;
}

bool CoreAlgorithm::normalizPixel( const vector<Point2f>& src
                                   ,vector<Point2f>& dst
                                   ,double fc[2]
,double cc[2]
,double kc[5]
,double alfpha_c )
{
    dst.resize(src.size());
    for (unsigned int i=0;i<src.size();i++)
    {
        //cwq图像像素坐标到图像物理坐标的转化
        dst[i].x = (src[i].x-cc[0])/fc[0];
        dst[i].y = (src[i].y-cc[1])/fc[1];
        dst[i].x = dst[i].x - alfpha_c/fc[0]*dst[i].y;//dst[i].x = dst[i].x - alfpha_c*dst[i].y;
    }
    vector<Point2f> temp;
    temp = dst;
    double norm2 = kc[0]*kc[0] + kc[1]*kc[1] + kc[2]*kc[2]+ kc[3]*kc[3]+ kc[4]*kc[4];
    if (norm2>0)
    {
        double r2,k_radial,delta_x,delta_y;
        for (unsigned int i=0;i<dst.size();i++)
        {
            for (int j=0;j<400;j++)///迭代２０次求解非畸变量
            {
                r2 = temp[i].x * temp[i].x + temp[i].y * temp[i].y;
                //径向
                k_radial = 1 + kc[0]*r2 + kc[1]*r2*r2 + kc[4]*r2*r2*r2;
                //切向
                delta_x = 2*kc[2]*temp[i].x*temp[i].y + kc[3]*(r2+2*temp[i].x*temp[i].x);
                delta_y = kc[2]*( r2 + 2*temp[i].y*temp[i].y ) + 2*kc[3]*temp[i].x*temp[i].y;
                //畸变校正公式
                temp[i].x = (dst[i].x-delta_x)/k_radial;
                temp[i].y = (dst[i].y-delta_y)/k_radial;
            }
        }
        dst = temp;
    }
    return true;
}

vector<Point2f> CoreAlgorithm::extraCorner(const Mat& img,const vector<Point2f>& mousePnts,const int r,const int c)
{
    vector<Point2f> ImageCorner;

    Point2f imgpntlefttop = mousePnts[0];
    Point2f imgpntrighttop = mousePnts[1];
    Point2f imgpntrightdown = mousePnts[2];
    Point2f imgpntleftdown = mousePnts[3];

    Point2f rowLength,colLength_l,colLength_r;
    colLength_l.x = imgpntleftdown.x-imgpntlefttop.x;
    colLength_l.y = imgpntleftdown.y-imgpntlefttop.y;
    colLength_r.x = imgpntrightdown.x-imgpntrighttop.x;
    colLength_r.y = imgpntrightdown.y-imgpntrighttop.y;
    Point2f tempPnt,leftPnt,rightPnt;

    tempPnt = imgpntlefttop;//cwq标注，tempPnt用来保存要push的点
    leftPnt = imgpntlefttop;
    rightPnt = imgpntrighttop;

    vector<Point2f> col_Pnt;// 行角点
    for (int row=1;row<r+1;row=row+1)
    {
        //col_Pnt.clear();
        for (int col=1;col<c+1;col=col+1)
        {
            //保存该点到行
            //col_Pnt.push_back(tempPnt);
            ImageCorner.push_back(tempPnt);
            //cwq标注，计算边长的x分量和y分量
            rowLength.x = rightPnt.x-leftPnt.x;
            rowLength.y = rightPnt.y-leftPnt.y;
            //cwq标注，行的初始点根据列数加上点间隔
            tempPnt.x = int ((float)col/(float)(c-1)*(float)rowLength.x)+leftPnt.x;
            tempPnt.y = int ((float)col/(float)(c-1)*(float)rowLength.y)+leftPnt.y;
        }
        //ImageCorner.push_back(col_Pnt);///把整行保存
        if (row<r-1)//如果不是最后一行
        {
            //cwq标注，移动行的左右端点到下一行
            leftPnt.x = int ((float)row/(float)(r-1)*(float)colLength_l.x)+imgpntlefttop.x;
            leftPnt.y = int ((float)row/(float)(r-1)*(float)colLength_l.y)+imgpntlefttop.y;
            rightPnt.x = int ((float)row/(float)(r-1)*(float)colLength_r.x)+imgpntrighttop.x;
            rightPnt.y = int ((float)row/(float)(r-1)*(float)colLength_r.y)+imgpntrighttop.y;
        }
        else//cwq标注，如果是最后一行
        {
            leftPnt = imgpntleftdown;
            rightPnt = imgpntrightdown;
        }

        tempPnt = leftPnt;
    }
    //*********************************************************************************//

    //亚像素角点提取
    Mat matImg_gray;
    cvtColor(img, matImg_gray, CV_BGR2GRAY);

    cornerSubPix(matImg_gray,ImageCorner, cvSize(12, 12), cvSize(-1, -1)
                 ,cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 60, 0.01));

    return ImageCorner;
}

bool CoreAlgorithm::extraCorner2(const Mat& img,const vector<Point2f>& mousePnts,vector<Point2f>& ImageCorner,int &row, int &col)
{
    Point2f imgpntlefttop = mousePnts[0];
    Point2f imgpntrighttop = mousePnts[1];
    Point2f imgpntrightdown = mousePnts[2];
    Point2f imgpntleftdown = mousePnts[3];

    double dircol,dirrow;
    double tranXY1[2],tranXY2[2];

    double lengthrow,lengthcol_l,lengthcol_r;
    lengthrow = sqrtf((imgpntrighttop.x - imgpntlefttop.x) * (imgpntrighttop.x - imgpntlefttop.x) + (imgpntrighttop.y - imgpntlefttop.y) * (imgpntrighttop.y - imgpntlefttop.y));
    lengthcol_l = sqrtf((imgpntleftdown.x - imgpntlefttop.x) * (imgpntleftdown.x - imgpntlefttop.x) + (imgpntleftdown.y - imgpntlefttop.y) * (imgpntleftdown.y - imgpntlefttop.y));
    lengthcol_r = sqrtf((imgpntrightdown.x - imgpntrighttop.x) * (imgpntrightdown.x - imgpntrighttop.x) + (imgpntrightdown.y - imgpntrighttop.y) * (imgpntrightdown.y - imgpntrighttop.y));

    Mat ImageGray;
    cvtColor(img, ImageGray, CV_BGR2GRAY);

    row = 0;
    col = 0;
    vector<Point2f> cornercol_l,conercol_r;

    if ( !(GetCornerAtRow(ImageGray,imgpntlefttop, imgpntleftdown, cornercol_l)
           && GetCornerAtRow(ImageGray,imgpntrighttop, imgpntrightdown, conercol_r)) )
    {
        return false;
    }

    if (cornercol_l.size() == conercol_r.size())
    {
        row = (int)cornercol_l.size();    // 获得角点行方向个数
    }
    else
    {
        return false;
    }

    for (int i = 0; i < row; i++)
    {
        Point2f templ,tempr;
        templ = cornercol_l[i];
        tempr = conercol_r[i];
        vector<Point2f> cornerrow;
        GetCornerAtRow(ImageGray,templ, tempr, ImageCorner);
        if (i==0)
            col = ImageCorner.size();				// 获得角点列方向个数
    }

    if (ImageCorner.size() != HEIGHT_CORNER_COUNT*WIDTH_CORNER_COUNT)
        return false;

    cornerSubPix(ImageGray,ImageCorner, cvSize(10, 10), cvSize(-1, -1)
                 ,cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 60, 0.01));

    ////为了与自动提取的焦点顺序相符，翻转corner的顺序
    //vector<Point2f> dst;
    //for (int i = ImageCorner.size()-1; i >=0 ;i--)
    //{
    //	dst.push_back(ImageCorner[i]);
    //}
    //ImageCorner = dst;
    return true;
}

bool CoreAlgorithm::GetCornerAtRow(const Mat& ImageGray, Point2f imgpnt_left, Point2f imgpnt_right, vector<Point2f> &cornerrow)
{
    double threhold;    // 自适应确定阈值，根据前两个格子确定
    bool signwb;
    int height = ImageGray.rows;
    int width = ImageGray.cols;
    int step = ImageGray.step;
    int chennels = ImageGray.channels();
    uchar* ImageGray_data = (uchar*)ImageGray.data;
    double pixel;

    double dir[2],length;
    dir[0] = imgpnt_right.x - imgpnt_left.x;
    dir[1] = imgpnt_right.y - imgpnt_left.y;
    length = sqrtf(dir[0] * dir[0] + dir[1] * dir[1]);

    //cwq防止两点重合
    if( 0 == length)
    {
        return false;
    }
    dir[0] = dir[0] / length;//cwq  sin\cos
    dir[1] = dir[1] / length;
    // 设定初始标志
    pixel = 0;
    //cwq 循环获得25个像素值，用来求平均阈值
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j > -5; j--)
        {
            // 在dir垂直的方向上加3个，所以反过来了。另外，图像上step相当于y
            //3/22 cwq检查是超出图像边缘
            if( (imgpnt_left.y - 10 * dir[1] + 10 * dir[0] + j) < 0  || (imgpnt_left.x - 10 * dir[0] - 10 * dir[1] + i ) < 0 )
            {
                return false;
            }
            pixel = pixel + ImageGray_data[(int)(imgpnt_left.y - 10 * dir[1] + 10 * dir[0] + j) * step + (int)(imgpnt_left.x - 10 * dir[0] - 10 * dir[1] + i) * chennels];
        }
    }
    double pretopleft = pixel / 25;
    pixel = 0;
    for (int i = 0;i < 5; i++)
    {
        for (int j = 0;j < 5; j++)
        {
            // 在dir垂直的方向上加3个，所以反过来了。另外，图像上step相当于y
            pixel = pixel + ImageGray_data[(int)(imgpnt_left.y + 10 * dir[1] + 10 * dir[0] + j) * step + (int)(imgpnt_left.x + 10 * dir[0] - 10 * dir[1] + i) * chennels];
        }
    }
    double aftertopleft = pixel / 25;
    threhold = (pretopleft + aftertopleft) / 2;
    if (pretopleft < threhold)
    {
        signwb = FALSE;
    }
    else
    {
        signwb = TRUE;
    }

    // 寻找角点
    for (int l = 0;l < length + 5; l++)
    {
        Point2f presentpnt;
        //cwq根据sin、cos获得扫描直线上的下一点
        presentpnt.x = imgpnt_left.x + l * dir[0];
        presentpnt.y = imgpnt_left.y + l * dir[1];
        pixel = 0;
        for (int i = 0;i < 5; i++)
        {
            for (int j = 0;j < 5; j++)
            {
                // 在dir垂直的方向上加3个，所以反过来了。另外，图像上step相当于y
                pixel = pixel+ImageGray_data[(int)(presentpnt.y + 10 * dir[0] + j) * step + (int)(presentpnt.x - 10 * dir[1] + i) * chennels];
            }
        }
        if (pixel / 25 < threhold && signwb == TRUE)
        {
            signwb = FALSE;
            cornerrow.push_back(presentpnt);
            // 进行最大值抑制，跳跃一个步长
            l = l + 4;
            continue;
        }
        if (pixel / 25 > threhold && signwb == FALSE)
        {
            signwb = TRUE;
            cornerrow.push_back(presentpnt);
            // 进行最大值抑制，跳跃一个步长
            l = l + 5;
        }
    }

    return TRUE;
}



bool CoreAlgorithm::rigidTransform(const std::vector<Point3f>& oriPoints,
                                   const std::vector<Point3f>& terminatePoints,
                                   cv::Mat& Rot,cv::Mat& Tra)
{
    //检查输入点的数目是否相同,且不为0
    if( oriPoints.size()<3 || terminatePoints.size()<3 )
        return FALSE;
    if( oriPoints.size() != terminatePoints.size() )
        return FALSE;

    //获得点的总数
    int pointNum = oriPoints.size();
    vector<Point3f> X1 = oriPoints;
    vector<Point3f> X2 = terminatePoints;

    //求起始点和终止点的重心
    Point3f C1(0,0,0),C2(0,0,0);
    for( int i=0; i<pointNum; i++ )
    {
        C1 += X1[i];
        C2 += X2[i];
    }
    C1.x = C1.x/pointNum;	C2.x = C2.x/pointNum;
    C1.y = C1.y/pointNum;	C2.y = C2.y/pointNum;
    C1.z = C1.z/pointNum;	C2.z = C2.z/pointNum;

    //原始点减去重心点坐标，获得新的点
    for( int i=0; i<pointNum; i++ )
    {
        X1[i] -= C1;
        X2[i] -= C2;
    }

    //创建N矩阵
    double Sxx, Sxy, Sxz, Syx, Syy, Syz, Szx, Szy, Szz;
    Sxx=Sxy=Sxz=Syx=Syy=Syz=Szx=Szy=Szz=0;

    for( int i=0; i<pointNum; i++ )
    {
        Sxx += X2[i].x * X1[i].x;
        Sxy += X2[i].x * X1[i].y;
        Sxz += X2[i].x * X1[i].z;

        Syx += X2[i].y * X1[i].x;
        Syy += X2[i].y * X1[i].y;
        Syz += X2[i].y * X1[i].z;

        Szx += X2[i].z * X1[i].x;
        Szy += X2[i].z * X1[i].y;
        Szz += X2[i].z * X1[i].z;
    }

    //对N矩阵赋值
    Mat N(4,4,CV_64F);
    N.at<double>(0,0) = Sxx + Syy + Szz;
    N.at<double>(0,1) = Syz - Szy;
    N.at<double>(0,2) = Szx - Sxz;
    N.at<double>(0,3) = Sxy - Syx;

    N.at<double>(1,0) = Syz - Szy;
    N.at<double>(1,1) = Sxx - Syy - Szz;
    N.at<double>(1,2) = Sxy + Syx;
    N.at<double>(1,3) = Szx + Sxz;

    N.at<double>(2,0) = Szx - Sxz;
    N.at<double>(2,1) = Sxy + Syx;
    N.at<double>(2,2) = Syy - Sxx - Szz;
    N.at<double>(2,3) = Syz + Szy;

    N.at<double>(3,0) = Sxy - Syx;
    N.at<double>(3,1) = Szx + Sxz;
    N.at<double>(3,2) = Syz + Szy;
    N.at<double>(3,3) = Szz - Sxx - Syy;

    //计算N矩阵的特征向量和特征值
    vector<double> eigenvalues;
    Mat eigenvectors;
    if( !eigen(N,eigenvalues,eigenvectors) )
        return FALSE;

    //求最大正特征值对应的特征向量
    double maximal_vaule = -1;
    int maximal_index = 0;
    for(int i=0; i<eigenvalues.size(); i++ )
    {
        if( eigenvalues[i]>maximal_vaule && eigenvalues[i]>0 )
        {
            maximal_vaule = eigenvalues[i];
            maximal_index = i;
        }
    }

    if ( maximal_vaule <= 0 )
    {
        return FALSE;
    }

    //unit quaternion Q
    double Q0,Q1,Q2,Q3;
    Q0 = eigenvectors.at<double>(maximal_index,0);
    Q1 = eigenvectors.at<double>(maximal_index,1);
    Q2 = eigenvectors.at<double>(maximal_index,2);
    Q3 = eigenvectors.at<double>(maximal_index,3);

    //创建旋转矩阵
    Rot.create(3,3,CV_64F);
    Rot.at<double>(0,0) = Q0*Q0 + Q1*Q1 - Q2*Q2 - Q3*Q3;
    Rot.at<double>(0,1) = 2 * (Q1*Q2 - Q0*Q3);
    Rot.at<double>(0,2) = 2 * (Q1*Q3 + Q0*Q2);

    Rot.at<double>(1,0) = 2 * (Q2*Q1 + Q0*Q3);
    Rot.at<double>(1,1) = Q0*Q0 - Q1*Q1 + Q2*Q2 - Q3*Q3 ;
    Rot.at<double>(1,2) = 2 * (Q2*Q3 - Q0*Q1) ;

    Rot.at<double>(2,0) = 2 * (Q3*Q1 - Q0*Q2);
    Rot.at<double>(2,1) = 2 * (Q3*Q2 + Q0*Q1);
    Rot.at<double>(2,2) = Q0*Q0 - Q1*Q1 - Q2*Q2 + Q3*Q3;

    //计算平移矩阵
    Mat C1_mat(3,1,CV_64F);
    Mat C2_mat(3,1,CV_64F);
    C1_mat.at<double>(0,0) = C1.x;
    C1_mat.at<double>(1,0) = C1.y;
    C1_mat.at<double>(2,0) = C1.z;

    C2_mat.at<double>(0,0) = C2.x;
    C2_mat.at<double>(1,0) = C2.y;
    C2_mat.at<double>(2,0) = C2.z;

    transpose(Rot,Rot);
    gemm(Rot,C1_mat,-1,C2_mat,1,Tra); //Tra = C2_mat - Rot*C1_mat

    return TRUE;
}

bool CoreAlgorithm::calculateProbeCenter(const vector<Mat>& R_vec, const vector<Mat>& T_vec,Point3f& center)
{
    if (R_vec.size() != T_vec.size()
            || R_vec.size() == 0)
    {
        return false;
    }

    //src1*X - src2 = min
    Mat src1, src2, dst, R1, T1;
    R1 = R_vec[0];
    T1 = T_vec[0];
    src1 = Mat::zeros(R1.size(),CV_64F);
    src2 = Mat::zeros(T1.size(), CV_64F);

    //P(R2-R1)=T1-T2
    //P(R3-R1)=T1-T3
    //...
    //P(Rn-R1)=T1-Tn
    for (int i = 1; i < R_vec.size(); i++)
    {
        add(src1, R_vec[i], src1);
        add(src2, T_vec[i], src2);
    }

    subtract((T_vec.size() - 1)*T1, src2, src2);
    subtract(src1, (R_vec.size() - 1)*R1, src1);

    solve(src1,src2,dst);

    center = Point3f(dst);

    return true;
}



bool CoreAlgorithm::getPntsWithTag(const vector<Point2f>& L_pnts, const vector<Point2f>& S_pnts,vector<TagPoint2f>& tagPnts)
{
    if (L_pnts.size() < 4 || S_pnts.size()<1)
        return false;

    //STEP-1:长轴点直线拟合
    Vec4f L_Axis;//(xx,xy)前两项为与直线平行的单位向量，(x0,y0)后两项为直线上的一个点
    float xx, xy, x0, y0;
    fitLine(L_pnts, L_Axis, CV_DIST_L2, 0, 0.01, 0.01);//最小二乘拟合
    xx = L_Axis[0]; xy = L_Axis[1]; x0 = L_Axis[2]; y0 = L_Axis[3];

    //STEP-2:求出十字交叉点（虚），求垂足算法网上拷的，还没看懂和测试
    Point2f crossPnt;
    Point2f interceptPnt;//长轴直线与Y轴角点
    interceptPnt.x = 0; interceptPnt.y = y0 - (xy/xx) * x0;//y1=y0-(xy/xx)*x0;

    Point2f pt1, pt2, pt3;//pt1,pt2为直线上两点，pt3为直线外一点
    pt1 = interceptPnt; pt2.x = x0; pt2.y = y0; pt3 = S_pnts[0];
    double dba,dbb;
    //a = sqr( (x2 - x1)^2 +(y2 - y1)^2 )
    dba = sqrt((long double)((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y) ));
    //b = (x2-x1) * (x3-x1) +(y2 -y1) * (y3 -y1)
    dbb = ((pt2.x - pt1.x) * (pt3.x -pt1.x) + (pt2.y - pt1.y) * (pt3.y - pt1.y) );
    //a = b / (a*a)
    dba = dbb / (dba * dba);
    //x4 = x1 +(x2 - x1)*a
    crossPnt.x = pt1.x + (pt2.x - pt1.x) * dba;
    //y4 = y1 +(y2 - y1)*a
    crossPnt.y = pt1.y + (pt2.y - pt1.y) * dba;

    //STEP-3:短轴上的点，在十字交叉点左边就是tag6，右边就是tag7
    for (int i = 0; i < S_pnts.size(); i++)
    {
        TagPoint2f _tagPnt;

        _tagPnt[1] = S_pnts[i].x;
        _tagPnt[2] = S_pnts[i].y;

        if (S_pnts[i].x < crossPnt.x)//在十字交叉左边
            _tagPnt[0] = TAG6;
        else//在十字交叉右边
            _tagPnt[0] = TAG7;

        tagPnts.push_back(_tagPnt);
    }

    //STEP-4:判断长轴中有无十字交叉点（实），有，该点为tag2点
    float dis_tag2Pnt_threshold = 10;//长轴上的某点到垂足的距离小于某值，则认为该点为真的十字交叉点
    for (int i = 0; i < L_pnts.size(); i++)
    {
        float _d = sqrt((crossPnt.x-L_pnts[i].x)*(crossPnt.x-L_pnts[i].x)
                        +(crossPnt.y-L_pnts[i].y)*(crossPnt.y-L_pnts[i].y));
        if (_d < dis_tag2Pnt_threshold)
        {
            TagPoint2f _tagPnt;
            _tagPnt[0] = TAG2;
            _tagPnt[1] = L_pnts[i].x;
            _tagPnt[2] = L_pnts[i].y;

            crossPnt = L_pnts[i];
            tagPnts.push_back(_tagPnt);
            break;
        }
    }

    //STEP-5:长轴在十字交叉点（实或虚）上方的点为tag1，既y值较小
    for (int i = 0; i < L_pnts.size(); i++)
    {
        if (L_pnts[i].y<crossPnt.y)
        {
            TagPoint2f _tagPnt;
            _tagPnt[0] = TAG1;
            _tagPnt[1] = L_pnts[i].x;
            _tagPnt[2] = L_pnts[i].y;
            tagPnts.push_back(_tagPnt);
            break;
        }
    }

    //STEP-7:长轴在十字交叉点（实或需）下方的点，既y值较大，根据距离判断tag345
    float unit_dis;//短轴上某点到十字交叉点的距离
    unit_dis = sqrt((S_pnts[0].x - crossPnt.x)*(S_pnts[0].x - crossPnt.x)
            + (S_pnts[0].y - crossPnt.y)*(S_pnts[0].y - crossPnt.y));
    for (int i = 0; i < L_pnts.size(); i++)
    {
        if (L_pnts[i].y > crossPnt.y)
        {
            TagPoint2f _tagPnt;
            _tagPnt[1] = L_pnts[i].x;
            _tagPnt[2] = L_pnts[i].y;
            float _d = sqrt((L_pnts[i].x - crossPnt.x)*(L_pnts[i].x - crossPnt.x)
                            + (L_pnts[i].y - crossPnt.y)*(L_pnts[i].y - crossPnt.y));
            float _k = _d / unit_dis;

            if (_k>0.5 && _k<1.5)
                _tagPnt[0] = TAG3;
            else if (_k>1.5 && _k < 2.5)
                _tagPnt[0] = TAG4;
            else if (_k>2.5 && _k < 3.8)
                _tagPnt[0] = TAG5;

            tagPnts.push_back(_tagPnt);
        }
    }
    return true;
}

bool CoreAlgorithm::pointsSort2D_3(vector<Point2f>& pnts,vector<Point2f>& L_pnts,vector<Point2f>& S_pnts)
{
    if( pnts.size()<6 )
        return false;
    L_pnts.clear();
    S_pnts.clear();
    float dis_threshold = 8;
    float noisePnt_threshold = 100;
    int pntsSize = pnts.size();
    vector<int> longAxisPntsIndexVec;
    vector<int> noisePntsIndexVec;
    //STEP-1 找出不在长轴上的点,判断剩下的点是否在一条直线上
    for(int i=0;i<pntsSize;i++)
    {
        int count;//落在长轴上的点计数
        for (int j = i + 1; j < pntsSize; j++)
        {
            count = 0;
            noisePntsIndexVec.clear();
            longAxisPntsIndexVec.clear();
            float a, b;//直线公式ax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
            Point2f pnt1 = pnts[i];
            Point2f pnt2 = pnts[j];
            a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
            b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);

            for (int n = 0; n < pnts.size(); n++)
            {
                //点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
                float dis = fabs(a*pnts[n].x + b*pnts[n].y + 1) / sqrt(a*a + b*b);
                if (dis < dis_threshold)//当中心找的不准的时候这个阈值要放宽一点
                {
                    count++; longAxisPntsIndexVec.push_back(n);
                }
                if (dis>noisePnt_threshold)
                    noisePntsIndexVec.push_back(n);
            }
            if (count >= 4)
                break;
        }
        if (count >= 4)
            break;
    }

    if( longAxisPntsIndexVec.size()==0 )
        return false;

    //STEP-2 长轴上的点放到一个vector中;短轴上的点放到一个vector中
    vector<Point2f> longAxisPntsVec,shortAxisPntsVec;
    for(int i=0;i<pnts.size();i++)
    {
        if( isBelong2Vec(longAxisPntsIndexVec,i) )
        {
            longAxisPntsVec.push_back(pnts[i]);
        }
        else
        {
            shortAxisPntsVec.push_back(pnts[i]);
        }
    }

    if( shortAxisPntsVec.size()==0 )
        return false;

    //STEP-3 对长轴上的点按照y坐标从小到大排序,冒泡法
    sortByYMin2Max(longAxisPntsVec);

    //STEP-4 对短轴上点按照x坐标从小到大排序，冒泡法
    sortByXMin2Max(shortAxisPntsVec);

    //STEP-5 判断排好序的长轴上的首末点，距离短轴中的第一点哪一个近
    //光笔颠倒标志位,
    bool isUpSideDown = false;
    int longAxisSize = longAxisPntsVec.size() -1;
    //y最小点到短轴中点距离
    int dis_minY = sqrt((shortAxisPntsVec[0].x - longAxisPntsVec[0].x)*(shortAxisPntsVec[0].x - longAxisPntsVec[0].x)+
            (shortAxisPntsVec[0].y - longAxisPntsVec[0].y)*(shortAxisPntsVec[0].y - longAxisPntsVec[0].y));
    //y最大点到短轴中点距离
    int dis_maxY = sqrt((shortAxisPntsVec[0].x - longAxisPntsVec[longAxisSize].x)*(shortAxisPntsVec[0].x - longAxisPntsVec[longAxisSize].x)+
            (shortAxisPntsVec[0].y - longAxisPntsVec[longAxisSize].y)*(shortAxisPntsVec[0].y - longAxisPntsVec[longAxisSize].y));
    //y坐标最小的那个点距离短轴中点较远的话则认为光笔颠倒了
    if( dis_minY > dis_maxY )
        isUpSideDown = true;

    //STEP-6 如果光笔颠倒了则对短轴和长轴均重新排序
    if( isUpSideDown )
    {
        //将长轴上的点次序颠倒
        for(int i=0;i<longAxisPntsVec.size()-1;i++)
        {
            for(int j=0;j<longAxisPntsVec.size()-1;j++)
            {
                if( longAxisPntsVec[j].y<longAxisPntsVec[j+1].y )
                {
                    swap(longAxisPntsVec[j],longAxisPntsVec[j+1]);
                }
            }
        }

        for(int i=0;i<shortAxisPntsVec.size()-1;i++)
        {
            for (int j = 0; j<shortAxisPntsVec.size() - 1; j++)
            {
                if (shortAxisPntsVec[j].x<shortAxisPntsVec[j + 1].x)
                {
                    swap(shortAxisPntsVec[j], shortAxisPntsVec[j + 1]);
                }
            }
        }
    }

    //STEP-7 将排好序的点输出
    pnts.clear();
    L_pnts = longAxisPntsVec; S_pnts = shortAxisPntsVec;
    for(int i=0;i<longAxisPntsVec.size();i++)
    {
        pnts.push_back(longAxisPntsVec[i]);
    }
    for(int i=0;i<shortAxisPntsVec.size();i++)
    {
        pnts.push_back(shortAxisPntsVec[i]);
    }

    return true;
}

void CoreAlgorithm::createLightPenCoordinate(vector<TagPoint3f>& tagPnts, const Point3f& probeCenter)
{
    vector<Point3f> longAxisPnts,shortAxisPnts;
    for (int i = 0; i < tagPnts.size();i++)
    {
        if (tagPnts[i][0] <= TAG5)
            longAxisPnts.push_back(Point3f(tagPnts[i][1],tagPnts[i][2],tagPnts[i][3]));
        else
            shortAxisPnts.push_back(Point3f(tagPnts[i][1],tagPnts[i][2],tagPnts[i][3]));
    }

    //STEP-1:长轴点的点，按照5点到1点方向进行直线拟合，获得Z轴单位向量
    Vec6f Z_Line;
    fitLine(longAxisPnts, Z_Line, CV_DIST_L2, 0, 0.01, 0.01);//Z轴的方向
    Vec3f norm_Z;
    norm_Z[0] = Z_Line[0];norm_Z[1] = Z_Line[1];norm_Z[2] = Z_Line[2];
    float a, b, c;
    a = Z_Line[0];b = Z_Line[1];c = Z_Line[2];

    //STEP-2:求经过短轴上的点6，且垂直于Z轴的单位向量，获得Y轴单位向量
    Vec3f norm_Y;
    Point3f perpendicularFoot;

    Point3f pt0, pt1;//pt0为直线上一点，pt1为直线外一点
    pt0 = Point3f(Z_Line[0],Z_Line[1],Z_Line[2]);
    pt1 = shortAxisPnts[0];
    float k, u0, u1, u2;
    u1 = a*pt1.x + b*pt1.y + c*pt1.z;
    u2 = a*pt0.x + b*pt0.y + c*pt0.z;
    u0 = a*a + b*b + c*c;
    k = (u1 - u2) / u0;

    perpendicularFoot.x = k*a + pt0.x;
    perpendicularFoot.y = k*b + pt0.y;
    perpendicularFoot.z = k*c + pt0.z;

    float d = sqrt((perpendicularFoot.x - pt1.x)*(perpendicularFoot.x - pt1.x)
                   +(perpendicularFoot.y - pt1.y)*(perpendicularFoot.y - pt1.y)
                   +(perpendicularFoot.z - pt1.z)*(perpendicularFoot.z - pt1.z));
    norm_Y[0] = (perpendicularFoot.x - pt1.x) / d;
    norm_Y[1] = (perpendicularFoot.y - pt1.y) / d;
    norm_Y[2] = (perpendicularFoot.z - pt1.z) / d;


    //STEP-3:Z 叉乘 Y ，获得X轴单位向量
    //向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
    //向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
    Vec3f norm_X;
    norm_X[0] = norm_Z[1] * norm_Y[2] - norm_Z[2] * norm_Y[1];
    norm_X[1] = norm_Z[2] * norm_Y[0] - norm_Z[0] * norm_Y[2];
    norm_X[2] = norm_Z[0] * norm_Y[1] - norm_Z[1] * norm_Y[0];

    //STEP-4:三个单位向量保存为R，测头中心点为T
    Mat T(3, 1, CV_32F); Mat R(3, 3, CV_32F);
    T.at<float>(0, 0) = probeCenter.x;
    T.at<float>(1, 0) = probeCenter.y;
    T.at<float>(2, 0) = probeCenter.z;

    R.at<float>(0, 0) = norm_X[0];R.at<float>(1, 0) = norm_X[1];R.at<float>(2, 0) = norm_X[2];
    R.at<float>(0, 1) = norm_Y[0];R.at<float>(1, 1) = norm_Y[1];R.at<float>(2, 1) = norm_Y[2];
    R.at<float>(0, 2) = norm_Z[0];R.at<float>(1, 2) = norm_Z[1];R.at<float>(2, 2) = norm_Z[2];

    Mat R_invert;
    invert(R, R_invert);
    //STEP-5:将测量坐标系下的点转化到光笔坐标系下：P2 = R*P1 + T
    for (int i = 0; i < tagPnts.size(); i++)
    {
        Mat pnt(3,1,CV_32F);
        pnt.at<float>(0, 0) = tagPnts[i][1];
        pnt.at<float>(1, 0) = tagPnts[i][2];
        pnt.at<float>(2, 0) = tagPnts[i][3];

        pnt = R_invert*(pnt - T);

        tagPnts[i][1] = pnt.at<float>(0, 0);
        tagPnts[i][2] = pnt.at<float>(1, 0);
        tagPnts[i][3] = pnt.at<float>(2, 0);
    }
}

void CoreAlgorithm::iterativeCameraCalibration(const vector<string>& imgsPath, CalibrationData& cData,
                                               const CamPara& camParaSrc, CamPara& camParaDst)
{
    camParaDst = camParaSrc;
    CalibrationData _cData = cData;
    CamPara camPara = camParaSrc;
    generateParallelCamR(camPara.parallelCamR);

    for (int k = 4; k>0;k--)
    {
        Mat cameraMatrix(3, 3, CV_64F);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                cameraMatrix.at<double>(i, j) = camPara.CameraIntrinsic[i][j];
            }
        }

        vector<double> distCoeffs;
        for (int i = 0; i < 4; i++)
            distCoeffs.push_back(camPara.DistortionCoeffs[i]);

        //STEP-1 计算每幅标定板平行视图的R'（并不是旋转矩阵） 3x3
        vector<Mat> parallelImgR;
        calculateParallelViewR(camPara.imgRTVec, parallelImgR);

        //STEP-2 把所有特征点转换到平行视图上
        vector< vector<Point2f> > corner_Parallel;
        for (int i = 0; i < _cData.plane2dPntsVec.size(); i++)  ////   update 11.6  int i = 0; i < cData.plane2dPntsVec.size(); i++
        {
            vector<Point2f> pnts;
            undistortPoints2DifferentView(_cData.plane2dPntsVec[i], pnts,
                                          cameraMatrix,camPara.imgRTVec[i].R,camPara.imgRTVec[i].T,distCoeffs,
                                          cameraMatrix,camPara.parallelCamR,camPara.imgRTVec[i].T,vector<double>());  ////11.6 cData--> _cData
            corner_Parallel.push_back(pnts);
        }

        //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
        for (int i = 0; i < imgsPath.size();i++)  //// just for test (int i = 0; i < imgsPath.size();i++)
        {
            Mat srcImgMat;
            srcImgMat = cv::imread(imgsPath[i],CV_LOAD_IMAGE_GRAYSCALE);
            ////// image is too small
            //            Mat* resultImgMat = new Mat;
            //            resultImgMat->create(2*srcImgMat.rows, 2*srcImgMat.cols, srcImgMat.type());

            Mat resultImgMat = Mat(2*srcImgMat.rows, 2*srcImgMat.cols, srcImgMat.type());
            /////

            //undistortImg(srcImgMat, *resultImgMat, cameraMatrix, distCoeffs, parallelImgR[i]);
            undistortImg(srcImgMat, resultImgMat, cameraMatrix, distCoeffs, parallelImgR[i]);

            /// test
            //            Mat testmat = imread("E:/DLP LightCrafter 4500/English Paper/experiment data/5_Principle/bak/png5.bmp", CV_LOAD_IMAGE_GRAYSCALE);
            //            Mat testresultImgMat(2*testmat.rows, 2*testmat.cols, testmat.type());
            //            undistortImg(testmat, testresultImgMat, cameraMatrix, distCoeffs, parallelImgR[i]);
            //            namedWindow("Test1",WINDOW_AUTOSIZE);
            //            imshow("Test1",resultImgMat);
            //            waitKey();
            //            imshow("Test1",testresultImgMat);
            //            waitKey();
            /// end test

            ////// zhang xu added 增加一个 ring模版708X708，然后进行resize
            Mat ringImg;
            CoreAlgorithm::creatRingImg(ringImg, 708);
            ////end
            ringSubPix(resultImgMat,corner_Parallel[i],ringImg); ////ringSubPix(resultImgMat,corner_Parallel[i],Mat());
            //delete resultImgMat;
            //ringSubPix(resultImgMat,corner_Parallel[i],Mat());

            /// modified by zhaorui
            //ringSubPixAdv(resultImgMat,corner_Parallel[i],1);
            /// end

            int j = 0;
            /// test
            //            // resize
            //            //Mat testmat;
            //            Size resizevalue;
            //            resizevalue.height = resultImgMat.rows*5;
            //            resizevalue.width = resultImgMat.cols*5;
            //            //resize(resultImgMat, testmat, resizevalue);
            //            resize(testresultImgMat, testmat, resizevalue);
            //            // draw cross
            //            Point up, down, left, right;
            //            Scalar sclr(255,255,255);
            //            for(int u = 0; u < corner_Parallel[i].size(); u++)
            //            {
            //                Point2f pt = corner_Parallel[i][u];
            //                pt.x *= 5;
            //                pt.y *= 5;
            //                up = down = left = right = pt;
            //                up.y = up.y - 50;
            //                down.y = down.y + 50;
            //                left.x=left.x-50;
            //                right.x=right.x+50;
            //                line(testmat, up, down, sclr, 2);
            //                line(testmat, left, right, sclr, 2);
            //            }
            //            // show and save
            //            namedWindow("testresize",CV_WINDOW_AUTOSIZE);
            //            imwrite("testresize.bmp", testmat);
            //            imshow("testresize",testmat);
            //            waitKey();
            /// end test
        }

        //STEP-4 将平行视图上检测到的亚像素角点转换到原始视图
        for (int i = 0; i < corner_Parallel.size(); i++)
        {
            vector<Point2f> pnts;
            undistortPoints2DifferentView(corner_Parallel[i], pnts,
                                          cameraMatrix,camPara.parallelCamR,camPara.imgRTVec[i].T,vector<double>(),
                                          cameraMatrix,camPara.imgRTVec[i].R,camPara.imgRTVec[i].T,distCoeffs);
            corner_Parallel[i] = (pnts);
        }

        //STEP-5 使用张正友标定法标定
        _cData.plane2dPntsVec.clear();
        _cData.plane2dPntsVec = corner_Parallel;

        ZhangCalibrationMethod_CPP(_cData,camParaDst);
        //// added by zhang xu 迭代终止条件，如果迭代没有使得totalReproNormErr 减少，则停止迭代
        if(camParaDst.totalReproNormErr> camPara.totalReproNormErr)
        {
            camParaDst = camPara;
            return;  /// 返回
        }
        ////end
        camParaDst.parallelCamR = camPara.parallelCamR;
        camPara = camParaDst;

        //////  update 11.6
        cData.plane2dPntsVec.clear();
        cData.plane2dPntsVec = _cData.plane2dPntsVec;
    }
}

void CoreAlgorithm::undistortPoints2DifferentView(const vector<Point2f>& src,vector<Point2f>& dst,
                                                  const Mat& cameraMatrix1,const Mat& R1,const Mat& T1,const vector<double>& distCoeffs1,
                                                  const Mat& cameraMatrix2,const Mat& R2,const Mat& T2,const vector<double>& distCoeffs2)
{
    //STEP-1 使用cam1的参数获得特征点的三维平面点
    vector<Point3f> pnts3d;
    project2CalibrationBoard(src,pnts3d,cameraMatrix1,distCoeffs1,R1,T1);
    //STEP-2 将特征点的三维平面点重投影到图像上，采用cam2的R和T
    Mat _R2;
    R2.copyTo(_R2);
    if (R2.cols==3 && R2.rows==3)
    {
        Rodrigues(R2, _R2);
    }
    projectPoints(pnts3d, _R2, T2, cameraMatrix2, distCoeffs2, dst);
}

void CoreAlgorithm::undistortImg(const Mat& src, Mat& dst,
                                 const Mat& cameraMatrix, const vector<double>& distCoeffs,const Mat& R)
{
    Mat map1, map2;
    Mat new_cameraMatrix;
    cameraMatrix.copyTo(new_cameraMatrix);
    ///// destination size
    if(dst.rows<1)   ///// dst 不能为空
    {
        cout<<"dst.rows<1"<<endl;
        return;
    }
    /////// 以后 重新优化这个函数
    initUndistortRectifyMap(cameraMatrix, distCoeffs, R, new_cameraMatrix, dst.size(), CV_32FC1, map1, map2);
	remap(src, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void CoreAlgorithm::calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR)
{
    for (int i = 0; i < rt_vec.size(); i++)
    {
        Mat R = rt_vec[i].R;
        Mat T = rt_vec[i].T;
        Mat _R;
        Rodrigues(R, _R);
        Mat H0(3,3,CV_64F);
        _R.copyTo(H0);
        T.col(0).copyTo(H0.col(2));
        ///// just for test  观察T的值
        float t1,t2,t3;
        t1 = T.at<double>(0,0);
        t2 = T.at<double>(1,0);
        t3 = T.at<double>(2,0);
        //// end the test

        Mat H1 = Mat::zeros(3,3,CV_64F);
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

//根据文档《》生成平顶视图的转换矩阵
//const Mat &PoseR,<in>  3X1
//const Mat &PoseT,<in>  3X1
//Mat &parallelR<in & out>  输入平顶视图相对于摄像机的外参数R，3X3矩阵，输出是文档要求的H1 * inv(H0), 3X3
void CoreAlgorithm::calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR)
{
    /// 判断矩阵的尺寸，不正确及跳出

    if(parallelR.rows ==0 ||(parallelR.cols == 0))
    {
        parallelR = Mat::eye(3,3,CV_64F);
    }
    ///  step  生成H1
    Mat H1(3,3,CV_64F);
    parallelR.copyTo(H1);
    PoseT.col(0).copyTo(H1.col(2));
    /// step 生成H0
    Mat temR = Mat::zeros(3,3,CV_64F);
    Rodrigues(PoseR, temR);
    Mat H0(3,3,CV_64F);
    temR.copyTo(H0);
    PoseT.col(0).copyTo(H0.col(2));

    //// zhangxu 修改
    //H1.at<double>(0, 2) = -SQUARE_SIZE*WIDTH_CORNER_COUNT/2;
    //H1.at<double>(1, 2) = -SQUARE_SIZE*HEIGHT_CORNER_COUNT/2;
    /*H1.at<double>(2, 2) = 1.5*H1.at<double>(2, 2);*/
    //////

    ///////  以下的标定板坐标系是  x向下，y向右，z标定板向前，和matlabtool box camera calibration一样
    //H1.at<double>(0, 1) = 1;
    //H1.at<double>(1, 0) = 1;
    ///////  以下的标定板坐标系是  x向右，y向下，z标定板向后，
    //H1.at<double>(0, 0) = 1;
    //H1.at<double>(1, 1) = 1;
    ///// end
    Mat H0_invert;
    invert(H0, H0_invert);
    parallelR = H1*H0_invert;
}

void CoreAlgorithm::project2CalibrationBoard(const vector<Point2f>& src, vector<Point3f>& pnts3d,
                                             const Mat& cameraMatrix, vector<double> distCoeffs, const Mat& R,const Mat& T)
{
	if (src.size() == 0)
		return;
    pnts3d.clear();
    vector<Point2f> pnts2d;
    Mat _R;
    R.copyTo(_R);
    if (R.cols==1 || R.rows==1)
    {
        Rodrigues(R,_R);
    }
    T.col(0).copyTo(_R.col(2));
    invert(_R, _R);
    undistortPoints(src, pnts2d, cameraMatrix, distCoeffs,_R);

    for (int i = 0; i < pnts2d.size(); i++)
    {
        pnts3d.push_back(Point3f(pnts2d[i].x,pnts2d[i].y,0));
    }
	return;
}

bool CoreAlgorithm::creatCosImg(Mat& Img, float T, float fai, float bias, float range, float gamma, bool isX, bool isGrid)
{
    int height = Img.rows;
    int width = Img.cols;
    int step = Img.step;
    int channels = Img.channels();
    uchar* Img_data = (uchar*)Img.data;

    if (height<1 || width <1)
    {
        ///// 图像不能为空
        return false;
    }

    ///// 设置一些参数
    double PI = acos(double(-1));
    float A = bias/(bias+range);
    float B = range/(bias+range);
    //////增大信噪比的方法，即加大B的方法
    float Xmin = pow((A-B),gamma);
    float A2 = (A+B+Xmin)/2;
    float B2 = (A+B-Xmin)/2;

    ///  对grid图像  x轴 生成图像Img，channel 有图像来控制  x指的就是col
    if (isGrid && isX)
    {
        for (int col=0;col<width; col++)
        {
            ///// 计算亮度
            float norm = A2+B2*cos(2*PI*col/T-fai);
            float power = 1/gamma;
            float temp = pow(norm,power);
            float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

            ///// 赋值到图像
            for (int row=0; row<height; row++)
            {
                for (int i=0;i<channels;i++)
                {
                    Img_data[row*step + col*channels + i] = int(lus);
                }
            }
        }
    }
    ///  对grid图像  y轴 生成图像Img，channel 有图像来控制  y指的就是row
    if (isGrid && !isX)
    {
        for (int row = 0;row<height; row++)
        {
            ///// 计算亮度
            float norm = A2+B2*cos(2*PI*row/T-fai);
            float power = 1/gamma;
            float temp = pow(norm,power);
            float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI
            ///// 赋值到图像
            for (int col=0; col<width; col++)
            {
                for (int i=0;i<channels;i++)
                {
                    Img_data[row*step + col*channels + i] = int(lus);
                }
            }
        }

    }

    ///  对diamond 像素网格图像  x轴 生成图像Img，channel 有图像来控制  x指的就是col
    /// 根据 LIghtcragter 4500 page11 说明 第0行是缩进半个像素，第1行是左突出半个像素，因而可的规律，偶数行像素的列坐标都是大于
    /// 奇数行两坐标半个像素（5.4微米），行与行的间距是5.4微米，列与列的间距是10.8微米,每一列不是正好尖尖对角的列，而是弯曲弯曲的组成一列。
    /// 总结起来就是 每列坐标上，奇数行列相对偶数行更靠近左边半个像素，即 偶数行列坐标加0.5，奇数行等于列坐标
    if (!isGrid && isX)
    {
        for (int col=0;col<width; col++)
        {
            ///// 赋值到图像奇数行
            for (int row=1; row<height; row=row+2)
            {
                ///// 计算亮度
                float tempcol = col;
                float norm = A2+B2*cos(2*PI*tempcol/T-fai);
                float power = 1/gamma;
                float temp = pow(norm,power);
                float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

                for (int i=0;i<channels;i++)
                {
                    Img_data[row*step + col*channels + i] = int(lus);
                }
            }

            ///// 赋值到图像偶数行
            for (int row=0; row<height; row=row+2)
            {
                ///// 计算亮度
                float tempcol = col+0.5;
                float norm = A2+B2*cos(2*PI*tempcol/T-fai);
                float power = 1/gamma;
                float temp = pow(norm,power);
                float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

                for (int i=0;i<channels;i++)
                {
                    Img_data[row*step + col*channels + i] = int(lus);
                }
            }
        }
    }

    ///  对diamond 像素网格图像  y轴 生成图像Img，channel 有图像来控制  x指的就是col
    /// 根据 LIghtcragter 4500 page11 说明 第0行是缩进半个像素，第1行是左突出半个像素，因而可的规律，偶数行像素的列坐标都是大于
    /// 奇数行两坐标半个像素（5.4微米），行与行的间距是5.4微米，列与列的间距是10.8微米,每一列不是正好尖尖对角的列，而是弯曲弯曲的组成一列。
    /// 总结起来就是 每行	坐标上，奇数行列相对偶数行更靠近左边半个像素，即 偶数行列坐标加除以2，奇数行等于行坐标加上1除以2，即比上一个偶数行多了0.5，


    if (!isGrid && !isX)
    {
        for (int row = 0;row<height; row++)
        {
            for (int col=1; col<width; col = col+1)
            {
                //// 判断row 是奇数还是偶数
                if (row%2==0) //// 如果为偶数
                {
                    ///// 计算亮度
                    float temprow = row/2;
                    float norm = A2+B2*cos(2*PI*temprow/T-fai);
                    float power = 1/gamma;
                    float temp = pow(norm,power);
                    float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

                    for (int i=0;i<channels;i++)
                    {
                        Img_data[row*step + col*channels + i] = int(lus);
                    }
                }
                else  //// 如果为奇数
                {
                    ///// 计算亮度
                    float temprow = (row+1)/2;
                    float norm = A2+B2*cos(2*PI*temprow/T-fai);
                    float power = 1/gamma;
                    float temp = pow(norm,power);
                    float lus = (bias+range)*temp;//// 这里直接使用col除以pitch 乘以2PI

                    for (int i=0;i<channels;i++)
                    {
                        Img_data[row*step + col*channels + i] = int(lus);
                    }
                }
            }
        }
    }
    return true;
}

bool CoreAlgorithm::gcd(int a, int b, int& x, int& y, int& q)
{
    int temp, xt, yt, qt;
    if (b==0)
    {
        x=1;
        y=0;
        q=a;
        return true;
    }
    else
    {
        temp = a%b;
        gcd (b, temp, xt, yt, qt );
        x = yt;
        y = xt-(a/b)*yt;
        q = qt;
    }

    return true;
}

bool CoreAlgorithm::phaseshift (vector<Mat> images, float threshold, Mat& mod_phase, Mat& mean_lus, Mat& lus_range, Mat& mask)
{
    int frameNum = images.size();  //// 图片数量

    if (frameNum<3)
    {
        return false;  /// 图片数量不能少于3
    }

    int height = images[0].rows;
    int width = images[0].cols;
    int channels = images[0].channels();
    if (channels>1)
    {
        return false;  /// 本函数只接受灰度图像
    }

    /*if (height!= mod_phase.rows || height!= mean_lus.rows || height!= lus_range.rows || height!=mask.rows)
    {
        return false;  /// 输入参数的 行数不一致
    }

    if(width != mod_phase.cols || width!= mean_lus.cols || width != lus_range.cols || width != mask.cols  )
    {
        return false;  /// 输入的参数 列数不一致
    }*/


    ///////
    Mat lusin = Mat::zeros(height,width,CV_32FC1);
    Mat lucos = Mat::zeros(height,width,CV_32FC1);
    mean_lus = Mat::zeros(height,width,CV_32FC1);  ////将它赋值为0
    mod_phase = Mat::zeros(height,width,CV_32FC1);  ////将它赋值为0
    lus_range = Mat::zeros(height,width,CV_32FC1);  ////将它赋值为0
    mask =  Mat::zeros(height,width,CV_8UC1); ////将它赋值为0
    Mat temMat ;  ////将它赋值为0   = Mat::zeros(height,width,CV_32FC1)

    double PI = acos(double(-1));
    for (int i=0;i<frameNum; i++)
    {
        double fai = 2*PI*i/frameNum;
        images[i].convertTo(temMat,CV_32FC1);
        scaleAdd(temMat, sin(fai), lusin, lusin);
        scaleAdd(temMat, cos(fai), lucos, lucos);
        scaleAdd(temMat,1.0/frameNum, mean_lus, mean_lus);
    }

    for (int row=0;row<height; row++)
    {
        for(int col=0;col<width;col++)
        {
            mod_phase.at<float>(row,col) = atan2(lusin.at<float>(row,col), lucos.at<float>(row,col));
            if (mod_phase.at<float>(row,col)<0)
            {
                mod_phase.at<float>(row,col) = mod_phase.at<float>(row,col)+2*PI;
            }
        }
    }
    multiply(lusin, lusin, lusin);
    multiply(lucos, lucos, lucos);
    add(lusin,lucos,lus_range);
    sqrt(lus_range,lus_range);
    lus_range = 2*lus_range/frameNum;

    //// 判断mask
    for (int row=0;row<height; row++)
    {
        for(int col=0;col<width;col++)
        {
            if (lus_range.at<float>(row,col)>threshold && mean_lus.at<float>(row,col)>threshold)
            {
                mask.at<char>(row,col) = 1;
            }
        }
    }
    return true;
}

bool CoreAlgorithm::robustCRM(vector<Mat> mod_phases,vector<float> lamda, float m, Mat& cor)
{
    int freNum = mod_phases.size();
    if (freNum!=lamda.size())
    {
        return false; //// 频率个数不一致
    }
    int height = mod_phases[0].rows;
    int width = mod_phases[0].cols;

    double PI = acos(double(-1));

    vector<Mat> q; /// 保存相减除以最大公约数的结果
    vector<int> taobari1;
    int gamma1 = 1; /// 所有质数的成绩除以第一个质数，即除了第一个质数之外的所有质数的乘积
    ///// 将卷绕相位转换成余数
    for(int i=0;i<freNum;i++)
    {
        mod_phases[i] = mod_phases[i]*lamda[i]/2/PI;  /// 卷绕相位转换成余数
        lamda[i] = lamda[i]/m;  //// 转换成互质的数
        ////// 计算余数的差
        Mat temQ,temQ2;  ///临时矩阵
        if (i>0)
        {
            ////
            gamma1 = gamma1*lamda[i]; /// 计算所有质数的乘积除以 tao1
            ///// 计算模乘系数
            int temtaoi1,temtao11,temq;
            gcd(lamda[0],lamda[i],temtaoi1,temtao11,temq);
            taobari1.push_back(temtaoi1);
            ///// 计算余数差，除以公约数m，乘以模乘系数
            subtract(mod_phases[i], mod_phases[0], temQ); //// 相减
            temQ = temQ/m;
            temQ.convertTo(temQ2,CV_32SC1);  //// 这里转换成整数
            q.push_back(temQ2);  /// 保存
        }
    }
    ///// 			//// 计算bi1
    vector<int> b;
    for (int i=0;i<freNum-1;i++)
    {
        int bi1, b11,q11;
        gcd(gamma1/lamda[i+1], lamda[i+1],bi1,b11,q11);
        b.push_back(bi1);
    }
    ////计算	 cor
    cor = Mat::zeros(height,width,CV_32FC1);
    for (int row=0;row<height; row++)
    {
        for (int col=0;col<width; col++)
        {
            float y =0;
            int zn = 0;
            for (int i=0;i<freNum-1; i++)
            {
                int epiu, tqt1,tqt2;
                tqt1 = q[i].at<int>(row,col);
                epiu = tqt1*taobari1[i]%int(lamda[i+1]);
                if (epiu<0)
                {
                    epiu = epiu + int(lamda[i+1]);
                }
                zn = zn + epiu*b[i]*int(gamma1)/int(lamda[i+1]); ///公式（25） 求和
            }
            int tqt1,tqt2,n1,ni1;
            n1 = zn%gamma1;
            if (n1<0)
            {
                n1 = n1+gamma1;
            }
            y = n1*lamda[0]*m + mod_phases[0].at<float>(row,col);


            for (int i=0;i<freNum-1; i++)
            {
                tqt1 = q[i].at<int>(row,col);
                tqt2 = int(lamda[0]);
                ni1 = (n1*tqt2-tqt1)/lamda[i+1];
                float yi1;
                yi1 = ni1*lamda[i+1]*m + mod_phases[i+1].at<float>(row,col);
                y =y+ yi1;
            }
            cor.at<float>(row,col) = y/freNum;
        }
    }
    return true;
}

bool CoreAlgorithm::undistorPixcor(const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c )
{
    normalizPixel( src, dst,  fc , cc ,kc ,alfpha_c );
    int pntNum = dst.size();
    Point2f temPnt;
    for (int i=0;i<pntNum; i++)
    {
        temPnt.x = fc[0]*dst[i].x + alfpha_c*dst[i].y + cc[0];
        temPnt.y = fc[1]*dst[i].y + cc[1];
        dst[i] = temPnt;
    }
    return true;
}

bool CoreAlgorithm::undistorPixcor(const vector<Point2f>& src, vector<Point2f>& dst, Mat& intrinsicPara, Mat& distCoeffs)
{

	double fc[2], cc[2], kc[5];
	double alfpha_c = 0;

	fc[0] = intrinsicPara.at<double>(0, 0);
	fc[1] = intrinsicPara.at<double>(1, 1);

	cc[0] = intrinsicPara.at<double>(0, 2);
	cc[1] = intrinsicPara.at<double>(1, 2);
	for (int col = 0; col < distCoeffs.cols;++col)
	{
		kc[col] = distCoeffs.at<double>(0, col);
	}

	if (distCoeffs.cols<5)
	{
		for (int num = distCoeffs.cols; num < 5; ++num)
		{
			kc[num] = 0;
		}
	}

	normalizPixel(src, dst, fc, cc, kc, alfpha_c);
	int pntNum = dst.size();
	Point2f temPnt;
	for (int i = 0; i < pntNum; i++)
	{
		temPnt.x = fc[0] * dst[i].x + alfpha_c*dst[i].y + cc[0];
		temPnt.y = fc[1] * dst[i].y + cc[1];
		dst[i] = temPnt;
	}
	return true;
}


/// 根据摄像机参数计算畸变图像，已知无畸变图像的模型，本函数适用于投影机三维计算时，对应无畸变投影图像的畸变图像，这样利用透镜的畸变
/// 抵消了这个畸变，直接使用无畸变模型参与三维计算
/// Mat& img,<in and out>  输出带有畸变的图像
/// double fc[2] ,<in> 摄像机参数focal lengh
/// double cc[2] ,<in> 摄像机参数主点位置
/// double kc[5] ,<in> 摄像机畸变参数
/// double alfpha_c  <in>  摄像机参数纵横比例因子
/// float meanLus,<in>  均值亮度
/// float rangelus,<in>  幅值亮度
/// float T,<in> 周期
/// float fai, <in> 起始相位
//  bool isGrid <in>  x y坐标是否垂直成正交grid， true 为是，false 表示为六边形排布，类似Lightcrafter的DMD形式
/// int method =0  默认为0，表示模型是 meanLus + rangelus*cos(2*pi*x/T -fai)
/// method = 1 Gray code  周期为T， 起始的位置在fai，ranglus<0 表示先 黑后白，反之 先白后黑； 如周期是10，fai等于5，ranglus=1
/// 则一开就是黑，相当于移动了半个周期
bool CoreAlgorithm::creatdistorImg(Mat& img,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c, float meanLus, float rangelus,float T,float fai,bool isGrid, int method )
{
    //// 判断要输出的图像大小是否大于0
    int height,width;
    height = img.rows;
    width  = img.cols;
    if (height<1 || width <1)
    {
        return false;  //// 图像输入为0，不是有效的图像
    }
    //// 判断图像的通道数，如果是3个通道的 报错
    int channel = img.channels();
    if (channel>1)
    {
        return false;
    }
    float PI = acos(double(-1));
    vector<Point2f> src, dst,src_diacor;
    src.clear();
    dst.clear();
    src_diacor.clear();
    Point2f tempPnt;
    if(isGrid) //// 是正交的坐标系
    {
        //// 将图像的坐标转换层 vector<point2f>
        for (float i=0;i<height; i++)
        {
            for (float j=0;j<width; j++)
            {
                tempPnt.x = j;
                tempPnt.y = i;
                src.push_back(tempPnt);
            }
        }
    }
    else   /// diamond 像素排列，专门针对 lightcrafter 进行的排列方式；列的宽度是行的宽度两倍，且偶数行  比 奇数行 的列坐标 向右偏移0.5个像素
    {
        //// 将图像的坐标转换层 vector<point2f>
        src.clear();
        dst.clear();
        for (int i=0;i<height; i++)  //// 行
        {
            if(i%2==0) /// 偶数行
            {
                for (float j=0;j<width; j++) //// 列
                {
                    tempPnt.x = j+0.5;
                    tempPnt.y = float(i/2);
                    src.push_back(tempPnt);

                    tempPnt.x = j;
                    tempPnt.y = float(i);
                    src_diacor.push_back(tempPnt);
                }
            }
            else  /// 奇数数行
            {
                for (float j=0;j<width; j++) //// 列
                {
                    tempPnt.x = j;
                    tempPnt.y = (i+1)/2;
                    src.push_back(tempPnt);

                    tempPnt.x = j;
                    tempPnt.y = float(i);
                    src_diacor.push_back(tempPnt);
                }
            }
        }
    }
    ///// 计算无畸变的像素坐标
    undistorPixcor(src ,dst ,fc ,cc ,kc ,alfpha_c);
    //// 根据无畸变的像素坐标计算亮度
    int intensty;
    ///////////////////////////  method =0  表示模型是 meanLus + rangelus*cos(2*pi*x/T -fai)
    float temp;
    if (method == 0)
    {
        for(int i=0;i<dst.size();i++)
        {
            tempPnt = dst[i];
            temp = meanLus + rangelus*cos(2*PI*tempPnt.x/T -fai);
            for (int j=0;j<channel;j++)
            {
                intensty = int (temp);
                ///// 根据此亮度对图像进行设置
                if(isGrid)
                {
                    img.at<uchar>(src_diacor[i].y,src_diacor[i].x) = intensty;
                }
                else
                {
                    img.at<uchar>(src_diacor[i].y,src_diacor[i].x) = intensty;
                }
            }
        }
    }
    /////////  method =1 Gray code
    if(method ==1)
    {
        return false;  /// 留在后边实现
    }
    return true;
}

void CoreAlgorithm::generateParallelCamR(Mat& R)
{
    Mat r(3, 3, CV_64F);
    r = Mat::zeros(3, 3, CV_64F);
    ///////  以下的标定板坐标系是  x向下，y向右，z标定板向前，和matlabtool box camera calibration一样
    //r.at<double>(0, 1) = 1;
    //r.at<double>(1, 0) = 1;
    //r.at<double>(2, 2) = -1;
    ///////  以下的标定板坐标系是  x向右，y向下，z标定板向后，
    r.at<double>(0, 0) = 1;
    r.at<double>(1, 1) = 1;
    r.at<double>(2, 2) = 1;

    r.copyTo(R);
}

bool CoreAlgorithm::triangulatePnts(const vector<Point2f>& pnts2d1, const vector<Point2f>& pnts2d2,
                                    const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
                                    const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
                                    vector<Point3f>& pnts3d)
{
    Mat _R1, _R2;
    R1.copyTo(_R1);
    R2.copyTo(_R2);
    if (R1.rows==1||R1.cols==1)
        Rodrigues(_R1,_R1);
    if (R2.rows==1||R2.cols==1)
        Rodrigues(_R2,_R2);

    Mat RT1(3,4,CV_64F), RT2(3,4,CV_64F);
    _R1.colRange(0,3).copyTo(RT1.colRange(0, 3));
    _R2.colRange(0,3).copyTo(RT2.colRange(0, 3));
    T1.copyTo(RT1.col(3));
    T2.copyTo(RT2.col(3));

    Mat P1, P2;//3x4
    P1 = cameraMatrix1 * RT1;
    P2 = cameraMatrix2 * RT2;

    vector<Point2f> _pnts2d1, _pnts2d2;

    undistortPoints2DifferentView(pnts2d1,_pnts2d1,
                                  cameraMatrix1,R1,T1,distCoeffs1,
                                  cameraMatrix1,R1,T1,vector<double>());

    undistortPoints2DifferentView(pnts2d2,_pnts2d2,
                                  cameraMatrix2,R2,T2,distCoeffs2,
                                  cameraMatrix2,R2,T2,vector<double>());

    Mat pnts4d;
    triangulatePoints(P1,P2,_pnts2d1,_pnts2d2,pnts4d);

    for (int i = 0; i < pnts4d.cols; i++)
    {
        double a = pnts4d.at<float>(3,i);
        pnts3d.push_back(Point3f(pnts4d.at<float>(0,i)/a,
                                 pnts4d.at<float>(1,i)/a,
                                 pnts4d.at<float>(2,i)/a));
    }

    return true;
}

bool CoreAlgorithm::quadSubPixel(const Mat& win, float& dx, float& dy)
{
    Mat A(9,6,CV_32F), B(9,1,CV_32F);
    int row = 0;
    for (int y = -1; y <=1 ; y++)
    {
        for (int x = -1; x <= 1; x++,row++)
        {
            A.at<float>(row,0)=x*x;		A.at<float>(row,1)=y*y;
            A.at<float>(row,2)=x;		A.at<float>(row,3)=y;
            A.at<float>(row,4)=x*y;		A.at<float>(row,5)=1;

            B.at<float>(row, 0) = win.at<float>(y + 1, x + 1);
        }
    }

    Mat X(6,1,CV_32F);

    //这里A不是方阵
	solve(A, B, X, cv::DECOMP_QR);

    Mat H(2,2,CV_32F);
    H.at<float>(0, 0) = X.at<float>(0, 0);
    H.at<float>(0, 1) = X.at<float>(1, 0) / 2.0;
    H.at<float>(1, 0) = X.at<float>(1, 0) / 2.0;
    H.at<float>(1, 1) = X.at<float>(2, 0);

    vector<double> eigenvalues;
    Mat eigenvectors(2,2,CV_32F);
    eigen(H, eigenvalues, eigenvectors);

    //后续添加
    /*if (eigenvectors.at<float>(0,0) < 0 && eigenvectors.at<float>(0,1) < 0)
    {
    float a = X.at<float>(0, 0); float b = X.at<float>(1, 0); float c = X.at<float>(2, 0);
    float d = X.at<float>(3, 0); float e = X.at<float>(4, 0); float f = X.at<float>(5, 0);

    dx = -1*(2*b*c - d*e)/(4.0*a*b - e*e);
    dy = -1*(2*a*d - c*e)/(4.0*a*b - e*e);
    }
    else
    {
    dx = 0;
    dy = 0;
    }*/
    //X*V = V*D;
    Mat aa = H*eigenvectors.row(0).t();
    Mat bb = eigenvalues[0] * eigenvectors.row(0).t();

    float a = X.at<float>(0, 0); float b = X.at<float>(1, 0); float c = X.at<float>(2, 0);
    float d = X.at<float>(3, 0); float e = X.at<float>(4, 0); float f = X.at<float>(5, 0);

    dx = -1*(2*b*c - d*e)/(4.0*a*b - e*e);
    dy = -1*(2*a*d - c*e)/(4.0*a*b - e*e);
    return true;

}

/////Modified by Chengwei
bool CoreAlgorithm::extraRingCenter(const Mat& img, const vector<Point2f>& mousePnts,
                                    vector<Point2f>& plane2dPnts, int& row, int& col)
{
    //转化成灰度图
    Mat ImageGray;
    if(img.channels()==1)
    {
        ImageGray = img;
    }
    else
    {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
    }
    //创建标定板区域的ROI，向外围阔出去一圈
    RotatedRect ROI = minAreaRect(mousePnts);
	cv::Rect roi = ROI.boundingRect();
    ///// 检查ROI区域 如果超过范围，则赋值图像最大尺寸
    if(roi.x<0)
    {
        roi.x=0;
    }
    if(roi.y<0)
    {
        roi.y=0;
    }
    if(roi.x+roi.width>ImageGray.cols-1)
    {
        roi.width = ImageGray.cols-roi.x -1;
    }
    if(roi.y+roi.height>ImageGray.rows-1)
    {
        roi.height=ImageGray.rows-roi.y-1;
    }
    Mat imgGray_ROI = ImageGray(roi).clone();
    //    namedWindow("roi",2);
    //    imshow("roi",imgGray_ROI);
    //    waitKey();
    //// 均值滤波
    //blur(imgGray_ROI,imgGray_ROI,Size(5,5),Point(-1,-1));//需要均值滤波去除圆环上的白色噪声点
    cv::GaussianBlur(imgGray_ROI,imgGray_ROI,cv::Size(3,3),0);//高斯核根据图像大小设定
    //chengwei changed
    float areaHeight = abs(mousePnts[0].y-mousePnts[2].y);
    unsigned int multiples = int(ceil(areaHeight/300));
    ///// 自适应二值化
    int block_size = 100*multiples+1;
    int C_threshold = 10;
    Mat imgroi2;
    imgGray_ROI.copyTo(imgroi2);
	adaptiveThreshold(imgGray_ROI, imgGray_ROI, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, block_size, C_threshold);
    //cv::threshold(imgGray_ROI,imgGray_ROI,130,255,CV_THRESH_BINARY);
    //对白色区域膨胀再腐蚀
	int close_type = cv::MORPH_ELLIPSE;
    //int dilate_size = multiples/3+1;
	int dilate_size = 1; //核根据图像大小设定
    Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
    dilate(imgGray_ROI, imgGray_ROI, element,Point(-1, -1),1);
    erode(imgGray_ROI, imgGray_ROI, element,Point(-1, -1),1);
    //// 边缘检测
    Mat img_threshold;
    Canny(imgGray_ROI,img_threshold,30,255,3,true);/// 255,  3

    //test by zhaorui
    //    namedWindow("window", WINDOW_NORMAL);
    //    imshow("window",img_threshold);
    //    waitKey();
    //end test

    //寻找轮廓
    vector< vector<Point> > contours;
    findContours(img_threshold,contours,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);
    //椭圆拟合,找中心,并进行筛选
    RotatedRect rotatedBox;
    vector<RotatedRect> rotatedBoxVec;
    vector<RotatedRect> temp_rotatedBoxVec;

    //test by zhaorui
    int num = 0;
    Mat testImg;
    testImg.create(img_threshold.rows, img_threshold.cols, CV_8UC3);
    testImg.zeros(img_threshold.rows, img_threshold.cols, CV_8UC3);
    //end test
    for( vector< vector<Point> >::iterator itr=contours.begin();itr!=contours.end();++itr)
    {
        static int n=0;
        n++;
        num++;
        if(itr->size()<5||itr->size()>450)//
        {
            continue;
        }
        //        vector<Point> contours1;
        //        contours1 = *itr;
        //        Point distancep = contours1[0]-contours1[contours1.size()-1];
        //        int distance = abs(distancep.x)+abs(distancep.y);
        //        if(distance>3)
        //            continue;
        try
        {
            rotatedBox = fitEllipse((*itr));
        }
        catch (...)//三个点表示任何异常
        {
            continue;
        }
        //CvPoint centerInSrcImg;
        //centerInSrcImg.x = rotatedBox.center.x + roi.x;
        //centerInSrcImg.y = rotatedBox.center.y + roi.y;
        //if (containsPnt(mousePnts, centerInSrcImg) < 0)//检测到的椭圆中心点不在给定的ROI区域内则剔除
        //{
        //    //test by zhaorui
        //    num--;
        //    Point tempcenter;
        //    tempcenter.x = rotatedBox.center.x;
        //    tempcenter.y = rotatedBox.center.y;
        //    //circle(testImg, tempcenter, 5, Scalar(0,0,255), 3);
        //    //end test
        //    continue;
        //}
        temp_rotatedBoxVec.push_back(rotatedBox);

        //test by zhaorui
        Point tempcenter;
        tempcenter.x = rotatedBox.center.x;
        tempcenter.y = rotatedBox.center.y;
        //circle(testImg, tempcenter, 5, Scalar(255,0,0), 3);
        //        cout<<"rotatedBox: ( "<<tempcenter.x<<", "<<tempcenter.y<<" )"<<endl;
        //        namedWindow("window", WINDOW_NORMAL);
        //        imshow("window",testImg);
        //        waitKey();
        //end test

        float height = rotatedBox.size.height;
        float width = rotatedBox.size.width;
        //plane2dPnts.push_back(rotatedBox.center);
        if ( height>width? height/width<2.5 : width/height<2.5 )
        {
            //x此处加上gradient的椭圆检测的算法，看下检测精度
            //just for test
            cv::Rect ROIsingle = cv::Rect(Point2i((int)abs(rotatedBox.center.x-width/2-3),(int)abs(rotatedBox.center.y-height/2-3)),cv::Size(int(width+6),int(height+6)));
            //防止越界判断
            if(ROIsingle.x<0)
            {
                ROIsingle.x=0;
            }
            if(ROIsingle.y<0)
            {
                ROIsingle.y=0;
            }
            if(ROIsingle.x+ROIsingle.width>img_threshold.cols-1)
            {
                ROIsingle.width = img_threshold.cols-ROIsingle.x -1;
            }
            if(ROIsingle.y+ROIsingle.height>img_threshold.rows-1)
            {
                ROIsingle.height=img_threshold.rows-ROIsingle.y-1;
            }
            vector<RotatedRect> findResulttemp;
            if(!CoreAlgorithm::findEllipses(imgroi2,ROIsingle,findResulttemp,0.1,false,3))
            {
                plane2dPnts.push_back(rotatedBox.center);
                continue;
            }
            if(findResulttemp.size()==2)
            {
                if(findResulttemp[0].size.width>2&&findResulttemp[1].size.width>2)
                {
					if (findResulttemp[0].size.width > findResulttemp[1].size.width)
					{

						Point2f centerTemp;
						centerTemp.x = findResulttemp[1].center.x;
						centerTemp.y = findResulttemp[1].center.y;
						plane2dPnts.push_back(centerTemp);
					}
					else
					{
						Point2f centerTemp;
						centerTemp.x = findResulttemp[0].center.x;
						centerTemp.y = findResulttemp[0].center.y;
						plane2dPnts.push_back(centerTemp);
					}
                }
            }
			else if (findResulttemp.size()==1)
			{
				if (findResulttemp[0].size.width > 2)
				{
					Point2f centerTemp;
					centerTemp.x = findResulttemp[0].center.x;
					centerTemp.y = findResulttemp[0].center.y;
					plane2dPnts.push_back(centerTemp);
				}
			}
            else
            {
                continue;
            }
            //just for test
            //plane2dPnts.push_back(rotatedBox.center);
        }
    }
    //test by zhaorui
    //    cout<<"num = "<<num<<endl;
    //    namedWindow("window", WINDOW_NORMAL);
    //    imshow("window",testImg);
    //    waitKey();
    //end test

    for (unsigned int i = 0; i < plane2dPnts.size();i++)
    {
        plane2dPnts[i].x += roi.x;
        plane2dPnts[i].y += roi.y;
    }
	if (plane2dPnts.size() == 0)
		return false;
    if (!sortRingCenter(mousePnts,plane2dPnts, row, col))
        return false;
    //再进行一次高精度的同心圆检测
    //if(!ringSubPixAdv(img,plane2dPnts,3))//3代表内外圆取平均
    // return false;
    return true;
}

bool CoreAlgorithm::extraRingCenterXu(const Mat& img, const vector<Point2f>& mousePnts
                                      , vector<Point2f>& plane2dPnts, int& row, int& col)
{

    //    //向外围阔出去一圈
    //    Mat ImageGray;
    //    cvtColor(img, ImageGray, CV_BGR2GRAY);

    //转化成灰度图
    Mat ImageGray;
    if(img.channels()==1)
    {
        ImageGray = img;
    }
    else
    {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
    }

    ///// 二值化
    //// 直方图均衡化
    //equalizeHist(ImageGray,ImageGray);

    //    /// test
    //    namedWindow("window1");
    //    imshow("window1",ImageGraydst);
    //    waitKey();
    //    /// end test

    //// 均值滤波
    blur(ImageGray,ImageGray,Size(5,5),Point(-1,-1));

    //    /// test
    //    namedWindow("window1");
    //    imshow("window1",ImageGray);
    //    waitKey();
    //    /// end test

    int block_size = 205;//105;
	adaptiveThreshold(ImageGray, ImageGray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, block_size, 0);

    //    /// test
    //    namedWindow("window2");
    //    imshow("window2",ImageGray);
    //    waitKey();
    //    /// end test

    //// 选择ROI
	int close_type = cv::MORPH_ELLIPSE;
    int dilate_size = 2;
    Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
    dilate(ImageGray, ImageGray, element,Point(-1, -1),1);

    //    /// test
    //    imshow("window2",ImageGray);
    //    waitKey();
    //    /// end test
    erode(ImageGray, ImageGray, element,Point(-1, -1),1);



    Point2f leftTop = mousePnts[0]; Point2f leftDown = mousePnts[3];

    /*int out = 0;
    while (true)
    {
        ImageGray.at<char>()
    }*/

    int scaleSize = 1;
    vector<Point2f> mousePnts_half;
    for (int i = 0; i < mousePnts.size();i++)
    {
        mousePnts_half.push_back(Point2f(mousePnts[i].x/scaleSize,mousePnts[i].y/scaleSize));
    }


    if (scaleSize!=1)
        resize(ImageGray, ImageGray,Size(ImageGray.cols/scaleSize,ImageGray.rows/scaleSize));

    //创建标定板区域的ROI
    RotatedRect ROI = minAreaRect(mousePnts_half);
	cv::Rect roi = ROI.boundingRect();

    ///// 检查ROI区域 如果超过范围，则赋值图像最大尺寸
    if(roi.x<0)
    {
        roi.x=0;
    }
    if(roi.y<0)
    {
        roi.y=0;
    }
    if(roi.x+roi.width>ImageGray.cols-1)
    {
        roi.width = ImageGray.cols-roi.x -1;
    }
    if(roi.y+roi.height>ImageGray.rows-1)
    {
        roi.height=ImageGray.rows-roi.y-1;
    }
    Mat imgGray_ROI = ImageGray(roi).clone();


    //    /// test
    //    imshow("window2",imgGray_ROI);
    //    waitKey();
    //    /// end test

    //从两边扫描，减少遍历时间
    for (int i = 0; i < imgGray_ROI.rows; i++)
    {
        bool minArea = false;
        for (int j = 0; j < imgGray_ROI.cols;j++)
        {
            int row = i + roi.y;
            int col = j + roi.x;
            if ( containsPnt(mousePnts_half, Point2i(col,row))<0)
            {
                imgGray_ROI.at<char>(i, j) = 255;
            }
            else
            {
                minArea = true;
                break;
            }
        }
        if (minArea = true)
        {
            for (int j = imgGray_ROI.cols-1; j >=0 ;j--)
            {
                int row = i + roi.y;
                int col = j + roi.x;
                if ( containsPnt(mousePnts_half, Point2i(col,row))<0)
                {
                    imgGray_ROI.at<char>(i, j) = 255;
                }
                else
                {
                    break;
                }
            }
        }
    }

    //    /// test

    //    imshow("window2",imgGray_ROI);
    //    waitKey();
    //    /// end test


    Mat img_threshold;
    Canny(imgGray_ROI,img_threshold,50,255,3,true);/// 255,  3

    //    /// test
    //    namedWindow("window1");
    //    imshow("window1",img_threshold);
    //    //imwrite("img_threshold.bmp",img_threshold);
    //    waitKey();
    //    /// end test
    //寻找轮廓
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(img_threshold,contours,hierarchy,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);

    Mat contourImg = Mat::zeros(img_threshold.size(),CV_8U);
    drawContours(contourImg, contours, -1, Scalar(255, 255, 255));

    //椭圆拟合,找中心,并进行筛选
    RotatedRect rotatedBox;
    vector<RotatedRect> rotatedBoxVec;
    vector<RotatedRect> temp_rotatedBoxVec;

    for( vector< vector<Point> >::iterator itr=contours.begin();itr!=contours.end();++itr)
    {
        static int n=0;
        n++;
        //if(itr->size()<3)
        if(itr->size()<15) 
            //if(itr->size()<80)
            //if(itr->size()<80 || itr->size()>450)
            continue;

        try
        {
            rotatedBox = fitEllipse((*itr));
			//zzl add  滤除不是椭圆的区块
			double contour_area = contourArea(*itr);
			float ellipse_S = 0.25*3.14159*rotatedBox.size.width*rotatedBox.size.height;//用面积的筛选方法，效果比较好
			cout << abs(ellipse_S - contour_area) << endl;
			if (abs(ellipse_S - contour_area) > 30)
				continue;

        }
        catch (...)//三个点表示任何异常
        {
            //qDebug()<<"fitEllipse error:"<< n ;
            continue;
        }
        temp_rotatedBoxVec.push_back(rotatedBox);
        float height = rotatedBox.size.height;
        float width = rotatedBox.size.width;
        plane2dPnts.push_back(rotatedBox.center);
        //        if ( height>width? height/width<2.5 : width/height<2.5 )
        //        {
        //            //x此处加上gradient的椭圆检测的算法，看下检测精度
        //            //just for test
        //            cv::Rect ROIsingle = cv::Rect(Point2i((int)abs(rotatedBox.center.x-width/2-10),(int)abs(rotatedBox.center.y-height/2-10)),cv::Size(int(width+20),int(height+20)));
        //            //防止越界判断
        //            if(ROIsingle.x<0)
        //            {
        //                ROIsingle.x=0;
        //            }
        //            if(ROIsingle.y<0)
        //            {
        //                ROIsingle.y=0;
        //            }
        //            if(ROIsingle.x+ROIsingle.width>img_threshold.cols-1)
        //            {
        //                ROIsingle.width = img_threshold.cols-ROIsingle.x -1;
        //            }
        //            if(ROIsingle.y+ROIsingle.height>img_threshold.rows-1)
        //            {
        //                ROIsingle.height=img_threshold.rows-ROIsingle.y-1;
        //            }
        //            vector<RotatedRect> findResulttemp;
        //            if(!CoreAlgorithm::findEllipses(imgroi2,ROIsingle,findResulttemp,5,1,3))
        //            {
        //                continue;
        //            }
        //            if(findResulttemp.size()==2)
        //            {
        //                if(findResulttemp[0].size.width>40&&findResulttemp[1].size.width>40)
        //                {
        //                    Point2f centerTemp;
        //                    centerTemp.x = (findResulttemp[0].center.x+findResulttemp[1].center.x)/2;
        //                    centerTemp.y = (findResulttemp[0].center.y+findResulttemp[1].center.y)/2;
        //                    plane2dPnts.push_back(centerTemp);
        //                }
        //                else
        //                {
        //                    continue;
        //                }
        //            }
        //            else
        //            {
        //                continue;
        //            }
        //            //just for test
        //            //plane2dPnts.push_back(rotatedBox.center);
        //        }
    }

    for (int i = 0; i < plane2dPnts.size();i++)
    {
        plane2dPnts[i].x += roi.x;
        plane2dPnts[i].y += roi.y;
    }

    if (!sortRingCenter(mousePnts_half,plane2dPnts, row, col))
        return false;

    for (int i = 0; i < plane2dPnts.size();i++)
    {
        plane2dPnts[i].x *= scaleSize;
        plane2dPnts[i].y *= scaleSize;
        circle(ImageGray, plane2dPnts[i], 1, Scalar(0, 0, 0));
        //putText(ImageGray,QString("%1").arg(i).toStdString(),plane2dPnts[i],FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,0,0));
    }
    return true;
}

bool CoreAlgorithm::isOnLine(const Point2f& pnt1, const Point2f& pnt2, const Point2f& pnt3)
{
    float dis_threshold = 15;//0.1*sqrt((pnt1.x-pnt2.x)*(pnt1.x-pnt2.x) + (pnt1.y-pnt2.y)*(pnt1.y-pnt2.y));
    float a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
    float b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);

    //点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
    float dis = fabs(a*pnt3.x + b*pnt3.y + 1) / sqrt(a*a + b*b);

    if (dis > dis_threshold)
        return false;

    return true;
}

bool CoreAlgorithm::sortRingCenter(const vector<Point2f>& mousePnts,vector<Point2f>& center,int& row,int& col)
{
    //找出四个角上的点  即分别距离四个角点最近的点
    vector<Point2f> cornerRingPnts;
    for (int i = 0; i < mousePnts.size(); i++)
    {
        Point2f pnt = center[0];
        float minDis = sqrtf((pnt.x-mousePnts[i].x)*(pnt.x-mousePnts[i].x)+
                             (pnt.y-mousePnts[i].y)*(pnt.y-mousePnts[i].y));
        for (int n = 1; n < center.size(); n++)
        {
            float dis = sqrtf((center[n].x-mousePnts[i].x)*(center[n].x-mousePnts[i].x)+
                              (center[n].y-mousePnts[i].y)*(center[n].y-mousePnts[i].y));
            if (dis < minDis)
            {
                minDis = dis;
                pnt = center[n];
            }
        }
        cornerRingPnts.push_back(pnt);
    }

    //找出在左右两列上的点
    Point2f pntlefttop = cornerRingPnts[0];
    Point2f pntrighttop = cornerRingPnts[1];
    Point2f pntrightdown = cornerRingPnts[2];
    Point2f pntleftdown = cornerRingPnts[3];

    vector<Point2f> leftColPnts,rightColPnts;
    for (int i = 0; i < center.size();i++)
    {
        if (isOnLine(pntlefttop, pntleftdown, center[i]))
        {
            leftColPnts.push_back(center[i]);
        }
        else if (isOnLine(pntrighttop, pntrightdown, center[i]))
        {
            rightColPnts.push_back(center[i]);
        }
    }

    if ( rightColPnts.size()!=leftColPnts.size())
        return false;

    row = rightColPnts.size();

    //左右两列的点按照top2down的顺序排序,既按照向量方向排序
    //左右两列点先按照Y坐标排序，再判断到top点的距离
    sortByYMin2Max(rightColPnts);
    float dis2Top = distance(rightColPnts[row - 2], pntrighttop);
    float dis2Top2 = distance(rightColPnts[1] , pntrighttop);
    if ( dis2Top < dis2Top2)
        reverse(rightColPnts);

    sortByYMin2Max(leftColPnts);
    dis2Top = distance(leftColPnts[row - 2], pntlefttop);
    dis2Top2 = distance(leftColPnts[1] , pntlefttop);
    if ( dis2Top < dis2Top2)
        reverse(leftColPnts);

    //从第最后一行开始找
    vector<Point2f> result;
    int colSize = 0;
    for (int n = 0; n <leftColPnts.size() ;n++)
    {
        vector<Point2f> rowPnts;
        for (int i = 0; i < center.size(); i++)
        {
            if (isOnLine(leftColPnts[n], rightColPnts[n], center[i]))
            {
                rowPnts.push_back(center[i]);
            }
        }

        sortByXMin2Max(rowPnts);

        float dis2left = distance(rowPnts[row - 2],leftColPnts[n]);
        float dis2left2 = distance(rowPnts[1], leftColPnts[n]);

        if (dis2left < dis2left2)
            reverse(rowPnts);

        if (colSize == 0)
            colSize = rowPnts.size();
        else
        {
            if (colSize != rowPnts.size())
                return false;
        }

        for (int i = 0; i <rowPnts.size() ;i++)
        {
            result.push_back(rowPnts[i]);
        }
    }

    col = colSize;
    center = result;
    return true;
}

bool CoreAlgorithm::extraRingCenter2(const Mat& img, const vector<Point2f>& mousePnts, vector<Point2f>& ImageCorner, int& row, int& col)
{

    return true;
}

void CoreAlgorithm::sortByXMin2Max(vector<Point2f>& pnts)
{
    for(int i=0;i<pnts.size()-1;i++)
    {
        for (int j = 0; j<pnts.size() - 1; j++)
        {
            if (pnts[j].x>pnts[j + 1].x)
            {
                swap(pnts[j], pnts[j + 1]);
            }
        }
    }
}

void CoreAlgorithm::sortByYMin2Max(vector<Point2f>& pnts)
{
    for(int i=0;i<pnts.size()-1;i++)
    {
        for (int j = 0; j<pnts.size() - 1; j++)
        {
            if (pnts[j].y>pnts[j + 1].y)
            {
                swap(pnts[j], pnts[j + 1]);
            }
        }
    }
}

void CoreAlgorithm::reverse(vector<Point2f>& pnts)
{
    vector<Point2f> result;
    for (int i = pnts.size()-1; i >=0 ; i--)
    {
        result.push_back(pnts[i]);
    }
    pnts = result;
}

bool CoreAlgorithm::ringSubPix(const Mat& img, vector<Point2f>& centers,const Mat& tampl)
{
    //获得中心间距
    int count = 0;
    float space = 0;
    float dis_max = distance(centers[0], centers[1]) + 30;
    for (int i = 0; i < centers.size()-1; i++)
    {
        float _dis = distance(centers[i], centers[i+1]);
        if (_dis < dis_max)
        {
            space += _dis;
            count++;
        }
    }
    space /= count;

    space  = cvRound(space);

    //如果输入的模板为空由中心距创建模板
    vector<float> dx_vec, dy_vec;
    Mat _tampl;
    if (tampl.empty())
    {
        _tampl = img(Rect(Point2f(centers[0].x - (space-1) / 2, centers[0].y - (space-1) / 2),
                Point2f(centers[0].x + (space+1) / 2, centers[0].y + (space+1) / 2))).clone();
    }
    else
    {
        ///// 对图像模板tampl  按照中心距离space进行resize
        cv::resize(tampl,_tampl,Size(int(space),int(space)),0,0,cv::INTER_CUBIC );
    }


    //创建模板匹配ROI，这个ROI的边界扩展一个space是不够的，应该是1.5个space
    Mat img_ROI;
    Rect ROI = boundingRect(centers);

    ROI.x = ROI.x - 1.5*space;
    ROI.y = ROI.y - 1.5*space;
    ROI.width = ROI.width + 3 * space;
    ROI.height = ROI.height + 3 * space;
    /// zhangxu added 判断ROI是否越界
    if(ROI.x<0)
    {
        ROI.x = 0;
    }
    if(ROI.y<0)
    {
        ROI.y = 0;
    }
    if(ROI.x+ROI.width>img.cols-1)
    {
        ROI.width = img.cols - ROI.x -1;
    }
    if(ROI.y+ROI.height>img.rows-1)
    {
        ROI.height = img.rows - ROI.y -1;
    }
    ////// end the test and obtain the ROI
    img_ROI = img(ROI);

    //CV_TM_CCORR_NORMED、CV_TM_CCOEFF、CV_TM_CCOEFF_NORMED，越亮匹配度越高
    Mat result;
    matchTemplate(img_ROI, _tampl, result, CV_TM_CCOEFF);  ///// 这个matchTemplate要求输入的两个矩阵的变量类型是一致的，要么都是无符号整型8位，要么都是浮点型32位
    normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, Mat());
    int centerArea_Size = 3;

    for (int n = 0; n < centers.size(); n++)
    {
        //获得中心点在匹配结果result图像中的坐标，因为result 的尺寸是  (W-w+1) X (H-h+1)，但是转换成坐标就不能使用用size来计算

        float x = centers[n].x - float(_tampl.cols-1) / 2 - ROI.x;
        float y = centers[n].y - float(_tampl.rows-1) / 2 - ROI.y;

        //在几何中心点附近找出匹配中心点
        double minVal; double maxVal; Point minLoc; Point maxLoc;
        ///进行越界检查
        Rect mask_roi = Rect(Point2f(cvRound(x)-centerArea_Size,cvRound(y)-centerArea_Size),
                             Point2f(cvRound(x)+centerArea_Size+1,cvRound(y)+centerArea_Size+1));
        if(mask_roi.x<0)
        {
            mask_roi.x = 0;
        }
        if(mask_roi.y<0)
        {
            mask_roi.y = 0;
        }
        if(mask_roi.x+mask_roi.width>result.cols-1)
        {
            mask_roi.width = result.cols - ROI.x -1;
        }
        if(mask_roi.y+mask_roi.height>result.rows-1)
        {
            mask_roi.height = result.rows - ROI.y -1;
        }

        Mat centerArea = result(mask_roi);
        minMaxLoc( centerArea, &minVal, &maxVal, &minLoc, &maxLoc );

        Point matchLoc = maxLoc;
        ////// 此处计算出像素级最大点，在result中的坐标
        matchLoc.x += cvRound(x-centerArea_Size);
        matchLoc.y += cvRound(y-centerArea_Size);

        //创建亚像素检测区域
        Mat win(3,3,CV_32F);
        for (int i = 0; i < 3;i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int col = matchLoc.x+(j-1);
                int row = matchLoc.y+(i-1);
                //// test
                if(row<0 ||row>result.rows -1 || col<0  || col >result.cols-1)
                {
                    return false;
                }
                float dd = result.at<float>(row, col);
                ////end  test
                win.at<float>(i, j) = result.at<float>(row, col);

            }
        }

        float dx=0, dy=0;
        quadSubPixel(win,dx,dy);
        //// zhang xu added 得到此时的像素精度级的坐标
        centers[n].x = matchLoc.x + float(_tampl.cols-1) / 2 + ROI.x;
        centers[n].y = matchLoc.y + float(_tampl.rows-1) / 2 + ROI.y;
        //// 增加亚像素的坐标偏移量
        centers[n].x += dx;
        centers[n].y += dy;
    }
    return true;
}

bool CoreAlgorithm::ringSubPixAdv(const Mat& img, vector<Point2f>& centers, uint mode)
{
    //判断行数和列数,即当中心距突然大于多个中心距之和时,该中心距为行末点和下行起始点之距
    uint cols = 1;//列数初始化为1
    float sum_dis = 0;
    for (uint i = 0; i < centers.size()-1; i++)
    {
        float _dis = distance(centers[i], centers[i+1]);
        if (i >= 2)
        {
            if (_dis > sum_dis)
                break;//该中心距为行末点和下行起始点之距
        }
        sum_dis += _dis;
        cols++;
    }

    //获取每个中心点的ROI
    vector<Rect> centers_rect;
    float roiWidth,roiHeight;
    roiWidth = distance(centers[0], centers[1]);//计算中心点间距
    if (centers.size() > cols)
        roiHeight = distance(centers[0], centers[cols]);
    else//只有一行
        roiHeight = roiWidth;
    for(uint n = 0; n < centers.size(); n++)
    {
        Point2i leftTop, rightBottom;
        leftTop.x = cvRound(centers[n].x - roiWidth/2);
        leftTop.y = cvRound(centers[n].y - roiHeight/2);
        rightBottom.x = cvRound(centers[n].x + roiWidth/2);
        rightBottom.y = cvRound(centers[n].y + roiHeight/2);
        Rect rectROI(leftTop,rightBottom);
        if(rectROI.x<0)//ROI越界检查
        {
            rectROI.x = 0;
        }
        if(rectROI.y<0)
        {
            rectROI.y = 0;
        }
        if(rectROI.x+rectROI.width>img.cols-1)
        {
            rectROI.width = img.cols - rectROI.x - 1;
        }
        if(rectROI.y+rectROI.height>img.rows-1)
        {
            rectROI.height = img.rows - rectROI.y - 1;
        }
        centers_rect.push_back(rectROI);
    }

    for(uint n = 0; n < centers.size(); n++)
    {
        vector<RotatedRect> findResults;
        if (mode == 0)
        {
            findEllipses(img,centers_rect[n],findResults,0.05,0,3);//内圆
            if (findResults.size() != 2)
                cerr<<endl<<"warning, findResults.size() = "<<findResults.size()<<endl;
            float areamin = findResults[0].size.area();
            if (findResults[1].size.area()<=areamin)
            {
                centers[n].x = findResults[1].center.x;
                centers[n].y = findResults[1].center.y;
            }
            else
            {
                centers[n].x = findResults[0].center.x;
                centers[n].y = findResults[0].center.y;
            }
        }
        else if(mode == 1)
        {
            findEllipses(img,centers_rect[n],findResults,0.05,0,3);//外圆
            if (findResults.size() != 2)
                cerr<<endl<<"warning, findResults.size() = "<<findResults.size()<<endl;
            float areamax = findResults[0].size.area();
            if (findResults[1].size.area()>=areamax)
            {
                centers[n].x = findResults[1].center.x;
                centers[n].y = findResults[1].center.y;
            }
            else
            {
                centers[n].x = findResults[0].center.x;
                centers[n].y = findResults[0].center.y;
            }
        }
        else if(mode == 2)
        {
            findEllipses(img,centers_rect[n],findResults,0.05,0,3);//内外圆平均值
            if (findResults.size() != 2)
                cerr<<endl<<"warning, findResults.size() = "<<findResults.size()<<endl;
            centers[n].x = (findResults[0].center.x+findResults[1].center.x)/2;
            centers[n].y = (findResults[0].center.y+findResults[1].center.y)/2;
        }
        else
        {
            findEllipses(img,centers_rect[n],findResults,0.05,1,3);//内外圆优化平均值
            if (findResults.size() != 2)
                cerr<<endl<<"warning, findResults.size() = "<<findResults.size()<<endl;
            centers[n].x = (findResults[0].center.x+findResults[1].center.x)/2;
            centers[n].y = (findResults[0].center.y+findResults[1].center.y)/2;
        }
    }
    return true;
}

cv::Mat CoreAlgorithm::generateRingTemp(const vector<Point2f>& centers)
{
    Mat tempImg;
    float dis_threshold = 100;

    int count = 0;
    float space = 0;
    for (int i = 0; i < centers.size()-1; i++)
    {
        float _dis = distance(centers[i], centers[i+1]);
        if (_dis < dis_threshold)
        {
            space += _dis;
            count++;
        }
    }
    space /= count;

    float R,r;
    R = 0.25*space; r = 0.5*R;//根据固定的比例

    //R = 27; r = 0.5*R;//test ,这个半径不对的话问题会很大
    int white_size=7;
    tempImg = Mat(2*R+white_size,2*R+white_size,CV_8UC1);
    for (int i = 0; i < tempImg.rows;i++)
    {
        for (int j = 0; j < tempImg.cols;j++)
        {
            tempImg.at<char>(i, j) = 255;
        }
    }

    Point2f center = Point2f((tempImg.cols - 1) / 2,(tempImg.rows - 1) / 2);
    cv::circle( tempImg, center,R, Scalar(0), -1, 8 );
    cv::circle( tempImg, center,r, Scalar(255), -1, 8 );

    return tempImg.clone();
}

float CoreAlgorithm::distance(const Point2f& pnt1, const Point2f& pnt2)
{
    return sqrtf(pow(pnt1.x - pnt2.x,2) + pow(pnt1.y - pnt2.y,2));
}

double CoreAlgorithm::containsPnt(vector<Point2f> mousePnts, Point2f pnt)
{
    return pointPolygonTest(mousePnts,pnt,false);
}

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// 张召瑞 //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/// 机器人手眼标定
/// vector<Mat> ObjToCam, <in> 标定板到摄像机坐标变换
/// vector<Mat> HandToBase, <in> 机器人手爪到机器人基座坐标变换
/// vector<Mat>& Outputs, <out> 输出标定要求的变换矩阵
/// bool type, 0为eye-in-hand模型，1为eye-to-hand模型
bool CoreAlgorithm::HandEyeCalibration(const vector<Mat> ObjToCamSrc, const vector<Mat> HandToBaseSrc, vector<Mat>& Outputs, bool type)
{
    //bool gflag = false;//桌面机器人的情况标志
    if (ObjToCamSrc.size() != HandToBaseSrc.size())//如果ObjToCam和HandToBase的数量不等，则出错
    {
        cout<<"Input Error!"<<endl;
        return false;
    }

    if((2 >= ObjToCamSrc.size()) || (2 >= HandToBaseSrc.size()))//如果ObjToCam的数量或HandToBase的数量不足3，则出错
    {
        cout<<"Input Error!"<<endl;
        return false;
    }

    //定义分别存储Rc、Rd、tc和td矩阵及其容器
    vector< vector<Mat> > RT_Cs;
    vector< vector<Mat> > RT_Ds;

    //定义分别存储C矩阵和D矩阵的容器
    vector<Mat> Cs;
    vector<Mat> Ds;

    /// added at 15-05-07
    //为了消除微小误差带来的秩计算错误,先将HandToBase矩阵集作预处理,同时判断是否是纯移动
    vector<Mat> ObjToCam = ObjToCamSrc;
    vector<Mat> HandToBase = HandToBaseSrc;
    //    for(uint i = 0; i < HandToBaseScr.size(); i++)
    //    {
    //        Mat tempMat = HandToBaseScr[i];
    //        HandToBase.push_back(tempMat);
    //    }
    vector<uint> translationalMotion;
    uint RotationCnt = 0;//所有运动中带旋转运动的总次数
    for (uint i = 1; i < HandToBase.size(); i++)
    {
        double disum = 0.0;
        RotationCnt++;
        for(uint m = 0; m < 3; m++)
        {
            for(uint n = 0; n < 3; n++)
            {
                double dis = abs(HandToBase[i].at<double>(m,n) - HandToBase[i-1].at<double>(m,n));
                disum += dis;
            }
        }
        if (disum*disum < 0.01)//两个HanToBase矩阵的旋转矩阵部分可认为相同,则认为是纯平移,强行将他们相等
        {
            uint translationalMotionFlag = 1;
            translationalMotion.push_back(translationalMotionFlag);
            RotationCnt--;
            for(uint m = 0; m < 3; m++)
            {
                for(uint n = 0; n < 3; n++)
                {
                    HandToBase[i].at<double>(m,n) = HandToBase[i-1].at<double>(m,n);
                }
            }
        }
        else
        {
            uint translationalMotionFlag = 0;
            translationalMotion.push_back(translationalMotionFlag);
        }
    }
    cout<<"RotationCnt = "<<RotationCnt<<endl;
    cout<<"translationalMotionFlag : ";
    for(uint i = 0; i < translationalMotion.size(); i++)
        cout<<translationalMotion[i]<<", ";
    cout<<endl;
    if (RotationCnt < 1)//至少需要1个带旋转的运动，否则无法计算
    {
        cout<<"Error! Need more rotational Pose!"<<endl;
        return false;
    }
    /// end added

    //计算C和D矩阵
    if(0 == type)//判断是eye-in-hand(0)还是eye-to-hand(1)问题，并求不同的D矩阵
    {
        for(int i = 0; i < (ObjToCam.size() - 1); i++)
        {
            Mat C = Mat::zeros(4, 4, CV_64FC1);
            Mat TempInv = Mat::zeros(4, 4, CV_64FC1);
            invert(HandToBase[i+1], TempInv);
            C = TempInv * HandToBase[i];
            Cs.push_back(C);
        }
    }
    else
    {
        for(int i = 0; i < (ObjToCam.size() - 1); i++)
        {
            Mat C = Mat::zeros(4, 4, CV_64FC1);
            Mat TempInv = Mat::zeros(4, 4, CV_64FC1);
            invert(HandToBase[i], TempInv);
            C = HandToBase[i+1] * TempInv;
            Cs.push_back(C);
            /// test
            /// cout<<"C"<<i<<" = "<<C<<endl;
        }
    }
    for(int i = 0; i < (ObjToCam.size() - 1); i++)
    {
        Mat D = Mat::zeros(4, 4, CV_64FC1);
        Mat TempInv = Mat::zeros(4, 4, CV_64FC1);
        invert(ObjToCam[i], TempInv);
        D = ObjToCam[i+1] * TempInv;
        Ds.push_back(D);

        /// test
        /// cout<<"D"<<i<<" = "<<D<<endl;
    }

    //将对应的Rc、Rd、tc和td矩阵提取出来并存到容器里
    for(int i = 0; i < (ObjToCam.size() - 1); i++)
    {
        vector<Mat> RT_C;
        vector<Mat> RT_D;

        Mat Rc = Mat::zeros(3, 3, CV_64FC1);
        Mat Rd = Mat::zeros(3, 3, CV_64FC1);
        Mat tc = Mat::zeros(3, 1, CV_64FC1);
        Mat td = Mat::zeros(3, 1, CV_64FC1);

        Mat M34_1 = Mat::zeros(3, 4, CV_64FC1);
        Mat M34_2 = Mat::zeros(3, 4, CV_64FC1);
        for(int j = 0; j < 3; j++)
        {
            M34_1.row(j) = M34_1.row(j) + Cs[i].row(j);
            M34_2.row(j) = M34_2.row(j) + Ds[i].row(j);
        }
        for(int j = 0; j < 3; j++)
        {
            Rc.col(j) = Rc.col(j) + M34_1.col(j);//提取Rc
            Rd.col(j) = Rd.col(j) + M34_2.col(j);//提取Rd

            //提取tc
            Mat M41_1 = Mat::zeros(4, 1, CV_64FC1);
            M41_1 = M41_1 + Cs[i].col(3);
            tc.row(j) = tc.row(j) + M41_1.row(j);

            //提取td
            Mat M41_2 = Mat::zeros(4, 1, CV_64FC1);
            M41_2 = M41_2 + Ds[i].col(3);
            td.row(j) = td.row(j) + M41_2.row(j);
        }

        RT_C.push_back(Rc);
        RT_C.push_back(tc);
        RT_D.push_back(Rd);
        RT_D.push_back(td);

        RT_Cs.push_back(RT_C);
        RT_Ds.push_back(RT_D);

        /// test
        // cout<<"Rd"<<i<<" = "<<Rd<<endl;
        //cout<<"td"<<i<<" = "<<td<<endl;
        // cout<<"Rc"<<i<<" = "<<Rc<<endl;
        //cout<<"tc"<<i<<" = "<<tc<<endl;
    }

    //定义待求矩阵X1的R和T矩阵
    Mat R = Mat::zeros(3, 3, CV_64FC1);
    Mat T = Mat::zeros(3, 1, CV_64FC1);
    vector<Mat> RT;


    //-------------------------求X1的R矩阵----------------------------//

    //求各自矩阵的旋转矩阵相对应的全部特征向量（旋转向量），并组成矩阵
    vector<Mat> kcs;
    vector<Mat> kds;
    for(int i = 0; i < RT_Cs.size(); i++)
    {
        if (translationalMotion[i])//仅仅是纯平动
        {
            Mat kc = Mat((cv::Mat_ <double> (3,1) <<0,0,0));
			Mat kd = Mat((cv::Mat_ <double>(3, 1) << 0, 0, 0));

            //将T矩阵压入M1和M2中
            kc = RT_Cs[i][1];
            kd = RT_Ds[i][1];

            //单位化
            normalize(kc, kc);
            normalize(kd, kd);

            kcs.push_back(kc);
            kds.push_back(kd);
        }
        else//有旋转运动
        {
			Mat kc = Mat((cv::Mat_ <double>(3, 1) << 0, 0, 0));
			Mat kd = Mat((cv::Mat_ <double>(3, 1) << 0, 0, 0));
            Rodrigues(RT_Cs[i][0], kc);
            Rodrigues(RT_Ds[i][0], kd);

            //单位化
            normalize(kc, kc);
            normalize(kd, kd);

            kcs.push_back(kc);
            kds.push_back(kd);

            /// test
            //cout<<"kc"<<i<<" = "<<kc<<endl;
            //cout<<"kd"<<i<<" = "<<kd<<endl;
        }
    }

    /// 定义标定板到相机以及机器人手爪到机器人基座的旋转矩阵的特征向量组矩阵
    Mat M1, M2, InvM2;
    if (RT_Cs.size() == 2)
    {
        int size = 3;
        M1 = Mat::zeros(3, size, CV_64FC1);
        M2 = Mat::zeros(3, size, CV_64FC1);
        InvM2 = Mat::zeros(size, 3, CV_64FC1);
    }
    else
    {
        int size = RT_Cs.size();
        M1 = Mat::zeros(3, size, CV_64FC1);
        M2 = Mat::zeros(3, size, CV_64FC1);
        InvM2 = Mat::zeros(size, 3, CV_64FC1);
    }

    //    if (2 == RT_Cs.size())//如果只有2次运动,则先假设这两次运动都有旋转运动,然后第三个旋转运动由他们叉乘得出
    //    {
    //        Mat kc3 = kcs[0].cross(kcs[1]);
    //        Mat kd3 = kds[0].cross(kds[1]);

    //        //单位化
    //        normalize(kc3, kc3);
    //        normalize(kd3, kd3);

    //        for(int i = 0; i < RT_Cs.size(); i++)
    //        {
    //            M1.col(i) = M1.col(i) + kcs[i];
    //            M2.col(i) = M2.col(i) + kds[i];
    //        }
    //        M1.col(2) = M1.col(2) + kc3;
    //        M2.col(2) = M2.col(2) + kd3;

    //        //求秩并判断是否满秩
    //        uint rankM1 = rankCompute(M1);
    //        if (rankM1 != 3)
    //        {
    //            cout<<"Error! Need more rotational Pose!"<<endl;//只运动2次时,旋转向量最多2个，和叉乘得到的旋转向量秩不为3
    //            return false;
    //        }
    //    }

    uint rankM1 = rankCompute(M1);
    for(int i = 0; i < RT_Cs.size(); i++)
    {
        M1.col(i) = M1.col(i) + kcs[i];
        M2.col(i) = M2.col(i) + kds[i];
    }

    //求秩并判断是否满秩
    rankM1 = rankCompute(M1);
    uint rankM2 = rankCompute(M2);

    /// test
    cout<<"M1 = "<<M1<<endl;
    cout<<"rankM1 = "<<rankM1<<endl;
    cout<<"M2 = "<<M2<<endl;

    double mindot;//点积最小值
    Mat crossnewkc, crossnewkd;//叉乘得到的新向量tc和td
    int mindotlocate[2] = {0,0};//点积最小值的两向量在容器中的位置
    bool flag = false;//用于判断第一组点乘不为0的标志
    if (rankM1 == 2 && rankM2 >= 2)//此情况需要两个向量叉乘得到第三个垂直于它们的向量
    {
        //找到最不平行的两组旋转向量并叉乘
        for(int i = 0; i < kcs.size(); i++)
        {
            if (norm(kcs[i]) == 0)//判断旋转向量是否为0
                continue;
            for(int j = i+1; j < kcs.size(); j++)
            {
                if (norm(kcs[j]) == 0)//判断旋转向量是否为0
                    continue;
                double dottemp = kcs[i].dot(kcs[j]);
                if (!flag)
                {
                    mindot = dottemp;
                    mindotlocate[0] = i;
                    mindotlocate[1] = j;
                    flag = true;
                }
                else if(dottemp < mindot)
                {
                    mindot = dottemp;
                    mindotlocate[0] = i;
                    mindotlocate[1] = j;
                }
            }
        }
        crossnewkc = kcs[mindotlocate[0]].cross(kcs[mindotlocate[1]]);
        crossnewkd = kds[mindotlocate[0]].cross(kds[mindotlocate[1]]);

        //单位化
        normalize(crossnewkc, crossnewkc);
        normalize(crossnewkd, crossnewkd);

        Mat TM1,TM2,Tcrossnewkc,Tcrossnewkd;
        TM1 = M1.t();
        TM2 = M2.t();
        Tcrossnewkc = crossnewkc.t();
        Tcrossnewkd = crossnewkd.t();
        TM1.push_back(Tcrossnewkc);
        TM2.push_back(Tcrossnewkd);
        M1 = TM1.t();
        M2 = TM2.t();
        rankM1 = rankCompute(M1);
    }
    else if(rankM1 < 2)//姿态数量太少
    {
        cout<<"Error! Need more Pose!"<<endl;
        return false;
    }
    //    else if (rankM1 == 1 || rankM2 >= 1)//桌面式机器人手眼标定
    //    {
    //        vector< vector<Mat> >::iterator iterRc = RT_Cs.begin();
    //        vector< vector<Mat> >::iterator iterRd = RT_Ds.begin();
    //        //找到旋转向量为0的情况，并做相应处理
    //        Mat TM1,TM2,Ttc,Ttd;
    //        for(int i = 0; iterRc != RT_Cs.end(); i++)
    //        {
    //            if (norm(kcs[i]) == 0)//判断旋转向量是否为0
    //            {
    //                Mat tc,td;
    //                //                    vector<Mat> TempRc = *iterRc;
    //                //                    vector<Mat> TempRd = *iterRd;

    //                //平移向量先单位化
    //                normalize((*iterRc)[1], tc);
    //                normalize((*iterRd)[1], td);

    //                Ttc = tc.t();
    //                Ttd = td.t();
    //                TM1.push_back(Ttc);
    //                TM2.push_back(Ttd);

    //                iterRc = RT_Cs.erase(iterRc);
    //                iterRd = RT_Ds.erase(iterRd);

    //                //                    vector< vector<Mat> >::iterator iterTemp = RT_Cs.erase(iterRc);
    //                //                    TempRc = iterTemp;
    //                //                    iterTemp = RT_Cd.erase(iterRd);
    //                //                    TempRd = iterTemp;
    //            }
    //            else
    //            {
    //                Ttc = kcs[i].t();
    //                Ttd = kds[i].t();
    //                TM1.push_back(Ttc);
    //                TM2.push_back(Ttd);

    //                iterRc++;
    //                iterRd++;
    //            }
    //        }
    //        M1 = TM1.t();
    //        M2 = TM2.t();

    //        /// test
    //        cout<<"M1 = "<<M1<<endl;
    //        cout<<"M2 = "<<M2<<endl;

    //        if (RT_Cs.size() < 2)//至少需要2个平行的旋转向量，否则无法计算
    //        {
    //            cout<<"Error! Need more rotational Pose!"<<endl;
    //            return false;
    //        }

    //        //求秩并做相应处理
    //        uint rankM1 = rankCompute(M1);
    //        uint rankM2 = rankCompute(M2);

    //        double mindot;//点积最小值
    //        Mat crossnewtc, crossnewtd;//叉乘得到的新向量tc和td
    //        int mindotlocate[2] = {0,0};//点积最小值的两向量在容器中的位置
    //        bool flag = false;//用于判断第一组点乘不为0的标志
    //        if (rankM1 == 2 && rankM2 >= 2)//此情况需要两个向量叉乘得到第三个垂直于它们的向量
    //        {
    //            //找到最不平行的两组向量并叉乘
    //            for(int i = 0; i < M1.cols; i++)
    //            {
    //                if (norm(M1.col(i)) == 0)//判断向量是否为0
    //                    continue;
    //                for(int j = i+1; j < kcs.size(); j++)
    //                {
    //                    if (norm(kcs[j]) == 0)//判断向量是否为0
    //                        continue;
    //                    double dottemp = abs((M1.col(i)).dot(M1.col(j)));
    //                    if (!flag)
    //                    {
    //                        mindot = dottemp;
    //                        mindotlocate[0] = i;
    //                        mindotlocate[1] = j;
    //                        flag = true;
    //                    }
    //                    if (i == 0 && j == 1)
    //                        mindot = dottemp;
    //                    else if(dottemp < mindot)
    //                    {
    //                        mindot = dottemp;
    //                        mindotlocate[0] = i;
    //                        mindotlocate[1] = j;
    //                    }
    //                }
    //            }
    //            crossnewtc = (M1.col(mindotlocate[0])).cross(M1.col(mindotlocate[1]));
    //            crossnewtd = (M2.col(mindotlocate[0])).cross(M2.col(mindotlocate[1]));

    //            //单位化
    //            normalize(crossnewtc, crossnewtc);
    //            normalize(crossnewtd, crossnewtd);

    //            Mat TM1,TM2,Tcrossnewtc,Tcrossnewtd;
    //            TM1 = M1.t();
    //            TM2 = M2.t();
    //            Tcrossnewtc = crossnewtc.t();
    //            Tcrossnewtd = crossnewtd.t();
    //            TM1.push_back(Tcrossnewtc);
    //            TM2.push_back(Tcrossnewtd);
    //            M1 = TM1.t();
    //            M2 = TM2.t();
    //        }
    //        else if(rankM1 < 2 && rankM2 < 2)
    //        {
    //            cout<<"Rank Error two! Need more panning!"<<endl;//线性无关的旋转向量只有1个，和平移向量得到的向量组秩小于2
    //            return false;
    //        }
    //        gflag = true;
    //    }
    //    else if (rankM1 ==0)//无唯一解的手眼标定
    //    {
    //        cout<<"Rank Error! Rotational vectors' rank is 0!"<<endl;
    //        return false;
    //    }
    //    else if(rankM1 == 3 && rankM2 == 2)
    //    {
    //        cout<<"Rank Error two! "<<endl;//意外错误
    //        return false;
    //    }

    /// test
    cout<<"M1 = "<<M1<<endl;
    cout<<"rankM1 = "<<rankM1<<endl;
    cout<<"M2 = "<<M2<<endl;

    //解矩阵方程，得出矩阵待求矩阵X1的R阵
	invert(M2, InvM2, cv::DECOMP_SVD);
    R = M1 * InvM2;

    //进行正交化
    Mat OrthR = Mat::zeros(3, 3, CV_64FC1);
	cv::SVD tempSVD = cv::SVD(R);
    OrthR = tempSVD.u * tempSVD.vt;
    //    OrthR = Mat((Mat_ <double> (3,3) <<0.2966,-0.0529,-0.9535,
    //                 0.9549,-0.0019,0.2971,
    //                -0.0176,-0.9986,0.0500));
    RT.push_back(OrthR);

    /// test
    //cout<<OrthR<<endl;
    cout<<"temSVD "<<tempSVD.w<<endl;
    /// end


    //-------------------------求X1的T矩阵----------------------------//

    Mat I = Mat((Mat_ <double> (3,3) <<1,0,0,0,1,0,0,0,1));//定义单位矩阵

    //定义运算需要的相关矩阵
    Mat M3 = Mat::zeros(3*RotationCnt, 3, CV_64FC1);
    Mat M4 = Mat::zeros(3*RotationCnt, 1, CV_64FC1);
    Mat tempM = Mat((Mat_ <double> (3,3) <<0,0,0,0,0,0,0,0,0));

    //得出运算需要的相关矩阵值
    for (int i = 0, num = 0; i < RT_Cs.size(); i++)
    {
        if (!translationalMotion[i])//如果不为纯平移运动,则用该组tc和td计算M3和M4
        {
            tempM = RT_Cs[i][0] - I;
            M3.row(0 + num * 3) = M3.row(0 + num * 3) + tempM.row(0);
            M3.row(1 + num * 3) = M3.row(1 + num * 3) + tempM.row(1);
            M3.row(2 + num * 3) = M3.row(2 + num * 3) + tempM.row(2);

            tempM = RT[0] * RT_Ds[i][1] - RT_Cs[i][1];
            M4.row(0 + num * 3) = M4.row(0 + num * 3) + tempM.row(0);
            M4.row(1 + num * 3) = M4.row(1 + num * 3) + tempM.row(1);
            M4.row(2 + num * 3) = M4.row(2 + num * 3) + tempM.row(2);

            num++;
        }
    }

    /// added at 15-05-11
    //检查是否满秩,否则无法计算出唯一解,只能得到解集
    uint rankM3 = rankCompute(M3);

    /// test
    cout<<"M3 = "<<endl<<M3<<endl;
    cout<<"rankM3 = "<<rankM3<<endl;
    cout<<"M4 = "<<endl<<M4<<endl;
    /// end test

    bool deskflag = false;
    if (rankM3 == 2)//桌面机器人的情况，无唯一解
    {
        deskflag = true;

        //先对矩阵进行预处理
        Mat A = Mat::zeros(2*RotationCnt, 2, CV_64FC1);
        Mat b = Mat::zeros(2*RotationCnt, 1, CV_64FC1);
        for(int i = 0; i < M3.rows; i++)
        {
            if (i%3 != 2)
            {
                A.at<double>(i,0) = M3.at<double>(i,0);
                A.at<double>(i,1) = M3.at<double>(i,1);
                b.at<double>(i,0) = M4.at<double>(i,0);
            }
        }

        Mat tempT = Mat::zeros(2, 1, CV_64FC1);
        if (!solve(A, b, tempT, cv::DECOMP_SVD))
            return false;
        else
        {
            T.at<double>(0,0) = tempT.at<double>(0,0);
            T.at<double>(1,0) = tempT.at<double>(1,0);
            T.at<double>(2,0) = 0;//如果是桌面机器人手眼标定,则只能标定出5个自由度,剩下的那一个设为0
            RT.push_back(T);
        }

        //        //使用奇异值分解的方法计算矩阵方程，并得出待求矩阵X1的T阵
        //        if (!solve(M3, M4, T, DECOMP_SVD))
        //            return false;
        //        else
        //        {
        //            T.at<double>(2,0) = 0;
        //            RT.push_back(T);
        //        }

        //        Mat invUsvd, invVtsvd;
        //        SVD svdM3 = SVD(M3, SVD::FULL_UV);
        //        invert(svdM3.u, invUsvd);
        //        invert(svdM3.vt, invVtsvd);
        //        Mat UMat = invUsvd * M4;
        //        Mat UMat22 = Mat((Mat_ <double> (2,1) <<UMat.at<double>(0,0), UMat.at<double>(1,0)));
        //        Mat invVtsvd22 = Mat((Mat_ <double> (2,2) <<invVtsvd.at<double>(0,0), 0, 0, invVtsvd.at<double>(1,1)));
        //        Mat invSsvd22 = Mat((Mat_ <double> (2,2) <<1/((svdM3.w).at<double>(0,0)), 0, 0, 1/((svdM3.w).at<double>(1,0))));
        //        Mat tMat22 = invVtsvd22 * invSsvd22 * UMat22;
        //        Mat T1 = Mat((Mat_ <double> (3,1) <<tMat22.at<double>(0,0), tMat22.at<double>(1,0), 0));
        //        Mat T2 = (svdM3.vt).col(2);
        //        cout<<"T1 = "<<T1<<endl;
        //        cout<<"T2 = "<<T2<<endl;
        //        //Mat T = T1 + (random()/double(RAND_MAX)*1000.0-500.0) * T2;
        //        Mat T = T1 + 0 * T2;
        //        cout<<"T = "<<T<<endl;
        //        RT.push_back(T);
        //        //test
        //        cout<<"invVtsvd22 = "<<invVtsvd22<<endl;
        //        cout<<"invSsvd22 = "<<invSsvd22<<endl;
        //        cout<<"UMat22 = "<<UMat22<<endl;

        //        Mat InvM3;
        //        invert(M3, InvM3, DECOMP_SVD);
        //        T = InvM3 * M4;
        //        cout<<"T = "<<T<<endl;
        //        RT.push_back(T);
    }
    else if(rankM3 != 3)
    {
        cout<<"rank of M3 error!"<<endl;
        return false;
    }
    else//rankM3 == 3
    {
        //使用奇异值分解的方法计算矩阵方程，并得出待求矩阵X1的T阵
		if (!solve(M3, M4, T, cv::DECOMP_SVD))
            return false;
        else
            RT.push_back(T);
    }
    /// end added

    //求出X1矩阵并存入容器
    Mat X1 = Mat::zeros(4, 4, CV_64FC1);
    {
        Mat M34 = Mat::zeros(3, 4, CV_64FC1);
        Mat M14 = Mat((Mat_ <double> (1,4) <<0,0,0,1));

        M34.col(0) = M34.col(0) + RT[0].col(0);
        M34.col(1) = M34.col(1) + RT[0].col(1);
        M34.col(2) = M34.col(2) + RT[0].col(2);
        M34.col(3) = M34.col(3) + RT[1];

        X1.row(0) = X1.row(0) + M34.row(0);
        X1.row(1) = X1.row(1) + M34.row(1);
        X1.row(2) = X1.row(2) + M34.row(2);
        X1.row(3) = X1.row(3) + M14;
    }

    Outputs.push_back(X1);


    //-------------------------求X2矩阵----------------------------//

    //求出X2矩阵并存入容器
    Mat X2 = Mat::zeros(4, 4, CV_64FC1);
    Mat InvE1 = Mat::zeros(4, 4, CV_64FC1);
    invert(HandToBase[0], InvE1);
    if (!type)
        X2 = HandToBase[0] * X1 * ObjToCam[0];
    else
    {
        X2 = InvE1 * X1 * ObjToCam[0];

        //如果是桌面机器人手眼标定,则X只能标定出5个自由度,Y矩阵平移量暂时无法正确计算
        //可能算法:确定Y中的平移量[tx,ty,f(z)],使得对所有的X中的平移量分量z,
        //都满足矩阵方程组Y = InvHandToBase[i] * X1 * ObjToCam[i],
        //其中,tx和ty是常数,f(z)是z的函数,z为全体实数,f(z)=-z?和X一样,z一般设为0
        if (deskflag)
        {
            X2.at<double>(0,3) = 0;
            X2.at<double>(1,3) = 0;
            X2.at<double>(2,3) = 0;
        }
    }

    Outputs.push_back(X2);

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// 张召瑞 //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/// 获得投影机对应性数据
/// const vector<Mat>& Images, <in> 输入的一个位置姿态的多幅(12)图像
/// const float Threshold, <in>
/// const int N, <in> 歩数
/// const vector<float> T, <in> 周期
/// vector<Point2f>& pointCam, <out> 一个位置姿态的多幅(12)图像转化的点云
/// vector<float>& proImg_x, <out> 投影机对应性数据,点云的X坐标
bool CoreAlgorithm::ObtainData4ProX(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Point2f>& pointCam, vector<float>& proImg_x)
{
    vector<Mat> CorAll;//用来存放X方向对应性
    vector<Mat> MaskAll;//用来存放对应性的mask
    //引用标定板三维坐标点及二维坐标点
    //    m_CalibrationDataPro.imgHeight=m_CalibrationData.imgHeight;
    //    m_CalibrationDataPro.imgWidth=m_CalibrationData.imgWidth;
    //    m_CalibrationDataPro.frameNumList=m_CalibrationData.frameNumList;


    //获得各个位姿标定板上的对应性及mask
    if(GetCorrespondanceX(Images, Threshold, N, T, CorAll, MaskAll)==false)
    {
        cout<<"Unable to get correspondance"<<endl;
        return false;
    }
    ///// test zx
    //        namedWindow("test2dpro",CV_WINDOW_AUTOSIZE);
    //        imshow("test2dpro",CorAll[0]/1200);
    //        waitKey();
    //        imshow("test2dpro",MaskAll[0]*255);
    //        waitKey();
    //// end test

    //将输入图像Mat的点位置赋值给vector<Point2f>  vector<float>
    //// zx test
    int height = MaskAll[0].rows;
    int width = MaskAll[0].cols;
    //// end test
    for(int i=0;i<MaskAll[0].rows;i++)
    {
        for(int j=0;j<MaskAll[0].cols;j++)
        {
            if(MaskAll[0].at<uchar>(i,j)==1)
            {
                Point2f temPnt;
                temPnt.y = float(i);
                temPnt.x = float(j);
                pointCam.push_back(temPnt);

                float pointCorTemp;
                Mat cor_x=CorAll[0];
                pointCorTemp=cor_x.at<float>(i, j) ;
                proImg_x.push_back(pointCorTemp);
            }
        }
    }

    //// zx test
    //int number = proImg_x.size();
    //// end test

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////// 张召瑞 ////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// 得到单一标定板的投影仪坐标系的物理坐标X
/// const vector<Mat>& Images， <in> 所有相机拍摄到的图像
/// const float Threshold, <in>
/// const int N, <in> 歩数
/// const vector<float> T， <in> 周期
/// Cor_X， <out> 所有图像上x的对应性
/// MaskAll, <out> 所有条纹图像对应性的mask
bool CoreAlgorithm::GetCorrespondanceX(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Mat>& Cor_X, vector<Mat>& Mask_All)
{
    //images[in]所有位姿相机拍摄到的标定板图像
    //cor_All[out]所有位姿标定板上x,y的对应性,在容器中的排序方式为第一个位姿对应性x,y，第二个，x,y，第三个...以此类推
    //MaskAll[out]所有位姿标定板上条纹图像对应性的mask
    vector<Mat> Imagesgray;
    vector<Mat> mod_phasemid;//用于存放三个周期的卷绕相位
    vector<Mat> image_ph_st;//读去某一周期，单一坐标的条纹图像
    Mat mean_lus;
    Mat lus_range;
    Mat mask_x;
    //    Mat mask_xy;
    //    Mat cor_x;
    //    Mat cor_y;
    int temx,temy,tg;
    int M;
    gcd(int(T[0]),int(T[1]),temx,temy,tg);
    gcd(int(T[2]),tg,temx,temy,M);
    //  int M = gcd(int(T[2]),gcd(int(T[0]),int(T[1])));
    int PoseNum = 1;//只对一个位置姿态进行三维测量
    int PeriodNum = T.size();
    int StepNum = N;  ////  Images.size()/(PoseNum*PeriodNum)/2.0;  //// ZHE GE BU YONG JISUAN   chen yuan bian liang
    Mat imagejudged = Images[0];
    if (imagejudged.channels()>1)
    {
        for(vector<Mat>::size_type i =0;i<Images.size();++i)
        {
            imagejudged=Images[i];
            Mat tempGrayImage(imagejudged.rows,imagejudged.cols,CV_32FC1);
            cvtColor(imagejudged,tempGrayImage,CV_BGR2GRAY);
            Imagesgray.push_back(tempGrayImage);
        }
        //Images.clear();
    }
    for(int n=0;n<PoseNum;n++)
    {
        Mat cor_x;
        Mat masktemp_x = Mat::ones(imagejudged.rows,imagejudged.cols,CV_8UC1);
        mod_phasemid.clear();
        for(int j=n*PeriodNum*StepNum;j<(n+1)*PeriodNum*StepNum;j+=StepNum)
        {
            Mat mod_phasetemp;//单一周期卷绕相位
            image_ph_st.clear();
            for(int i=j;i<j+StepNum;i++)
            {
                image_ph_st.push_back(Imagesgray[i]);
            }
            if(CoreAlgorithm::phaseshift(image_ph_st,Threshold,mod_phasetemp,mean_lus,lus_range,mask_x)==false)
            {
                //error message
                cout<<"Unable to get mod_phase"<<endl;
                return false;
            }
            masktemp_x=masktemp_x.mul(mask_x);
            ///// test
            //            namedWindow("test2dpro1",CV_WINDOW_AUTOSIZE);
            //            imshow("test2dpro1",mod_phasetemp/6.19);
            //            waitKey();
            ///// end test
            mod_phasemid.push_back(mod_phasetemp);
        }
        if(CoreAlgorithm::robustCRM(mod_phasemid,T,M,cor_x)==false)
        {
            //error message
            cout<<"Unable to using robustCRM"<<endl;
            return false;
        }
        Cor_X.push_back(cor_x);
        mod_phasemid.clear();
        ///// test
        //        namedWindow("test2corx",CV_WINDOW_AUTOSIZE);
        //        imshow("test2corx",cor_x/1140);
        //        waitKey();
        ///// end test

        Mask_All.push_back(masktemp_x);
        //        ///// test
        //        namedWindow("test2cory",CV_WINDOW_AUTOSIZE);
        //        imshow("test2cory",cor_y/1140);
        //        waitKey();
        //        Cor_All.push_back(cor_y);

    }
    return true;

}

/// 保存成点云文件
/// const string filename, <in> 保存的文件名
/// const vector<Point3f>& Points, <in> 要保存的点
bool CoreAlgorithm::WritePointsCloud(const string filename, const vector<Point3f>& Points, bool modeAdd)
{
    //    char* filename;
    //    QByteArray tmp = qfilename.toLatin1();
    //    filename = tmp.data();
    ofstream outFile;
    if (modeAdd)
        outFile.open(filename, ios::app, (int)ios_base::_Openprot);
    else
        outFile.open(filename, ios::out, (int)ios_base::_Openprot);
    if (!outFile)
    {
        cerr<<"error::can't open this file"<<endl;
        return false;
    }
    for (unsigned int i=0;i<Points.size();i++)
    {
        outFile<<Points[i].x<<"  "<<Points[i].y<<"  "<<Points[i].z<<endl;
    }
    outFile.close();
    return true;
}

/// 保存成点云文件
/// const string filename, <in> 保存的文件名
/// const vector<Point2f>& Points, <in> 要保存的点
bool CoreAlgorithm::WritePointsCloud(const string filename, const vector<Point2f>& Points, bool modeAdd)
{
    //    char* filename;
    //    QByteArray tmp = qfilename.toLatin1();
    //    filename = tmp.data();
    ofstream outFile;
    if (modeAdd)
        outFile.open(filename, ios::app, (int)ios_base::_Openprot);
    else
        outFile.open(filename, ios::out, (int)ios_base::_Openprot);
    if (!outFile)
    {
        cerr<<"error::can't open this file"<<endl;
        return false;
    }
    for (unsigned int i=0;i<Points.size();i++)
    {
        outFile<<Points[i].x<<"  "<<Points[i].y<<endl;
    }
    outFile.close();
    return true;
}

/// 读取点云文件
/// const string filename, <in> 保存的文件名
/// vector<Point3f>& Points, <out> 要保存的点
bool CoreAlgorithm::ReadPointsCloud(const string filename, vector<Point3f>& Points)
{
    ifstream inFile(filename, ios::in);
    if (!inFile)
    {
        cerr<<"error::can't open this file"<<endl;
        return false;
    }
    Points.clear();
    int count = 0;
    while(!inFile.eof())
    {
        count++;
        if (1000 == count)
            break;//防止死循环
        Point3f pt;
        for(int i = 0; i < 3; i++)
        {
            char str[128] = {0};
            if (2 != i)
                inFile.get(str, 128, ' ');
            else
                inFile.get(str, 128, '\n');
            if (0 == i)
            {
                pt.x = atof(str);
                inFile.seekg(2, ios::cur);//有两个空格
                if (inFile.eof())
                    return true;//已到文件结尾
            }
            if (1 == i)
            {
                pt.y = atof(str);
                inFile.seekg(2, ios::cur);//有两个空格
            }
            if (2 == i)
            {
                pt.z = atof(str);
            }
        }
        //cout<<fixed<<setprecision(4)<<pt<<endl;
        Points.push_back(pt);
    }
    inFile.close();
    return true;
}

/// 读取点云文件
/// const string filename, <in> 保存的文件名
/// vector<Point2f>& Points, <out> 要保存的点
bool CoreAlgorithm::ReadPointsCloud(const string filename, vector<Point2f>& Points)
{
    ifstream inFile(filename, ios::in);
    if (!inFile)
    {
        cerr<<"error::can't open this file"<<endl;
        return false;
    }
    Points.clear();
    int count = 0;
    while(!inFile.eof())
    {
        count++;
        if (1000 == count)
            break;//防止死循环
        Point2f pt;
        for(int i = 0; i < 2; i++)
        {
            char str[128] = {0};
            if (1 != i)
                inFile.get(str, 128, ' ');
            else
                inFile.get(str, 128, '\n');
            if (0 == i)
            {
                pt.x = atof(str);
                inFile.seekg(2, ios::cur);//有两个空格
                if (inFile.eof())
                    return true;//已到文件结尾
            }
            if (1 == i)
            {
                pt.y = atof(str);
            }
        }
        //cout<<fixed<<setprecision(4)<<pt<<endl;
        Points.push_back(pt);
    }
    inFile.close();
    return true;
}

bool CoreAlgorithm::ObtainData4Pro(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Point2f>& pointCam, vector<Point2f>& pointPro)
{
    vector<Mat> CorAll;//用来存放X、Y方向对应性
    vector<Mat> MaskAll;//用来存放对应性的mask
    //引用标定板三维坐标点及二维坐标点
    //    m_CalibrationDataPro.imgHeight=m_CalibrationData.imgHeight;
    //    m_CalibrationDataPro.imgWidth=m_CalibrationData.imgWidth;
    //    m_CalibrationDataPro.frameNumList=m_CalibrationData.frameNumList;
    //获得这个位姿标定板上的对应性及mask
    if (GetCorrespondance(Images, Threshold, N, T, CorAll, MaskAll)==false)
    {
        cout<<"Unable to get correspondance"<<endl;
        return false;
    }

    //    /// test
    //    namedWindow("testmask",CV_WINDOW_AUTOSIZE);
    //    imshow("testmask",MaskAll[0]*255);
    //    waitKey();
    //    /// end test

    int height = MaskAll[0].rows;
    int width = MaskAll[0].cols;

    for(int i=0;i<MaskAll[0].rows;i++)
    {
        for(int j=0;j<MaskAll[0].cols;j++)
        {
            if(MaskAll[0].at<uchar>(i,j)==1)
            {
                Point2f temPnt;
                temPnt.y = float(i);
                temPnt.x = float(j);
                pointCam.push_back(temPnt);

                Point2f pointCorTemp;
                Mat cor_x=CorAll[0];
                Mat cor_y=CorAll[1];
                pointCorTemp.x=cor_x.at<float>(i, j);
                pointCorTemp.y=cor_y.at<float>(i, j);
                pointPro.push_back(pointCorTemp);
            }
        }
    }

    return true;
}

bool CoreAlgorithm::GetCorrespondance(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Mat>& Cor_All, vector<Mat>& Mask_All)
{
    //images[in]所有位姿相机拍摄到的标定板图像
    //cor_All[out]所有位姿标定板上x,y的对应性,在容器中的排序方式为第一个位姿对应性x,y，第二个，x,y，第三个...以此类推
    //MaskAll[out]所有位姿标定板上条纹图像对应性的mask
    vector<Mat> Imagesgray;
    vector<Mat> mod_phasemid;//用于存放三个周期的卷绕相位
    vector<Mat> image_ph_st;//读去某一周期，单一坐标的条纹图像
    Mat mean_lus;
    Mat lus_range;
    Mat mask_x;
    Mat mask_y;
    //    Mat mask_xy;
    //    Mat cor_x;
    //    Mat cor_y;
    //int M=gcd(int(T[2]),gcd(int(T[0]),int(T[1])));
    int temx,temy,tg;
    int M;
    gcd(int(T[0]),int(T[1]),temx,temy,tg);
    gcd(int(T[2]),tg,temx,temy,M);
    int PoseNum = 1;//只对一个位置姿态进行三维测量
    int PeriodNum = T.size();
    int StepNum = N;  ////  Images.size()/(PoseNum*PeriodNum)/2.0;  //// ZHE GE BU YONG JISUAN   chen yuan bian liang
    Mat imagejudged=Images[0];
    if(imagejudged.channels()>1)
    {
        for(vector<Mat>::size_type i =0;i<Images.size();++i)
        {
            imagejudged=Images[i];
            Mat tempGrayImage(imagejudged.rows,imagejudged.cols,CV_32FC1);
            cvtColor(imagejudged,tempGrayImage,CV_BGR2GRAY);
            Imagesgray.push_back(tempGrayImage);
        }
        //Images.clear();
    }
    for(int n=0;n<PoseNum;n++)
    {
        Mat mask_xy;
        Mat cor_x;
        Mat cor_y;
        Mat masktemp_x = Mat::ones(imagejudged.rows,imagejudged.cols,CV_8UC1);
        Mat masktemp_y = Mat::ones(imagejudged.rows,imagejudged.cols,CV_8UC1);
        mod_phasemid.clear();
        for(int j=2*n*PeriodNum*StepNum;j<(2*n+1)*PeriodNum*StepNum;j+=StepNum)
        {
            Mat mod_phasetemp;//单一周期卷绕相位
            image_ph_st.clear();
            for(int i=j;i<j+StepNum;i++)
            {
                image_ph_st.push_back(Imagesgray[i]);
            }
            if(CoreAlgorithm::phaseshift(image_ph_st,Threshold,mod_phasetemp,mean_lus,lus_range,mask_x)==false)
            {
                //error message
                cout<<"Unable to get mod_phase"<<endl;
                return false;
            }
            masktemp_x=masktemp_x.mul(mask_x);
            ///// test
            //            namedWindow("test2dpro1",CV_WINDOW_AUTOSIZE);
            //            imshow("test2dpro1",mod_phasetemp/6.19);
            //            waitKey();
            ///// end test
            /// test
            //            namedWindow("testmask",CV_WINDOW_AUTOSIZE);
            //            imshow("testmask",mask_x*255);
            //            waitKey();
            /// end test
            mod_phasemid.push_back(mod_phasetemp);
        }
        if(CoreAlgorithm::robustCRM(mod_phasemid,T,M,cor_x)==false)
        {
            //error message
            cout<<"Unable to using robustCRM"<<endl;
            return false;
        }
        Cor_All.push_back(cor_x);
        mod_phasemid.clear();
        ///// test
        //        namedWindow("test2corx",CV_WINDOW_AUTOSIZE);
        //        imshow("test2corx",cor_x/912);
        //        Mat row464 = Mat::eye(1, 900, CV_32FC1);
        //        Mat sub_1 = Mat::eye(1, 900, CV_32FC1);
        //        Mat sub_2 = Mat::eye(1, 900, CV_32FC1);
        //        for(int i = 100; i < 400; i++)
        //        {
        //            row464.at<float>(0,i-100) = cor_x.at<float>(464,i);
        //            //cout<<cor_x.at<float>(464,i)<<endl;
        //            if (100 == i) continue;
        //            sub_1.at<float>(0,i-100) = cor_x.at<float>(464,i+1) - cor_x.at<float>(464,i);
        //            //cout<<cor_x.at<float>(464,i+1) - cor_x.at<float>(464,i)<<endl;
        //            if (101 == i) continue;
        //            sub_2.at<float>(0,i-100) = cor_x.at<float>(464,i+2) - 2*cor_x.at<float>(464,i+1) + cor_x.at<float>(464,i);
        //            cout<<sub_2.at<float>(0,i-100)<<endl;
        //        }
        //        //imwrite("absPhaseX.bmp", cor_x/912*255);
        //        FileStorage fs_X(".\\absPhaseX.xml", FileStorage::WRITE);
        //        fs_X<<"absPhaseX"<<row464;
        //        fs_X.release();
        //        waitKey();
        ///// end test
        // 求出Y方向的对应性
        for(int j=(2*n+1)*PeriodNum*StepNum;j<2*(n+1)*PeriodNum*StepNum;j+=StepNum)
        {
            Mat mod_phasetemp;//单一周期卷绕相位
            image_ph_st.clear();
            for(int i=j;i<j+StepNum;i++)
            {
                image_ph_st.push_back(Imagesgray[i]);
            }
            if(CoreAlgorithm::phaseshift(image_ph_st,Threshold,mod_phasetemp,mean_lus,lus_range,mask_y)==false)
            {
                //error message
                cout<<"Unable to get mod_phase"<<endl;
                return false;
            }
            ///// test
            //            namedWindow("test2dpro1",CV_WINDOW_AUTOSIZE);
            //            imshow("test2dpro1",mod_phasetemp/6.29);
            //            waitKey();
            ////////////////////////////////////
            mod_phasemid.push_back(mod_phasetemp);
            image_ph_st.clear();
            masktemp_y=masktemp_y.mul(mask_y);

            /// test
            //            namedWindow("testmask",CV_WINDOW_AUTOSIZE);
            //            imshow("testmask",mask_y*255);
            //            waitKey();
            /// end test
        }
        if(CoreAlgorithm::robustCRM(mod_phasemid,T,M,cor_y)==false)
        {
            //error message
            cout<<"Unable to using robustCRM"<<endl;
            return false;
        }
        mask_xy=masktemp_x.mul(masktemp_y);
        Mask_All.push_back(mask_xy);
        ///// test
        //        namedWindow("test2cory",CV_WINDOW_AUTOSIZE);
        //        imshow("test2cory",cor_y/1140*2);
        //        //imwrite("absPhaseY.bmp", cor_y/1140*2*255);
        //        //FileStorage fs_Y(".\\absPhaseY.xml", FileStorage::WRITE);
        //        //fs_Y<<"absPhaseY"<<cor_y;
        //        //fs_Y.release();
        //        waitKey();
        /// test
        //        namedWindow("testmaskxy",CV_WINDOW_AUTOSIZE);
        //        imshow("testmaskxy",mask_xy*255);
        //        waitKey();
        /// end test
        Cor_All.push_back(cor_y);

    }
    return true;

}

/// 创建一个圆环图像，改图像保存在正方形的矩阵中，正方形尺寸为dx，外圆环半径为 dx/3, 内圆环半径为  dx/6
/// Mat& img,<out> 输出图像 double型
/// int dx <in>  正方形矩阵的尺
bool CoreAlgorithm::creatRingImg(Mat& img,int dx)
{
    ///// 如果dx 小于等于0，则返回错误
    if(dx<=0)
    {
        return false;
    }

    //// 生成的圆环图像尺寸为dx，外环半径为 dx/3， 内环半径为dx/6
    //// 两个圆之间的部分为黑0.其他部分亮255
    Mat _img = Mat::zeros(dx, dx, CV_8U);  ////Mat _img = Mat::zeros(dx, dx, CV_32F);
    //// 图像大小
    int height,width;
    height = _img.rows;
    width  = _img.cols;
    double rmax,rmin,dist,cen_x,cen_y;
    rmax = dx/3;
    rmin = dx/6;
    ////// ，由于C++中矩阵的起始坐标为0，而不是1，所以坐标size减去1，则等效于（size-1）/2
    cen_x = (dx-1)/2;
    cen_y = (dx-1)/2;


    for(int i=0;i<height; i++)
    {
        for(int j=0;j<width;j++)
        {
            dist  = double(i-cen_y)*double(i-cen_y) + double(j-cen_x)*double(j-cen_x);
            dist = sqrt(dist);
            if(dist<rmin || dist>rmax) //// 设置成为白色 255
            {
                _img.at<char>(i,j) = 255;
            }
            //// 否则，就是为黑色，不用做任何操作
        }
    }
    img = _img.clone();
    return true;
}

//void CoreAlgorithm::PnPMethod(vector<Point3f> objectPoints,
//                              vector<Point2f> imagePoints,
//                              Mat cameraMatrix, Mat distCoeffs,
//                              Mat &PoseR,Mat &PoseT,
//                              vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam)
//{
//    //第一步，使用sovlePnP得到旋转向量和平移向量
//    //opencv2.44
//    //if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,CV_ITERATIVE)))
//    //opencv3.0
//    if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,SOLVEPNP_ITERATIVE)))
//    {
//        return;
//    }
//    //第二步，使用projectPoints得到重投影的二维特征点，得到雅可比矩阵，仅仅抽取前六列（ 旋转向量，平移向量）则构成J
//    vector<Point2f> imageReprojected;
//    Mat jacobian;
//    Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
//    projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
//    //将drot dt放到doutpara中(抽取前六列)
//    for( int i=0;i<6;i++ )
//    {
//        jacobian.col(i).copyTo(dpoutpara.col(i));
//    }
//    //第三步，输出的特征点图像坐标，与检测的图像特征点坐标相减则为反向投射误差，即为delta他是2nX1的矩阵。
//    Mat delta(dpoutpara.rows,1,CV_64FC1);
//    for (unsigned int i = 0;i < imagePoints.size();i++)
//    {
//        delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
//        delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

//    }
//    //第四步，根据公式求得一个方差值，6X6的矩阵，取对角线元素即为方差。为6X6的矩阵，其求平方根这位标准差，乘以3进行输出，为3被标准差结果。
//    Mat covariance_pre;
//    Mat dpoutpara_invert;
//    Mat covariance;
//    double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
//    gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
//    //covariance_pre = dpoutpara_invert*delta;//这种矩阵相乘方式也是可行的
//    mulTransposed(covariance_pre,covariance,0);
//    //第五步，求出3倍标准差
//    Mat diag_covariance(covariance.rows,1,CV_64FC1);
//    diag_covariance = covariance.diag(0);//取主对角线
//    for(int i =0;i<diag_covariance.rows;i++)
//    {
//        if(i<3)
//        {
//            dpdrot.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
//        }
//        else
//        {
//            dpdt.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
//        }
//    }
//    //第六步，求出转换到测量坐标系下的三维特征点，作为输出
//    Mat _R = Mat::zeros(3,3,CV_64FC1);
//    Rodrigues(PoseR,_R);
//    for(unsigned int i = 0;i < objectPoints.size();i++)
//    {
//        Mat pointtemp1;
//        Mat pointtemp4;
//        Point3f pointtemp2 = Point3f(0,0,0);
//        Point3d pointtemp3 = Point3d(0,0,0);
//        pointtemp1.setTo(Scalar(0));
//        pointtemp4.setTo(Scalar(0));
//        pointtemp3.x = objectPoints[i].x;
//        pointtemp3.y = objectPoints[i].y;
//        pointtemp3.z = objectPoints[i].z;
//        pointtemp1 = Mat(pointtemp3);
//        pointtemp4 = _R*pointtemp1 + PoseT;
//        pointtemp2 = Point3f(pointtemp4);
//        PointToCam.push_back(pointtemp2);
//    }
//    return;
//}

bool CoreAlgorithm::DualConicFitting(vector<Point2d>areaGrad,vector<Point2d>areaPos,Mat& dC,Mat& precision,double& angleIncertitude)
{
    precision = Mat::zeros(1,2,CV_64F);
    Mat a,b,c,_M;
    Mat areaGradmat = Mat(areaGrad).reshape(1);
    Mat areaPosmat = Mat(areaPos).reshape(1);
    a = areaGradmat.col(0);
    b = areaGradmat.col(1);
    Mat multitemp = areaGradmat.mul(areaPosmat);
    addWeighted(multitemp.col(0),-1,multitemp.col(1),-1,0,c);

    //为了高精度检测，对数据进行了线性归一化
    Mat M= Mat(a.rows,2,CV_64F);
    Mat tempb = -1*b;
    tempb.copyTo(M.col(0));
    a.copyTo(M.col(1));
    Mat B = -1*c;
    Mat mpts,Lnorm,Lnormt,Minvert;
    if(!solve(M,B,mpts,cv::DECOMP_SVD))
        return false;
    Mat H = Mat::eye(3,3,CV_64F);
    H.at<double>(0,2) = mpts.at<double>(0,0);
    H.at<double>(1,2) = mpts.at<double>(1,0);
    Mat abc=Mat(a.rows,3,CV_64F);
    a.copyTo(abc.col(0));
    b.copyTo(abc.col(1));
    c.copyTo(abc.col(2));

    Lnorm = H.t()*abc.t();
    Lnormt = Lnorm.t();
    a = Lnormt.col(0).clone();
    b = Lnormt.col(1).clone();
    c = Lnormt.col(2).clone();
    Mat AA = Mat(5,5,CV_64F);
    Mat BB = Mat(5,1,CV_64F);
    Mat a2 = a.mul(a);
    Mat ab = a.mul(b);
    Mat b2 = b.mul(b);
    Mat ac = a.mul(c);
    Mat bc = b.mul(c);
    Mat c2 = c.mul(c);
    //solution par least-square
    //AA*THITA=BB;
    //求AA
    Mat aaaa = a2.mul(a2);Mat aaab = a2.mul(ab);Mat aabb = a2.mul(b2);Mat aaac = a2.mul(ac);Mat aabc = a2.mul(bc);
    Mat abab = ab.mul(ab);Mat abbb = ab.mul(b2);Mat abac = ab.mul(ac);Mat abbc = ab.mul(bc);
    Mat bbbb = b2.mul(b2);Mat bbac = b2.mul(ac);Mat bbbc = b2.mul(bc);
    Mat acac = ac.mul(ac);Mat acbc = ac.mul(bc);
    Mat bcbc = bc.mul(bc);
    AA.at<double>(0,0)= sum(aaaa).val[0];AA.at<double>(0,1)= sum(aaab).val[0];AA.at<double>(0,2)= sum(aabb).val[0];AA.at<double>(0,3)= sum(aaac).val[0];AA.at<double>(0,4)= sum(aabc).val[0];
    AA.at<double>(1,0)= sum(aaab).val[0];AA.at<double>(1,1)= sum(abab).val[0];AA.at<double>(1,2)= sum(abbb).val[0];AA.at<double>(1,3)= sum(abac).val[0];AA.at<double>(1,4)= sum(abbc).val[0];
    AA.at<double>(2,0)= sum(aabb).val[0];AA.at<double>(2,1)= sum(abbb).val[0];AA.at<double>(2,2)= sum(bbbb).val[0];AA.at<double>(2,3)= sum(bbac).val[0];AA.at<double>(2,4)= sum(bbbc).val[0];
    AA.at<double>(3,0)= sum(aaac).val[0];AA.at<double>(3,1)= sum(abac).val[0];AA.at<double>(3,2)= sum(bbac).val[0];AA.at<double>(3,3)= sum(acac).val[0];AA.at<double>(3,4)= sum(acbc).val[0];
    AA.at<double>(4,0)= sum(aabc).val[0];AA.at<double>(4,1)= sum(abbc).val[0];AA.at<double>(4,2)= sum(bbbc).val[0];AA.at<double>(4,3)= sum(acbc).val[0];AA.at<double>(4,4)= sum(bcbc).val[0];
    //求BB
    Mat _ccaa = -1*(c2.mul(a2));Mat _ccab = -1*(c2.mul(ab));Mat _ccbb = -1*(c2.mul(b2));Mat _ccac = -1*(c2.mul(ac));Mat _ccbc = -1*(c2.mul(bc));
    BB.at<double>(0,0)= sum(_ccaa).val[0];BB.at<double>(1,0)= sum(_ccab).val[0];BB.at<double>(2,0)= sum(_ccbb).val[0];BB.at<double>(3,0)= sum(_ccac).val[0];BB.at<double>(4,0)= sum(_ccbc).val[0];
    if(determinant(AA)<10e-10)
    {
        //是否没有必要做下面工作，直接return false'
        dC = Mat::ones(3,3,CV_64F);
        dC = -1*dC;
        precision.at<double>(0,0) = -1;
        angleIncertitude = -1;
        return false;
    }
    //解AA*THITA=BB;
    Mat w,u,vt;
    Mat sol = Mat(5,1,CV_64F);
    if(!solve(AA,BB,sol))
        return false;
    //denormalisation
    Mat dCnorm = Mat(3,3,CV_64F);
    dCnorm.at<double>(0,0) = sol.at<double>(0,0);
    dCnorm.at<double>(0,1) = sol.at<double>(1,0)/2;
    dCnorm.at<double>(0,2) = sol.at<double>(3,0)/2;
    dCnorm.at<double>(1,0) = sol.at<double>(1,0)/2;
    dCnorm.at<double>(1,1) = sol.at<double>(2,0);
    dCnorm.at<double>(1,2) = sol.at<double>(4,0)/2;
    dCnorm.at<double>(2,0) = sol.at<double>(3,0)/2;
    dCnorm.at<double>(2,1) = sol.at<double>(4,0)/2;
    dCnorm.at<double>(2,2) = 1;
    dC = H*dCnorm*H.t();
    //误差估计
    Mat ss = sol.t();
    Mat cccc = c2.mul(c2);
    double BIB = sum(cccc).val[0];
    Mat R = (ss*AA*sol-2*ss*BB+BIB)/(a.rows-5);
    double RmatValue = R.at<double>(0,0);
    Mat cvar2_constantVariance = RmatValue*AA.inv();
    double vD = cvar2_constantVariance.at<double>(3,3);
    double vDE = cvar2_constantVariance.at<double>(3,4);
    double vE = cvar2_constantVariance.at<double>(4,4);
    Mat errorMatrics = Mat(2,2,CV_64F);
    errorMatrics.at<double>(0,0)= cvar2_constantVariance.at<double>(3,3);
    errorMatrics.at<double>(0,1)= cvar2_constantVariance.at<double>(3,4);
    errorMatrics.at<double>(1,0)= cvar2_constantVariance.at<double>(4,3);
    errorMatrics.at<double>(1,1)= cvar2_constantVariance.at<double>(4,4);
	cv::SVD::compute(errorMatrics, w, u, vt);
    Mat diagresult;
    sqrt(w,diagresult);
    diagresult = diagresult/4;
    precision = diagresult.t();
    angleIncertitude = atan2(vt.at<double>(1,1),vt.at<double>(0,1));
    return true;
}

bool CoreAlgorithm::conicaEllipseTostd(const Mat ellipsePara,RotatedRect& result)
{
    double thetarad,cost,sint,sin_squared,cos_squared,cos_sin;
    thetarad = 0.5*atan2(ellipsePara.at<double>(1,0),(ellipsePara.at<double>(0,0) - ellipsePara.at<double>(2,0)));
    cost = cos(thetarad);
    sint = sin(thetarad);
    sin_squared = sint*sint;
    cos_squared = cost*cost;
    cos_sin = sint*cost;
    double Ao,Au,Av,Auu,Avv;
    Ao = ellipsePara.at<double>(5,0);
    Au = ellipsePara.at<double>(3,0)*cost + ellipsePara.at<double>(4,0)*sint;
    Av = -ellipsePara.at<double>(3,0)*sint + ellipsePara.at<double>(4,0)*cost;
    Auu =ellipsePara.at<double>(0,0)*cos_squared + ellipsePara.at<double>(2,0)*sin_squared + ellipsePara.at<double>(1,0)*cos_sin;
    Avv =ellipsePara.at<double>(0,0)*sin_squared + ellipsePara.at<double>(2,0)*cos_squared - ellipsePara.at<double>(1,0)*cos_sin;
    if(Auu==0 || Avv==0)
    {
        //problem.  this is not a valid ellipse
        //make sure param are invalid and be easy to spot
        result.center.x = -1;
        result.center.y = -1;
        result.size.height = 0;
        result.size.width = 0;
        result.angle = 0;
        return false;
    }
    double tuCentre,tvCentre,wCentre,uCentre,vCentre,Ru,Rv;
    // ROTATED = [Ao Au Av Auu Avv]
    tuCentre = - Au/(2*Auu);
    tvCentre = - Av/(2*Avv);
    wCentre = Ao - Auu*tuCentre*tuCentre-Avv*tvCentre*tvCentre;
    uCentre = tuCentre * cost - tvCentre * sint;
    vCentre = tuCentre * sint + tvCentre * cost;

    Ru = -wCentre/Auu;
    Rv = -wCentre/Avv;
    if(Ru<0)
    {
        Ru =-1*sqrt(abs(Ru));
    }
    else
    {
        Ru =sqrt(abs(Ru));
    }
    if(Rv<0)
    {
        Rv =-1*sqrt(abs(Rv));
    }
    else
    {
        Rv =sqrt(abs(Rv));
    }
    result.center.x = (float)uCentre;
    result.center.y = (float)vCentre;
    if(Ru<Rv)
    {
        result.size.height =  (float)Ru;
        result.size.width =  (float)Rv;
        result.angle =  (float)thetarad;
    }
    else
    {
        result.size.height =  (float)Rv;
        result.size.width =  (float)Ru;
        result.angle =  (float)thetarad;
    }
    return true;
}

bool CoreAlgorithm::findEllipses(const Mat img,const cv::Rect mask,vector<RotatedRect>& findResults,const double precisionlevel,bool multi,int kenelsize)
{
    //step-1 将图像转化成灰度图
    Mat ImageGray;
    if(img.channels()==1)
    {
        ImageGray = img;
    }
    else
    {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
    }
    ImageGray = Mat(ImageGray,mask);

    //    namedWindow("window",1);
    //    imshow("window",ImageGray);
    //    waitKey();

    //step-2 计算图像梯度信息
    //step-2-1 生成高斯滤波器模版
    //判断滤波片模版大小是不是奇数
    if(kenelsize%2==0)
        return false;
    Mat X,Y;
    cv::Range xgv =cv::Range(-abs((kenelsize-1)/2),abs((kenelsize-1)/2));
    cv::Range ygv = xgv;
    std::vector<int> t_x, t_y;
    for(int i = xgv.start; i <= xgv.end; i++) t_x.push_back(i);
    for(int j = ygv.start; j <= ygv.end; j++) t_y.push_back(j);
    cv::repeat(cv::Mat(t_x),1,t_y.size(), X);
    cv::repeat(cv::Mat(t_y).t(), t_x.size(),1, Y);
    Mat GaussianKenelx(kenelsize,kenelsize,CV_64F);
    Mat GaussianKenely(kenelsize,kenelsize,CV_64F);
    for(int i=0;i<kenelsize;i++)
    {
        for(int j=0;j<kenelsize;j++)
        {
            GaussianKenelx.at<double>(i,j) = double(-X.at<int>(i,j)*exp(pow(double(X.at<int>(i,j)),2)/-2)*exp(pow(double(Y.at<int>(i,j)),2)/-2));
            GaussianKenely.at<double>(i,j) = double(-Y.at<int>(i,j)*exp(pow(double(X.at<int>(i,j)),2)/-2)*exp(pow(double(Y.at<int>(i,j)),2)/-2));
        }
    }
    //step-2-2 二维滤波器操作
    Mat dx,dy;
    filter2D(ImageGray,dx,CV_64F,GaussianKenelx);
    filter2D(ImageGray,dy,CV_64F,GaussianKenely);
    //step-2-3 梯度范数计算
    Mat gradientnorm,gradientnormBinary;;
    magnitude(dx,dy,gradientnorm);//计算梯度范数
    double minvalue,maxvalue;
    cv::minMaxLoc(gradientnorm,&minvalue,&maxvalue);
    //step-3 高置信梯度区域的选择
    //step-3-1 针对求出的梯度矩阵进行二值化
    int thresholdvalue = int(minvalue+maxvalue/2);
    //尝试直接二值化的方法
    //imwrite("F:/test3.jpg",ImageGray);
    //imwrite("F:/test2.jpg",gradientnorm);
    gradientnorm.convertTo(gradientnorm,CV_32F);
    double value = threshold(gradientnorm,gradientnormBinary,thresholdvalue,255, CV_THRESH_BINARY);
    gradientnormBinary.convertTo(gradientnormBinary,CV_8UC1);

    //以上部分是不是可以通过先高斯滤波处理再使用canny算子进行边缘检测？？？？？？？？？
    //step-3-2 联通区域标识
    Mat contoursMask;
    int contoursNum = connectedComponents(gradientnormBinary,contoursMask,8);
    contoursMask.convertTo(contoursMask,CV_8UC1);
    contoursNum--;
    //step-3-3 数据整理
    vector<vector<Point2d>> conectAreasPos;
    vector<vector<Point2d>> conectAreasGrad;
    conectAreasGrad.resize(contoursNum);
    conectAreasPos.resize(contoursNum);
    for(int i=0;i<contoursMask.rows;i++)
    {
        for(int j=0;j<contoursMask.cols;j++)
        {
            int tempnum = contoursMask.at<uchar>(i,j)-1;
            if(contoursMask.at<uchar>(i,j)!=0)
            {
                conectAreasPos[tempnum].push_back(Point2d(double(i),double(j)));
                conectAreasGrad[tempnum].push_back(Point2d(dx.at<double>(i,j),dy.at<double>(i,j)));
            }
        }
    }
    //step-4 利用对偶椭圆算子
    vector<Mat> dCVec,precisionVec;
    vector<double> angleIncertitudeVec;
    if(!multi)
    {
        for(int i =0;i<contoursNum;i++)
        {
            Mat dC,precision;
            double AngleIncertitude;
            if(conectAreasPos[i].size()<10)
                continue;
            if(!DualConicFitting(conectAreasGrad[i],conectAreasPos[i],dC,precision,AngleIncertitude))
                continue;
            if(precision.at<double>(0,0)==-1||precision.at<double>(0,0)>precisionlevel)
                continue;
            double num1 = precision.at<double>(0,0);
            dCVec.push_back(dC);
            precisionVec.push_back(precision);
            angleIncertitudeVec.push_back(AngleIncertitude);
        }
    }
    else
    {
        vector<vector<Point2d> > newAreasGradvec;
        vector<vector<Point2d> > newAreasPosvec;
        if (2 < conectAreasGrad.size() || 2 < conectAreasPos.size())
        {
            int account = 0;
            for(uint i = 0; i < conectAreasGrad.size(); i++)
            {
                if (100 < conectAreasGrad[i].size())
                {
                    newAreasGradvec.push_back(conectAreasGrad[i]);
                    newAreasPosvec.push_back(conectAreasPos[i]);
                    account++;
                    if (2 == account)
                        break;
                }
            }
        }
        else if(2 > conectAreasGrad.size() || 2 > conectAreasPos.size())
        {
            cout<<"MultiEllipseFitting() input size error!!!"<<endl;
            return false;
        }
        else
        {
            newAreasGradvec = conectAreasGrad;
            newAreasPosvec = conectAreasPos;
        }
        if(!MultiEllipseFitting(newAreasGradvec,newAreasPosvec,dCVec,precisionVec,angleIncertitudeVec))
        {
            cout<<"MultiEllipseFitting() failed!!!"<<endl;
            return false;
        }
    }
    //step-5 椭圆参数计算 Ax^2+Bxy+Cy^2+Dx+Ey+F=0
    vector<Mat> EllipsesparaVec;
    for(unsigned int i =0;i<dCVec.size();i++)
    {
        Mat Ellipsespara = Mat(6,1,CV_64F);
        Mat _C =  dCVec[i].inv();
        _C = _C/_C.at<double>(2,2);
        Ellipsespara.at<double>(0,0) = _C.at<double>(1,1);
        Ellipsespara.at<double>(1,0) = _C.at<double>(0,1)*2;
        Ellipsespara.at<double>(2,0) = _C.at<double>(0,0);
        Ellipsespara.at<double>(3,0) = _C.at<double>(1,2)*2;
        Ellipsespara.at<double>(4,0) = _C.at<double>(2,0)*2;
        Ellipsespara.at<double>(5,0) = _C.at<double>(2,2);
        EllipsesparaVec.push_back(Ellipsespara);
    }
    //step-6 由椭圆一般方程求解椭圆标准方程参数
    vector<RotatedRect> findResultstemp;
    for(unsigned int i=0;i<EllipsesparaVec.size();i++)
    {
        RotatedRect temppara;
        if(!conicaEllipseTostd(EllipsesparaVec[i],temppara))
            continue;
        findResultstemp.push_back(temppara);
    }
    for(unsigned int i=0;i<findResultstemp.size();i++)
    {
        findResultstemp[i].center.x += mask.x;
        findResultstemp[i].center.y += mask.y;
    }
    findResults = findResultstemp;
    return true;
}

bool CoreAlgorithm::MultiEllipseFitting(vector<vector<cv::Point2d>>areaGrads,
                                        vector<vector<cv::Point2d>>areaPoses,
                                        vector<Mat>& dCs,vector<Mat>& precisions,
                                        vector<double> angleIncertitudes)
{
    if (areaGrads.size() != areaPoses.size())//意外错误检测
        return false;

    //对数据统一进行了线性归一化,计算统一的H矩阵
    Mat a,b,c;
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat ctemp;
        Mat areaGradmat = Mat(areaGrads[i]).reshape(1);
        Mat areaPosmat = Mat(areaPoses[i]).reshape(1);
        a.push_back(areaGradmat.col(0));
        b.push_back(areaGradmat.col(1));
        Mat multitemp = areaGradmat.mul(areaPosmat);
        addWeighted(multitemp.col(0),-1,multitemp.col(1),-1,0,ctemp);
        c.push_back(ctemp);
    }
    Mat M= Mat(a.rows,2,CV_64F);
    Mat tempb = -1*b;
    tempb.copyTo(M.col(0));
    a.copyTo(M.col(1));
    Mat B = -1*c;
    Mat mpts,Lnorm,Lnormt,Minvert;
	if (!solve(M, B, mpts, cv::DECOMP_SVD))
        return false;
    Mat H = Mat::eye(3,3,CV_64F);
    H.at<double>(0,2) = mpts.at<double>(0,0);
    H.at<double>(1,2) = mpts.at<double>(1,0);
    //// 构造完毕H

    vector<Mat> AAVec,BBVec,aVec,cVec;
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat a,b,c;
        Mat areaGradmat = Mat(areaGrads[i]).reshape(1);
        Mat areaPosmat = Mat(areaPoses[i]).reshape(1);
        a = areaGradmat.col(0);
        b = areaGradmat.col(1);
        Mat multitemp = areaGradmat.mul(areaPosmat);
        addWeighted(multitemp.col(0),-1,multitemp.col(1),-1,0,c);

        //构造[a,b,c]
        Mat abc=Mat(a.rows,3,CV_64F);
        a.copyTo(abc.col(0));
        b.copyTo(abc.col(1));
        c.copyTo(abc.col(2));

        //得到归一化后的a,b和c
        Lnorm = H.t()*abc.t();
        Lnormt = Lnorm.t();
        a = Lnormt.col(0).clone();
        b = Lnormt.col(1).clone();
        c = Lnormt.col(2).clone();
        Mat AA = Mat(5,5,CV_64F);
        Mat BB = Mat(5,1,CV_64F);
        Mat a2 = a.mul(a);
        Mat ab = a.mul(b);
        Mat b2 = b.mul(b);
        Mat ac = a.mul(c);
        Mat bc = b.mul(c);
        Mat c2 = c.mul(c);
        aVec.push_back(a);//存储a和c,用于误差计算
        cVec.push_back(c);

        //solution par least-square
        //求AA
        Mat aaaa = a2.mul(a2);Mat aaab = a2.mul(ab);Mat aabb = a2.mul(b2);Mat aaac = a2.mul(ac);Mat aabc = a2.mul(bc);
        Mat abab = ab.mul(ab);Mat abbb = ab.mul(b2);Mat abac = ab.mul(ac);Mat abbc = ab.mul(bc);
        Mat bbbb = b2.mul(b2);Mat bbac = b2.mul(ac);Mat bbbc = b2.mul(bc);
        Mat acac = ac.mul(ac);Mat acbc = ac.mul(bc);
        Mat bcbc = bc.mul(bc);
        AA.at<double>(0,0)= sum(aaaa).val[0];AA.at<double>(0,1)= sum(aaab).val[0];AA.at<double>(0,2)= sum(aabb).val[0];AA.at<double>(0,3)= sum(aaac).val[0];AA.at<double>(0,4)= sum(aabc).val[0];
        AA.at<double>(1,0)= sum(aaab).val[0];AA.at<double>(1,1)= sum(abab).val[0];AA.at<double>(1,2)= sum(abbb).val[0];AA.at<double>(1,3)= sum(abac).val[0];AA.at<double>(1,4)= sum(abbc).val[0];
        AA.at<double>(2,0)= sum(aabb).val[0];AA.at<double>(2,1)= sum(abbb).val[0];AA.at<double>(2,2)= sum(bbbb).val[0];AA.at<double>(2,3)= sum(bbac).val[0];AA.at<double>(2,4)= sum(bbbc).val[0];
        AA.at<double>(3,0)= sum(aaac).val[0];AA.at<double>(3,1)= sum(abac).val[0];AA.at<double>(3,2)= sum(bbac).val[0];AA.at<double>(3,3)= sum(acac).val[0];AA.at<double>(3,4)= sum(acbc).val[0];
        AA.at<double>(4,0)= sum(aabc).val[0];AA.at<double>(4,1)= sum(abbc).val[0];AA.at<double>(4,2)= sum(bbbc).val[0];AA.at<double>(4,3)= sum(acbc).val[0];AA.at<double>(4,4)= sum(bcbc).val[0];

        //求BB
        Mat _ccaa = -1*(c2.mul(a2));Mat _ccab = -1*(c2.mul(ab));Mat _ccbb = -1*(c2.mul(b2));Mat _ccac = -1*(c2.mul(ac));Mat _ccbc = -1*(c2.mul(bc));
        BB.at<double>(0,0)= sum(_ccaa).val[0];BB.at<double>(1,0)= sum(_ccab).val[0];BB.at<double>(2,0)= sum(_ccbb).val[0];BB.at<double>(3,0)= sum(_ccac).val[0];BB.at<double>(4,0)= sum(_ccbc).val[0];
        if(determinant(AA)<10e-10)
        {
            //是否没有必要做下面工作，直接return false'
            //            dC = Mat::ones(3,3,CV_64F);
            //            dC = -1*dC;
            //            precision.at<double>(0,0) = -1;
            //            angleIncertitude = -1;
            return false;
        }
        AAVec.push_back(AA);
        BBVec.push_back(BB);
    }

    //求AAU和BBU
    Mat AAU;// = Mat(5*areaGrads.size(),3*areaGrads.size()+2,CV_64F);
    Mat BBU;// = Mat(5*areaGrads.size(),1,CV_64F);
    for(uint j = 0; j < areaGrads.size(); j++)//5行5行的压入到AAU和BBU中
    {
        Mat AAURow,AAURowT;
        for(uint k = 0; k < 3*areaGrads.size(); k++)//先压入AA的前三列到AAU中
        {
            if (j != k%areaGrads.size())
            {
                Mat O = Mat::zeros(1,5,CV_64F);
                AAURowT.push_back(O);
            }
            else
            {
                Mat tempM = Mat::zeros(5,5,CV_64F);
                AAVec[j].copyTo(tempM);
                tempM = tempM.t();
                AAURowT.push_back((tempM.row(k/areaGrads.size())));
            }
        }
        for(uint k = 0; k < 2; k++)//再压入AA的后两列到AAU中
        {
            Mat tempM = Mat::zeros(5,5,CV_64F);
            AAVec[j].copyTo(tempM);
            tempM = tempM.t();
            AAURowT.push_back((tempM.row(k+3)));
        }
        AAURow = AAURowT.t();//转置并压入到AAU中
        AAU.push_back(AAURow);
        BBU.push_back(BBVec[j]);//直接将相应的BB压入到BBU中
    }

    //解AAU*THITAU=BBU;
    //    cout<<AAU<<endl;
    //    cout<<BBU<<endl;
    Mat solu;// = Mat(3*areaGrads.size()+2,1,CV_64F);
	if (!solve(AAU, BBU, solu, cv::DECOMP_SVD))
        return false;

    //先拆解
    vector<Mat> dCnormVec;
    vector<Mat> solVec;
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat dCnorm = Mat::ones(3,3,CV_64F);
        Mat sol = Mat::ones(5,1,CV_64F);
        dCnorm.at<double>(0,2) = solu.at<double>(3*areaGrads.size(),0)/2;
        dCnorm.at<double>(2,0) = solu.at<double>(3*areaGrads.size(),0)/2;
        dCnorm.at<double>(1,2) = solu.at<double>(3*areaGrads.size()+1,0)/2;
        dCnorm.at<double>(2,1) = solu.at<double>(3*areaGrads.size()+1,0)/2;
        sol.at<double>(3,0) = solu.at<double>(3*areaGrads.size(),0);
        sol.at<double>(4,0) = solu.at<double>(3*areaGrads.size()+1,0);
        dCnormVec.push_back(dCnorm);
        solVec.push_back(sol);
    }
    for(uint i = 0; i < 3*areaGrads.size(); i++)
    {

        uint j = 0;//对应于A,B,C
        if (0 == i%areaGrads.size())
            j++;
        if (j == 1)//A
        {
            dCnormVec[i%areaGrads.size()].at<double>(0,0) = solu.at<double>(i,0);
            solVec[i%areaGrads.size()].at<double>(0,0) = solu.at<double>(i,0);
        }
        else if (j == 2)//B
        {
            dCnormVec[i%areaGrads.size()].at<double>(0,1) = solu.at<double>(i,0)/2;
            dCnormVec[i%areaGrads.size()].at<double>(1,0) = solu.at<double>(i,0)/2;
            solVec[i%areaGrads.size()].at<double>(1,0) = solu.at<double>(i,0);
        }
        else if (j == 3)//C
        {
            dCnormVec[i%areaGrads.size()].at<double>(1,1) = solu.at<double>(i,0);
            solVec[i%areaGrads.size()].at<double>(2,0) = solu.at<double>(i,0);
        }
    }

    //denormalisation
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat dC;
        dC = H*dCnormVec[i]*H.t();
        dCs.push_back(dC);
    }

    //误差估计
    for(uint i = 0; i < areaGrads.size(); i++)
    {
        Mat w,u,vt,precision;
        Mat ss = solVec[i].t();
        Mat c2 = cVec[i].mul(cVec[i]);
        Mat cccc = c2.mul(c2);
        double angleIncertitude;
        double BIB = sum(cccc).val[0];
        Mat R = (ss*AAVec[i]*solVec[i]-2*ss*BBVec[i]+BIB)/(aVec[i].rows-5);
        double RmatValue = R.at<double>(0,0);
        Mat cvar2_constantVariance = RmatValue*AAVec[i].inv();
        double vD = cvar2_constantVariance.at<double>(3,3);
        double vDE = cvar2_constantVariance.at<double>(3,4);
        double vE = cvar2_constantVariance.at<double>(4,4);
        Mat errorMatrics = Mat(2,2,CV_64F);
        errorMatrics.at<double>(0,0)= cvar2_constantVariance.at<double>(3,3);
        errorMatrics.at<double>(0,1)= cvar2_constantVariance.at<double>(3,4);
        errorMatrics.at<double>(1,0)= cvar2_constantVariance.at<double>(4,3);
        errorMatrics.at<double>(1,1)= cvar2_constantVariance.at<double>(4,4);
		cv::SVD::compute(errorMatrics, w, u, vt);
        Mat diagresult;
        sqrt(w,diagresult);
        diagresult = diagresult/4;
        precision = diagresult.t();
        precisions.push_back(precision);
        angleIncertitude = atan2(vt.at<double>(1,1),vt.at<double>(0,1));
        angleIncertitudes.push_back(angleIncertitude);
    }

    return true;
}

//计算矩阵的秩
uint CoreAlgorithm::rankCompute(const Mat& M)
{
    SVD svdM = SVD(M);
    Mat sM = svdM.w;//矩阵的奇异值
    double minValue,maxValue;
    uint rankM = 0;
    minMaxLoc(sM, &minValue, &maxValue);
    const double eps = pow((long double)1/2,(int)52);//double型的一个很小的数，详见Matlab的Help
    //double tolerance = max(3,int(M.cols))*eps*maxValue;//计算可允许误差
    double tolerance = 0.001;
    for(int i = 0; i < 3; i++)//计算矩阵的秩
    {
        if (sM.at<double>(i,0) > tolerance)
            rankM++;
    }
    return rankM;
}

//根据已经标定好的相机内部参数和畸变参数将圆环特征点二维坐标投影到平行视图上进行圆环匹配的特征点检测,从而得到更为精确的圆环特征点
//input:  imgsPath 输入的圆环标定板图像路径
//        cData输入的圆环中心点二维坐标和三维坐标
//        camPara输入的摄像机参数
//outout: cData输出的圆环中心点二维坐标和三维坐标
bool CoreAlgorithm::undistortRingCenter(const vector<string> &imgsPath, CalibrationData &cData, const CamPara& camPara)
{
    //合法性检查
    if (!imgsPath.size())
    {
        cout<<"Unable to undistortRingCenter, imgsPath is Null!"<<endl;
        return false;
    }
    if (imgsPath.size() != cData.plane2dPntsVec.size())
    {
        cout<<"Unable to undistortRingCenter, imgsPath error!"<<endl;
        return false;
    }

    //STEP-1 计算标定板平行视图的R'（并不是旋转矩阵） 3x3  初始化相机内部参数和畸变参数
    CamPara camparaParallel = camPara;
    generateParallelCamR(camparaParallel.parallelCamR);

    Mat cameraMatrix(3, 3, CV_64F);
    vector<double> distCoeffs;
    Mat DistortionCoeffs = Mat::zeros(1,4,CV_64FC1);
    for (int i = 0; i < 3; i++)
    {
        distCoeffs.push_back(camPara.DistortionCoeffs[i]);
        DistortionCoeffs.at<double>(0,i) = camPara.DistortionCoeffs[i];
        for (int j = 0; j < 3; j++)
        {
            cameraMatrix.at<double>(i, j) = camPara.CameraIntrinsic[i][j];
        }
    }
    distCoeffs.push_back(camPara.DistortionCoeffs[3]);
    DistortionCoeffs.at<double>(0,3) = camPara.DistortionCoeffs[3];

    //用PnP计算相机外部参数
    for(uint i = 0; i < imgsPath.size(); i++)
    {
        RT rt;
        Mat PoseR, PoseT;
        vector<double> dpdrot, dpdt;
        vector<Point3f> tmpPoint3fs;
        PnPMethod(cData.plane3dPntsVec[i], cData.plane2dPntsVec[i],
                  cameraMatrix, DistortionCoeffs,
                  PoseR, PoseT,
                  dpdrot, dpdt, tmpPoint3fs);
        rt.R = PoseR;
        rt.T = PoseT;
        camparaParallel.imgRTVec.push_back(rt);
    }

    //STEP-2 把所有特征点转换到平行视图上
    vector< vector<Point2f> > corner_Parallel;
    for (int i = 0; i < imgsPath.size(); i++)
    {
        vector<Point2f> pnts;
        undistortPoints2DifferentView(cData.plane2dPntsVec[i], pnts,
                                      cameraMatrix,camparaParallel.imgRTVec[i].R,
                                      camparaParallel.imgRTVec[i].T,distCoeffs,
                                      cameraMatrix,camparaParallel.parallelCamR,
                                      camparaParallel.imgRTVec[i].T,vector<double>());
        corner_Parallel.push_back(pnts);
    }

    //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
    vector<Mat> parallelImgR;
    calculateParallelViewR(camparaParallel.imgRTVec, parallelImgR);
    for (int i = 0; i < imgsPath.size();i++)  //// just for test (int i = 0; i < imgsPath.size();i++)
    {
        Mat srcImgMat;
        srcImgMat = cv::imread(imgsPath[i],CV_LOAD_IMAGE_GRAYSCALE);
        ////// image is too small
        //            Mat* resultImgMat = new Mat;
        //            resultImgMat->create(2*srcImgMat.rows, 2*srcImgMat.cols, srcImgMat.type());

        Mat resultImgMat = Mat(2*srcImgMat.rows, 2*srcImgMat.cols, srcImgMat.type());
        /////

        //undistortImg(srcImgMat, *resultImgMat, cameraMatrix, distCoeffs, parallelImgR[i]);
        undistortImg(srcImgMat, resultImgMat, cameraMatrix, distCoeffs, parallelImgR[i]);

        /// test
        //            Mat testmat = imread("E:/DLP LightCrafter 4500/English Paper/experiment data/5_Principle/bak/png5.bmp", CV_LOAD_IMAGE_GRAYSCALE);
        //            Mat testresultImgMat(2*testmat.rows, 2*testmat.cols, testmat.type());
        //            undistortImg(testmat, testresultImgMat, cameraMatrix, distCoeffs, parallelImgR[i]);
        //            namedWindow("Test1",WINDOW_AUTOSIZE);
        //            imshow("Test1",resultImgMat);
        //            waitKey();
        //            imshow("Test1",testresultImgMat);
        //            waitKey();
        /// end test

        ////// zhang xu added 增加一个 ring模版708X708，然后进行resize
        Mat ringImg;
        CoreAlgorithm::creatRingImg(ringImg, 708);
        ////end
        ringSubPix(resultImgMat,corner_Parallel[i],ringImg); ////ringSubPix(resultImgMat,corner_Parallel[i],Mat());
        //delete resultImgMat;
        //ringSubPix(resultImgMat,corner_Parallel[i],Mat());

        /// modified by zhaorui
        //ringSubPixAdv(resultImgMat,corner_Parallel[i],1);
        /// end

        int j = 0;
        /// test
        //            // resize
        //            //Mat testmat;
        //            Size resizevalue;
        //            resizevalue.height = resultImgMat.rows*5;
        //            resizevalue.width = resultImgMat.cols*5;
        //            //resize(resultImgMat, testmat, resizevalue);
        //            resize(testresultImgMat, testmat, resizevalue);
        //            // draw cross
        //            Point up, down, left, right;
        //            Scalar sclr(255,255,255);
        //            for(int u = 0; u < corner_Parallel[i].size(); u++)
        //            {
        //                Point2f pt = corner_Parallel[i][u];
        //                pt.x *= 5;
        //                pt.y *= 5;
        //                up = down = left = right = pt;
        //                up.y = up.y - 50;
        //                down.y = down.y + 50;
        //                left.x=left.x-50;
        //                right.x=right.x+50;
        //                line(testmat, up, down, sclr, 2);
        //                line(testmat, left, right, sclr, 2);
        //            }
        //            // show and save
        //            namedWindow("testresize",CV_WINDOW_AUTOSIZE);
        //            imwrite("testresize.bmp", testmat);
        //            imshow("testresize",testmat);
        //            waitKey();
        /// end test
    }

    //STEP-4 将平行视图上检测到的亚像素角点转换到原始视图
    for (int i = 0; i < corner_Parallel.size(); i++)
    {
        vector<Point2f> pnts;
        undistortPoints2DifferentView(corner_Parallel[i], pnts,
                                      cameraMatrix,camparaParallel.parallelCamR,
                                      camparaParallel.imgRTVec[i].T,vector<double>(),
                                      cameraMatrix,camparaParallel.imgRTVec[i].R,
                                      camparaParallel.imgRTVec[i].T,distCoeffs);
        corner_Parallel[i] = pnts;
    }

    //STEP-5 更新二维点信息
    cData.plane2dPntsVec.clear();
    cData.plane2dPntsVec = corner_Parallel;

    return true;
}

//对Mat类型的数据按照某一列数据按大小进行排序  从小到大冒泡  注意只支持CV_8UC1 CV_32FC1 CV_64FC1 CV_32SC1
//Input:  Matrix        待排序的Mat矩阵
//        Col           按照该列数据排序
//Output: Matrix        将输入的矩阵排序后的矩阵
bool CoreAlgorithm::SortInMatByCol(Mat& Matrix, const int Col)
{
    if (Col >= Matrix.cols)
    {
        cout<<"Error Col in SortInMatByCol!\n";
        return false;//出错
    }
    for(uint j = 0; j < Matrix.rows; j++)
    {
        for (uint i = 0; i < Matrix.rows - j - 1; i++)
        {
            Mat tempMat;
            tempMat.create(1, Matrix.cols, Matrix.type());
            tempMat.zeros(1, Matrix.cols, Matrix.type());
            switch (Matrix.type())
            {
            case CV_8UC1:
            {
                if (Matrix.at<uchar>(i,Col) > Matrix.at<uchar>(i+1,Col))
                {
                    for(uint m = 0; m < Matrix.cols; m++)
                    {
                        tempMat.at<uchar>(0,m) = Matrix.at<uchar>(i,m);
                        Matrix.at<uchar>(i,m) = Matrix.at<uchar>(i+1,m);
                        Matrix.at<uchar>(i+1,m) = tempMat.at<uchar>(0,m);
                    }
                }
                break;
            }
            case CV_32FC1:
            {
                if (Matrix.at<float>(i,Col) > Matrix.at<float>(i+1,Col))
                {
                    for(uint m = 0; m < Matrix.cols; m++)
                    {
                        tempMat.at<float>(0,m) = Matrix.at<float>(i,m);
                        Matrix.at<float>(i,m) = Matrix.at<float>(i+1,m);
                        Matrix.at<float>(i+1,m) = tempMat.at<float>(0,m);
                    }
                }
                break;
            }
            case CV_64FC1:
            {
                if (Matrix.at<double>(i,Col) > Matrix.at<double>(i+1,Col))
                {
                    for(uint m = 0; m < Matrix.cols; m++)
                    {
                        tempMat.at<double>(0,m) = Matrix.at<double>(i,m);
                        Matrix.at<double>(i,m) = Matrix.at<double>(i+1,m);
                        Matrix.at<double>(i+1,m) = tempMat.at<double>(0,m);
                    }
                }
                break;
            }
            case CV_32SC1:
            {
                if (Matrix.at<int>(i,Col) > Matrix.at<int>(i+1,Col))
                {
                    for(uint m = 0; m < Matrix.cols; m++)
                    {
                        tempMat.at<int>(0,m) = Matrix.at<int>(i,m);
                        Matrix.at<int>(i,m) = Matrix.at<int>(i+1,m);
                        Matrix.at<int>(i+1,m) = tempMat.at<int>(0,m);
                    }
                }
                break;
            }
            default:
            {
                cout<<"Error type in SortInMatByCol!\n";
                return false;//出错
            }
            }
        }
    }
    return true;
}

//计算两个点形成的直线与X轴正向形成的角度  逆时针为正
bool CoreAlgorithm::CalculateLineAngle(Point2f inputPnt1, Point2f inputPnt2, float& angleInDeg)
{
    if (inputPnt2.x - inputPnt1.x == 0)//异常处理
    {
        //test
        //cout<<"Warning, CalculateLineAngle abnormal!"<<endl;
        //end test

        return false;
    }
    float angle = abs(atan((inputPnt2.y - inputPnt1.y) /
                           (inputPnt2.x - inputPnt1.x)));
    if (angle <= -3.15/2 || angle >= 3.15/2)//异常处理
    {
        //test
        //cout<<"Warning, CalculateLineAngle abnormal!"<<endl;
        //end test

        return false;
    }
    angle = angle*180/3.141592653;
    if ((inputPnt2.y - inputPnt1.y) >= 0 &&
            (inputPnt2.x - inputPnt1.x) > 0)
        angle = (90 - angle) + 270;
    else if ((inputPnt2.y - inputPnt1.y) >= 0 &&
             (inputPnt2.x - inputPnt1.x) < 0)
        angle = angle + 180;
    else if ((inputPnt2.y - inputPnt1.y) < 0 &&
             (inputPnt2.x - inputPnt1.x) > 0)
        angle = angle + 0;
    else if ((inputPnt2.y - inputPnt1.y) < 0 &&
             (inputPnt2.x - inputPnt1.x) < 0)
        angle = (90 - angle) + 90;

    angleInDeg = angle;

    return true;
}

//自动探测所有椭圆封闭轮廓  并输出其中心点  注意已排除了重复点
bool CoreAlgorithm::AutoDetectRingCenters(const Mat& srcImg, vector<Point2f>& centerPointsVec)
{
    Mat cannyMat, imgGray;
    if (srcImg.empty())
    {
        cout<<"srcImg is NULL!"<<endl;
        return false;
    }
    if (srcImg.channels() != 1)
        cvtColor(srcImg, imgGray, CV_BGR2GRAY);
    else
        imgGray = srcImg.clone();

    //自适应二值化
    //    int block_size = 205;//105;
    //    adaptiveThreshold(imgGray,imgGray,255,ADAPTIVE_THRESH_MEAN_C ,THRESH_BINARY,block_size,0 );

    //寻找轮廓
    Canny(imgGray,cannyMat,50,255,3,true);
    vector< vector<Point> > contoursvec;
    vector<Vec4i> hierarchy;
    findContours(cannyMat, contoursvec, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    Mat contourImg;
    contourImg.create(cannyMat.rows, cannyMat.cols, CV_8U);
    contourImg.zeros(cannyMat.rows, cannyMat.cols, CV_8U);
    drawContours(contourImg, contoursvec, -1, Scalar(255, 255, 255));

    //test
    //    namedWindow("cannyMat", 0);
    //    imshow("cannyMat", contourImg);
    //    cvWaitKey(0);
    //end test

    //椭圆拟合,找中心,并进行筛选
    centerPointsVec.clear();
    float SqrtHeightWidth = sqrt(float(cannyMat.rows * cannyMat.cols));
    int min_D = SqrtHeightWidth / 60;
    int max_D = SqrtHeightWidth / 10;//允许的最大最小直径
    float max_width = 0;
    float max_height = 0;//最大长和宽
    int num = 0;
    for( vector< vector<Point> >::iterator itr=contoursvec.begin();itr!=contoursvec.end();++itr)
    {
        RotatedRect rotatedBox;
        uint contoursvecSize = itr->size();

        if (contoursvecSize < min_D * 3.14 || contoursvecSize > max_D * 3.14)
            continue;//剔除大小不符合要求的轮廓

        try
        {
            rotatedBox = fitEllipse((*itr));
        }
        catch (...)//三个点表示任何异常
        {
            //qDebug()<<"fitEllipse error:"<< n ;
            continue;
        }

        //test
        //        Mat contourImg;
        //        vector<vector<Point> > contourTemp;
        //        contourTemp.push_back(*itr);
        //        contourImg.create(cannyMat.rows, cannyMat.cols, CV_8UC3);
        //        contourImg.zeros(cannyMat.rows, cannyMat.cols, CV_8UC3);
        //        drawContours(contourImg, contourTemp, -1, Scalar(255, 255, 255));
        //        namedWindow("contourTemp", 0);
        //        imshow("contourTemp", contourImg);
        //        imwrite("contourTemp.bmp", contourImg);
        //        cvWaitKey(0);
        //end test

        //剔除不封闭的轮廓
        bool isClose = true;
        int angleArea[36];//每10度为一个检查区域
        for(uint p = 0; p < 36; p++)
            angleArea[p] = 0;
        for(uint p = 0; p < contoursvecSize; p++)
        {
            Point pointTemp = (*itr)[p];
            float angleInDeg = 0;
            Point2f inputPnt1(rotatedBox.center.x, rotatedBox.center.y);
            Point2f inputPnt2(pointTemp.x, pointTemp.y);
            if (!CalculateLineAngle(inputPnt1, inputPnt2, angleInDeg))
            {
                isClose = false;
                break;
            }
            int angleInt = (int)angleInDeg / 10;
            angleArea[angleInt]++;
        }
        for(uint p = 0; p < 36; p++)
        {
            if (angleArea[p] == 0)
            {
                isClose = false;
                break;
            }
        }
        if (isClose == false)
            continue;

        //test
        //        Mat contourImg;
        //        vector<vector<Point> > contourTemp;
        //        contourTemp.push_back(*itr);
        //        contourImg.create(cannyMat.rows, cannyMat.cols, CV_8UC3);
        //        contourImg.zeros(cannyMat.rows, cannyMat.cols, CV_8UC3);
        //        drawContours(contourImg, contourTemp, -1, Scalar(255, 255, 255));
        //        namedWindow("contourTemp", 0);
        //        imshow("contourTemp", contourImg);
        //        cvWaitKey(0);
        //end test

        float height = rotatedBox.size.height;
        float width = rotatedBox.size.width;
        bool isRepeat = false;
        if (rotatedBox.center.x > width &&
                rotatedBox.center.x < cannyMat.cols - width &&
                rotatedBox.center.y > height &&
                rotatedBox.center.y < cannyMat.rows - height)//越界判断
        {
            Point2f point2fTemp(rotatedBox.center.x, rotatedBox.center.y);
            for(uint m = 0; m < centerPointsVec.size(); m++)//剔除重复点
            {
                if (sqrt((point2fTemp.x - centerPointsVec[m].x)*
                         (point2fTemp.x - centerPointsVec[m].x)+
                         (point2fTemp.y - centerPointsVec[m].y)*
                         (point2fTemp.y - centerPointsVec[m].y)) < width/2)
                {
                    isRepeat = true;
                    break;
                }

            }
            if (isRepeat)
                continue;
            if (0 == num)//获取最大椭圆的长和宽
            {
                max_width = width;
                max_height = height;
            }
            else
            {
                if (width > max_width)
                    max_width = width;
                if (height > max_height)
                    max_height = height;
            }
            centerPointsVec.push_back(point2fTemp);//存储中心点
            num++;
        }
    }

    ///test
    //    Mat centerPoints;
    //    centerPoints.create(cannyMat.rows, cannyMat.cols, CV_8UC3);
    //    centerPoints.zeros(cannyMat.rows, cannyMat.cols, CV_8UC3);
    //    for(uint i = 0; i < centerPointsVec.size(); i++)
    //    {
    //        CvPoint point;
    //        point.x = int(centerPointsVec[i].x);
    //        point.y = int(centerPointsVec[i].y);
    //        circle(centerPoints, point ,3 , CV_RGB(255,0,0),1, 8, 3);
    //        cout<<"point:( "<<point.x<<" , "<<point.y<<" )"<<endl;
    //    }
    //    cout<<"num = "<<num<<endl;
    //    namedWindow("CenterPoints", 0);
    //    imshow("CenterPoints", centerPoints);
    //    imwrite("CenterPoints.bmp", centerPoints);
    //    cvWaitKey(0);
    ///end test

    return true;
}

//对给定的一系列点去除孤立点
static void RemoveIsolatePoints(vector<Point2f>& Pnts)
{

}

//粗略的判断某一点是否在一系列点所组成的直线上  返回true表示在直线上
bool CoreAlgorithm::IsOnline(const vector<Point2f>& pntsLine, Point2f pnt)
{
    if (2 > pntsLine.size())
    {
        cout<<"Error in IsOnline!"<<endl;
        return false;
    }
    for(uint i = 0; i < pntsLine.size() - 1; i++)
    {
        float angleDeg = 0;
        float angleDegX = 0;
        Point2f pnt1 = pntsLine[i];
        Point2f pnt2 = pntsLine[i+1];
        CalculateLineAngle(pnt1, pnt2, angleDeg);
        CalculateLineAngle(pnt1, pnt, angleDegX);
        if (180 < angleDeg)
            angleDeg = angleDeg - 180;
        if (180 < angleDegX)
            angleDegX = angleDegX - 180;
        if (5 > abs(angleDeg - angleDegX))//小于5度满足条件一
        {
            //点到直线的距离小于一定值满足条件二
            float dis_threshold = 0.2*sqrt((pnt1.x-pnt2.x)*(pnt1.x-pnt2.x) + (pnt1.y-pnt2.y)*(pnt1.y-pnt2.y));
            float a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
            float b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);
            float dis = fabs(a*pnt.x + b*pnt.y + 1) / sqrt(a*a + b*b);//点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
            if (dis < dis_threshold)
                return true;
        }
    }
    return false;
}

//将有序点集按行或列排列  并输出所有行或列的点集
//输入:  const Mat& PointsMat 按某个坐标排序的有序点集  其中每一行为一个点的各个坐标值
//输出:  vector<Mat>& correctLinesVec 每行或每列的点集
bool CoreAlgorithm::SortPointsLineByLine(const Mat& PointsMat, vector<Mat>& correctLinesVec)
{
    if (!PointsMat.rows)
    {
        cout<<"PointsMat is NULL!"<<endl;
        return false;
    }

    //test
    //cout<<PointsMat<<endl;
    //end test

    vector<float> anglesVec;//存储每条有效直线的斜率
    Mat PointsMatClone;//判断出无效的起点后  用于还原已清除的点
    PointsMatClone = PointsMat.clone();
    Mat PointsMatBak;//判断出无效的起点或有效的直线点后  清除该点不再判断
    PointsMatBak = PointsMat.clone();
    //KDTree kdtree(PointsMatBak);//建立kd树结构  以便查找最近点  OpenCV2.4.4
	cv::flann::KDTreeIndexParams indexParams;//建立kd树结构  以便查找最近点  OpenCV3.0
	cv::flann::Index kdtree(PointsMatBak, indexParams);

    //判断以该点为起点建立直线时  该点是否有效  若无法找到2个点与该点建立有效直线  则无效
    for(uint i = 0; i < PointsMatBak.rows - 1; i++)
    {
        if (0 == PointsMatBak.at<float>(i,0))
            continue;//无需判断的点

        //test
        //        Mat CenterPoints_i = imread("test.bmp");
        //        CvPoint point;
        //        point.x = int(PointsMatBak.at<float>(i,0));
        //        point.y = int(PointsMatBak.at<float>(i,1));
        //        circle(CenterPoints_i, point ,3 , CV_RGB(255,255,0),3);
        //end test

        if (correctLinesVec.size())//漏掉的拐点及其之前的点的检查
        {
            Point2f pnt(PointsMatBak.at<float>(i,0), PointsMatBak.at<float>(i,1));
            vector<Point2f> pnts;
            for(uint k = 0; k < (correctLinesVec[correctLinesVec.size()-1]).rows; k++)
            {
                Mat temp = (correctLinesVec[correctLinesVec.size()-1]).clone();
                Point2f tempPnt(temp.at<float>(k,0), temp.at<float>(k,1));

                //test
                //                CvPoint point;
                //                point.x = int(tempPnt.x);
                //                point.y = int(tempPnt.y);
                //                circle(CenterPoints_i, point ,3 , CV_RGB(255,0,0),3);
                //end test

                pnts.push_back(tempPnt);
            }

            //test
            //            namedWindow("CenterPoints_i", 0);
            //            imshow("CenterPoints_i", CenterPoints_i);
            //            cvWaitKey(0);
            //end test

            if (IsOnline(pnts, pnt))
            {
                (correctLinesVec[correctLinesVec.size()-1]).push_back(PointsMatBak.row(i));
                PointsMatBak.at<float>(i,0) = 0;//以后不需要再判断该点
                PointsMatBak.at<float>(i,1) = 0;

                //test
                //                CvPoint point;
                //                point.x = int(pnt.x);
                //                point.y = int(pnt.y);
                //                circle(CenterPoints_i, point ,3 , CV_RGB(0,0,255),3);
                //                namedWindow("CenterPoints_i", 0);
                //                imshow("CenterPoints_i", CenterPoints_i);
                //                cvWaitKey(0);
                //end test

                continue;
            }
        }

        Mat findResultsMat;//建立的一个列点集
        uint j = i + 1;
        uint vote = 3;//对于同一个起点  若之后的3个点都不能和其建立有效直线  则该起点为无效点
        for(uint numInLine = 1; numInLine < PointsMatBak.rows; )
        {
            Point2f pnt1(0,0), pnt2(0,0);
            if (1 == numInLine)
            {
                pnt1.x = PointsMatBak.at<float>(i,0);
                pnt1.y = PointsMatBak.at<float>(i,1);
                pnt2.x = PointsMatBak.at<float>(j,0);
                pnt2.y = PointsMatBak.at<float>(j,1);
            }
            else//每次循环更新pnt1和pnt2
            {
                int lastrow = findResultsMat.rows - 1;
                pnt1.x = findResultsMat.at<float>(lastrow-1,0);
                pnt1.y = findResultsMat.at<float>(lastrow-1,1);
                pnt2.x = findResultsMat.at<float>(lastrow,0);
                pnt2.y = findResultsMat.at<float>(lastrow,1);
            }
            Point2f predictPnt(2*pnt2.x - pnt1.x, 2*pnt2.y - pnt1.y);//预测直线上的下一点  以便查找与该点最近的点
            float disThreshold = 0.2*sqrt((pnt2.x - pnt1.x)*(pnt2.x - pnt1.x) +
                                          (pnt2.y - pnt1.y)*(pnt2.y - pnt1.y));//其中0.33表示可允许的误差百分比

            //test
            //cout<<"px = "<<predictPnt.x<<", py = "<<predictPnt.y<<endl;
            //end test

            Mat target, findresult, idx;
            target.create(1,2,CV_32FC1);
            target.zeros(1,2,CV_32FC1);
            float minDifference = 0;
            target.at<float>(0,0) = predictPnt.x;//要用kd树查找的目标点
            target.at<float>(0,1) = predictPnt.y;
            //kdtree.findNearest(target, 1, INT_MAX, idx);//找到一个离预测点最近的点  //OpenCV2.4.4
            //kdtree.getPoints(idx, findresult);
            Mat dists;//OpenCV 3.0
            kdtree.knnSearch(target, idx, dists, 1);//找到一个离预测点最近的点
            findresult = (PointsMat.row(idx.at<int>(0,0))).clone();
            Mat disTemp = findresult - target;
            minDifference = sqrt((disTemp.at<float>(0,0)*disTemp.at<float>(0,0)) +
                                 (disTemp.at<float>(0,1)*disTemp.at<float>(0,1)));

            //test
            //cout << findresult <<endl;
            //end test

            //kdMinPnt.x = findresult.at<float>(0,0);
            //kdMinPnt.y = findresult.at<float>(0,1);
            if (minDifference > disThreshold)//没有找到离预测点有效的最近点
            {
                if (4 <= numInLine)//整条直线上有效点大于等于4个时  该直线有效
                {
                    float angle = 0;
                    Point2f pnt1(PointsMatBak.at<float>(i,0), PointsMatBak.at<float>(i,1));
                    Point2f pnt2(PointsMatBak.at<float>(j,0), PointsMatBak.at<float>(j,1));
                    if (!CalculateLineAngle(pnt1, pnt2, angle))
                        cout<<"Warning, CalculateLineAngle abnormal!"<<endl;
                    anglesVec.push_back(angle);
                    correctLinesVec.push_back(findResultsMat);
                    PointsMatBak.at<float>(j,0) = 0;//以后不需要再判断该点
                    PointsMatBak.at<float>(j,1) = 0;
                    PointsMatBak.at<float>(i,0) = 0;//以后不需要再判断该点
                    PointsMatBak.at<float>(i,1) = 0;
                    PointsMatClone = PointsMatBak.clone();
                }
                else
                {
                    if(0 == vote)
                    {
                        PointsMatBak = PointsMatClone.clone();//还原
                        PointsMatBak.at<float>(i,0) = 0;//以后不需要再判断该点
                        PointsMatBak.at<float>(i,1) = 0;
                        PointsMatClone.at<float>(i,0) = 0;//以后不需要再判断该点
                        PointsMatClone.at<float>(i,1) = 0;
                    }
                    else
                    {
                        vote--;
                        j++;
                        if (j == PointsMatBak.rows)
                        {
                            PointsMatBak = PointsMatClone.clone();//还原
                            PointsMatBak.at<float>(i,0) = 0;//以后不需要再判断该点
                            PointsMatBak.at<float>(i,1) = 0;
                            PointsMatClone.at<float>(i,0) = 0;//以后不需要再判断该点
                            PointsMatClone.at<float>(i,1) = 0;
                            break;
                        }
                        numInLine = 1;
                        continue;
                    }
                }

                //test
                //cout<<PointsMatBak<<endl;
                //end test

                break;
            }
            else
            {
                //test
                //Mat CenterPoints_i = imread("test.bmp");
                //                CvPoint point;
                //                point.x = int(findresult.at<float>(0,0));
                //                point.y = int(findresult.at<float>(0,1));
                //                circle(CenterPoints_i, point ,3 , CV_RGB(0,0,255),3);
                //                namedWindow("CenterPoints_i", 0);
                //                imshow("CenterPoints_i", CenterPoints_i);
                //                cvWaitKey(0);
                //end test

                if (1 == numInLine)
                {
                    Mat tempMat;
                    tempMat.create(1,2,CV_32FC1);
                    tempMat.zeros(1,2,CV_32FC1);
                    tempMat.at<float>(0,0) = pnt1.x;
                    tempMat.at<float>(0,1) = pnt1.y;
                    findResultsMat.push_back(tempMat);
                    tempMat.at<float>(0,0) = pnt2.x;
                    tempMat.at<float>(0,1) = pnt2.y;
                    findResultsMat.push_back(tempMat);
                    tempMat.at<float>(0,0) = findresult.at<float>(0,0);
                    tempMat.at<float>(0,1) = findresult.at<float>(0,1);
                    findResultsMat.push_back(tempMat);//保存该直线上的这三个点
                    numInLine = 3;

                    for(uint k = 0; k < PointsMatBak.rows; k++)
                    {
                        if ( (PointsMatBak.at<float>(k,0) == tempMat.at<float>(0,0)) &&
                             (PointsMatBak.at<float>(k,1) == tempMat.at<float>(0,1)) )
                        {
                            PointsMatBak.at<float>(k,0) = 0;//以后不需要再判断该点
                            PointsMatBak.at<float>(k,1) = 0;
                        }
                    }
                }
                else
                {
                    Mat tempMat;
                    tempMat.create(1,2,CV_32FC1);
                    tempMat.zeros(1,2,CV_32FC1);
                    tempMat.at<float>(0,0) = findresult.at<float>(0,0);
                    tempMat.at<float>(0,1) = findresult.at<float>(0,1);
                    findResultsMat.push_back(tempMat);//保存这条直线上的新的点
                    numInLine++;

                    for(uint k = 0; k < PointsMatBak.rows; k++)
                    {
                        if ( (PointsMatBak.at<float>(k,0) == tempMat.at<float>(0,0)) &&
                             (PointsMatBak.at<float>(k,1) == tempMat.at<float>(0,1)) )
                        {
                            PointsMatBak.at<float>(k,0) = 0;//以后不需要再判断该点
                            PointsMatBak.at<float>(k,1) = 0;
                        }
                    }
                }

                //test
                //cout<<"right"<<endl;
                //cout<<findResultsMat<<endl;
                //end test

            }
        }
        //test
        //cout<<findResultsMat<<endl;
        //end test
    }

    //错误检查
    if (!anglesVec.size())
    {
        cout<<"correctLinesVec is NULL!"<<endl;
        return false;
    }
    return true;
}

//自动识别圆环中心点阵  并按一定顺序给出圆环中心点阵区域ROI的四个角点  要求所有图片最上一行外法线始终向上
bool CoreAlgorithm::AutoDetermineRingsROI(const Mat& srcImg, vector<Point2f>& mousePnts)
{
    Mat imgGray;
    if (srcImg.empty())
        return false;
    if (srcImg.channels() != 1)
        cvtColor(srcImg, imgGray, CV_BGR2GRAY);
    else
        imgGray = srcImg.clone();

    //自动探测所有椭圆封闭轮廓中心点
    vector<Point2f> centerPointsVec;
    if (!AutoDetectRingCenters(imgGray, centerPointsVec))
    {
        cout<<"Error in AutoDetectRingCenters!"<<endl;
        return false;
    }

    ///test
    //    Mat centerPoints;
    //    centerPoints.create(srcImg.rows, srcImg.cols, CV_8UC3);
    //    centerPoints.zeros(srcImg.rows, srcImg.cols, CV_8UC3);
    //    for(uint i = 0; i < centerPointsVec.size(); i++)
    //    {
    //        CvPoint point;
    //        point.x = int(centerPointsVec[i].x);
    //        point.y = int(centerPointsVec[i].y);
    //        circle(centerPoints, point ,3 , CV_RGB(255,0,0),1);
    //        cout<<"point:( "<<point.x<<" , "<<point.y<<" )"<<endl;
    //    }
    //    cout<<"num = "<<centerPointsVec.size()<<endl;
    //    namedWindow("CenterPoints", 0);
    //    imshow("CenterPoints", centerPoints);
    //    imwrite("CenterPoints.bmp", centerPoints);
    //    cvWaitKey(0);
    ///end test

    ///去除孤立点和点集  待添加

    //对中心点先按X坐标排序  然后再按Y坐标排序  目的是为了确定哪个是行哪个是列  设行点数大于列点数  否则出错
    Mat centerPointsMat;
    centerPointsMat.create(0,0,CV_32FC1);
    for(uint i = 0; i < centerPointsVec.size(); i++)
    {
        Mat temp;
        temp.create(1,2,CV_32FC1);
        temp.zeros(1,2,CV_32FC1);
        temp.at<float>(0,0) = centerPointsVec[i].x;
        temp.at<float>(0,1) = centerPointsVec[i].y;
        centerPointsMat.push_back(temp.row(0));
    }
    SortInMatByCol(centerPointsMat, 0);//对Mat类型数据按X坐标进行排序  以剔除参差不齐的列

    //test
    //cout<<centerPointsMat<<endl;
    //cout<<"centerPointsMat.rows = "<<centerPointsMat.rows<<endl;
    //end test

    //根据所有点的列信息  建立以列为单位的表型结构
    vector<Mat> correctColsVec;
    if (!SortPointsLineByLine(centerPointsMat, correctColsVec))
    {
        cout<<"Error in SortPointsLineByLine!"<<endl;
        return false;
    }

    ///test
    //    cout<<"cols = "<<correctColsVec.size()<<endl;
    //    Mat sortedCenterPoints;
    //    sortedCenterPoints.create(imgGray.rows, imgGray.cols, CV_8UC3);
    //    sortedCenterPoints.zeros(imgGray.rows, imgGray.cols, CV_8UC3);
    //    for(uint i = 0; i < correctColsVec.size(); i++)
    //    {
    //        for(uint j = 0; j < (correctColsVec[i]).rows; j++)
    //        {
    //            CvPoint point;
    //            point.x = int((correctColsVec[i]).at<float>(j,0));
    //            point.y = int((correctColsVec[i]).at<float>(j,1));
    //            circle(sortedCenterPoints, point, 3, CV_RGB(255,0,0),1);
    //            Point2f point2f;
    //            point2f.x = (correctColsVec[i]).at<float>(j,0);
    //            point2f.y = (correctColsVec[i]).at<float>(j,1);
    //            //cout<<point2f<<endl;
    //        }
    //        cout<<endl<<endl;
    //        namedWindow("sortedCenterPoints", 0);
    //        imshow("sortedCenterPoints", sortedCenterPoints);
    //        imwrite("sortedCenterPoints.bmp", sortedCenterPoints);
    //        cvWaitKey(0);
    //    }
    ///end test

    //根据行点数大于列点数  判断出行和列
    float rows = 0;
    for(uint i = 0; i < correctColsVec.size(); i++)
        rows += float((correctColsVec[i]).rows);
    rows /= correctColsVec.size();
    if (correctColsVec.size() < rows)//correctColsVec存储的是一行一行的中心点  不利于剔除两侧无效点
    {
        //对中心点再按Y坐标排序
        centerPointsMat.create(0,0,CV_32FC1);
        for(uint i = 0; i < centerPointsVec.size(); i++)
        {
            Mat temp;
            temp.create(1,2,CV_32FC1);
            temp.zeros(1,2,CV_32FC1);
            temp.at<float>(0,0) = centerPointsVec[i].x;
            temp.at<float>(0,1) = centerPointsVec[i].y;
            centerPointsMat.push_back(temp.row(0));
        }
        SortInMatByCol(centerPointsMat, 1);//对Mat类型数据按Y坐标进行排序  以剔除参差不齐的列

        //test
        //cout<<centerPointsMat<<endl;
        //cout<<"centerPointsMat.rows = "<<centerPointsMat.rows<<endl;
        //end test

        vector<Mat> correctLinesVec;//根据所有点的列信息  建立以列为单位的表型结构
        if (!SortPointsLineByLine(centerPointsMat, correctLinesVec))
        {
            cout<<"Error in SortPointsLineByLine!"<<endl;
            return false;
        }

        ///test
        //        cout<<"cols = "<<correctLinesVec.size()<<endl;
        //        Mat sortedCenterPoints;
        //        sortedCenterPoints.create(imgGray.rows, imgGray.cols, CV_8UC3);
        //        sortedCenterPoints.zeros(imgGray.rows, imgGray.cols, CV_8UC3);
        //        for(uint i = 0; i < correctLinesVec.size(); i++)
        //        {
        //            for(uint j = 0; j < (correctLinesVec[i]).rows; j++)
        //            {
        //                CvPoint point;
        //                point.x = int((correctLinesVec[i]).at<float>(j,0));
        //                point.y = int((correctLinesVec[i]).at<float>(j,1));
        //                circle(sortedCenterPoints, point, 3, CV_RGB(255,0,0),1);
        //            }
        //            namedWindow("sortedCenterPoints", 0);
        //            imshow("sortedCenterPoints", sortedCenterPoints);
        //            imwrite("sortedCenterPoints.bmp", sortedCenterPoints);
        //            cvWaitKey(0);
        //        }
        ///end test

        correctColsVec.clear();
        correctColsVec = correctLinesVec;
    }

    //将每列的点按Y坐标由小到大重新排列
    for(uint i = 0; i < correctColsVec.size(); i++)
    {
        if (!SortInMatByCol(correctColsVec[i], 1))
            return false;
    }

    //将每列的重复点剔除
    for(uint i = 0; i < correctColsVec.size(); i++)
    {
        Mat tempMat;
        tempMat.create(0,0,CV_32FC1);
        for(uint j = 0; j < (correctColsVec[i]).rows; j++)
        {
            if (0 == j)
                tempMat.push_back((correctColsVec[i]).row(j));
            else
            {
                if ((tempMat.at<float>(tempMat.rows-1,0) == (correctColsVec[i]).at<float>(j,0)) &&
                        (tempMat.at<float>(tempMat.rows-1,1) == (correctColsVec[i]).at<float>(j,1)) )
                    continue;
                else
                    tempMat.push_back((correctColsVec[i]).row(j));
            }
        }
        correctColsVec[i] = tempMat.clone();
    }


    //剔除两侧参差不齐的列
    float medianLinefloat = correctColsVec.size() / 2;
    int medianLineNum1 = int(medianLinefloat) - 1 ;
    int medianLineNum2 = int(medianLinefloat + 1) - 1;
    if ((correctColsVec[medianLineNum1]).rows != (correctColsVec[medianLineNum2]).rows)
    {
        cout<<"Error in medianLineNum!"<<endl;
        return false;
    }
    int medianLineRows = (correctColsVec[medianLineNum1]).rows;
    vector<Mat> correctColsVecClone;
    for(uint i = 0; i < correctColsVec.size(); i++)
    {
        Mat temp = (correctColsVec[i]).clone();
        correctColsVecClone.push_back(temp);
    }
    correctColsVec.clear();
    for(uint i = 0; i < correctColsVecClone.size(); i++)
    {
        if ((correctColsVecClone[i]).rows == medianLineRows)
        {
            Mat temp = (correctColsVecClone[i]).clone();
            correctColsVec.push_back(temp);
        }
    }
    correctColsVecClone.clear();

    //检查异常情况
    for(uint i = 0; i < correctColsVec.size() - 1; i++)
    {
        if ((correctColsVec[i]).rows != (correctColsVec[i+1]).rows)
        {
            cout<<"Error size in correctColsVec!\n";
            return false;
        }
    }

    //按顺序获取确定ROI的四个点
    Point2f leftTopPnt(0,0), rightTopPnt(0,0), leftDownPnt(0,0), rightDownPnt(0,0);
    uint colEnd = (correctColsVec[0]).rows - 1;
    uint rowEnd = correctColsVec.size() - 1;

    leftTopPnt.x = (3*correctColsVec[0].at<float>(0,0) - correctColsVec[1].at<float>(1,0))/2;
    leftTopPnt.y = (3*correctColsVec[0].at<float>(0,1) - correctColsVec[1].at<float>(1,1))/2;
    rightTopPnt.x = (3*correctColsVec[rowEnd].at<float>(0,0) - correctColsVec[rowEnd-1].at<float>(1,0))/2;
    rightTopPnt.y = (3*correctColsVec[rowEnd].at<float>(0,1) - correctColsVec[rowEnd-1].at<float>(1,1))/2;
    leftDownPnt.x = (3*correctColsVec[0].at<float>(colEnd,0) - correctColsVec[1].at<float>(colEnd-1,0))/2;
    leftDownPnt.y = (3*correctColsVec[0].at<float>(colEnd,1) - correctColsVec[1].at<float>(colEnd-1,1))/2;
    rightDownPnt.x = (3*correctColsVec[rowEnd].at<float>(colEnd,0) - correctColsVec[rowEnd-1].at<float>(colEnd-1,0))/2;
    rightDownPnt.y = (3*correctColsVec[rowEnd].at<float>(colEnd,1) - correctColsVec[rowEnd-1].at<float>(colEnd-1,1))/2;

    if (0 > leftTopPnt.x)//越界检查
        leftTopPnt.x = 1;
    if (0 > leftTopPnt.y)
        leftTopPnt.y = 1;
    if (srcImg.cols < rightTopPnt.x)
        rightTopPnt.x = srcImg.cols - 1;
    if (0 > rightTopPnt.x)
        rightTopPnt.x = 1;
    if (srcImg.cols < rightDownPnt.x)
        rightDownPnt.x = srcImg.cols - 1;
    if (srcImg.rows < rightDownPnt.y)
        rightDownPnt.y = srcImg.rows - 1;
    if (0 > leftDownPnt.x)
        leftDownPnt.x = 1;
    if (srcImg.rows < leftDownPnt.x)
        leftDownPnt.x = srcImg.rows - 1;

    mousePnts.clear();
    mousePnts.push_back(leftTopPnt);//按顺序得到要点选的角点
    mousePnts.push_back(rightTopPnt);
    mousePnts.push_back(rightDownPnt);
    mousePnts.push_back(leftDownPnt);

    ///test
    //    Mat imgClr;
    //    cvtColor(imgGray, imgClr, CV_GRAY2RGB);
    //    for(uint i = 0; i < mousePnts.size(); i++)
    //    {
    //        CvPoint point;
    //        point.x = int(mousePnts[i].x);
    //        point.y = int(mousePnts[i].y);
    //        circle(imgClr, point ,5 , CV_RGB(85*(i+1),0,0), 5);
    //        if (0 != i)
    //        {
    //            CvPoint point0;
    //            point0.x = int(mousePnts[i-1].x);
    //            point0.y = int(mousePnts[i-1].y);
    //            line(imgClr, point0, point, CV_RGB(0,0,255), 2);
    //            if (3 == i)
    //            {
    //                CvPoint point00;
    //                point00.x = int(mousePnts[0].x);
    //                point00.y = int(mousePnts[0].y);
    //                line(imgClr, point, point00, CV_RGB(0,0,255), 2);
    //            }
    //        }
    //        cout<<"\n\npoint:( "<<point.x<<" , "<<point.y<<" )"<<endl;
    //    }
    //    namedWindow("pntsImg", 0);
    //    imshow("pntsImg", imgClr);
    //    cvWaitKey(0);
    ///end test

    return true;
}

//获取指定目录下所有指定扩展名的所有文件路径
void CoreAlgorithm::GetFiles(string path, string exd, std::vector<string>& files)
{
    //文件句柄
    long   hFile   =   0;
    //文件信息
    struct _finddata_t fileinfo;
    string pathName, exdName;

    if (0 != strcmp(exd.c_str(), ""))
    {
        exdName = "\\*." + exd;
    }
    else
    {
        exdName = "\\*";
    }

    if((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(),&fileinfo)) !=  -1)
    {
        do
        {
            //如果是文件夹中仍有文件夹,迭代之
            //如果不是,加入列表
            if((fileinfo.attrib &  _A_SUBDIR))
            {
                if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
                    GetFiles( pathName.assign(path).append("\\").append(fileinfo.name), exd, files );
            }
            else
            {
                if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)
                    files.push_back(pathName.assign(path).append("\\").append(fileinfo.name));
            }
        }while(_findnext(hFile, &fileinfo)  == 0);
        _findclose(hFile);
    }
}

bool CoreAlgorithm::findPntsWithTag(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma)
{
    if(centerPnt.size()<7)
        return false;
    vector<TagPoint2f> TagPnts;
    if(centerPnt.size()!=longaxisRadius.size())
        return false;
    int pntsSize = centerPnt.size();
    Mat distanceMatrix1 = Mat::zeros(pntsSize,pntsSize,CV_64F);
    Mat distanceMatrix = Mat::zeros(pntsSize,pntsSize,CV_64F);
    //step-2 计算椭圆中心之间的距离并除以主椭圆长轴半径，将其存储在矩阵中
    for(int i = 0;i<pntsSize;i++)
    {
        for(int j=i+1;j<pntsSize;j++)
        {
            distanceMatrix1.at<double>(i,j) = distancePoints(centerPnt[i],centerPnt[j])/(longaxisRadius[i]*gamma);
        }
    }
    Mat distanceMatrix1_T = distanceMatrix1.t();
    cv::add(distanceMatrix1,distanceMatrix1_T,distanceMatrix);
    //step-3 从矩阵中的找出存在三个元素值在0.7-1.3之间的行数及相应元素所对应的行标及列标
    vector<int> pointsIndextemp;
    int rowIndex=0;
    for(int i= 0;i<distanceMatrix.rows;i++)
    {
        rowIndex=i;
        pointsIndextemp.clear();
        int n=0;
        for(int j=0;j<distanceMatrix.cols;j++)
        {
            if(distanceMatrix.at<double>(i,j)>0.8&&1.2>distanceMatrix.at<double>(i,j)
                    &&longaxisRadius[i]/longaxisRadius[j]>0.9&&longaxisRadius[i]/longaxisRadius[j]<1.1)
            {
                n++;
                pointsIndextemp.push_back(j);
            }
        }
        if(n>=3)
        {
            pair<int,vector<int>> possiblefistPoints;
            possiblefistPoints.first=i;
            possiblefistPoints.second=pointsIndextemp;
            //判断1267点
            vector<Point2f> footpointNearpoints;
            TagPnts.clear();
            for(unsigned int i = 0;i<possiblefistPoints.second.size();i++)
            {
                footpointNearpoints.push_back(centerPnt[possiblefistPoints.second[i]]);
            }
            if(!(findshortAxisPoints(footpointNearpoints[0],footpointNearpoints[1],footpointNearpoints[2],centerPnt[possiblefistPoints.first],TagPnts)))
            {
                continue;
            }
            firstFeaturelength = longaxisRadius[possiblefistPoints.first];
            if(!TagPnts.size())
            {
                continue;
            }
            //根据求得12两点确定的长轴直线，求出长轴上的点
            float dis_threshold = 10;
            vector<Point2f> longAxisPntsVec;
            float a, b;//直线公式ax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
            Point2f pnt1 = Point2f(TagPnts[0].val[1],TagPnts[0].val[2]);
            Point2f pnt2 = Point2f(TagPnts[1].val[1],TagPnts[1].val[2]);
            Mat ZaxisVector = Mat::zeros(3,1,CV_64F);
            Mat axisVectorTemp = Mat::zeros(3,1,CV_64F);
            a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
            b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);
            ZaxisVector.at<double>(0,0)= pnt2.x - pnt1.x;
            ZaxisVector.at<double>(1,0)= pnt2.y - pnt1.y;
            for (size_t n = 0; n < centerPnt.size(); n++)
            {
                //点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);

                float dis = float(fabs(a*centerPnt[n].x + b*centerPnt[n].y + 1) / pow((a*a + b*b),(float)0.5));
                axisVectorTemp.at<double>(0,0)= centerPnt[n].x-pnt1.x;
                axisVectorTemp.at<double>(1,0)= centerPnt[n].y-pnt1.y;
                double tempnum = ZaxisVector.dot(axisVectorTemp);
                if (dis < dis_threshold&&tempnum>0)//当中心找的不准的时候这个阈值要放宽一点
                {
                    longAxisPntsVec.push_back(centerPnt[n]);
                }
            }
            //step-5 确定出3，4，5点
            float unit_dis;//2点到1点的距离作为比较单位距离
            unit_dis = (float)pow((pow((TagPnts[1][1] - TagPnts[0][1]),2)+ pow((TagPnts[1][2] - TagPnts[0][2]),2)),(float)0.5);
            for (unsigned int i = 0; i < longAxisPntsVec.size(); i++)
            {
                TagPoint2f _tagPnt;
                float dis = (float)pow((pow((longAxisPntsVec[i].x - TagPnts[0][1]),2)+pow((longAxisPntsVec[i].y - TagPnts[0][2]),2)),(float)0.5);
                float _k = dis/ unit_dis;
                if (_k>1.8 && _k<2.2)
                {
                    _tagPnt[0] = TAG3;
                    _tagPnt[1] =longAxisPntsVec[i].x;
                    _tagPnt[2] =longAxisPntsVec[i].y;
                    TagPnts.push_back(_tagPnt);
                }
                if (_k>2.8 && _k < 3.2)
                {
                    _tagPnt[0] = TAG4;
                    _tagPnt[1] =longAxisPntsVec[i].x;
                    _tagPnt[2] =longAxisPntsVec[i].y;
                    TagPnts.push_back(_tagPnt);
                }
                if (_k>3.7 && _k < 4.2)
                {
                    _tagPnt[0] = TAG5;
                    _tagPnt[1] =longAxisPntsVec[i].x;
                    _tagPnt[2] =longAxisPntsVec[i].y;
                    TagPnts.push_back(_tagPnt);
                }
            }
            if(TagPnts.size()!=7)
            {
                if(rowIndex==distanceMatrix.rows-1)
                {
                    return false;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                break;
            }
        }
    }
    // step-6对标记好的点，按照标记点进行排序
    //由小到大排序//chengwei added
    if(TagPnts.size()!=7)
    {
        return false;
    }
    for(unsigned int i=0;i<TagPnts.size()-1;i++)
    {
        for (unsigned int j = i+1; j<TagPnts.size(); j++)
        {
            if (TagPnts[i][0] > TagPnts[j][0])
            {
                swap(TagPnts[i], TagPnts[j]);
            }
        }
    }
    tagPnts = TagPnts;
    return true;
}

bool CoreAlgorithm::findRingBoardCenter(const Mat& img, Size patternSize, vector<Point2f>& centers)
{

    //转化成灰度图
    Mat ImageGray;
    if(img.channels()==1)
    {
        ImageGray = img;
    }
    else
    {
        cvtColor(img, ImageGray, CV_BGR2GRAY);
    }
    //chengwei changed

    ////直方图均衡化
    //equalizeHist(ImageGray,ImageGray);

    //// 均值滤波
    blur(ImageGray,ImageGray,Size(5,5),Point(-1,-1));


    //自适应阈值处理
    Mat thresh_img;
    int block_size = 501;
	adaptiveThreshold(ImageGray, thresh_img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, block_size, 0);

	int close_type = cv::MORPH_ELLIPSE;
    int dilate_size = 3;
    Mat element = getStructuringElement(close_type,Size(2*dilate_size+1,2*dilate_size+1),Point(-1,-1));
    dilate(thresh_img, thresh_img, element,Point(-1, -1),1);

    //找轮廓
    vector<vector<Point>> all_contours;
    vector<int> contoursIndex;
    vector<Vec4i> all_hierarchy;
    Mat temp_LED;
    thresh_img.copyTo(temp_LED);
    findContours(temp_LED,all_contours,all_hierarchy,CV_RETR_EXTERNAL ,CV_CHAIN_APPROX_SIMPLE);

    //轮廓过滤
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    for (unsigned int n = 0; n<all_contours.size(); n++)
    {
        double area = contourArea(all_contours[n]);
        //按照组成轮廓的点的数量
        // if(  all_contours[n].size() < 80 || all_contours[n].size() > 700)
        if(  all_contours[n].size() < 80 )
        {continue;}
        //按照轮廓的面积，既包含的像素个数
        /*else if(  area < area_threshold_min || area > area_threshold_max )
        {continue;}*/
        else
        {
            RotatedRect rotatedBox;
            rotatedBox = fitEllipse(all_contours[n]);
            float height = rotatedBox.size.height;
            float width = rotatedBox.size.width;
            Point2f center = rotatedBox.center;
            if( height > width ? (height/width<1.5) : (width/height<1.5) )
            {
                //判断第一个子轮廓的中心是否与其父轮廓接近
                int k = all_hierarchy[n][2];
                if (k>0)//若k<0则该轮廓没有子轮廓
                {
                    if(  all_contours[k].size() < 30 || all_contours[k].size() > 350)
                    {continue;}

                    Point2f _center = fitEllipse(all_contours[k]).center;
                    if (abs(_center.x - center.x) < 2 && abs(_center.y - center.y) < 2)
                    {
                        if (roundness(center, all_contours[n], (height + width) / 4) < 5)
                        {
                            centers.push_back(rotatedBox.center);
                            contoursIndex.push_back(n);
                            contours.push_back(all_contours[n]);
                        }
                    }
                }
            }
        }
    }

    if (centers.size()!=patternSize.width * patternSize.height)
    {
        return false;
    }

    return sortRingCenter2(centers,patternSize.height,patternSize.width);
}

bool CoreAlgorithm::getTagPoint2f(const Mat& img, vector<TagPoint2f>& tagPnts2f)
{
    Mat imagep;
    cv::medianBlur(img,imagep,5);
    //STEP-1:中心点提取
    vector<Point2f> centerPnts;
    vector<float> longaxisRadius;
    CoreAlgorithm::detectLightSpot_LED(imagep,centerPnts,longaxisRadius);
    if (centerPnts.size() < 7)
        return false;

    //STEP-2:中心点排序,并加上标签，加标签的目的是为了进行过滤
    vector<TagPoint2f>  tagPnts;
    double a = 25; //25,45
    double d = 10.0;//10,15
    double gamma = 2*(a+d)/d;
    float Featurelength;
    if(!CoreAlgorithm::findPntsWithTag(centerPnts,longaxisRadius,tagPnts,Featurelength,gamma))
    {
        return false;
    }
    //STEP-3 针对特征点进行高精度的圆检测
    for(size_t j=0;j<tagPnts.size();j++)
    {
        cv::Rect ROI = cv::Rect(Point2i((int)abs(tagPnts[j].val[1]-2*Featurelength),(int)abs(tagPnts[j].val[2]-2*Featurelength)),
                cv::Size(int(4*Featurelength),int(4*Featurelength)));
        vector<RotatedRect> findResulttemp;
        if(!CoreAlgorithm::findEllipses(imagep,ROI,findResulttemp,0.08))
        {
            return false;
        }
        if(!findResulttemp.size())
            return false;
        if(findResulttemp.size()==1)
        {
            if(findResulttemp[0].size.width>2)
            {
                tagPnts[j].val[1] = findResulttemp[0].center.x;
                tagPnts[j].val[2] = findResulttemp[0].center.y;
            }
            else
                return false;
        }
        else
        {

            RotatedRect tempRotateRect = findResulttemp[0];
            for(size_t k=1;k<findResulttemp.size();k++)
            {
                if(tempRotateRect.size.width<findResulttemp[k].size.width)
                    tempRotateRect = findResulttemp[k];
            }
            tagPnts[j].val[1] = tempRotateRect.center.x;
            tagPnts[j].val[2] = tempRotateRect.center.y;
        }

    }
    tagPnts2f = tagPnts;
    return true;
}

//bool CoreAlgorithm::getTagPoint3f(const Mat& img1,const CamPara _campara1, const Mat& img2, const CamPara _campara2,
//                                  const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3f)
//{
//    //step-1 计算左右相机的二维特征点点集
//    vector<TagPoint2f> leftTagsPnts2f;
//    vector<TagPoint2f> rightTagsPnts2f;
//    if(!getTagPoint2f(img1,leftTagsPnts2f))
//        return false;
//    if(!getTagPoint2f(img2,rightTagsPnts2f))
//        return false;
//    /*if(!(getTagPoint2f(img1,leftTagsPnts2f)&&getTagPoint2f(img2,rightTagsPnts2f)))
//        return false;*/
//    //step-2 将特征点的标签信息去掉
//    vector<Point2f> leftPnts2f,rightPnts2f;
//    vector<int> TagVec;
//    pntsTagConfirm(leftTagsPnts2f,rightTagsPnts2f,leftPnts2f,rightPnts2f,TagVec);
//    vector<vector<Point2f>> pntsvec;
//    pntsvec.push_back(leftPnts2f);
//    pntsvec.push_back(rightPnts2f);
//    //XMLWriter::writePnts2fData("F:/FeaturePoints.xml",pntsvec);
//    //step-3 计算左相机摄像机坐标系下的三维点信息
//    vector<Point3f> pnts3fVec;
//    //第一种计算三维点信息的方法
//    if(!Cal3dPoint(leftPnts2f,_campara1,rightPnts2f,
//                   _campara2,_camgrouppara.right2LeftRotVector,_camgrouppara.right2LeftTraVector,pnts3fVec))
//    {
//        return false;
//    }
//    //第二种计算三维点信息的方法
//    //转化相机参数及相机外参数为输入格式
//    /*Mat cameraMatrix1(3, 3, CV_64FC1);
//    Mat distCoeffs1(1, 4, CV_64FC1);
//    Mat cameraMatrix2(3, 3, CV_64FC1);
//    Mat distCoeffs2(1, 4, CV_64FC1);
//    for (int i=0; i < 3; i++)
//    {
//        for (int j=0; j < 3; j++)
//        {
//             cameraMatrix1.at<double>(i,j) = _campara1.CameraIntrinsic[i][j];
//              cameraMatrix2.at<double>(i,j) = _campara2.CameraIntrinsic[i][j];
//        }
//    }
//    for (int i = 0; i < 4; i++)
//    {
//         distCoeffs1.at<double>(0,i) = _campara1.DistortionCoeffs[i];
//          distCoeffs2.at<double>(0,i) = _campara2.DistortionCoeffs[i];
//    }
//    Mat R1 = Mat::eye(3,3,CV_64F);
//    Mat T1 = Mat::zeros(3,1,CV_64F);
//    Mat R2 = Mat::zeros(3,1,CV_64F);
//    Mat T2 = Mat::zeros(3,1,CV_64F);
//    R2.at<double>(0,0) = _camgrouppara.right2LeftRotVector[0];
//    R2.at<double>(1,0) = _camgrouppara.right2LeftRotVector[1];
//    R2.at<double>(2,0) = _camgrouppara.right2LeftRotVector[2];
//    T2.at<double>(0,0) = _camgrouppara.right2LeftTraVector[0];
//    T2.at<double>(1,0) = _camgrouppara.right2LeftTraVector[1];
//    T2.at<double>(2,0) = _camgrouppara.right2LeftTraVector[2];
//    Rodrigues(R2,R2);
//    if(!triangulatePnts(leftPnts2f,rightPnts2f,cameraMatrix1,R1,T1,distCoeffs1,
//cameraMatrix2,R2,T2,distCoeffs2,pnts3fVec))
//    {
//        return false;
//    }*/
//    vector<TagPoint3f> result;
//    addPntsTag(pnts3fVec,TagVec,result);
//    tagPnts3f = result;
//    return true;
//}

bool CoreAlgorithm::getTagPoint2f2(const Mat& img, vector<TagPoint2f>& tagPnts2f)
{
    cv::Rect mask =cv::Rect(Point2f(0,0),img.size());
    vector<RotatedRect> findResults;
    int kenelsize =5;
    if(!CoreAlgorithm::findEllipses(img,mask,findResults,kenelsize))
    {
        return false;
    }
    vector<Point2f> centerPnts;
    vector<float> longaxisRadius;
    for(unsigned int i =0;i<findResults.size();i++)
    {
        if(findResults[i].size.width<3)
            continue;
        centerPnts.push_back(findResults[i].center);
        longaxisRadius.push_back(findResults[i].size.width);
    }
    vector<TagPoint2f>  tagPnts;
    double a = 25.0;
    double d = 10.0;   ////  fang can shu
    double gamma = 2*(a+d)/d;
    float Featurelength;
    if(!CoreAlgorithm::findPntsWithTag(centerPnts,longaxisRadius,tagPnts,Featurelength,gamma))
    {
        return false;
    }
    if (tagPnts.size() != 7)
        return false;
    tagPnts2f = tagPnts;
    return true;
}

bool CoreAlgorithm::getTagPoint2f2(const Mat& img, vector<TagPoint2f>& tagPnts2f,cv::Rect mask)
{
    vector<RotatedRect> findResults;
    int kenelsize =5;
    if(!CoreAlgorithm::findEllipses(img,mask,findResults,kenelsize))
    {
        return false;
    }
    vector<Point2f> centerPnts;
    vector<float> longaxisRadius;
    for(unsigned int i =0;i<findResults.size();i++)
    {
        if(findResults[i].size.width<5)
            continue;
        centerPnts.push_back(findResults[i].center);
        longaxisRadius.push_back(findResults[i].size.width);
    }
    vector<TagPoint2f>  tagPnts;
    double a = 25.0;
    double d = 10.0;   ////  fang can shu
    double gamma = 2*(a+d)/d;
    float Featurelength;
    if(!CoreAlgorithm::findPntsWithTag(centerPnts,longaxisRadius,tagPnts,Featurelength,gamma))
    {
        return false;
    }
    if (tagPnts.size() != 7)
        return false;
    tagPnts2f = tagPnts;
    return true;
}

void CoreAlgorithm::PnPMethod(vector<Point3f> objectPoints,
                              vector<Point2f> imagePoints,
                              Mat cameraMatrix, Mat distCoeffs,
                              Mat &PoseR,Mat &PoseT,
                              vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag)
{
    PoseR = Mat::zeros(3,1,CV_64F);
    PoseT = Mat::zeros(3,1,CV_64F);
    //第一步，使用sovlePnP得到旋转向量和平移向量
    if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,Flag)))
    {
        return;
    }
    //第二步，使用projectPoints得到重投影的二维特征点，得到雅可比矩阵，仅仅抽取前六列（ 旋转向量，平移向量）则构成J
    vector<Point2f> imageReprojected;
    Mat jacobian;
    Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
    projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
    //将drot dt放到doutpara中(抽取前六列)
    for( int i=0;i<6;i++ )
    {
        jacobian.col(i).copyTo(dpoutpara.col(i));
    }
    //第三步，输出的特征点图像坐标，与检测的图像特征点坐标相减则为反向投射误差，即为delta他是2nX1的矩阵。
    Mat delta(dpoutpara.rows,1,CV_64FC1);
    for (unsigned int i = 0;i < imagePoints.size();i++)
    {
        delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
        delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

    }
    //第四步，根据公式求得一个方差值，6X6的矩阵，取对角线元素即为方差。为6X6的矩阵，其求平方根这位标准差，乘以3进行输出，为3被标准差结果。
    Mat covariance_pre;
    Mat dpoutpara_invert;
    Mat covariance;
    double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
    gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
    //covariance_pre = dpoutpara_invert*delta;//这种矩阵相乘方式也是可行的
    mulTransposed(covariance_pre,covariance,0);
    //第五步，求出3倍标准差
    Mat diag_covariance(covariance.rows,1,CV_64FC1);
    diag_covariance = covariance.diag(0);//取主对角线
    for(int i =0;i<diag_covariance.rows;i++)
    {
        if(i<3)
        {
            dpdrot.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
        }
        else
        {
            dpdt.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
        }
    }
    //第六步，求出转换到测量坐标系下的三维特征点，作为输出
    Mat _R = Mat::zeros(3,3,CV_64FC1);
    Rodrigues(PoseR,_R);
    for(unsigned int i = 0;i < objectPoints.size();i++)
    {
        Mat pointtemp1;
        Mat pointtemp4;
        Point3f pointtemp2 = Point3f(0,0,0);
        Point3d pointtemp3 = Point3d(0,0,0);
        pointtemp1.setTo(Scalar(0));
        pointtemp4.setTo(Scalar(0));
        pointtemp3.x = objectPoints[i].x;
        pointtemp3.y = objectPoints[i].y;
        pointtemp3.z = objectPoints[i].z;
        pointtemp1 = Mat(pointtemp3);
        pointtemp4 = _R*pointtemp1 + PoseT;
        pointtemp2 = Point3f(pointtemp4);
        PointToCam.push_back(pointtemp2);
    }
    return;
}

void CoreAlgorithm::PnPMethod(vector<Point3f> objectPoints,
                              vector<Point2f> imagePoints,
                              Mat cameraMatrix, Mat distCoeffs,
                              Mat &PoseR,Mat &PoseT,
                              vector<double> &dpdrot,vector<double> &dpdt,int Flag)
{
    float xsum=0,ysum=0,zsum=0;
    for(size_t h=0;h<objectPoints.size();h++)
    {
        xsum +=objectPoints[h].x;
        ysum +=objectPoints[h].y;
        zsum +=objectPoints[h].z;
    }
    Point3f centroid = Point3f(xsum/objectPoints.size(),ysum/objectPoints.size(),zsum/objectPoints.size());
    vector<Point3f> objectPointsnew;
    objectPointsnew.resize(objectPoints.size());
    for(size_t l=0;l<objectPoints.size();l++)
    {
        objectPointsnew[l] = objectPoints[l]-centroid;
    }
    PoseR = Mat::zeros(3,1,CV_64F);
    PoseT = Mat::zeros(3,1,CV_64F);
    //第一步，使用sovlePnP得到旋转向量和平移向量
    if(!(solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR,PoseT,false,Flag)))
    {
        return;
    }
    //第二步，使用projectPoints得到重投影的二维特征点，得到雅可比矩阵，仅仅抽取前六列（ 旋转向量，平移向量）则构成J
    vector<Point2f> imageReprojected;
    Mat jacobian;
    Mat dpoutpara(objectPoints.size()*2,6,CV_64FC1);
    projectPoints(objectPoints,PoseR,PoseT,cameraMatrix,distCoeffs,imageReprojected,jacobian);
    //将drot dt放到doutpara中(抽取前六列)
    for( int i=0;i<6;i++ )
    {
        jacobian.col(i).copyTo(dpoutpara.col(i));
    }
    //第三步，输出的特征点图像坐标，与检测的图像特征点坐标相减则为反向投射误差，即为delta他是2nX1的矩阵。
    Mat delta(dpoutpara.rows,1,CV_64FC1);
    for (unsigned int i = 0;i < imagePoints.size();i++)
    {
        delta.at<double>(2*i,0) = fabs(double(imagePoints[i].x-imageReprojected[i].x));
        delta.at<double>(2*i+1,0) =fabs(double(imagePoints[i].y-imageReprojected[i].y));

    }
    //XMLWriter::writeMatData("F:/1.xml",delta);
    //第四步，根据公式求得一个方差值，6X6的矩阵，取对角线元素即为方差。为6X6的矩阵，其求平方根这位标准差，乘以3进行输出，为3被标准差结果。
    Mat covariance_pre;
    Mat dpoutpara_invert;
    Mat covariance;
    double error = invert(dpoutpara,dpoutpara_invert,CV_SVD);
    gemm(dpoutpara_invert,delta,1,0,0,covariance_pre);
    //covariance_pre = dpoutpara_invert*delta;//这种矩阵相乘方式也是可行的
    mulTransposed(covariance_pre,covariance,0);
    //第五步，求出3倍标准差
    Mat diag_covariance(covariance.rows,1,CV_64FC1);
    diag_covariance = covariance.diag(0);//取主对角线
    for(int i =0;i<diag_covariance.rows;i++)
    {
        if(i<3)
        {
            dpdrot.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
        }
        else
        {
            dpdt.push_back(3*sqrt(abs(diag_covariance.at<double>(i,0))));
        }
    }
    //将坐标系元原点转化到原来的坐标原点上
    Point3f pointtemp = Point3f(PoseT);
    pointtemp = pointtemp-centroid;
    PoseT = Mat(pointtemp);
    return;
}

bool CoreAlgorithm::iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                                       vector<Point2f>imagePoints,
                                       Mat cameraMatrix, Mat distCoeffs,
                                       Mat &PoseR,Mat &PoseT,
                                       vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag,int iterativeTimes)
{
    Mat srcImgMat;
    if(img.channels()==1)
    {
        srcImgMat = img;
    }
    else
    {
        cvtColor(img, srcImgMat, CV_BGR2GRAY);
    }
    Mat PoseR_Src = Mat::zeros(3,1,CV_64F);
    Mat PoseT_Src = Mat::zeros(3,1,CV_64F);
    Mat PoseR_Dst = Mat::zeros(3,1,CV_64F);
    Mat PoseT_Dst = Mat::zeros(3,1,CV_64F);
    Mat parallelCamR;
    vector<double> dpdrot_Src,dpdt_Src,dpdrot_Dst,dpdt_Dst;
    generateParallelCamR(parallelCamR);
    Mat argaMat = Mat::zeros(3,3,CV_64F);
    Mat argaMatt = Mat::zeros(3,1,CV_64F);
    argaMat.at<double>(1,2) =1;argaMat.at<double>(0,1) =1;argaMat.at<double>(2,0) =1;
    vector<Point3f> pnts3d;
    pointPosetranslation(objectPoints,pnts3d,argaMat,argaMatt);
    PnPMethod(pnts3d,imagePoints,cameraMatrix,distCoeffs,PoseR_Src,PoseT_Src,dpdrot_Src,dpdt_Src,Flag);
    for(unsigned int k = iterativeTimes;k>0;k--)
    {
        //STEP-1 计算每幅特征点图像的平行视图的R'（并不是旋转矩阵） 3x3
        Mat parallelImgR,t1;
        calculateParallelViewR(PoseR_Src,PoseT_Src,parallelImgR);
        //STEP-2 把所有特征点转换到平行视图上
        vector<Point2f> corner_Parallel;
        //Mat t1 = Mat::zeros(3,1,CV_64F);
        PoseT_Src.copyTo(t1);
        undistortPoints2DifferentView(imagePoints,corner_Parallel,cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs,
                                      cameraMatrix,parallelCamR,t1,vector<double>());
        //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
        ////// image is too small
        Mat resultImgMat(2*srcImgMat.rows, 2*srcImgMat.cols, srcImgMat.type());
        undistortImg(srcImgMat, resultImgMat, cameraMatrix, distCoeffs, parallelImgR);
        /*	namedWindow("window",2);
        imshow("window",resultImgMat);
        waitKey();*/
        //在平行视图上进行亚像素圆检测
        double roisize = distancePoints(corner_Parallel[0],corner_Parallel[5]);
        double roiwidth = distancePoints(corner_Parallel[5],corner_Parallel[6])+roisize;
        double roiheight = distancePoints(corner_Parallel[0],corner_Parallel[4])+roisize;
        cv::Rect baseROI = cv::Rect(int(corner_Parallel[5].x-roisize/2),int(corner_Parallel[5].y-roisize/2),(int)roiwidth,(int)roiheight);
        vector<TagPoint2f> tagPnts2f;
        vector<Point2f> detectorResult;
        if(!CoreAlgorithm::getTagPoint2f2(resultImgMat,tagPnts2f,baseROI))return false;
        pntsTagConfirm(tagPnts2f,detectorResult);
        undistortPoints2DifferentView(detectorResult,detectorResult,
                                      cameraMatrix,parallelCamR,t1,vector<double>(),
                                      cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs);
        //STEP-5 使用PNP算法计算位姿
        dpdrot_Dst.clear();
        dpdt_Dst.clear();
        PnPMethod(pnts3d,detectorResult,cameraMatrix,distCoeffs,PoseR_Dst,PoseT_Dst,dpdrot_Dst,dpdt_Dst,Flag);
        //// 迭代终止条件，如果迭代没有使得位姿误差三倍标准差减小，则停止迭代
        //分别求出迭代前后位姿误差三倍标准差的模
        double dpdrot_Dst_value =0;
        double dpdt_Dst_value =0;
        double dpdrot_Src_value =0;
        double dpdt_Src_value =0;
        for(unsigned int i=0;i<3;i++)
        {
            dpdrot_Dst_value +=pow(dpdrot_Dst[i],2);
            dpdt_Dst_value +=pow(dpdt_Dst[i],2);
            dpdrot_Src_value +=pow(dpdrot_Src[i],2);
            dpdt_Src_value +=pow(dpdt_Src[i],2);
        }
        if(!(dpdrot_Dst_value<dpdrot_Src_value||dpdt_Dst_value<dpdt_Src_value))
        {
            PoseR = argaMat.inv()*PoseR_Dst;
            PoseT = PoseT_Dst;
            dpdrot = dpdrot_Dst;
            dpdt = dpdt_Dst;
            //第六步，求出转换到测量坐标系下的三维特征点，作为输出
            Mat _R = Mat::zeros(3,3,CV_64FC1);
            Rodrigues(PoseR,_R);
            pointPosetranslation(objectPoints,PointToCam,_R,PoseT);
            return true;
        }
        else
        {
            PoseR_Src = PoseR_Dst;
            PoseT_Src = PoseT_Dst;
            dpdrot_Src = dpdrot_Dst;
            dpdt_Src = dpdt_Dst;
        }
    }
    return true;
}

bool CoreAlgorithm::iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                                       vector<Point2f>imagePoints,
                                       Mat cameraMatrix, Mat distCoeffs,
                                       Mat &PoseR,Mat &PoseT,
                                       vector<double> &dpdrot,vector<double> &dpdt,int Flag,int iterativeTimes)
{
    Mat srcImgMat;
    if(img.channels()==1)
    {
        srcImgMat = img;
    }
    else
    {
        cvtColor(img, srcImgMat, CV_BGR2GRAY);
    }
    Mat PoseR_Src = Mat::zeros(3,1,CV_64F);
    Mat PoseT_Src = Mat::zeros(3,1,CV_64F);
    Mat PoseR_Dst = Mat::zeros(3,1,CV_64F);
    Mat PoseT_Dst = Mat::zeros(3,1,CV_64F);
    Mat parallelCamR;
    vector<double> dpdrot_Src,dpdt_Src,dpdrot_Dst,dpdt_Dst;
    generateParallelCamR(parallelCamR);
    Mat argaMat = Mat::zeros(3,3,CV_64F);
    Mat argaMatt = Mat::zeros(3,1,CV_64F);
    //argaMat.at<double>(1,2) =1;argaMat.at<double>(0,1) =1;argaMat.at<double>(2,0) =1;
    vector<Point3f> pnts3d;
    //pointPosetranslation(objectPoints,pnts3d,argaMat,argaMatt);
    //求出特征点的ROI区域
    cv::Rect featureROI;
    vector<Point2f> imagePointsROI;
    getFeatureROI(imagePoints,featureROI,imagePointsROI,distancePoints(imagePoints[0],imagePoints[1]));
    Mat imgROI = Mat(srcImgMat,featureROI);
    PnPMethod(objectPoints,imagePoints,cameraMatrix,distCoeffs,PoseR_Src,PoseT_Src,dpdrot_Src,dpdt_Src,Flag);
    for(unsigned int k = iterativeTimes;k>0;k--)
    {
        //STEP-1 计算每幅特征点图像的平行视图的R'（并不是旋转矩阵） 3x3
        Mat parallelImgR,t1;
        calculateParallelViewR(PoseR_Src,PoseT_Src,parallelImgR);
        //STEP-2 把所有特征点转换到平行视图上
        vector<Point2f> corner_Parallel;
        //Mat t1 = Mat::zeros(3,1,CV_64F);
        PoseT_Src.copyTo(t1);
        undistortPoints2DifferentView(imagePoints,corner_Parallel,cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs,
                                      cameraMatrix,parallelCamR,t1,vector<double>());
        //STEP-3 根据平行视图上的特征点，在平行视图上进行亚像素角点检测
        ////// image is too small
        //求出特征ROI

        Mat resultImgMat(2*srcImgMat.rows, 2*srcImgMat.cols, srcImgMat.type());
        undistortImg(srcImgMat, resultImgMat, cameraMatrix, distCoeffs, parallelImgR);
        /*	namedWindow("window",2);
        imshow("window",resultImgMat);
        waitKey();*/
        //在平行视图上进行亚像素圆检测
        double roisize = distancePoints(corner_Parallel[0],corner_Parallel[5]);
        double roiwidth = distancePoints(corner_Parallel[5],corner_Parallel[6])+roisize;
        double roiheight = distancePoints(corner_Parallel[0],corner_Parallel[4])+roisize;
        cv::Rect baseROI = cv::Rect(int(corner_Parallel[5].x-roisize/2),int(corner_Parallel[5].y-roisize/2),(int)roiwidth,(int)roiheight);
        vector<TagPoint2f> tagPnts2f;
        vector<Point2f> detectorResult;
        if(!CoreAlgorithm::getTagPoint2f2(resultImgMat,tagPnts2f,baseROI))return false;
        pntsTagConfirm(tagPnts2f,detectorResult);
        undistortPoints2DifferentView(detectorResult,detectorResult,
                                      cameraMatrix,parallelCamR,t1,vector<double>(),
                                      cameraMatrix,PoseR_Src,PoseT_Src,distCoeffs);
        //STEP-5 使用PNP算法计算位姿
        dpdrot_Dst.clear();
        dpdt_Dst.clear();
        PnPMethod(objectPoints,detectorResult,cameraMatrix,distCoeffs,PoseR_Dst,PoseT_Dst,dpdrot_Dst,dpdt_Dst,Flag);
        //// 迭代终止条件，如果迭代没有使得位姿误差三倍标准差减小，则停止迭代
        //分别求出迭代前后位姿误差三倍标准差的模
        double dpdrot_Dst_value =0;
        double dpdt_Dst_value =0;
        double dpdrot_Src_value =0;
        double dpdt_Src_value =0;
        for(unsigned int i=0;i<3;i++)
        {
            dpdrot_Dst_value +=pow(dpdrot_Dst[i],2);
            dpdt_Dst_value +=pow(dpdt_Dst[i],2);
            dpdrot_Src_value +=pow(dpdrot_Src[i],2);
            dpdt_Src_value +=pow(dpdt_Src[i],2);
        }
        //if(!(dpdrot_Dst_value<dpdrot_Src_value||dpdt_Dst_value<dpdt_Src_value))
        if(!(dpdt_Dst_value<dpdt_Src_value))
        {
            //PoseR = argaMat.inv()*PoseR_Dst;
            PoseR = PoseR_Src;
            PoseT = PoseT_Src;
            dpdrot = dpdrot_Src;
            dpdt = dpdt_Src;
            return true;
        }
        else
        {

            PoseR_Src = PoseR_Dst;
            PoseT_Src = PoseT_Dst;
            dpdrot_Src = dpdrot_Dst;
            dpdt_Src = dpdt_Dst;
        }
    }
    return true;
}

void CoreAlgorithm::GetSurfaceNormal(Point3f point1,Point3f point2,Point3f point3,SurfaceNormal &Normal)
{
    //得到的表面法线符合右手原则

    float  w0,w1,w2,v0,v1,v2,nx,ny,nz;
    w0=point2.x-point1.x;
    w1=point2.y-point1.y;
    w2=point2.z-point1.z;
    v0=point3.x-point1.x;
    v1=point3.y-point1.y;
    v2=point3.z-point1.z;
    nx=w1*v2-w2*v1;
    ny=w2*v0-w0*v2;
    nz=w0*v1-w1*v0;
    Normal.x = nx;
    Normal.y = ny;
    Normal.z = nz;
}

bool CoreAlgorithm::PlaneFitting(vector<Point3f> obsPoints,Plane& model)
{
    Mat coefficient_matrix = Mat::zeros(3,3,CV_32F);
    Mat variable_matrix = Mat::zeros(3,1,CV_32F);
    Mat equation_right = Mat::zeros(3,1,CV_32F);
    for(unsigned int i =0;i<obsPoints.size();i++)
    {
        coefficient_matrix.at<float>(0,0) += pow(obsPoints[i].x,2);
        coefficient_matrix.at<float>(0,1) += obsPoints[i].x*obsPoints[i].y;
        coefficient_matrix.at<float>(0,2) += obsPoints[i].x;
        coefficient_matrix.at<float>(1,1) += pow(obsPoints[i].y,2);
        coefficient_matrix.at<float>(1,2) += obsPoints[i].y;
        equation_right.at<float>(0,0) += obsPoints[i].x*obsPoints[i].z;
        equation_right.at<float>(1,0) += obsPoints[i].y*obsPoints[i].z;
        equation_right.at<float>(2,0) += obsPoints[i].z;

    }
    coefficient_matrix.at<float>(1,0) = coefficient_matrix.at<float>(0,1);
    coefficient_matrix.at<float>(2,0) = coefficient_matrix.at<float>(0,2);
    coefficient_matrix.at<float>(2,1) = coefficient_matrix.at<float>(1,2);
    coefficient_matrix.at<float>(2,2) = float(obsPoints.size());
	if (!solve(coefficient_matrix, equation_right, variable_matrix, cv::DECOMP_CHOLESKY))//高斯消元法?此处系数矩阵为对称矩阵
    {
        return false;
    }
    //方向向量单位化
    float dist = sqrt(pow(variable_matrix.at<float>(0,0),2)+ pow(variable_matrix.at<float>(1,0),2)+float(1.0));
    if ( dist == 0 )
    {
        model.normal.x=1;
        return false;

    }
    else
    {
        model.normal.x=variable_matrix.at<float>(0,0)/dist;
        model.normal.y=variable_matrix.at<float>(1,0)/dist;
        model.normal.z=float(-1.0)/dist;
    }
    model.orignal.x =0;
    model.orignal.y =0;
    model.orignal.z =variable_matrix.at<float>(2,0);
    return true;
}

bool CoreAlgorithm::CircleCylinderFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank)
{
    //step-1 解出一般曲面方程的十个待定系数
    //设置一个判定系数K.此系数指标还有待确定，误差？
    float K;
    switch (errorRank)
    {
    case 0:
        K = float(1e-5);
        break;
    case 1:
        K = float(1e-4);
        break;
    case 2:
        K = float(1e-3);
        break;
    default:
        break;
    }

    Mat coefficient_matrix(obsPoints.size(),10,CV_32F);
    Mat variable_matrix = Mat::zeros(10,1,CV_32F);
    //Mat equation_right = Mat::zeros(10,1,CV_32F);
    for(int i = 0;i<coefficient_matrix.rows;i++)
    {
        coefficient_matrix.at<float>(i,0)=pow(obsPoints[i].x,2);
        coefficient_matrix.at<float>(i,1)=pow(obsPoints[i].y,2);
        coefficient_matrix.at<float>(i,2)=pow(obsPoints[i].z,2);
        coefficient_matrix.at<float>(i,3)=obsPoints[i].x*obsPoints[i].y;
        coefficient_matrix.at<float>(i,4)=obsPoints[i].y*obsPoints[i].z;
        coefficient_matrix.at<float>(i,5)=obsPoints[i].x*obsPoints[i].z;
        coefficient_matrix.at<float>(i,6)=obsPoints[i].x;
        coefficient_matrix.at<float>(i,7)=obsPoints[i].y;
        coefficient_matrix.at<float>(i,8)=obsPoints[i].z;
        coefficient_matrix.at<float>(i,9)=1;
    }
    SVD::solveZ(coefficient_matrix,variable_matrix);
    float lamada;
    //第一种情况，方向向量有两个为零，即圆截面平行于某个二维坐标平面
    //根据解析解判断方向向量的值是否为零
    if(abs(variable_matrix.at<float>(3,0))<K&&abs(variable_matrix.at<float>(4,0))<K&&abs(variable_matrix.at<float>(5,0))<K)
    {
        //都为零，则方向向量必有两个量是零，一个为非零
        //step-1判断哪一个量为非零
        if(abs(variable_matrix.at<float>(0,0))-abs(variable_matrix.at<float>(1,0))<K||abs(variable_matrix.at<float>(0,0))-abs(variable_matrix.at<float>(2,0))<K)
        {
            lamada = variable_matrix.at<float>(0,0);
        }
        else
        {
            lamada = variable_matrix.at<float>(1,0);
        }
        models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
        models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
        models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
    }
    //第二种情况，方向向量有一个为零，其余不为零
    if((abs(variable_matrix.at<float>(3,0))>K||abs(variable_matrix.at<float>(4,0))>K||abs(variable_matrix.at<float>(5,0))>K)&&
            (abs(variable_matrix.at<float>(3,0))<K||abs(variable_matrix.at<float>(4,0))<K||abs(variable_matrix.at<float>(5,0))<K))
    {
        for(int i =3;i<6;i++)
        {
            if(abs(variable_matrix.at<float>(i,0))>K)
            {
                switch (i)
                {
                case 3:
                    lamada = variable_matrix.at<float>(2,0);
                    if(variable_matrix.at<float>(i,0)*lamada>0)
                        models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
                    models.axisNormal.z = -1*sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
                    models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
                    break;
                case 4:
                    lamada = variable_matrix.at<float>(0,0);
                    if(variable_matrix.at<float>(i,0)*lamada>0)
                        models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
                    models.axisNormal.z = -1*sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
                    models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
                    break;
                case 5:
                    lamada = variable_matrix.at<float>(1,0);
                    if(variable_matrix.at<float>(i,0)*lamada>0)
                        models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
                    models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
                    models.axisNormal.x = -1*sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));

                    break;
                default:
                    break;
                }
                if(variable_matrix.at<float>(i,0)*lamada<0)
                {
                    models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
                    models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
                    models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
                }
            }
        }
    }
    //第三种情况，方向向量都不为零
    if(abs(variable_matrix.at<float>(3,0))>K&&abs(variable_matrix.at<float>(4,0))>K&&abs(variable_matrix.at<float>(5,0))>K)
    {
        if(abs(pow(variable_matrix.at<float>(3,0),2)-pow(variable_matrix.at<float>(5,0),2))<K)
        {
            if(abs(variable_matrix.at<float>(1,0))<K&&abs(variable_matrix.at<float>(2,0)<K))
            {
                models.axisNormal.y =1;
                models.axisNormal.z =1;
                lamada =  variable_matrix.at<float>(4,0)/(-2);
                models.axisNormal.x = variable_matrix.at<float>(5,0)/(lamada*models.axisNormal.z*(-2));
            }
            else
            {
                lamada = (variable_matrix.at<float>(1,0)+variable_matrix.at<float>(2,0)-variable_matrix.at<float>(4,0))/2;
                models.axisNormal.y =sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
                models.axisNormal.z =sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
                models.axisNormal.x =sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
            }
        }
        else
        {
            lamada =(pow(variable_matrix.at<float>(5,0),2)*variable_matrix.at<float>(1,0)-variable_matrix.at<float>(2,0)*pow(variable_matrix.at<float>(3,0),2))/(pow(variable_matrix.at<float>(5,0),2)-pow(variable_matrix.at<float>(3,0),2));
            models.axisNormal.y = sqrt(abs(1-variable_matrix.at<float>(1,0)/lamada));
            models.axisNormal.z = sqrt(abs(1-variable_matrix.at<float>(2,0)/lamada));
            models.axisNormal.x = sqrt(abs(1-variable_matrix.at<float>(0,0)/lamada));
        }
        //判断方向向量的正负号
        if(variable_matrix.at<float>(3,0)*lamada>0)
        {
            models.axisNormal.y = -1*models.axisNormal.y;
            if(variable_matrix.at<float>(5,0)*lamada>0)
            {
                models.axisNormal.z = -1*models.axisNormal.z;
            }
        }
        else
        {
            if(variable_matrix.at<float>(5,0)*lamada>0)
            {
                models.axisNormal.z = -1*models.axisNormal.z;
            }
        }

    }
    Mat coefficient_matrix2(3,3,CV_32F);
    Mat variable_matrix2(3,1,CV_32F);
    Mat equation_right2(3,1,CV_32F);
    coefficient_matrix2.at<float>(0,0)=variable_matrix.at<float>(0,0)*2;
    coefficient_matrix2.at<float>(0,1)=variable_matrix.at<float>(3,0);
    coefficient_matrix2.at<float>(0,2)=variable_matrix.at<float>(5,0);
    coefficient_matrix2.at<float>(1,0)=variable_matrix.at<float>(3,0);
    coefficient_matrix2.at<float>(1,1)=variable_matrix.at<float>(1,0)*2;
    coefficient_matrix2.at<float>(1,2)=variable_matrix.at<float>(4,0);
    coefficient_matrix2.at<float>(2,0)=variable_matrix.at<float>(5,0);
    coefficient_matrix2.at<float>(2,1)=variable_matrix.at<float>(4,0);
    coefficient_matrix2.at<float>(2,2)=variable_matrix.at<float>(2,0)*2;
    equation_right2.at<float>(0,0)=-1*variable_matrix.at<float>(6,0);
    equation_right2.at<float>(1,0)=-1*variable_matrix.at<float>(7,0);
    equation_right2.at<float>(2,0)=-1*variable_matrix.at<float>(8,0);
	if (!solve(coefficient_matrix2, equation_right2, variable_matrix2, cv::DECOMP_LU))//高斯消元法?此处系数矩阵为对称矩阵
        return false;
    models.orignal.x =variable_matrix2.at<float>(0,0);
    models.orignal.y =variable_matrix2.at<float>(1,0);
    models.orignal.z =variable_matrix2.at<float>(2,0);
    //求出r
    models.r = sqrt(abs((variable_matrix.at<float>(0,0)*pow(models.orignal.x,2)+variable_matrix.at<float>(1,0)*pow(models.orignal.y,2)+
                         variable_matrix.at<float>(2,0)*pow(models.orignal.z,2)+variable_matrix.at<float>(3,0)*models.orignal.x*models.orignal.y+
                         variable_matrix.at<float>(4,0)*models.orignal.z*models.orignal.y+variable_matrix.at<float>(5,0)*models.orignal.x*models.orignal.z-
                         variable_matrix.at<float>(9,0))/lamada));
    //方向向量单位化
    float dist = sqrt(pow(models.axisNormal.x,2)+ pow(models.axisNormal.y,2)+pow(models.axisNormal.z,2));
    if ( dist == 0 )
    {
        return false;
    }
    else
    {
        models.axisNormal.x = models.axisNormal.x / dist;
        models.axisNormal.y = models.axisNormal.y / dist;
        models.axisNormal.z =models.axisNormal.z / dist;
    }
    return true;
}

bool CoreAlgorithm::CalPlaneLineIntersectPoint(const Plane _plane,const Line _line,Point3f &IntersectPoint)
{
    float vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t,vpt;
    vp1 = _plane.normal.x;
    vp2 = _plane.normal.y;
    vp3 = _plane.normal.z;
    n1 = _plane.orignal.x;
    n2 = _plane.orignal.y;
    n3 = _plane.orignal.z;
    v1 = _line.normal.x;
    v2 = _line.normal.y;
    v3 = _line.normal.z;
    m1 = _line.orignal.x;
    m2 = _line.orignal.y;
    m3 = _line.orignal.z;
    vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
    //首先判断直线是否与平面平行
    if (vpt == 0)
    {
        return false;
    }
    else
    {
        t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
        IntersectPoint.x = m1 + v1 * t;
        IntersectPoint.y = m2 + v2 * t;
        IntersectPoint.z = m3 + v3 * t;
        return true;
    }
}

void CoreAlgorithm::findTtypeROI(vector<Point2f>& imagePoints,cv::Rect& ROI)
{
    Mat PointMat = Mat(imagePoints).reshape(1);
    Mat PointX,PointY;
    PointMat.col(0).copyTo(PointX);
    PointMat.col(1).copyTo(PointY);
    Point2d minPoint,maxPoint;
    Point2d PointOffset(abs(imagePoints[0].x-imagePoints[1].x),abs(imagePoints[0].y-imagePoints[1].y));
    cv::minMaxLoc(PointX,&minPoint.x,&maxPoint.x);
    cv::minMaxLoc(PointY,&minPoint.y,&maxPoint.y);
    minPoint -=PointOffset;
    maxPoint +=PointOffset;
    Mat minPointMat = Mat(7,1,CV_32FC2,Scalar(minPoint.x,minPoint.y)).reshape(1);
    PointMat -=minPointMat;
    ROI = Rect(Point2i(minPoint),Point2i(maxPoint));
    imagePoints = Mat_<Point2f>(PointMat);
}

double CoreAlgorithm::distancePoints(const Point2f pnt1,const Point2f pnt2)
{
    double distance = sqrt(pow((pnt1.x-pnt2.x),2)+pow((pnt1.y-pnt2.y),2));
    return distance;
}

double CoreAlgorithm::distancePoints(const Point3f pnt1,const Point3f pnt2)
{
    double distance = sqrt(pow((pnt1.x-pnt2.x),2)+pow((pnt1.y-pnt2.y),2)+pow((pnt1.z-pnt2.z),2));
    return distance;
}

float CoreAlgorithm::distancePlanes(const Plane plane1,const Plane plane2)
{
    //现将平面参数转换成Ax+By+Cz+D=0表达形式
    Mat inplanepnts = Mat(plane2.orignal-plane1.orignal);
    Mat direction1 = Mat(plane1.normal);
    Mat direction2 = Mat(plane2.normal);
    cv::normalize(direction1,direction1);
    cv::normalize(direction2,direction2);
    double tempnum = abs(direction1.dot(direction2));
    if(tempnum<0.9||tempnum>1.1)
        return -1;
    tempnum = abs(direction1.dot(inplanepnts));
    return (float)tempnum;
}

bool CoreAlgorithm::findshortAxisPoints(const Point2f footpointNearPnts1,const Point2f footpointNearPnts2,const Point2f footpointNearPnts3,const Point2f footPoint,vector<TagPoint2f>& TagPoints)
{
    vector<TagPoint2f> tagPnts;
    TagPoint2f _tagPnt1, _tagPnt2,_tagPnt6,_tagPnt7;
    _tagPnt1[0] = TAG1;
    _tagPnt1[1] = footPoint.x;
    _tagPnt1[2] = footPoint.y;
    tagPnts.push_back(_tagPnt1);
    //找出不在短轴上的点
    float dis_threshold = 15;
    float a, b;//直线公式ax+by+1=0;a=(y1-y2)/(x1y2-x2y1),b=(x1-x2)/(x2y1-x1y2);
    Point2f pointIndex2,pnt1,pnt2,pnt3;
    for(int i=0;i<3;i++)
    {
        if(i==0)
        {
            pnt1 = footpointNearPnts1;
            pnt2 = footpointNearPnts2;
            pnt3 = footpointNearPnts3;
        }
        if(i==1)
        {
            pnt1 = footpointNearPnts2;
            pnt2 = footpointNearPnts3;
            pnt3 = footpointNearPnts1;
        }
        if(i==2)
        {
            pnt1 = footpointNearPnts1;
            pnt2 = footpointNearPnts3;
            pnt3 = footpointNearPnts2;
        }
        a = (pnt1.y - pnt2.y) / (pnt1.x*pnt2.y - pnt2.x*pnt1.y);
        b = (pnt1.x - pnt2.x) / (pnt2.x*pnt1.y - pnt1.x*pnt2.y);
        //点到直线的距离d=fabs(a*x+b*y+1)/sqrt(a*a+b*b);
        float dis = float(fabs(float(a*footPoint.x + b*footPoint.y + 1))/pow((a*a + b*b),(float)0.5));
        if (dis < dis_threshold)//当中心找的不准的时候这个阈值要放宽一点
        {
            _tagPnt2[0] = TAG2;
            _tagPnt2[1] = pnt3.x;
            _tagPnt2[2] = pnt3.y;
            tagPnts.push_back(_tagPnt2);
            //向量AB＝（x1,y1,z1）, 向量CD＝（x2,y2,z2）
            //向量AB×向量CD＝（y1z2-z1y2，z1x2-x1z2，x1y2-y1x2）
            Vec3f norm_X,norm_Y,norm_Z;//norm_X为长轴的方向向量，定义为1到5点为正方向
            norm_X[0] = pnt3.x - footPoint.x;
            norm_X[1] = pnt3.y - footPoint.y;
            norm_X[2] = 0;
            norm_Y[0] = pnt1.x -footPoint.x;
            norm_Y[1] = pnt1.y -footPoint.y;
            norm_Y[2] = 0;
            norm_Z[0] = norm_X[1] * norm_Y[2] - norm_X[2] * norm_Y[1];
            norm_Z[1] = norm_X[2] * norm_Y[0] - norm_X[0] * norm_Y[2];
            norm_Z[2] = norm_X[0] * norm_Y[1] - norm_X[1] * norm_Y[0];
            if(norm_Z[2]>0)
            {
                _tagPnt6[0] = TAG6;
                _tagPnt6[1] = pnt1.x;
                _tagPnt6[2] = pnt1.y;
                tagPnts.push_back(_tagPnt6);
                _tagPnt7[0] = TAG7;
                _tagPnt7[1] = pnt2.x;
                _tagPnt7[2] = pnt2.y;
                tagPnts.push_back(_tagPnt7);
                TagPoints = tagPnts;
            }
            else
            {
                _tagPnt6[0] = TAG6;
                _tagPnt6[1] = pnt2.x;
                _tagPnt6[2] = pnt2.y;
                tagPnts.push_back(_tagPnt6);
                _tagPnt7[0] = TAG7;
                _tagPnt7[1] = pnt1.x;
                _tagPnt7[2] = pnt1.y;
                tagPnts.push_back(_tagPnt7);
                TagPoints = tagPnts;
            }
            return true;
        }
    }
    return false;
}

void CoreAlgorithm::pointPosetranslation(const vector<Point3f> PointSrc,vector<Point3f>& PointDst,Mat R,Mat t)
{
    for(unsigned int i = 0;i < PointSrc.size();i++)
    {
        Mat pointtemp1;
        Mat pointtemp4;
        Point3f pointtemp2 = Point3f(0,0,0);
        Point3d pointtemp3 = Point3d(0,0,0);
        pointtemp1.setTo(Scalar(0));
        pointtemp4.setTo(Scalar(0));
        pointtemp3.x = PointSrc[i].x;
        pointtemp3.y = PointSrc[i].y;
        pointtemp3.z = PointSrc[i].z;
        pointtemp1 = Mat(pointtemp3);
        pointtemp4 = R*pointtemp1+t;
        pointtemp2 = Point3f(pointtemp4);
        PointDst.push_back(pointtemp2);
    }
}

void CoreAlgorithm::getFeatureROI(const vector<Point2f> pnts2fVec,cv::Rect& ROI,vector<Point2f>& pnts2fVecNew,double lengthOffset)
{
    Mat data = Mat(pnts2fVec).reshape(1);
    double minvalueX,minvalueY,maxvalueX,maxvalueY;
    cv::minMaxLoc(data.col(0),&minvalueX,&maxvalueX);
    cv::minMaxLoc(data.col(1),&minvalueY,&maxvalueY);
    ROI = cv::Rect(Point2d(minvalueX-lengthOffset,minvalueY-lengthOffset),Point2d(maxvalueX+lengthOffset,maxvalueY+lengthOffset));
    pnts2fVecNew.resize(pnts2fVec.size());
    for(size_t i=0;i<pnts2fVec.size();i++)
    {
        pnts2fVecNew[i].x = float(pnts2fVec[i].x-minvalueX+lengthOffset);
        pnts2fVecNew[i].y = float(pnts2fVec[i].y-minvalueY+lengthOffset);
    }
}

float CoreAlgorithm::roundness(const Point2f& pnt1, const vector<Point>& circle, const float r)
{
    float res=0;
    for (unsigned int i = 0; i < circle.size(); i++)
    {
        float dis = distance(pnt1, circle[i]);
        res += pow(abs(dis-r),2);
    }
    float non_roundness = sqrtf(res/circle.size());
    return non_roundness;
}

bool CoreAlgorithm::sortRingCenter2(vector<Point2f>& center, const int row, const int col)
{
    //找出四个角上的点
    Point2f rectAnglePnts[4];
    vector<Point2f> cornerRingPnts;
    minAreaRect(center).points(rectAnglePnts);//输出的点是按照顺时针排序的
    for (int i = 0; i < 4; i++)
    {
        Point2f pnt = center[0];
        float minDis = distance(pnt,rectAnglePnts[i]);
        for (unsigned int n = 1; n < center.size(); n++)
        {
            float dis = distance(center[n], rectAnglePnts[i]);
            if (dis < minDis)
            {
                minDis = dis;
                pnt = center[n];
            }
        }
        cornerRingPnts.push_back(pnt);
    }

    //找出左上角点
    int dis=100000;
    int leftTopIndex;
    for (int n = 0; n < 4;n++)
    {
        int _dis = int(cornerRingPnts[n].x + cornerRingPnts[n].y);
        if (_dis<dis)
        {
            dis = _dis;
            leftTopIndex = n;
        }
    }

    //找出在左右两列上的点
    Point2f pntlefttop = cornerRingPnts[leftTopIndex];
    Point2f pntrighttop = cornerRingPnts[(leftTopIndex+1)%4];
    Point2f pntrightdown = cornerRingPnts[(leftTopIndex+2)%4];
    Point2f pntleftdown = cornerRingPnts[(leftTopIndex+3)%4];

    vector<Point2f> leftColPnts,rightColPnts;
    for (unsigned int i = 0; i < center.size();i++)
    {
        if (isOnLine(pntlefttop, pntleftdown, center[i]))
        {
            leftColPnts.push_back(center[i]);
        }
        else if (isOnLine(pntrighttop, pntrightdown, center[i]))
        {
            rightColPnts.push_back(center[i]);
        }
    }

    if ( rightColPnts.size()!=leftColPnts.size())
    //if ( rightColPnts.size()!=leftColPnts.size() || rightColPnts.size()!=row || leftColPnts.size()!=row)
        return false;

    //左右两列点先按照Y坐标排序
    sortByYMin2Max(rightColPnts);

    sortByYMin2Max(leftColPnts);

    //从第最后一行开始找
    vector<Point2f> result;
    int colSize = 0;
    for (unsigned int n = 0; n <leftColPnts.size() ;n++)
    {
        vector<Point2f> rowPnts;
        for (unsigned int i = 0; i < center.size(); i++)
        {
            if (isOnLine(leftColPnts[n], rightColPnts[n], center[i]))
            {
                rowPnts.push_back(center[i]);
            }
        }

        sortByXMin2Max(rowPnts);

        /*if (rowPnts.size() != col)
            return false;*/

        if (colSize == 0)
            colSize = rowPnts.size();
        else
        {
            if (colSize != rowPnts.size())
                return false;
        }

        for (unsigned int i = 0; i <rowPnts.size() ;i++)
        {
            result.push_back(rowPnts[i]);
        }
    }

    center = result;
    return true;
}

//描述:从相机参数类CamPara中提取相机内参和畸变参数
bool CoreAlgorithm::ExtraCamMatrixAndDist(const CamPara& Camparam,Mat& CamMatrix,Mat& DistCoeffs)
{
    CamMatrix=Mat::zeros(3,3,CV_64FC1);
    DistCoeffs=Mat::zeros(1,4,CV_64FC1);
    for (int i=0; i < 3; i++)
    {
        for (int j=0; j < 3; j++)
        {
            CamMatrix.at<double>(i,j) = Camparam.CameraIntrinsic[i][j];
        }
    }
    for (int i = 0; i < 4; i++)
    {
        DistCoeffs.at<double>(0,i) = Camparam.DistortionCoeffs[i];
    }
    return true;
}

//描述：提取立体主动光标靶特征点圆心
bool CoreAlgorithm::DiscernLSCircle(Mat img,vector<SH_Ellipse> &Fea_circle,Mat &imge_contour,int Threshold,int areamin,int areamax,float rate,int flag)
{
	//input check
	if (img.channels()>1)
	{
		cvtColor(img,img,CV_RGB2GRAY);//将三通道转化为单通道
	}
	if (img.empty())
	{
		return false;
	}

	//var define
	int cirthreshold = 10; //圆度阈值，推荐值10
	EllipseBox rotatedBox;	//用来存储所提取的椭圆的信息
	Mat img_threshold;
	int i=0;
	int j =0;//椭圆提取跟踪
	int x = 0;
	char word[20];
	vector<vector <Point>> contours;
	vector<cv::Vec4i> hierarchy;
	vector <double> contour_area;
	int e_center_id=0;
	int thresh_level=cvRound(Threshold);
	/*ofstream txt_file("C:/Users/Administrator/Desktop/test.txt",ios::ate);*/
	//
	imge_contour = Mat::zeros(img.rows, img.cols, CV_8UC3);
	threshold(img,img_threshold,thresh_level,255,CV_THRESH_BINARY);
	//adaptiveThreshold(img,img_threshold,35,CV_ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,3,0);
	findContours(img_threshold,
		contours,					//所获得的轮廓点
		hierarchy,
		CV_RETR_LIST,				//获取轮廓的方法
		CV_CHAIN_APPROX_NONE);		//轮廓近似的方法
	//Mat imge_contour = Mat::zeros(img.rows, img.cols, CV_8UC3);
	if(!contours.empty()&&!hierarchy.empty())
	{
		int idx=0;
		for(;idx>=0;idx= hierarchy[idx][0])
		{
			contour_area.push_back(contourArea(contours[idx]));
			if (contour_area[idx]>areamin && contour_area[idx]<areamax)//第一个筛选条件：轮廓的像素面积大小
			{
				Scalar color( (rand()&255), (rand()&255), (rand()&255) );//表示颜色
				try
				{
					x++;
					RotatedRect _rotatebox;
					_rotatebox=fitEllipse(contours[idx]);

					if ((_rotatebox.size.height/_rotatebox.size.width)<LRATE)//第二个筛选条件：椭圆的长短轴的比值，去掉偏转角度较大的面（1.732为三条棱在投影平面上成120度时，的长短轴比）
					{
						i++;
						float ellipse_S=0.25*3.14159*_rotatebox.size.width*_rotatebox.size.height;//用面积的筛选方法，效果比较好
						float ellipse_L=3.14159*_rotatebox.size.width+2*(_rotatebox.size.height-_rotatebox.size.width);//计算椭圆周长，第三个筛选条件（实验效果不佳）
						sprintf(word,"%d",i);
						putText(imge_contour,word,_rotatebox.center,CV_FONT_HERSHEY_COMPLEX,1,Scalar(244,249,59),1);
						//...
						if (ellipse_S-contour_area[idx]<cirthreshold)
						{
							j++;
							rotatedBox.push_back(_rotatebox);
							drawContours(imge_contour,
								contours,
								idx,		//轮廓编号，当为负数时表示所有轮廓
								color,
								2);			//线的粗细
						}
					}
				}
				catch(...)		//三个点表示任何异常
				{
					continue;
				}

			}
		}
		//std::cout<<"经过最大最小面积筛选条件后椭圆个数："<<contours.size()<<endl;
		//std::cout<<"经过LRATE筛选条件后椭圆个数："<<i<<endl;
		//std::cout<<"经过cirthreshold筛选条件后椭圆个数："<<j<<endl;
	}
	if (flag == 0)
	{
		for (int i=0;i<rotatedBox.size();i++)
		{
			circle(imge_contour,rotatedBox[i].center,2,Scalar(255,0,0),2);
		}
		SH_Ellipse _temellipse;
		for (int i=0;i<rotatedBox.size();i++)
		{
			_temellipse.center=rotatedBox[i].center;
			_temellipse.macroaxis=rotatedBox[i].size.height;
			_temellipse.brachyaxis=rotatedBox[i].size.width;
			Fea_circle.push_back(_temellipse);
		}
		if (Fea_circle.size()<10)
		{
			return false;
		}
		return true;
	}
	else if(flag == 1)
	{
		if (rotatedBox.size()<10)
		{
			return false;
		}
		EllipseBox newRotatedBox,tempbox;	//用来存储椭圆的信息
		try
		{
			for (int i = 0; i < rotatedBox.size(); i ++)
			{
				//提取特征圆ROI
				cv::Rect ellRect, centerAreaRect;
				ellRect.x = rotatedBox[i].center.x - rotatedBox[i].size.height; 
				ellRect.y = rotatedBox[i].center.y - rotatedBox[i].size.height; 
				ellRect.width = 2*rotatedBox[i].size.height;
				ellRect.height = 2*rotatedBox[i].size.height;
				//use to limit the center area
				centerAreaRect.x = rotatedBox[i].center.x - rotatedBox[i].size.width; 
				centerAreaRect.y = rotatedBox[i].center.y - rotatedBox[i].size.width; 
				centerAreaRect.width = 2*rotatedBox[i].size.width;
				centerAreaRect.height = 2*rotatedBox[i].size.width;
				tempbox.clear();
				findEllipses(img,ellRect,tempbox,0.05,false,3);
				//abandon the spare feature

				for ( int j = 0; j < tempbox.size(); j++)
				{
					//check if the center out the range
					if (tempbox[j].center.x < centerAreaRect.x+centerAreaRect.width && tempbox[j].center.y < centerAreaRect.y+centerAreaRect.height && 
						tempbox[j].center.x > centerAreaRect.x && tempbox[j].center.y > centerAreaRect.y)
					{
						newRotatedBox.push_back(tempbox[j]);
					}
				}
			}
		}
		catch (...)
		{
			int j = 12;
		}
		SH_Ellipse _temellipse;
		for (int i=0;i<newRotatedBox.size();i++)
		{
			circle(imge_contour,newRotatedBox[i].center,2,Scalar(255,0,0),2);
			_temellipse.center=newRotatedBox[i].center;
			_temellipse.macroaxis=newRotatedBox[i].size.width*2;
			_temellipse.brachyaxis=newRotatedBox[i].size.height*2;
			Fea_circle.push_back(_temellipse);
		}
		if (Fea_circle.size()<10)
		{
			return false;
		}
		return true;
	}
}

//描述：对提取的立体标靶的特征点进行特征圆识别和面识别
bool CoreAlgorithm::FindMarkArea(Mat img,Mat& outimage,vector<SH_Ellipse> Fea_circle,MarkArea& MarkedArea,string& err_str)
{
    //参数定义
    float dis_restrian=1.4;//大小圆距离约束设定值
    Scalar drawcolor = Scalar(0,0,255);//颜色值分别是蓝，绿，红
    outimage=Mat::Mat(img.rows, img.cols, CV_8UC3);
    img.copyTo(outimage);
    if (img.channels()>1)
    {
        cvtColor(img,img,CV_RGB2GRAY);//将三通道转化为单通道
    }
    err_str="No Error!";
    vector <circle_inf> Cir_inf;
    //vector<SH_Ellipse>::iterator largeitr;
    vector<FeaCircle> LCIR;			//暂存已识别大圆
    vector<FeaCircle> SCIR;			//暂存已识别小圆

    //遍历所有椭圆，找出所有一号圆及其环绕小圆

    //把椭圆的基本信息存入特征圆属性结构体中
    for (int i=0;i<Fea_circle.size();i++)
    {
        circle_inf _cir;
        _cir.center=Fea_circle[i].center;
        _cir.height=Fea_circle[i].macroaxis;
        _cir.width=Fea_circle[i].brachyaxis;
        _cir.flag=CIR_0;
        _cir.dist2large1[0]=0;
        _cir.dist2large1[1]=0;
        _cir.markflag=0;
        _cir.dist2large1Flag=0;
        Cir_inf.push_back(_cir);
    }
    int cir_num=Cir_inf.size();

    //算出每个圆到其他圆的距离并分别对每个圆到其他圆的距离和1.25D进行比较，如果在1.25D距离范围内则标记
    for(int i=0;i<cir_num;i++)
    {
        float _dis;
        for(int j=0;j<cir_num;j++)
        {
            if (i==j)
                continue;
            _dis=sqrt(pow(Cir_inf[i].center.x-Cir_inf[j].center.x,2)+pow(Cir_inf[i].center.y-Cir_inf[j].center.y,2));
            Cir_inf[i].dis2othercir.push_back(_dis);
            if (_dis<dis_restrian*Cir_inf[i].height && pow(Cir_inf[i].height/Cir_inf[j].height,5)>RATE)//dis_restrian这个值有待确定，初定1.4
            {
                Cir_inf[i].markflag++;
                Cir_inf[i].cirflag.push_back(j);
            }
        }
    }

    //通过已找到的一号圆及其环绕小圆找出其他的特征圆
    float height_cir0=0;
    for (int m=0;m<cir_num;m++)
    {
        if (Cir_inf[m].markflag==2)//判断是否有两个环绕小圆
        {
            //对一号圆进行提取，提取到LCIR变量中
            Cir_inf[m].flag=CIR_1;
            height_cir0=Cir_inf[m].height;
            FeaCircle _cir;
            _cir.center = Cir_inf[m].center;
            _cir.flag=CIR_1;
            _cir.height=Cir_inf[m].height;
            _cir.width=Cir_inf[m].width;
            LCIR.push_back(_cir);
            //对该圆的两个环绕小圆进行提取（未编号）提取到SCIR变量中
            for (int j=0;j<2;j++)
            {
                int ord= Cir_inf[m].cirflag[j];
                _cir.center= Cir_inf[ord].center;
                _cir.flag= Cir_inf[ord].flag;
                SCIR.push_back(_cir);
            }
            //区分出一号圆的两个环绕小圆中的x轴方向的小圆和y方向的小圆，并定义X,Y,45°方向
            Vec3f norm_X,norm_Y,norm_Z;		//X,Y,Z轴单位向量
            Vec3f norm_45;					//45度方向单位向量
#if 0
			//标靶1.0
			norm_X[0]=LCIR[0].center.x-SCIR[0].center.x;
			norm_X[1]=LCIR[0].center.y-SCIR[0].center.y;
#endif
#if 1
			//标靶2.0
			norm_X[0]=SCIR[0].center.x-LCIR[0].center.x;
			norm_X[1]=SCIR[0].center.y-LCIR[0].center.y;
#endif

            norm_X[2]=0;
            norm_X=norm_X/sqrt(norm_X.dot(norm_X));

            norm_Y[0]=LCIR[0].center.x-SCIR[1].center.x;
            norm_Y[1]=LCIR[0].center.y-SCIR[1].center.y;
            norm_Y[2]=0;
            norm_Y=norm_Y/sqrt(norm_Y.dot(norm_Y));

            norm_Z=norm_X.cross(norm_Y);
            if(norm_Z[2]>0)
            {
                SCIR[0].flag=CIR_a;
                SCIR[1].flag=CIR_b;
            }
            else
            {
                SCIR[1].flag=CIR_a;
                SCIR[0].flag=CIR_b;
                swap(SCIR[0],SCIR[1]);
            }
            //第一次更新X,Y,Z，45度方向，用两个环绕小圆更新
            //45°方向
#if 0
			//标靶1.0
			norm_45[0]=SCIR[0].center.x-SCIR[1].center.x;
			norm_45[1]=SCIR[0].center.y-SCIR[1].center.y;
			norm_45[2]=0;
			float _norm_45=sqrt(norm_45.dot(norm_45));//全局变量
			norm_45=norm_45/_norm_45;
#endif
#if 1
			//标靶2.0
			//构造虚拟1号小圆
			float _x = LCIR[0].center.x-SCIR[0].center.x;
			float _y = LCIR[0].center.y-SCIR[0].center.y;
			Point2f virtualScir1;
			virtualScir1.x = LCIR[0].center.x + _x;
			virtualScir1.y = LCIR[0].center.y + _y;

			norm_45[0]=virtualScir1.x-SCIR[1].center.x;
			norm_45[1]=virtualScir1.y-SCIR[1].center.y;
			norm_45[2]=0;
			float _norm_45=sqrt(norm_45.dot(norm_45));//全局变量
			norm_45=norm_45/_norm_45;

#endif
			
            //X方向
#if 0
			//标靶1.0
			norm_X[0]=LCIR[0].center.x-SCIR[0].center.x;
			norm_X[1]=LCIR[0].center.y-SCIR[0].center.y;
#endif
#if 1
			//标靶2.0
			norm_X[0]=SCIR[0].center.x-LCIR[0].center.x;
			norm_X[1]=SCIR[0].center.y-LCIR[0].center.y;
#endif
            norm_X[2]=0;
            float _norm_X=sqrt(norm_X.dot(norm_X));
            norm_X=norm_X/_norm_X;
            //Y方向
            norm_Y[0]=LCIR[0].center.x-SCIR[1].center.x;
            norm_Y[1]=LCIR[0].center.y-SCIR[1].center.y;
            norm_Y[2]=0;
            float _norm_Y=sqrt(norm_Y.dot(norm_Y));
            norm_Y=norm_Y/_norm_Y;

            // 将一个面上其它的六个大圆（2，3，4，5，6，7号大圆）进行排序
            for (int i=0;i<Cir_inf.size();i++)
            {
                //对小圆及一号圆进行排除
                if (pow(height_cir0/Cir_inf[i].height,5)>RATE || Cir_inf[i].flag==CIR_1)
                {
                    continue;
                }
                Vec3f _norm;
                float _distance;
                _norm[0]=LCIR[0].center.x-Cir_inf[i].center.x;
                _norm[1]=LCIR[0].center.y-Cir_inf[i].center.y;
                _norm[2]=0;
                _distance=sqrt(_norm.dot(_norm));
                _norm=_norm/_distance;
                float dot_X,dot_Y,dot_45;
                dot_X = _norm.dot(norm_X);
                dot_Y = _norm.dot(norm_Y);
                dot_45 = _norm.dot(norm_45);
                //cout<<"dot_X="<<dot_X<<" "<<"dot_Y="<<dot_Y<<" "<<"dot_45="<<dot_45<<endl;
                if(abs((_norm.cross(norm_X))[2])<ALLOWERROR && dot_X<0)
                {
                    Cir_inf[i].dist2large1[0]=_distance;
                    /*cout<<"X_FLAG"<<endl;*/
                }
                else if (abs((_norm.cross(norm_Y))[2])<ALLOWERROR && dot_Y>0)
                {
                    Cir_inf[i].dist2large1[1]=_distance;
                    /*cout<<"Y_FLAG"<<endl;*/
                }
                else if (abs((_norm.cross(norm_45))[2])<ALLOWERROR && dot_45>0)
                {
                    Cir_inf[i].dist2large1[0]=_distance;
                    Cir_inf[i].dist2large1[1]=_distance;
                    /*cout<<"45_FLAG"<<endl;*/
                }
            }

            for (int i=0;i<Cir_inf.size();i++)
            {
                if(Cir_inf[i].dist2large1[0]!=0 && Cir_inf[i].dist2large1[1]==0&&
                        pow(height_cir0/Cir_inf[i].height,5)<RATE)
                {
                    for(int j=0;j<Cir_inf.size();j++)
                    {
                        if(Cir_inf[i].dist2large1[0]>=Cir_inf[j].dist2large1[0]
                                && Cir_inf[j].dist2large1[0]!=0 && Cir_inf[j].dist2large1[1]==0
                                && pow(height_cir0/Cir_inf[j].height,5)<RATE)
                        {
                            Cir_inf[i].dist2large1Flag++;
                        }
                    }

                }
                if(Cir_inf[i].dist2large1[1]!=0 && Cir_inf[i].dist2large1[0]==0
                        && pow(height_cir0/Cir_inf[i].height,5)<RATE)
                {
                    for(int j=0;j<Cir_inf.size();j++)
                    {
                        if(Cir_inf[i].dist2large1[1]>=Cir_inf[j].dist2large1[1]
                                && Cir_inf[j].dist2large1[1]!=0 && Cir_inf[j].dist2large1[0]==0
                                && pow(height_cir0/Cir_inf[j].height,5)<RATE)
                        {
                            Cir_inf[i].dist2large1Flag++;
                        }
                    }

                }
                //45度角比较特殊，还要判断这个圆是否是大圆还是小圆
                if(Cir_inf[i].dist2large1[1]!=0 && Cir_inf[i].dist2large1[0]!=0 &&
                        pow(height_cir0/Cir_inf[i].height,5)<RATE)
                {
                    for(int j=0;j<Cir_inf.size();j++)
                    {
                        if(Cir_inf[i].dist2large1[0]>=Cir_inf[j].dist2large1[0]
                                && Cir_inf[j].dist2large1[1]!=0 && Cir_inf[j].dist2large1[0]!=0
                                && pow(height_cir0/Cir_inf[j].height,5)<RATE)
                        {
                            Cir_inf[i].dist2large1Flag++;
                        }
                    }

                }
            }
            int markcirclenum=0;
            for(int i=0;i<Cir_inf.size();i++)
            {
                if (Cir_inf[i].dist2large1[0]!=0 && Cir_inf[i].dist2large1[1]==0)
                {
                    if(Cir_inf[i].dist2large1Flag==1 && Cir_inf[i].dist2large1[0]<4.1*LCIR[0].height)
                    {
                        Cir_inf[i].flag=CIR_2;
                        markcirclenum++;
                    }
                    if(Cir_inf[i].dist2large1Flag==2 && Cir_inf[i].dist2large1[0]<8.1*LCIR[0].height)
                    {
                        Cir_inf[i].flag=CIR_3;
                        markcirclenum++;
                    }
                }
                if (Cir_inf[i].dist2large1[1]!=0 && Cir_inf[i].dist2large1[0]==0)
                {
                    if(Cir_inf[i].dist2large1Flag==1 && Cir_inf[i].dist2large1[0]<4.5*LCIR[0].height)
                    {
                        Cir_inf[i].flag=CIR_4;
                        markcirclenum++;
                    }
                    if(Cir_inf[i].dist2large1Flag==2 && Cir_inf[i].dist2large1[0]<8.5*LCIR[0].height)
                    {
                        Cir_inf[i].flag=CIR_5;
                        markcirclenum++;
                    }
                }

                if (Cir_inf[i].dist2large1[1]!=0 && Cir_inf[i].dist2large1[0]!=0)
                {
                    if(Cir_inf[i].dist2large1Flag==1 && Cir_inf[i].dist2large1[0]<6.5*LCIR[0].height)
                    {
                        Cir_inf[i].flag=CIR_6;
                        markcirclenum++;
                    }
                    if(Cir_inf[i].dist2large1Flag==2 && Cir_inf[i].dist2large1[0]<13*LCIR[0].height)
                    {
                        Cir_inf[i].flag=CIR_7;
                        markcirclenum++;
                    }
                }

            }
            //test:
            //验证是否有七个大圆被打上标记(作用在于：滤除某个面上特征点显示不全的问题)
            int sum=0;
            for (int i=0;i<Cir_inf.size();i++)
            {
                sum+=Cir_inf[i].flag;
            }
            if (sum!=28 || markcirclenum!=6)//七个圆都被标记，且标记正确的话和为28
            {
                //对编号信息，距离信息标识信息进行清理
                for (int i=0;i<Cir_inf.size();i++)
                {
                    Cir_inf[i].flag=CIR_0;
                    Cir_inf[i].dist2large1[0]=0;
                    Cir_inf[i].dist2large1[1]=0;
                    Cir_inf[i].dist2large1Flag=0;
                }
                LCIR.clear();
                SCIR.clear();
                continue;
            }
            //提取已经识别的2~7号特征圆
            for (int i=0;i<Cir_inf.size();i++)
            {
                if (Cir_inf[i].flag>1)
                {
                    FeaCircle _cir1;
                    _cir1.center=Cir_inf[i].center;
                    _cir1.flag=Cir_inf[i].flag;
                    _cir1.height=Cir_inf[i].height;
                    _cir1.width=Cir_inf[i].width;
                    LCIR.push_back(_cir1);
                }
                //清理已经提取的圆信息
                Cir_inf[i].flag=CIR_0;
                Cir_inf[i].dist2large1[0]=0;
                Cir_inf[i].dist2large1[1]=0;
                Cir_inf[i].dist2large1Flag=0;
            }

            //对提取的大圆进行从小到大重新排序
            for (int i=0;i<LCIR.size();i++)
            {
                for(int j=0;j<LCIR.size()-1;j++)
                {
                    if (LCIR[j].flag>LCIR[j+1].flag)
                        swap(LCIR[j],LCIR[j+1]);
                }
            }
            //进一步判断是否从一号到七号都有:让前一个的flag减去后一个的flag，看是否为1
            bool condflag=false;
            for (int i=0;i<LCIR.size()-1;i++)
            {
                if (LCIR[i+1].flag-LCIR[i].flag!=1)
                {
                    LCIR.clear();
                    SCIR.clear();
                    condflag=true;
                    break;
                }
            }
            if (condflag==true)
            {
                continue;
            }
            //确认一号圆~七号圆都找到后更新三个方向向量
            norm_45[0]=LCIR[0].center.x-LCIR[6].center.x;
            norm_45[1]=LCIR[0].center.y-LCIR[6].center.y;
            norm_45[2]=0;
            _norm_45=sqrt(norm_45.dot(norm_45));
            norm_45=norm_45/_norm_45;

            norm_X[0]=LCIR[0].center.x-LCIR[2].center.x;
            norm_X[1]=LCIR[0].center.y-LCIR[2].center.y;
            norm_X[2]=0;
            _norm_X=sqrt(norm_X.dot(norm_X));
            norm_X=norm_X/_norm_X;

            norm_Y[0]=LCIR[0].center.x-LCIR[4].center.x;
            norm_Y[1]=LCIR[0].center.y-LCIR[4].center.y;
            norm_Y[2]=0;
            _norm_Y=sqrt(norm_Y.dot(norm_Y));
            norm_Y=norm_Y/_norm_Y;

            //根据和六号圆的尺寸约束找出离群小圆，即与7号圆的环绕小圆
            for (int i=0;i<Cir_inf.size();i++)
            {
                auto _distance=sqrt(pow(LCIR[6].center.x-Cir_inf[i].center.x,2)
                        +pow(LCIR[6].center.y-Cir_inf[i].center.y,2));
                if (_distance<dis_restrian*LCIR[6].height && _distance>LCIR[6].width/2)//为了排除自身这个圆
                {
                    FeaCircle _cir2;
                    _cir2.flag=CIR_c;
                    _cir2.center=Cir_inf[i].center;
                    SCIR.push_back(_cir2);
                    break;
                }
            }
            //检查是否找到c号小圆
            if (SCIR.size()!=3)
            {
                LCIR.clear();
                SCIR.clear();
                continue;
            }
            //通过上面程序就可得到离群小圆为small_circle[2]
            //////////////////////对圆所在面进行判断////////////////////
            //step1:定义三号小圆的方位
            //由已知的X,Y,45度方向来判断具体是哪个面
            //六个面分别表示为：
            //一号面：3号小圆位于X轴方向
            //二号面：3号小圆位于-Y轴方向
            //三号面：3号小圆位于-X轴方向
            //四号面：3号小圆位于Y轴方向
            //五号面：3号小圆位于-45°轴方向

            //定义3号小圆的方位
            Vec3f DIR3;
            float _DIR3;
            DIR3[0]=LCIR[6].center.x-SCIR[2].center.x;
            DIR3[1]=LCIR[6].center.y-SCIR[2].center.y;
            DIR3[2]=0;
            _DIR3=sqrt(DIR3.dot(DIR3));
            DIR3=DIR3/_DIR3;

            //step2:确定面编号
            mark_area _temparea;
            Point2f _tempLcircle;
            Point2f _tempScircle;
            float resultX,resultY,result45,_tempresult;
            resultX=abs((DIR3.cross(norm_X))[2]);
            resultY=abs((DIR3.cross(norm_Y))[2]);
            result45=abs((DIR3.cross(norm_45))[2]);
            if(resultX>resultY)
                _tempresult=resultY;
            else
                _tempresult=resultX;
            if(_tempresult>result45)
                _tempresult=result45;
            if(_tempresult==resultX)
            {
                if (DIR3.dot(norm_X)<0)
                {
                    _temparea.Face=Face_3;
                }
                else
                {
                    _temparea.Face=Face_1;
                }
            }
            if(_tempresult==result45)
            {
                _temparea.Face=Face_5;
            }
            if(_tempresult==resultY)
            {
                if (DIR3.dot(norm_Y)<0)
                {
                    _temparea.Face=Face_2;
                }
                else
                {
                    _temparea.Face=Face_4;
                }
            }
            //对找到的特征图形组合进行标记
            cv::putText(outimage,"a",SCIR[0].center,CV_FONT_HERSHEY_COMPLEX,1,drawcolor,1);
            cv::putText(outimage,"b",SCIR[1].center,CV_FONT_HERSHEY_COMPLEX,1,drawcolor,1);
            cv::putText(outimage,"c",SCIR[2].center,CV_FONT_HERSHEY_COMPLEX,1,drawcolor,1);
            for (int i=0;i<7;i++)
            {
                char word[20];
                sprintf(word,"%d",i);
                cv::putText(outimage,word,LCIR[i].center,CV_FONT_HERSHEY_COMPLEX,1,drawcolor,1);
            }
            drawArrow(outimage,LCIR[0].center,LCIR[2].center,10,30,drawcolor,5,8);
            drawArrow(outimage,LCIR[0].center,LCIR[4].center,10,30,drawcolor,5,8);
            //对已识别的面和其上的圆进行提取储存
            for (int i=0;i<7;i++)
            {
                _tempLcircle=LCIR[i].center;
                _temparea.large_circle.push_back(_tempLcircle);
            }
            for (int i=0;i<3;i++)
            {
                _tempScircle=SCIR[i].center;
                _temparea.small_circle.push_back(_tempScircle);
            }
            MarkedArea.push_back(_temparea);
            //标记出识别的面
            char word[20];
            sprintf(word,"F_%d",_temparea.Face);
            Point2f _tem;
            _tem.x=_temparea.large_circle[5].x-100;
            _tem.y=_temparea.large_circle[5].y+100;
            cv::putText(outimage,word,_tem,CV_FONT_HERSHEY_COMPLEX,3,Scalar(0,255,0),2);
            //识别到三个markarea便停止识别
            if (MarkedArea.size()==3)
            {
                break;
            }
            //在每个continue之前都对编号信息，距离信息标识信息进行清理
            LCIR.clear();
            SCIR.clear();
            continue;
        }
    }
    ////////////////////////////对每个一号圆进行查找完毕后判断MarkedArea////////////////////////////
    if(MarkedArea.size()==0)
    {
        err_str="ERROR: can't find the feature face in the image!";
        return false;
    }
    return true;
}

//描述：提取图片中的特征信息
///@param img		 输入 待处理图片
///@param MarkedArea 输出 识别结果
///@param err_str    输出 识别错误提示信息
///@param Face		 输入 所要识别的面（可指定多个面，如face中存放1，2表示提取图像中的1，2面，若图片中没有指定面返回false）
bool CoreAlgorithm::FindImgFeature(const Mat &img, MarkArea & MarkedArea, string& err_str, const vector<int> Face)
{
	err_str = "no error!";
	if (Face.size()>6)
	{
		err_str = "face flag error!";
		return false;
	}
	if(img.empty()) {return false; err_str = "no image!";}
	vector<SH_Ellipse> Fea_circle;
	 MarkArea  _Marked;
	Mat _tempimg;
	int threadvalue = ThreshLevel;
	if (!DiscernLSCircle(img,Fea_circle,_tempimg))
	{
		err_str = "DiscernLSCircle error!";
		return false;
	}
	else
	{
		if (!FindMarkArea(img,_tempimg,Fea_circle,_Marked,err_str))
		{
			return false;
		}
		else
		{
			int temp =0;
			for (int i=0;i<Face.size();i++)
			{
				temp++;
			}
			if (temp == 0 || Face.empty())
			{
				MarkedArea = _Marked;
				return true;
			}
			else
			{
				for (int j=0;j<Face.size();j++)
				{
					for (int i = 0;i<_Marked.size();i++)
					{
						if (_Marked[i].Face == Face[j])
						{
							MarkedArea.push_back(_Marked[i]);
						}
					}
					if ( !MarkedArea.empty() && MarkedArea[MarkedArea.size()-1].Face == Face[j])
						continue;
					else
					{
						err_str = "error: can fit just one face in the image!";
						return false;
					}
				}
				return true;
			}
		}
		
	}
}

//描述：提取图片中的特征信息（重载函数，用于双相机）
///@param Limg		 输入 左图片
///@param Rimg		 输入 右图片
///@param MarkedArea 输出 识别结果
///@param err_str    输出 识别错误提示信息
///@param Face		 输入 所要识别的面（可指定多个面，如face中存放1，2表示提取图像中的1，2面，若图片中没有指定面返回false）
bool CoreAlgorithm::FindImgFeature(const Mat &Limg, const Mat &Rimg,MarkArea &LMarkedArea, MarkArea &RMarkedArea, string& err_str, const vector<int> &Face)
{
	RMarkedArea.clear();
	LMarkedArea.clear();
	err_str = "no error!";
	if (Face.size()>6)
	{
		err_str = "face flag error!";
		return false;
	}
	if(Limg.empty() || Rimg.empty()) {return false; err_str = "Limag or Rimg is no image!";}
	vector<SH_Ellipse> LFea_circle,RFea_circle;
	MarkArea  _LMarked,_RMarked;
	Mat _tempimg;
	if (!DiscernLSCircle(Limg,LFea_circle,_tempimg) || !DiscernLSCircle(Rimg,RFea_circle,_tempimg))
	{
		err_str = "DiscernLSCircle error!";
		return false;
	}
	else
	{
		if (!FindMarkArea(Limg,_tempimg,LFea_circle,_LMarked,err_str) || !FindMarkArea(Rimg,_tempimg,RFea_circle,_RMarked,err_str))
		{
			return false;
		}
		else
		{
			int temp = 0;
			for (int i=0;i<Face.size();i++)
			{
				temp++;
			}
			if (temp == 0 || Face.empty())
			{
				LMarkedArea = _LMarked;
				RMarkedArea = _RMarked;
				return true;
			}
			else
			{
				for (int j=0;j<Face.size();j++)
				{
					for (int i = 0;i<_LMarked.size();i++)
					{
						if (_LMarked[i].Face == Face[j])
						{
							LMarkedArea.push_back(_LMarked[i]);
						}
					}
					for (int i = 0;i<_RMarked.size();i++)
					{
						if (_RMarked[i].Face == Face[j])
						{
							RMarkedArea.push_back(_RMarked[i]);
						}
					}
					if (!LMarkedArea.empty() && LMarkedArea.size() == RMarkedArea.size() && LMarkedArea[LMarkedArea.size()-1].Face == Face[j] && RMarkedArea[RMarkedArea.size()-1].Face == Face[j])
						continue;
					else
					{
						err_str = "error: can fit just one face in the image!";
						return false;
					}
				}
				return true;
			}
		}

	}
}



//在图片中画箭头
void CoreAlgorithm::drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,
                              cv::Scalar& color, int thickness, int lineType)
{
    const double PI = 3.1415926;
    Point arrow;
    //计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面）
    double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
    line(img, pStart, pEnd, color, thickness, lineType);
    //计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置）
    arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
    line(img, pEnd, arrow, color, thickness, lineType);
    arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
    arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
    line(img, pEnd, arrow, color, thickness, lineType);
}

//描述：获得立体标靶（即标靶的一号面）在测量坐标系下的位姿
bool CoreAlgorithm::TransferFrame(const MarkArea& markedarea,const Mat& distCoeffs,const Mat& cameraMatrix,const ObjectPara& objparam, Mat& rtTOcam,string& err_str)
{
    //检查标靶参数是否有误
    if (objparam.FacePnts[0][1].x==0 || objparam.RT2one[0].at<double>(0,3)==0)
    {
        err_str="error in TransferFrame: the target's param is empty!";
        return false;
    }
    if (distCoeffs.at<double>(0,3)==0 || cameraMatrix.at<double>(0,0)==0)
    {
        err_str="error in TransferFrame: the camparam's param is empty!";
        return false;
    }
    //test 用于测试根据不同面求出来的一号面的位姿矩阵的差异程度
    vector <Mat> rtTOcams;
    double err[2]={0,0};//不同面求出的一号面位姿的无穷范数
    //...
    //标靶上特征点实际坐标
    //vector<Point3f> objpoints;
    //初始化rtTOcam
    rtTOcam = Mat::zeros(4,4,CV_64FC1);
    //for (int m=0;m<7;m++)
    //{
    // objpoints.push_back(Point3f(0,0,0));
    //}
    ////初始化标靶特征点的实际坐标值
    //objpoints[0]=Point3f(0,0,0);
    //objpoints[1]=Point3f(40,0,0);
    //objpoints[2]=Point3f(80,0,0);
    //objpoints[3]=Point3f(0,40,0);
    //objpoints[4]=Point3f(0,80,0);
    //objpoints[5]=Point3f(40,40,0);
    //objpoints[6]=Point3f(80,80,0);

    //各个面到一号面的位姿矩阵
    //Mat RT2one[4];
    ////各工作面与第一工作面的位姿转化矩阵初始化
    ////2号面到1号面的转化矩阵
    //RT2one[0]=(Mat_<double>(4,4)<<0.0000,    0,        1.0000,     118.5000,
    //						0,          1.0000,  0,          0,
    //						-1.0000,    0,       0.0000,     -38.5000,
    //						0,	        0,		 0,		     1.0000);
    ////3号面到1号面的转化矩阵
    //RT2one[1]=(Mat_<double>(4,4)<<-1.0000,    0,       0.0000,     80.0000,
    //						0,          1.0000,  0,          0,
    //						-0.0000,    0,       -1.0000,    -157.0000,
    //						0,          0,       0,          1.0000);
    ////4号面到1号面的转化矩阵
    //RT2one[2]=(Mat_<double>(4,4)<<0.0000,     0,		 -1.0000,	 -38.5000,
    //						0,			1.0000,	 0,			 0,
    //						1.0000,     0,       0.0000,     -118.5000,
    //						0,          0,       0,          1.0000);
    ////5号面到1号面的转化矩阵
    //RT2one[3]=(Mat_<double>(4,4)<<1.0000,         0,         0,         0,
    //							0,    0.0000,   -1.0000,  -34.5000,
    //							0,    1.0000,    0.0000, -118.5000,
    //							0,         0,         0,    1.0000);

    //特征向量和旋转向量的三倍标准差
    vector<double> dpdrot;
    vector<double> dpdt;
    //pnp计算所得的位姿定义
    Mat picR=Mat::zeros(3,1,CV_64FC1);//旋转向量
    Mat  picT=Mat::zeros(3,1,CV_64FC1);//平移向量

    //其他变量定义
    Mat Rs;
    vector<Point3f> PointToCam;

    for (int i=0;i<markedarea.size();i++)
    {
        Point2f _temppoint;
        vector<Point2f> _tempoints;
        //获取标靶的位姿
        for (int j=0;j<markedarea[i].large_circle.size();j++)
        {
            _temppoint.x=markedarea[i].large_circle[j].x;
            _temppoint.y=markedarea[i].large_circle[j].y;
            _tempoints.push_back(_temppoint);
        }
        CoreAlgorithm::PnPMethod(objparam.FacePnts[(int)(markedarea[i].Face-1)],_tempoints,cameraMatrix,distCoeffs,picR,picT,dpdrot,dpdt,PointToCam);
        cv::Rodrigues(picR,Rs);
        Rs.copyTo(rtTOcam(cv::Range(0,3),cv::Range(0,3)));
        picT.copyTo(rtTOcam(cv::Range(0,3),cv::Range(3,4)));
        rtTOcam.at<double>(3,3)=1;
        //进行坐标系转化
        Mat _tempRT=Mat::zeros(4,4,CV_64FC1);
        switch (markedarea[i].Face)
        {
        case 2:
            objparam.RT2one[0].copyTo(_tempRT);
            invert(_tempRT,_tempRT);
            rtTOcam=rtTOcam*_tempRT;
            //test
            rtTOcams.push_back(rtTOcam);
            break;
        case 3:
            objparam.RT2one[1].copyTo(_tempRT);
            invert(_tempRT,_tempRT);
            rtTOcam=rtTOcam*_tempRT;
            //test
            rtTOcams.push_back(rtTOcam);
            break;
        case 4:
            //_tempRT=objparam.RT2one[2];请慎用Mat的等号操作，等号操作传递的只是Mat的数据头，他们共用一块数据区（发现时间2015.4.6）
            //改正后标定结果正确
            objparam.RT2one[2].copyTo(_tempRT);
            invert(_tempRT,_tempRT);
            rtTOcam=rtTOcam*_tempRT;
            //test
            rtTOcams.push_back(rtTOcam);
            break;
        case 5:
            objparam.RT2one[3].copyTo(_tempRT);
            invert(_tempRT,_tempRT);
            rtTOcam=rtTOcam*_tempRT;
            //test
            rtTOcams.push_back(rtTOcam);
            break;
        default:
            break;
        }
        return true;
    }
    //test
    switch (rtTOcams.size())
    {
    case 2:
		err[0] = cv::norm(rtTOcams[0], rtTOcams[1], cv::NORM_INF);
        break;
    case 3:
		err[0] = cv::norm(rtTOcams[0], rtTOcams[1], cv::NORM_INF);
		err[1] = cv::norm(rtTOcams[0], rtTOcams[2], cv::NORM_INF);
        break;
    default:
        break;
    }
}

//描述：找出左右相机图片中同时都能看到的面
int CoreAlgorithm::FindObjectSameFace (const MarkArea LeftCamMarkFace, const MarkArea RightCamMarkFace,
                               MarkArea& LeftSameFace,MarkArea& RightSameFace)
{
    for (int i=0;i<LeftCamMarkFace.size();i++)
    {
        for (int j=0;j<RightCamMarkFace.size();j++)
        {
            if (LeftCamMarkFace[i].Face==RightCamMarkFace[j].Face)
            {
                LeftSameFace.push_back(LeftCamMarkFace[i]);
                RightSameFace.push_back(RightCamMarkFace[j]);
            }
        }
    }
    return LeftSameFace.size();
}

//描述：提取左右相机标靶图像中指定的特征面上的特征点坐标
bool CoreAlgorithm::GetObjectFeatureData(const vector<std::string> LeftCamImagePath,const vector<std::string> RightCamImagePath,const FaceFlag flag,
                                         vector<vector<Point2f> >& LeftImagePoints,vector<vector<Point2f> >& RightImagePoints,string &err_inf)
{
    if (LeftCamImagePath.size()!=RightCamImagePath.size())
    {
        err_inf = "error: the input image number don't match!";
        return false;
    }
    Mat leftimage,rightimage;
    //test:
    //ofstream out_point2D("..\\datafile\\point2D.txt",ios::trunc|ios::ate);
    for (int i=0;i<LeftCamImagePath.size();i++)
    {
        vector<SH_Ellipse> Lcircle, Rcircle;
        vector<Point2f> L_FeaturePoints,R_FeaturePoints;
        MarkArea L_markedarea,R_markedarea;
        string err_str;
        leftimage=cv::imread(LeftCamImagePath[i],0);
        rightimage=cv::imread(RightCamImagePath[i],0);
        Mat _tempimage;
        if (!CoreAlgorithm::DiscernLSCircle(leftimage,Lcircle,_tempimage) ||
                !CoreAlgorithm::DiscernLSCircle(rightimage,Rcircle,_tempimage))
        {
            continue;
        }
        else if(!CoreAlgorithm::FindMarkArea(leftimage,_tempimage,Lcircle,L_markedarea,err_str)||
                !CoreAlgorithm::FindMarkArea(rightimage,_tempimage,Rcircle,R_markedarea,err_str))
        {
            continue;
        }
        else
        {
            //TEST
            /*	 out_point2D<<"********************************************"<<endl;
             out_point2D<<"Image_"<<i<<endl;
             for (int k=0;k<L_markedarea.size();k++)
             {
             if (L_markedarea[k].Face==Face_1)
             {
             for (int j=0;j<7;j++)
             {
             out_point2D<<"LPNT_"<<j<<L_markedarea[k].large_circle[j]<<endl;
             out_point2D<<"RPNT_"<<j<<L_markedarea[k].large_circle[j]<<endl;
             }
             }
             }*/
            //
            for (int j=0;j<L_markedarea.size();j++)
            {
                if (L_markedarea[j].Face==flag)
                {
                    L_FeaturePoints = L_markedarea[j].large_circle;
                    L_markedarea.clear();
                    break;
                }
            }
            if (L_FeaturePoints.size()==7)
            {
                LeftImagePoints.push_back(L_FeaturePoints);
            }
            L_FeaturePoints.clear();
            for (int k=0;k<R_markedarea.size();k++)
            {
                if (R_markedarea[k].Face==flag)
                {
                    R_FeaturePoints = R_markedarea[k].large_circle;
                    R_markedarea.clear();
                    break;
                }
            }
            if (R_FeaturePoints.size()==7)
            {
                RightImagePoints.push_back(R_FeaturePoints);
            }
            R_FeaturePoints.clear();
            if (RightImagePoints.size()!=LeftImagePoints.size())
            {
                err_inf ="error: one of the image can't discern the specific face!";
                return false;
            }
        }
    }
    if (LeftImagePoints.size()!=RightImagePoints.size())
    {
        err_inf = "error: output image points number don't match!";
        return false;
    }
    if (LeftImagePoints.size()==0)
    {
        err_inf = "error: output image points number is zero";
        return false;
    }
    return true;
}

bool CoreAlgorithm::GetObjectFeatureData(const vector<std::string> LeftCamImagePath,const vector<std::string> RightCamImagePath,const int flag,
                                         vector<vector<Point2f> >& LeftImagePnt1s,vector<vector<Point2f> >& RightImagePnt1s,
                                         vector<vector<Point2f> >& LeftImagePnt2s,vector<vector<Point2f> >& RightImagePnt2s,string& err_inf)
{
    if (LeftCamImagePath.size()!=RightCamImagePath.size())
    {
        err_inf = "error: the input image number don't match!";
        return false;
    }

    for (int i=0;i<LeftCamImagePath.size();i++)
    {
        string err_str;
        vector<SH_Ellipse> Lcircle, Rcircle;
        Mat leftimage,rightimage;
        MarkArea L_markedarea,R_markedarea;
        leftimage=cv::imread(LeftCamImagePath[i],0);
        rightimage=cv::imread(RightCamImagePath[i],0);
        Mat _tempimage;
        if (!CoreAlgorithm::DiscernLSCircle(leftimage,Lcircle,_tempimage) ||
                !CoreAlgorithm::DiscernLSCircle(rightimage,Rcircle,_tempimage))
        {
            continue;
        }
        else if(!CoreAlgorithm::FindMarkArea(leftimage,_tempimage,Lcircle,L_markedarea,err_str)||
                !CoreAlgorithm::FindMarkArea(rightimage,_tempimage,Rcircle,R_markedarea,err_str))
        {
            continue;
        }
        else
        {
            vector<Point2f> L_FeaturePoints,R_FeaturePoints;
            switch (flag)
            {
#pragma region switch

            case FIR_TO_TW://二号面相对一号面
                //处理左图片特征点信息
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_1)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                if (L_FeaturePoints.size()!=7)
                {
                    continue;
                }
                LeftImagePnt1s.push_back(L_FeaturePoints);
                L_FeaturePoints.clear();
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_2)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                //如果得不到第二面的特征点则直接进行下一张图片处理，并清除已经存储的第一个面的特征点信息
                if (L_FeaturePoints.size()!=7)
                {
                    LeftImagePnt1s.erase(LeftImagePnt1s.end()-1);
                    continue;
                }
                LeftImagePnt2s.push_back(L_FeaturePoints);
                //处理右图片特征点信息
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_1)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                    continue;
                RightImagePnt1s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_2)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    RightImagePnt1s.erase(RightImagePnt1s.end()-1);
                    continue;
                }
                RightImagePnt2s.push_back(R_FeaturePoints);
                break;
            case FIR_TO_TH://三号面相对二号面：因为三号面和一号面无法同时看到，所以只能通过二号面来转化
                //处理左图片特征点信息
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_2)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                if (L_FeaturePoints.size()!=7)
                {
                    continue;
                }
                LeftImagePnt1s.push_back(L_FeaturePoints);
                L_FeaturePoints.clear();
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_3)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                //如果得不到第二面的特征点则直接进行下一张图片处理，并清除已经存储的第一个面的特征点信息
                if (L_FeaturePoints.size()!=7)
                {
                    LeftImagePnt1s.erase(LeftImagePnt1s.end()-1);
                    continue;
                }
                LeftImagePnt2s.push_back(L_FeaturePoints);
                L_FeaturePoints.clear();
                //处理右图片特征点信息
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_2)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    continue;
                }
                RightImagePnt1s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_3)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    RightImagePnt1s.erase(RightImagePnt1s.end()-1);
                    continue;
                }
                RightImagePnt2s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                break;
            case FIR_TO_FO://一号面到四号面
                //处理左图片特征点信息
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_1)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                if (L_FeaturePoints.size()!=7)
                {
                    continue;
                }
                LeftImagePnt1s.push_back(L_FeaturePoints);
                L_FeaturePoints.clear();
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_4)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                //如果得不到第二面的特征点则直接进行下一张图片处理，并清除已经存储的第一个面的特征点信息
                if (L_FeaturePoints.size()!=7)
                {
                    LeftImagePnt1s.erase(LeftImagePnt1s.end()-1);
                    continue;
                }
                LeftImagePnt2s.push_back(L_FeaturePoints);
                //处理右图片特征点信息
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_1)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    continue;
                }
                RightImagePnt1s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_4)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    RightImagePnt1s.erase(RightImagePnt1s.end()-1);
                    continue;
                }
                RightImagePnt2s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                break;
            case FIR_TO_FI://一号面到五号面
                //处理左图片特征点信息
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_1)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                if (L_FeaturePoints.size()!=7)
                {
                    continue;
                }
                LeftImagePnt1s.push_back(L_FeaturePoints);
                L_FeaturePoints.clear();
                for (int j=0;j<L_markedarea.size();j++)
                {
                    if (L_markedarea[j].Face==Face_5)
                        L_FeaturePoints = L_markedarea[j].large_circle;
                }
                //如果得不到第二面的特征点则直接进行下一张图片处理，并清除已经存储的第一个面的特征点信息
                if (L_FeaturePoints.size()!=7)
                {
                    LeftImagePnt1s.erase(LeftImagePnt1s.end()-1);
                    continue;
                }
                LeftImagePnt2s.push_back(L_FeaturePoints);
                L_FeaturePoints.clear();
                //处理右图片特征点信息
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_1)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    continue;
                }
                RightImagePnt1s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                for (int j=0;j<R_markedarea.size();j++)
                {
                    if (R_markedarea[j].Face==Face_5)
                        R_FeaturePoints = R_markedarea[j].large_circle;
                }
                if (R_FeaturePoints.size()!=7)
                {
                    RightImagePnt1s.erase(RightImagePnt1s.end()-1);
                    continue;
                }
                RightImagePnt2s.push_back(R_FeaturePoints);
                R_FeaturePoints.clear();
                break;
            default:
                break;

#pragma endregion
            }
            if (RightImagePnt1s.size()!=LeftImagePnt1s.size() || RightImagePnt2s.size()!=LeftImagePnt2s.size())
            {
                return false;
                /*			 LeftImagePnt1s.erase(RightImagePnt1s.end()-1);
                 LeftImagePnt2s.erase(RightImagePnt1s.end()-1);
                 RightImagePnt1s.erase(RightImagePnt1s.end()-1);
                 RightImagePnt2s.erase(RightImagePnt1s.end()-1);*/
            }
        }
    }
    if (LeftImagePnt1s.size()==0)
    {
        err_inf = "error: output image points number is zero";
        return false;
    }
    return true;
}



//描述：三角法获取特征点三维信息
bool CoreAlgorithm::CalibrateFeatureDim(const vector<vector<Point2f> > LeftImagePoints,const vector<vector<Point2f> > RightImagePoints,
                                        const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
                                        const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
                                        vector<Point3f>& pnts3d)
{
    if (LeftImagePoints.size()!=RightImagePoints.size())
    {
        return false;
    }
    //在相机坐标系下的特征点的位置信息
    vector<Point3f> pntsToCam;
    //在世界坐标系下（标靶工作面上）的特征点的位置信息
    vector<Point3f> pntsToObj;
    vector<vector<Point3f> > pntsToObjs;
    Point3f PVec[6];
    Mat MatPVec[6];
    for (int i=0;i<6;i++)
    {
        MatPVec[i]=Mat::zeros(3,1,CV_64FC1);
    }
    //世界坐标系到相机坐标系的旋转变化矩阵
    cv::Mat _tempR=cv::Mat::zeros(3,2,CV_64FC1);
    cv::Mat _tempR1=cv::Mat::zeros(3,3,CV_64FC1);
    cv::Mat _tempR2=cv::Mat::zeros(3,3,CV_64FC1);
    for (int i=0;i<6;i++)
    {
        PVec[i]=Point3f(0,0,0);
    }
    for (int i=0;i<LeftImagePoints.size();i++)
    {
        bool flag=CoreAlgorithm::triangulatePnts(LeftImagePoints[i],RightImagePoints[i],
                                                 cameraMatrix1,R1,T1,distCoeffs1,cameraMatrix2,R2,T2,distCoeffs2,pntsToCam);
        if (flag==true && pntsToCam.size()==7)
        {
            //让特征面做平移，使一号点的坐标移到相机坐标系的原点，求出其他六个点的坐标
            for (int j=0;j<6;j++)
            {
                PVec[j]=pntsToCam[j+1]-pntsToCam[0];

                MatPVec[j].at<double>(0,0)=PVec[j].x;
                MatPVec[j].at<double>(1,0)=PVec[j].y;
                MatPVec[j].at<double>(2,0)=PVec[j].z;
            }
            //确定相机坐标系与世界坐标系的转化矩阵
            _tempR.col(0)=((cv::Mat_ <double> (3,1) <<PVec[1].x, PVec[1].y, PVec[1].z));
            _tempR.col(1)=((cv::Mat_ <double> (3,1) <<PVec[3].x, PVec[3].y, PVec[3].z));
            CoreAlgorithm::GramSchmidt(_tempR,_tempR1);
            //_tempR1.col(2)=_tempR1.col(0).cross(_tempR1.col(1));
            cv::invert(_tempR1,_tempR2);
            //将特征点坐标从相机坐标系转到标靶坐标系（即世界坐标系下）
            for (int j=0;j<7;j++)
            {
                Mat _temp=Mat::zeros(3,1,CV_64FC1);
                Point3f _temppnt;
                if (j=0)
                    continue;
                _temp=_tempR2*MatPVec[j-1];
                _temppnt.x=_temp.at<double>(0,0);
                _temppnt.y=_temp.at<double>(1,0);
                _temppnt.z=_temp.at<double>(2,0);
                pntsToObj.push_back(_temppnt);
            }
            pntsToObjs.push_back(pntsToObj);
        }

    }
    //对多组特征点尺寸取平均，获得各特征点在世界坐标系下的坐标值
    Point3f sum=Point3f(0,0,0);
    Point3f _pnt;
    for (int j=1;j<7;j++)
    {
        for (int i =0;i<pntsToObjs.size();i++)
        {
            sum.x+=pntsToObjs[i][j-1].x;
            sum.y+=pntsToObjs[i][j-1].y;
            sum.z+=pntsToObjs[i][j-1].z;
        }
        _pnt.x=sum.x/pntsToObjs.size();
        _pnt.y=sum.y/pntsToObjs.size();
        _pnt.z=sum.z/pntsToObjs.size();
        pnts3d.push_back(_pnt);
        sum=Point3f(0,0,0);
    }
    _pnt=Point3f(0,0,0);//一号特征点定义为原点
    pnts3d.insert(pnts3d.begin(),_pnt);
    return true;
}

//(重载上面的函数)用Cal3dPoint（）方法获取特征面上七个特征点在世界坐标系下的坐标值（世界坐标系：以1号大圆的圆心为原点，以1号圆圆心和3,5号圆心的向量为x,y轴）
bool CoreAlgorithm::CalibrateFeatureDim(const vector<vector<Point2f> > LeftImagePoints,const vector<vector<Point2f> > RightImagePoints,
                                        const CamPara& camParaLeft,const CamPara& camParaRight,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& pnts3d)
{
    if (LeftImagePoints.size()!=RightImagePoints.size())
    {
        return false;
    }
    //在相机坐标系下的特征点的位置信息
    vector <Point3f> pntsToCam;
    //在世界坐标系下（标靶工作面上）的特征点的位置信息
    vector <Point3f> pntsToObj;
    vector<vector <Point3f>> pntsToObjs;
    Point3f PVec[6];
    Mat MatPVec[6];
    for (int i=0;i<6;i++)
    {
        MatPVec[i]=Mat::zeros(3,1,CV_64FC1);
    }
    //世界坐标系到相机坐标系的旋转变化矩阵
    cv::Mat _tempR=cv::Mat::zeros(3,2,CV_64FC1);
    cv::Mat _tempR1=cv::Mat::zeros(3,3,CV_64FC1);
    cv::Mat _tempR2=cv::Mat::zeros(3,3,CV_64FC1);
    for (int i=0;i<6;i++)
    {
        PVec[i]=Point3f(0,0,0);
    }
    for (int i=0;i<LeftImagePoints.size();i++)
    {
        bool flag=CoreAlgorithm::Cal3dPoint(LeftImagePoints[i],camParaLeft,RightImagePoints[i],camParaRight,rotVector,traVector,pntsToCam);
        if (flag==true && pntsToCam.size()==7)
        {
            //让特征面做平移，使一号点的坐标移到相机坐标系的原点，求出其他六个点的坐标
            for (int j=0;j<6;j++)
            {
                PVec[j]=pntsToCam[j+1]-pntsToCam[0];

                MatPVec[j].at<double>(0,0)=PVec[j].x;
                MatPVec[j].at<double>(1,0)=PVec[j].y;
                MatPVec[j].at<double>(2,0)=PVec[j].z;
            }
            _tempR.at<double>(0,0)=PVec[1].x;
            _tempR.at<double>(1,0)=PVec[1].y;
            _tempR.at<double>(2,0)=PVec[1].z;
            _tempR.at<double>(0,1)=PVec[3].x;
            _tempR.at<double>(1,1)=PVec[3].y;
            _tempR.at<double>(2,1)=PVec[3].z;
            CoreAlgorithm::GramSchmidt(_tempR,_tempR1);
            cv::invert(_tempR1,_tempR2);
            //将特征点坐标从相机坐标系转到标靶坐标系（即世界坐标系下）
            for (int j=0;j<7;j++)
            {
                Mat _temp=Mat::zeros(3,1,CV_64FC1);
                Point3f _temppnt=Point3f(0,0,0);
                if (j==0)
                    continue;
                _temp=_tempR2*MatPVec[j-1];
                _temppnt.x=_temp.at<double>(0,0);
                _temppnt.y=_temp.at<double>(1,0);
                _temppnt.z=_temp.at<double>(2,0);
                pntsToObj.push_back(_temppnt);
            }
            pntsToObjs.push_back(pntsToObj);
            pntsToObj.clear();
        }

    }
    //对多组特征点尺寸取平均，获得各特征点在世界坐标系下的坐标值
    Point3f sum=Point3f(0,0,0);
    Point3f _pnt;
    for (int j=1;j<7;j++)
    {
        for (int i =0;i<pntsToObjs.size();i++)
        {
            sum.x+=pntsToObjs[i][j-1].x;
            sum.y+=pntsToObjs[i][j-1].y;
            sum.z+=pntsToObjs[i][j-1].z;
        }
        _pnt.x=sum.x/pntsToObjs.size();
        _pnt.y=sum.y/pntsToObjs.size();
        _pnt.z=sum.z/pntsToObjs.size();
        pnts3d.push_back(_pnt);
        sum=Point3f(0,0,0);
    }
    _pnt=Point3f(0,0,0);//一号特征点定义为原点
    pnts3d.insert(pnts3d.begin(),_pnt);
    return true;
}


//描述：对输入的三维向量进行斯密特标准正交化
bool CoreAlgorithm::GramSchmidt(const vector<Vec3f> src,vector<Vec3f> dst)
{
    if (src.size()>3||src.size()<2||dst.size()!=3)
    {
        return false;
    }
    if (src.size()==3)
    {
        //斯密特正交化
        float x, y, z;
        dst[0]=src[0];
        dst[1]=src[1]-src[1].dot(src[0])/dst[0].dot(dst[0])*dst[0];
        dst[2]=src[2]-src[2].dot(src[0])/dst[0].dot(dst[0])*dst[0]-
                src[2].dot(dst[1])/dst[1].dot(dst[1])*dst[1];
        //单位化
        dst[0]=dst[0]/cv::norm(dst[0]);
        dst[1]=dst[1]/cv::norm(dst[1]);
        dst[2]=dst[2]/cv::norm(dst[2]);
        //检验正交化后向量是否两两正交
        x=dst[0].dot(dst[1]);
        y=dst[0].dot(dst[2]);
        z=dst[1].dot(dst[2]);
        if ((int)x*1000!=0 || (int)x*1000!=0 || (int)x*1000!=0)
        {
            return false;
        }
    }
    else if (src.size()==2)
    {
        float x;
        dst[0]=src[0];
        dst[1]=src[1]-src[1].dot(src[0])/dst[0].dot(dst[0])*dst[0];
        dst[0]=dst[0]/cv::norm(dst[0]);
        dst[1]=dst[1]/cv::norm(dst[1]);
        dst[2]=dst[0].cross(dst[1]);
        x=dst[0].dot(dst[1]);
        if ((int)x*1000!=0)
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    return true;
}

//重载GramSchmidt（）：对输入的三维向量进行斯密特标准正交化
bool CoreAlgorithm::GramSchmidt(const Mat src,Mat& dst)
{
    dst=cv::Mat::zeros(3,3,CV_64FC1);
    if (src.cols>3||src.cols<2||dst.cols!=3)
    {
        return false;
    }
    if (src.cols==3)
    {
        float x, y, z;
        dst.col(0)=src.col(0)*1;
        dst.col(1)=src.col(1)-src.col(1).dot(src.col(0))/dst.col(0).dot(dst.col(0))*dst.col(0);
        dst.col(2)=src.col(2)-src.col(2).dot(src.col(0))/dst.col(0).dot(dst.col(0))*dst.col(0)-
                src.col(2).dot(dst.col(1))/dst.col(1).dot(dst.col(1))*dst.col(1);
        dst.col(0)=dst.col(0)/cv::norm(dst.col(0));
        dst.col(1)=dst.col(1)/cv::norm(dst.col(1));
        dst.col(2)=dst.col(2)/cv::norm(dst.col(2));
        x=dst.col(0).dot(dst.col(1));
        y=dst.col(0).dot(dst.col(2));
        z=dst.col(1).dot(dst.col(2));
        if ((int)x*1000!=0 || (int)x*1000!=0 || (int)x*1000!=0)
        {
            return false;
        }
    }
    else if (src.cols==2)
    {
        float x;
        dst.col(0)=src.col(0)*1;//要乘以一个数才可以赋值
        dst.col(1)=src.col(1)-src.col(1).dot(src.col(0))/dst.col(0).dot(dst.col(0))*dst.col(0);
        dst.col(2)=dst.col(0).cross(dst.col(1))*1;//要乘以一个数才可以赋值
        dst.col(0)=dst.col(0)/cv::norm(dst.col(0));
        dst.col(1)=dst.col(1)/cv::norm(dst.col(1));
        dst.col(2)=dst.col(2)/cv::norm(dst.col(2));
        x=dst.col(0).dot(dst.col(1));
        if ((int)x*1000!=0)
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    return true;
}

//描述：由特征点的在相机坐标系三维坐标信息，建立标靶工作面坐标系，得到标靶工作面坐标系在相机坐标系下的位姿
bool  CoreAlgorithm::GetObjectFaceRT(const vector<Point3f> src_pnts, Mat& dst_RT)
{
    //输入判定
    if (src_pnts.size()!=7)
    {
        return false;
    }
    dst_RT=Mat::eye(4,4,CV_64FC1);
    //一号特征点到其他六个特征点的向量
    Point3f VecToPoint1[6];
    //正交化用矩阵
    Mat src_Mat=Mat::zeros(3,2,CV_64FC1);//正交前
    Mat dst_Mat;//正交后
    for (int i=0;i<6;i++)
    {
        VecToPoint1[i]=src_pnts[i+1]-src_pnts[0];
    }
    src_Mat.at<double>(0,0)=VecToPoint1[1].x;
    src_Mat.at<double>(1,0)=VecToPoint1[1].y;
    src_Mat.at<double>(2,0)=VecToPoint1[1].z;
    src_Mat.at<double>(0,1)=VecToPoint1[3].x;
    src_Mat.at<double>(1,1)=VecToPoint1[3].y;
    src_Mat.at<double>(2,1)=VecToPoint1[3].z;
    if ( !CoreAlgorithm::GramSchmidt(src_Mat,dst_Mat)==true)
    {
        return false;
    }
    //得到旋转量
    dst_Mat.copyTo(dst_RT(cv::Range(0,3),cv::Range(0,3)));
    //得到平移量
    dst_RT.at<double>(0,3)=src_pnts[0].x;
    dst_RT.at<double>(1,3)=src_pnts[0].y;
    dst_RT.at<double>(2,3)=src_pnts[0].z;
    return true;
}

//获取两个特征面的相对位姿
bool CoreAlgorithm::GetFaceToFaceRT(const vector<vector<Point2f> > &LeftImagePnt1s,const vector<vector<Point2f> > &RightImagePnt1s,
                                    const vector<vector<Point2f> > &LeftImagePnt2s,const vector<vector<Point2f> > &RightImagePnt2s,
                                    const CamPara& camParaLeft,const CamPara& camParaRight,const double rotVector[3] ,const double traVector[3] ,
									vector<Mat>&RTs,string err_str)
{

    //先判断左右图片的数据点是否一致
    err_str="";
    if (LeftImagePnt1s.size()!=RightImagePnt1s.size()||LeftImagePnt2s.size()!=RightImagePnt2s.size())
    {
        err_str="错误：左右图片的数据点不匹配！！";
        return false;
    }
    if (LeftImagePnt1s.size()==0||LeftImagePnt2s.size()==0)
    {
        err_str="错误：无数据点！";
        return false;
    }
    for (int i=0;i<LeftImagePnt1s.size();i++)
    {
        //获取标靶面的特征点的三维信息
        vector<Point3f>F1_Pnts,F2_Pnts;
        Mat RTToCam1,RTToCam2;
        Mat _RT;
        _RT=Mat::zeros(4,4,CV_64FC1);
        RTToCam1=Mat::zeros(4,4,CV_64FC1);
        RTToCam2=Mat::zeros(4,4,CV_64FC1);
        if (!CoreAlgorithm::Cal3dPoint(LeftImagePnt1s[i],camParaLeft,RightImagePnt1s[i],camParaRight,rotVector,traVector,F1_Pnts)
                || !CoreAlgorithm::Cal3dPoint(LeftImagePnt2s[i],camParaLeft,RightImagePnt2s[i],camParaRight,rotVector,traVector,F2_Pnts))
        {
            continue;
        }
        if (CoreAlgorithm::GetObjectFaceRT(F1_Pnts,RTToCam1) && CoreAlgorithm::GetObjectFaceRT(F2_Pnts,RTToCam2))
        {
            cv::invert(RTToCam1,RTToCam1);
            //_RT=RTToCam2*RTToCam1;
            _RT=RTToCam1*RTToCam2;
            // 			 求相对位姿时的问题所在：在进行位姿转化时，箭头一定是指向变换后的坐标系
            // 			（例如：如果箭头是从A坐标系指向B坐标，则箭头代表的RT为B坐标系在A坐标系下的位姿矩阵）这点一定不能搞混了，切记！
            RTs.push_back(_RT);
        }
    }
    if (RTs.size()!=0)
    {
        return true;
    }
    else
        return false;
    //对所有图片求出的rt进行求平均
}
bool CoreAlgorithm::GetFaceToFaceRT(const Mat &Limg,const Mat &Rimg,const int flag, const CamPara& Lcam,const CamPara& Rcam,
									const Mat &Camrt ,Mat &RT,string &err_str)
{
	err_str ="no error!";
	MarkArea  _LMarked,_RMarked;
	vector <int> Faceflag;
	vector <Point2f> Lpnt1;
	vector <Point2f> Lpnt2;
	vector <Point2f> Rpnt1;
	vector <Point2f> Rpnt2;
	RT = Mat::zeros(4,4,CV_64FC1);
	//相机相对位姿转化
	double rotVector[3];//右相机到左相机的旋转向量
	double traVector[3];//右相机到左相机的平移向量
	Mat CamR33,CamR31;
	Camrt(cv::Range(0, 3), cv::Range(0, 3)).copyTo(CamR33);
	cv::Rodrigues(CamR33,CamR31);
	for (int i=0;i<3;i++)
	{
		rotVector[i]=CamR31.at<double>(i,0);
		traVector[i]=Camrt.at<double>(i,3);
	}

	switch (flag)
	{
	case FIR_TO_TW:
		Faceflag.clear();
		Faceflag.push_back(1);
		Faceflag.push_back(2);
		break;
	case FIR_TO_TH:
		Faceflag.clear();
		Faceflag.push_back(2);
		Faceflag.push_back(3);
		break;
	case FIR_TO_FO:
		Faceflag.clear();
		Faceflag.push_back(1);
		Faceflag.push_back(4);
		break;
	case FIR_TO_FI:
		Faceflag.clear();
		Faceflag.push_back(1);
		Faceflag.push_back(5);
		break;
	default:
		err_str ="error flag!";
		return false;
		break;
	}

	if( !FindImgFeature(Limg,Rimg,_LMarked,_RMarked,err_str,Faceflag))
		return false;
	vector <Point3f> pntsToCam;
	Mat Face1RT = Mat::zeros(4,4,CV_64FC1);
	Mat Face2RT = Mat::zeros(4,4,CV_64FC1);
	Lpnt1 = _LMarked[0].large_circle;
	Lpnt2 = _LMarked[1].large_circle;
	Rpnt1 = _RMarked[0].large_circle;
	Rpnt2 = _RMarked[1].large_circle;
	//获取第一个面的姿态
	if(!Cal3dPoint(Lpnt1,Lcam,Rpnt1,Rcam,rotVector,traVector,pntsToCam))
	{
		err_str ="error in Cal3dPoint!";
		return false;
	}
	if(!GetObjectFaceRT(pntsToCam,Face1RT))
	{
		err_str ="error in GetObjectFaceRT!";
		return false;
	}
	//获取第二个面的姿态
	if(!Cal3dPoint(Lpnt2,Lcam,Rpnt2,Rcam,rotVector,traVector,pntsToCam))
	{
		err_str ="error in Cal3dPoint!";
		return false;
	}
	if(!GetObjectFaceRT(pntsToCam,Face2RT))
	{
		err_str ="error in GetObjectFaceRT!";
		return false;
	}
	//获取第二面相对第一个面的相对姿态
	cv::invert(Face1RT,Face1RT);
	RT=Face1RT*Face2RT;
	return true;
}

//描述：根据左右相机图片，获取标靶在测量坐标系下的位姿
bool CoreAlgorithm::GetObjectRT(const Mat &LeftImage,const Mat &RightImage, const CamPara &L_Camparam,
                                const CamPara &R_Camparam,const Mat &CamRT,const ObjectPara &objparam,Mat &RT2cam, std::string &err_inf,const int type)
{
    err_inf="OK_come from GetObjectRT()";
    //解析相机参数
    Mat L_distCoeffs,L_cameraMatrix;
    Mat R_distCoeffs,R_cameraMatrix;
    if (!ExtraCamMatrixAndDist(L_Camparam,L_cameraMatrix,L_distCoeffs) ||
            !ExtraCamMatrixAndDist(R_Camparam,R_cameraMatrix,R_distCoeffs))
    {
        err_inf ="Error from ExtraCamMatrixAndDist()";
        return false;
    }
    //单相机测量方法
    Mat _camrt = Mat::eye(4,4,CV_64FC1);
    invert(CamRT,_camrt);//用于单目测量中这里需要注意的是：相机联合标定的得到的RT是左相机在右相机坐标系下的位姿，然而测量系统的
    if (type==0)
    {
        if (LeftImage.empty() && RightImage.empty())
        {
            err_inf="Error from GetObjectRT(): Both of two camera image are empty!";
            return false;
        }
        vector<SH_Ellipse> Lcircle, Rcircle;;
        Mat L_imge_contour,R_imge_contour;
        MarkArea L_markarea,R_markarea;
        //进行大小圆提取
        bool L_flag=false,R_flag=false;
        L_flag=DiscernLSCircle(LeftImage,Lcircle,L_imge_contour);
        R_flag=DiscernLSCircle(RightImage,Rcircle,R_imge_contour);
        if (L_flag==false && R_flag==false)
        {
            err_inf="Error from GetObjectRT(): Both of two image can't discern right feature!";
            return false;
        }
        else
        {
            Mat _Limg,_Rimg;
            if (L_flag==true)
            {
                if (FindMarkArea(LeftImage,_Limg,Lcircle,L_markarea,err_inf))
                {
                    if (TransferFrame(L_markarea,L_distCoeffs,L_cameraMatrix,objparam,RT2cam,err_inf))
                        return true;
                }
            }
            if (R_flag == true)
            {
                if(FindMarkArea(RightImage,_Rimg,Rcircle,R_markarea,err_inf))
                {
                    if(!TransferFrame(R_markarea,R_distCoeffs,R_cameraMatrix,objparam,RT2cam,err_inf))
                    {
                        return false;
                    }
                }
                //转化到左相机坐标系下
                RT2cam = _camrt*RT2cam;
                return true;
            }
            return false;
        }
    }
    //双相机测量方法
    if (type==1)
    {
        if (LeftImage.empty() || RightImage.empty())
        {
            err_inf="Error from GetObjectRT(): One of two camera images is empty!";
            return false;
        }
        vector<SH_Ellipse> Lcircle, Rcircle;;
        Mat L_imge_contour,R_imge_contour;
        MarkArea L_markarea,R_markarea;
        //进行大小圆提取
        bool L_flag=false,R_flag=false;
        L_flag=DiscernLSCircle(LeftImage,Lcircle,L_imge_contour);
        R_flag=DiscernLSCircle(RightImage,Rcircle,R_imge_contour);
        if (L_flag==false || R_flag==false)
        {
            err_inf="Error from DiscernLSCircle()";
            return false;
        }
        else
        {
            Mat _Limg,_Rimg;
            bool flag_1,flag_2;
            flag_1=FindMarkArea(LeftImage,_Limg,Lcircle,L_markarea,err_inf);
            flag_2=FindMarkArea(RightImage,_Rimg,Rcircle,R_markarea,err_inf);
            if (flag_1==false || flag_2==false)
            {
                err_inf="Error from FindMarkArea()";
                return false;
            }
            //找出左右相机都识别到的特征面
            double R[3],T[3];
            vector<Point3f> Pnt3d;
            Mat _R;
            CamRT(cv::Range(0,3),cv::Range(0,3)).copyTo(_R);
            Rodrigues(_R,_R);
            for (int n=0;n<3;n++)
            {
                T[n]=CamRT.at<double>(n,3);
                R[n]=_R.at<double>(n,0);
            }

            for (int i=0;i<L_markarea.size();i++)
            {
                for (int j=0;j<R_markarea.size();j++)
                {
                    if (L_markarea[i].Face==R_markarea[j].Face)
                    {
                        Pnt3d.clear();
                        if (Cal3dPoint(L_markarea[i].large_circle,L_Camparam,R_markarea[j].large_circle,R_Camparam,R,T,Pnt3d)==true)
                        {
                            if (GetObjectFaceRT(Pnt3d,RT2cam)==false)
                            {
                                err_inf="Error from GetObjectFaceRT()";
                                return false;
                            }
                            //进行面转化
                            Mat _tempRT;
                            switch (L_markarea[i].Face)
                            {
                            case 2:
                                objparam.RT2one[0].copyTo(_tempRT);
                                invert(_tempRT,_tempRT);
                                RT2cam=RT2cam*_tempRT;
                                break;
                            case 3:
                                objparam.RT2one[1].copyTo(_tempRT);
                                invert(_tempRT,_tempRT);
                                RT2cam=RT2cam*_tempRT;
                                break;
                            case 4:
                                objparam.RT2one[2].copyTo(_tempRT);
                                invert(_tempRT,_tempRT);
                                RT2cam=RT2cam*_tempRT;
                                break;
                            case 5:
                                objparam.RT2one[3].copyTo(_tempRT);
                                invert(_tempRT,_tempRT);
                                RT2cam=RT2cam*_tempRT;
                                break;
                            default:
                                break;
                            }
                        }
                        else
                        {
                            err_inf="Error from Cal3dPoint()";
                            return false;
                        }
                        return true;
                    }
                }
                if (i==L_markarea.size()-1)
                {
                    err_inf="Error from GetObjectRT(): Can't find the same face in the two images!";
                    return false;
                }
            }
        }
    }
}

///用标靶的特征图形对评价双目系统的测量误差
bool CoreAlgorithm::StereMeasureAccuracy(const Mat &Leftimage,const Mat &Rightimage,  //左右相机图片
										const CamPara& camParaLeft,const CamPara& camParaRight, //左右相机参数
                                         const double rotVector[3] ,const double traVector[3],  //双目系统外参数
										vector <Point3f> &pnts3d,vector <Point3f> &pntsToCam,vector <float> &_mean,
										float &meanerror,float &dis,string &err_str,
										const float &Xdis,const float &Ydis)

{
    if (Leftimage.empty() || Rightimage.empty())
    {
        err_str ="error in StereMeasureAccuracy(): the input image is empty!";
        return false;
    }
    vector<SH_Ellipse> Lcircle, Rcircle;
    MarkArea L_markedarea,R_markedarea;
    Mat _tempimage;
    if (!CoreAlgorithm::DiscernLSCircle(Leftimage,Lcircle,_tempimage) ||
            !CoreAlgorithm::DiscernLSCircle(Rightimage,Rcircle,_tempimage))
    {
        err_str ="error in DiscernLSCircle(): feature extract fail!";
        return false;
    }
    else if(!CoreAlgorithm::FindMarkArea(Leftimage,_tempimage,Lcircle,L_markedarea,err_str)||
            !CoreAlgorithm::FindMarkArea(Rightimage,_tempimage,Rcircle,R_markedarea,err_str))
    {
        return false;
    }
    vector<Point2f> L_FeaturePoints,R_FeaturePoints;
    if (L_markedarea.size()==1 && R_markedarea.size()==1)
    {
        L_FeaturePoints = L_markedarea[0].large_circle;
        R_FeaturePoints = R_markedarea[0].large_circle;
    }

    //在相机坐标系下的特征点的位置信息
    //vector <Point3f> pntsToCam;
    Point3f PVec[6];
    Mat MatPVec[6];
    for (int i=0;i<6;i++)
    {
        MatPVec[i]=Mat::zeros(3,1,CV_64FC1);
    }
    //世界坐标系到相机坐标系的旋转变化矩阵
    cv::Mat _tempR=cv::Mat::zeros(3,2,CV_64FC1);
    cv::Mat _tempR1=cv::Mat::zeros(3,3,CV_64FC1);
    cv::Mat _tempR2=cv::Mat::zeros(3,3,CV_64FC1);
    for (int i=0;i<6;i++)
    {
        PVec[i]=Point3f(0,0,0);
    }
    bool flag=CoreAlgorithm::Cal3dPoint(L_FeaturePoints,camParaLeft,R_FeaturePoints,camParaRight,rotVector,traVector,pntsToCam);
    if (flag==true && pntsToCam.size()==7)
    {
        dis=sqrt(pntsToCam[0].dot(pntsToCam[0]));
        //让特征面做平移，使一号点的坐标移到相机坐标系的原点，求出其他六个点的坐标
        for (int j=0;j<6;j++)
        {
            PVec[j]=pntsToCam[j+1]-pntsToCam[0];

            MatPVec[j].at<double>(0,0)=PVec[j].x;
            MatPVec[j].at<double>(1,0)=PVec[j].y;
            MatPVec[j].at<double>(2,0)=PVec[j].z;
        }
        //确定相机坐标系与世界坐标系的转化矩阵
        _tempR.at<double>(0,0)=PVec[1].x;
        _tempR.at<double>(1,0)=PVec[1].y;
        _tempR.at<double>(2,0)=PVec[1].z;
        _tempR.at<double>(0,1)=PVec[3].x;
        _tempR.at<double>(1,1)=PVec[3].y;
        _tempR.at<double>(2,1)=PVec[3].z;
        CoreAlgorithm::GramSchmidt(_tempR,_tempR1);
        cv::invert(_tempR1,_tempR2);
        //将特征点坐标从相机坐标系转到标靶坐标系（即世界坐标系下）
        for (int j=0;j<7;j++)
        {
            Mat _temp=Mat::zeros(3,1,CV_64FC1);
            Point3f _temppnt=Point3f(0,0,0);
            if (j==0)
                continue;
            _temp=_tempR2*MatPVec[j-1];
            _temppnt.x=_temp.at<double>(0,0);
            _temppnt.y=_temp.at<double>(1,0);
            _temppnt.z=_temp.at<double>(2,0);
            pnts3d.push_back(_temppnt);
        }
        //求出每个特征圆到一号大圆的距离
        /*vector <float> _mean;*/
        for (int i = 0;i<6;i++)
        {
            float _dis=sqrt(pnts3d[i].dot(pnts3d[i]));
            switch (i)
            {
            case 0:
                _mean.push_back(_dis-Xdis);
                break;
            case 1:
                _mean.push_back(_dis-2*Xdis);
                break;
            case 2:
                _mean.push_back(_dis-Ydis);
                break;
            case 3:
                _mean.push_back(_dis-2*Ydis);
                break;
            case 4:
                _mean.push_back(_dis-sqrt(Xdis*Xdis+Ydis*Ydis));
                break;
            case 5:
                _mean.push_back(_dis-2*sqrt(Xdis*Xdis+Ydis*Ydis));
                break;
            default:
                break;
            }
        }
        meanerror=0;
        for (int i = 0;i<6;i++)
        {
            //meanerror+=abs(_mean[i]);
            meanerror+=_mean[i];
        }
        meanerror=meanerror/6;
        err_str="no error!";
        return true;
    }
    else
    {
        err_str="error in Cal3dPoint()";
        return false;
    }
}

//描述：标靶的特征图形
bool CoreAlgorithm::ShowFeaCircle(const int &imgheight, const int &imgwidth, const int &FeaSize, Mat &outimage)
{
    cv::Point2d offset;
    offset.x = imgheight/4-4*FeaSize;
    offset.y = imgwidth/4+4*FeaSize;
    Mat img =Mat(imgheight,imgwidth,CV_8UC1,Scalar(255));
    Mat ringimgsingle1,ringimgsingle2;
    //产生大圆和小圆
    bool isok1 = creatCircleImg(ringimgsingle1,FeaSize,4);
    bool isok2 = creatCircleImg(ringimgsingle2,FeaSize,2);
    //生成第一行第一个小圆
    int index=0;
    for(int k =0;k<ringimgsingle1.rows;k++)
    {
        for(int h = 0;h<ringimgsingle1.cols;h++)
        {
            img.at<uchar>(offset.x+k,offset.y-h) = ringimgsingle1.at<uchar>(k,h);
        }
    }
    //生成第一行三个大圆
    for(int i=offset.y-FeaSize*1.25;i<img.cols;i-=FeaSize*4)
    {
        if(i>img.cols-FeaSize||index>2)
            break;
        for(int k =0;k<ringimgsingle2.rows;k++)
        {
            for(int h = 0;h<ringimgsingle2.cols;h++)
            {
                img.at<uchar>(offset.x+k,i-h) = ringimgsingle2.at<uchar>(k,h);
            }
        }
        index++;
    }
    //生成第二行的小圆
    for(int k =0;k<ringimgsingle1.rows;k++)
    {
        for(int h = 0;h<ringimgsingle1.cols;h++)
        {
            img.at<uchar>(offset.x+FeaSize*1.25+k,offset.y-FeaSize*1.25-h) = ringimgsingle1.at<uchar>(k,h);
        }
    }
    //生成第三行的两个大圆
    index =0;
    for(int i=offset.y-FeaSize*1.25;i<img.cols;i-=FeaSize*4)
    {
        if(i>img.cols-FeaSize||index>1)
            break;
        for(int k =0;k<ringimgsingle2.rows;k++)
        {
            for(int h = 0;h<ringimgsingle2.cols;h++)
            {
                img.at<uchar>(offset.x+4*FeaSize+k,i-h) = ringimgsingle2.at<uchar>(k,h);
            }
        }
        index++;
    }
    //生成第五行的两个大圆
    index =0;
    for(int i=offset.y-FeaSize*1.25;i<img.cols;i-=FeaSize*8)
    {
        if(i>img.cols-FeaSize||index>1)
            break;
        for(int k =0;k<ringimgsingle2.rows;k++)
        {
            for(int h = 0;h<ringimgsingle2.cols;h++)
            {
                img.at<uchar>(offset.x+8*FeaSize+k,i-h) = ringimgsingle2.at<uchar>(k,h);
            }
        }
        index++;
    }
    //生成的第五行的小圆
    for(int k =0;k<ringimgsingle1.rows;k++)
    {
        for(int h = 0;h<ringimgsingle1.cols;h++)
        {
            img.at<uchar>(offset.x+8*FeaSize+k,offset.y-10.5*FeaSize-h) = ringimgsingle1.at<uchar>(k,h);
        }
    }
    img.copyTo(outimage);
    return true;
}

//描述：在图像上画实心圆
bool CoreAlgorithm::DrawCircle( Mat & inoutimage, const  Point2i & Center, const int & radius)
{
	//check center whether out of image range
	if (inoutimage.empty())
	{
		return false;
	}
	Point2i allowarea[2] = {(0,0), (0,0)};//the allow area for center
	int offset = 4;
	allowarea[0].x = radius+offset;
	allowarea[0].y = radius+offset;
	allowarea[1].x = inoutimage.cols - radius - offset;
	allowarea[1].y = inoutimage.rows - radius - offset;
	if (Center.x<allowarea[0].x || Center.x>allowarea[1].x ||Center.y<allowarea[0].y || Center.y>allowarea[1].y )
	{
		return false;
	}
	for (int i = Center.x - radius-offset; i < Center.x + radius+offset; i++)
	{
		for (int j = Center.y - radius - offset; j<Center.y + radius+offset; j++)
		{
			float dist = sqrt( (i-Center.x)*(i-Center.x) + (j-Center.y)*(j-Center.y) );
			if ( dist <= radius )
			{
				inoutimage.at<uchar>(j,i) = 0;
			}
		}
	}
	return true;
}

//函数描述：机器人关节转角和关节脉冲比例关系标定函数
bool CoreAlgorithm::CalibrateAngerPulse(const vector <Mat>RT44,const vector<Vec6f> &pose,const int aixsflag,float &ratio)
{
    if (RT44.size() < 2 && pose.size()<2)
        return false;
    if (RT44.size()!=pose.size())
        return false;
    vector<Mat> Vi_0;
    //vector <double> Puli_0;
    vector<float> rates;
    Mat RT_0 = Mat::zeros(4,4,CV_32FC1);
    invert(RT44[0],RT_0);
    for (int i =1;i<RT44.size();i++)
    {
        Mat _temp;
        _temp = RT_0(cv::Range(0,3),cv::Range(0,3))*RT44[i](cv::Range(0,3),cv::Range(0,3));
        cv::Rodrigues(_temp,_temp);
        float thetai_0 = cv::norm(_temp,cv::NORM_L2);
        //Vi_0.push_back(_temp);
        //Puli_0.push_back(pose[i][aixsflag-1]-pose[0][aixsflag-1]);
        float Puli_0 = pose[i][aixsflag-1]-pose[0][aixsflag-1];
        float _rate = abs(Puli_0/(thetai_0/3.141593*180));
        rates.push_back(_rate);
    }
    float sum = std::accumulate(std::begin(rates), std::end(rates), 0.0); //求和
    ratio =  sum/rates.size(); //均值
    return true;
}

//遍历路径中的图片文件
//bool CoreAlgorithm::ReadFileImg(CString filepath, CString fileFormat, vector<CString> &filename)
//{
//	CFileFind ff;
//	BOOL bfind=ff.FindFile(filepath+fileFormat);
//	while (bfind)
//	{
//		bfind=ff.FindNextFileA();
//		if (ff.IsDots()||ff.IsSystem()||ff.IsHidden()||ff.IsDirectory())
//		{
//			continue;
//		}
//		else
//		{
//			CString szFilePath = ff.GetFilePath();
//			/*		 CString szFileName = ff.GetFileName();*/
//			filename.push_back(szFilePath/*+szFileName*/);
//		}
//	}
//	return true;
//}

//由已经图片的绝对路径得到该文件中下一张图片的绝对路径
//bool CoreAlgorithm::GetNextImagePath(CString &InOutFilePath)
//{
//	CFileFind ff;
//	//左窗口图片++
//	int len = InOutFilePath.GetLength();
//	CString imagenum="";
//	//CString imageprex="";
//	//CString imagepath="";//只有路径没有文件名（结尾不含\\）
//	CString temstr = "";
//	int numpos = 0;
//	//获取图片编号
//	for (int i=0;i<len;i++)
//	{
//		if (InOutFilePath[i]=='.' && i==len-4)//注意：或许CString类型的元素,'.'表示字符，而"."表示字符串
//		{
//			string num = "0123456789";
//			for (int j = i-1; ; j--)
//			{
//				bool flag = false;
//				for (int m = 0; m<num.size();m++)
//				{
//					if (InOutFilePath[j] == num[m])
//					{
//						imagenum+=InOutFilePath[j];
//						break;
//					}
//					if (m == num.size()-1)
//					{
//						flag = true;
//						numpos = j;
//					}
//				}
//				if (flag == true)
//				{
//					break;
//				}
//			}
//			imagenum.MakeReverse();
//			//imagenum+=InOutFilePath[i-4];
//			//imagenum+=InOutFilePath[i-3];
//			//imagenum+=InOutFilePath[i-2];
//			//imagenum+=InOutFilePath[i-1];
//			break;
//		}
//		if (i==len-1)
//		{
//			return false;
//		}
//	}
//	//获取图片前缀和图片路径
//	for (int i = 0; i < numpos+1; i++)
//	{
//		temstr += InOutFilePath[i]; 
//	}
//	//for (int i=len-1;i>=0;i--)
//	//{
//	//	if (InOutFilePath[i]=='_'&& i==len-9)//注意：或许CString类型的元素,'.'表示字符，而"."表示字符串
//	//	{
//	//		for (int j=len-1;j>=0;j--)
//	//		{
//	//			if (InOutFilePath[j]=='\\')
//	//			{
//	//				for (int n=j-1;n>=0;n--)
//	//				{
//	//					imagepath+=InOutFilePath[n];
//	//				}
//	//				imagepath.MakeReverse();//字符串顺序颠倒
//	//				for (int k=j;k<i+1;k++)//包含了‘\\’,也含'_'
//	//				{
//	//					imageprex+=InOutFilePath[k];
//	//				}
//	//				break;
//	//			}
//	//		}
//	//		break;
//	//		if (i==len-1)
//	//		{
//	//			return false;
//	//		}
//	//	}
//	//}
//	//获取图片路径
//	//InOutFilePath.Format(imagepath+imageprex+"%04d.jpg",atoi(imagenum)+1);
//	InOutFilePath.Format(temstr+"%04d.bmp",atoi(imagenum)+1);
//	return true;
//}
//由已经图片的绝对路径得到该文件中上一张图片的绝对路径
//bool CoreAlgorithm::GetFormerImagePath(CString &InOutFilePath)
//{
//	CFileFind ff;
//	int len = InOutFilePath.GetLength();
//	CString imagenum="";
//	CString temstr = "";
//	int numpos = 0;
//	//获取图片编号
//	for (int i=0;i<len;i++)
//	{
//		if (InOutFilePath[i]=='.' && i==len-4)//注意：或许CString类型的元素,'.'表示字符，而"."表示字符串
//		{
//			string num = "-0123456789";
//			for (int j = i-1; ; j--)
//			{
//				bool flag = false;
//				for (int m = 0; m<num.size();m++)
//				{
//					if (InOutFilePath[j] == num[m])
//					{
//						imagenum+=InOutFilePath[j];
//						break;
//					}
//					if (m == num.size()-1)
//					{
//						flag = true;
//						numpos = j;
//					}
//				}
//				if (flag == true)
//				{
//					break;
//				}
//			}
//			imagenum.MakeReverse();
//			break;
//		}
//		if (i==len-1)
//		{
//			return false;
//		}
//	}
//	//获取图片前缀和图片路径
//	for (int i = 0; i < numpos+1; i++)
//	{
//		temstr += InOutFilePath[i]; 
//	}
//	InOutFilePath.Format(temstr+"%04d.bmp",atoi(imagenum)-1);
//	return true;
//}

//******空间圆拟合（无剔除偏离较大的点）**********//
StereoEllipse CoreAlgorithm::fitStereoEllipse(vector<Point3f> points)
{

	float X0=0,Y0=0,Z0=0;
	float SumR=0,Radius=0;
	int count = 0;
	float quan=0;
	Point3f temp;
	int n = points.size();

	CvMat* X  = cvCreateMat(3,1,CV_32FC1);
	CvMat* Q  = cvCreateMat(3,3,CV_32FC1);
	CvMat* NT = cvCreateMat(3,n,CV_32FC1);
	CvMat* T  = cvCreateMat(3,3,CV_32FC1);
	CvMat* S1 = cvCreateMat(3,n,CV_32FC1);
	CvMat* P  = cvCreateMat(n,n,CV_32FC1);
	CvMat* WT = cvCreateMat(3,n,CV_32FC1);
	CvMat* A  = cvCreateMat(3,n,CV_32FC1);
	CvMat* B  = cvCreateMat(3,3,CV_32FC1);
	CvMat* C  = cvCreateMat(3,3,CV_32FC1);
	CvMat* D  = cvCreateMat(3,n,CV_32FC1);
	CvMat* E  = cvCreateMat(3,n,CV_32FC1);
	CvMat* N  = cvCreateMat(n,3,CV_32FC1);
	CvMat* CirclePoint  = cvCreateMat(3,1,CV_32FC1);
	////512///
	int N_step = N->step/sizeof(float);
	float* N_data = N->data.fl;
	Point3f tmp;
	for (unsigned int i = 0; i < n; i++)
	{
		tmp = points[i];
		N_data[i * N_step]     = tmp.x;
		N_data[i * N_step + 1] = tmp.y;
		N_data[i * N_step + 2] = tmp.z;
	}

	CvMat *L2 = cvCreateMat(n,1,CV_32FC1);
	int L2_step = L2->step/sizeof(float);
	float* L2_data = L2->data.fl;
	for (unsigned int i = 0; i < n; i++)
	{
		L2_data[i * L2_step] = 1;
	}


	cvTranspose(N,NT);//求矩阵N的转置矩阵
	cvMulTransposed(N,Q,1,NULL);//计算矩阵与其转置矩阵的乘积
	if(cvDet(Q)!=0)
	{
		cvInvert(Q,T,CV_LU);
	}
	cvMatMul(T,NT,S1);
	cvMatMul(S1,L2, X);

	CvMat *W = cvCreateMat(n,3,CV_32FC1);
	int W_step = W->step/sizeof(float);
	float* W_data = W->data.fl;
	W_data[0 * W_step]   = cvGetReal2D(X,0,0);
	W_data[0 * W_step+1] = cvGetReal2D(X,1,0);
	W_data[0 * W_step+2] = cvGetReal2D(X,2,0);

	//测拟合的平面法向量

	Point3f nor;
	nor.x=cvGetReal2D(X,0,0);
	nor.y=cvGetReal2D(X,1,0);
	nor.z=cvGetReal2D(X,2,0);
	//
	int j=1;
	for (unsigned int i = 0; i < n-1; i++)
	{
		W_data[j * W_step ] = points[i+1].x-points[i].x;
		W_data[j * W_step + 1 ] = points[i+1].y-points[i].y;
		W_data[j * W_step + 2 ] = points[i+1].z-points[i].z;
		j++;
	}

	cvTranspose(W,WT);
	quan=n*(n-1)*(n-2)/6;
	cvSetIdentity(P);
	cvConvertScale(P,P,quan,0);
	//
	//
	cvMatMul(WT,P,A);//A=WT*P
	cvMatMul(A,W,B);//B=A*W=WT*P*W
	if(cvDet(B)!=0)
	{
		cvInvert(B,C,CV_LU);//C=B-1,(WT*P*W)-1
	}
	cvMatMul(C,WT,D);//D=C*WT
	cvMatMul(D,P, E);//E=D*P=(WT*P*W)-1*WT*P

	CvMat *L1 = cvCreateMat(n,1,CV_32FC1);
	int L1_step = L1->step/sizeof(float);
	float* L1_data = L1->data.fl;
	L1_data[0*L1_step] = 1;
	j=1;
	for (unsigned int i = 0; i < n-1; i++)
	{
		L1_data[j * L1_step ] = (points[i+1].x*points[i+1].x + points[i+1].y*points[i+1].y + points[i+1].z*points[i+1].z)/2
			-(points[i].x*points[i].x + points[i].y*points[i].y + points[i].z*points[i].z)/2;
		j++;
	}

	cvMatMul(E,L1,CirclePoint);//E=D*P=(WT*P*W)-1*WT*P*L

	X0=cvGetReal2D(CirclePoint,0,0);
	Y0=cvGetReal2D(CirclePoint,1,0);
	Z0=cvGetReal2D(CirclePoint,2,0);
	for(int i=0;i<n;i++)
	{
		float r=sqrt((X0-points[i].x)*(X0-points[i].x)+(Y0-points[i].y)*(Y0-points[i].y)+(Z0-points[i].z)*(Z0-points[i].z));
		SumR=SumR+r;
	}
	Radius=SumR/n;

	StereoEllipse mSEllipse_temp;
	mSEllipse_temp.NormalVector.x = cvGetReal2D(X,0,0);
	mSEllipse_temp.NormalVector.y = cvGetReal2D(X,1,0);
	mSEllipse_temp.NormalVector.z = cvGetReal2D(X,2,0);
	mSEllipse_temp.center.x = X0;
	mSEllipse_temp.center.y = Y0;
	mSEllipse_temp.center.z = Z0;
	mSEllipse_temp.r = Radius;

	cvReleaseMat(&X);
	cvReleaseMat(&Q); 
	cvReleaseMat(&NT);
	cvReleaseMat(&T);
	cvReleaseMat(&S1);
	cvReleaseMat(&P);
	cvReleaseMat(&WT);
	cvReleaseMat(&A);
	cvReleaseMat(&B);
	cvReleaseMat(&C);
	cvReleaseMat(&D);
	cvReleaseMat(&E);

	return(mSEllipse_temp);
}

//描述：
vector<Point3f> CoreAlgorithm::RansacMachine(vector<Point3f> SourcePoint)
{
	vector<CvPoint2D32f> kkk;
	int index1,index2,index3;
	int iterations=0;//迭代标记
	int flag=0;//用来判断符合模型点的个数
	double Len=0;//点到圆心的距离与半径的差值
	double thre_gap=0;//随机数的间隔
	double thre_dif=0;//与半径误差值的判断
	double thre_no =0;//通过点的个数
	double thre_flat=0;//点到平面的距离阈值
	float a=0,b=0,c=0;//拟合出的平面法向量
	double Dsum=0;
	vector<double> Dist;//用来存储各个点到圆心的距离
	vector<double> Dist_flat;//用来存储各个点到平面的距离
	vector<double> Distorder_flat;
	StereoEllipse result1;	//随机三个点圆拟合的结果
	vector<Point3f> TestData;//存放随机找出来的三个点
	vector<Point3f> ReturnPoint;//存放符合模型的最佳点集合
	vector<double> Distan;
	int countnum = SourcePoint.size()-1;//点集的尺寸
	int trytime=0;//累积迭代的次数

	//StereoEllipse result11;	
	//result11=fitStereoEllipse(SourcePoint);
	while(!iterations)
	{

		trytime++;
		TestData.clear();
		Distan.clear();
		Dist.clear();
		Dist_flat.clear();
		ReturnPoint.clear();
		//找出有一定间隔的随机数
		index1=rand()%countnum;
		index2=rand()%countnum;
		index3=rand()%countnum;
		thre_gap=0.2*countnum;//随机数的间隔
		thre_dif=0;//与半径误差值的判断
		/*thre_no=0.8*(countnum+1);*/
		thre_no=0.7*(countnum);//通过点的个数
		while(abs(index1-index2)<thre_gap||abs(index1-index3)<thre_gap||abs(index2-index3)<thre_gap)
		{
			index2=rand()%countnum;
			index3=rand()%countnum;

		}

		TestData.push_back(SourcePoint[index1]);
		TestData.push_back(SourcePoint[index2]);
		TestData.push_back(SourcePoint[index3]);
		result1=fitStereoEllipse(TestData);
		Point3f pointcenter;
		Point3f normalpoint;
		pointcenter=result1.center;
		normalpoint=result1.NormalVector;
		a=normalpoint.x;
		b=normalpoint.y;
		c=normalpoint.z;

		thre_dif=0.08*result1.r;
		for(int i=0;i<=countnum;i++)//样本集中点到圆心的距离
		{
			Dist.push_back(cv::sqrt((SourcePoint[i].x-pointcenter.x)*(SourcePoint[i].x-pointcenter.x)+
				(SourcePoint[i].y-pointcenter.y)*(SourcePoint[i].y-pointcenter.y)+
				(SourcePoint[i].z-pointcenter.z)*(SourcePoint[i].z-pointcenter.z)));
		}
		for(int i=0;i<=countnum;i++)//样本集中点到平面的距离
		{
			Dist_flat.push_back(abs(a*SourcePoint[i].x+b*SourcePoint[i].y+c*SourcePoint[i].z-1)/sqrt(a*a+b*b+c*c));
		}

		int lab=0;
		double temp;
		for(int i=1;i<=countnum;i++)
		{
			lab=0;
			for(int j=0;j<=countnum-i;j++)
				if(Dist_flat[j]>Dist_flat[j+1])
				{
					temp=Dist_flat[j];
					Dist_flat[j]=Dist_flat[j+1];
					Dist_flat[j+1]=temp;
					lab=1;
				}
				if(!lab)break;
		}
		//thre_flat=2*Dist_flat[countnum/2];
		for(int i=0;i<=countnum;i++)
		{
			Dsum+=Dist_flat[i];
		}
		thre_flat=Dsum/(5*countnum);
		for(int i=0;i<=countnum;i++)
		{
			Distan.push_back(abs(Dist[i]-result1.r));
		}


		for(int i=0;i<=countnum;i++)
		{
			Len=abs(Dist[i]-result1.r);
			if(Len<thre_dif&&Dist_flat[i]<thre_flat)//thre_flat
			{
				ReturnPoint.push_back(SourcePoint[i]);
				flag++;			
			}
			else
			{
				int jj=0;//just fit
			}
		}

		if(flag>thre_no)
		{
			iterations=1;	
		}
		else
		{
			iterations=0;
			flag=0;
		}
	}
	//for test
	StereoEllipse result2;
	result2=fitStereoEllipse(ReturnPoint);
	int kk=0;

	return ReturnPoint;
}
//
bool CoreAlgorithm::creatCircleImg(Mat& img,int dx,float ratio)
{
    ///// 如果dx 小于等于0，则返回错误
    if(dx<=0)
    {
        return false;
    }

    //// 生成的圆环图像尺寸为dx，外环半径为 dx/3， 内环半径为dx/6
    //// 两个圆之间的部分为黑0.其他部分亮255
    Mat _img = Mat::zeros(dx, dx, CV_8U);  ////Mat _img = Mat::zeros(dx, dx, CV_32F);
    //// 图像大小
    int height,width;
    height = _img.rows;
    width  = _img.cols;
    double radius,dist,cen_x,cen_y;
    radius = dx/ratio;
    ////// ，由于C++中矩阵的起始坐标为0，而不是1，所以坐标size减去1，则等效于（size-1）/2
    cen_x = (dx-1)/2;
    cen_y = (dx-1)/2;


    for(int i=0;i<height; i++)
    {
        for(int j=0;j<width;j++)
        {
            dist  = double(i-cen_y)*double(i-cen_y) + double(j-cen_x)*double(j-cen_x);
            dist = sqrt(dist);
            if(dist>radius) //// 设置成为白色 255
            {
                _img.at<uchar>(i,j) = 255;
            }
            //// 否则，就是为黑色，不用做任何操作
        }
    }
    img = _img.clone();
    return true;
}


///模板函数
///进行三维重建前确认点的tag匹配性，并进行数据类型转换
///进行刚体变换前确认点的tag匹配性，并进行数据类型转换
void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
								   vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2, vector<int>& tags)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point3f(pnts1[i][1], pnts1[i][2], pnts1[i][3]));
				out_pnts2.push_back(Point3f(pnts2[j][1], pnts2[j][2], pnts2[j][3]));
				tags.push_back(int(pnts1[i][0]));
			}
		}
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
								   vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point3f(pnts1[i][1], pnts1[i][2], pnts1[i][3]));
				out_pnts2.push_back(Point3f(pnts2[j][1], pnts2[j][2], pnts2[j][3]));
			}
		}
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts.size(); i++)
	{
		out_pnts.push_back(Point3f(pnts[i][1], pnts[i][2], pnts[i][3]));
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts,vector<int>& tags)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts.size(); i++)
	{
		out_pnts.push_back(Point3f(pnts[i][1], pnts[i][2], pnts[i][3]));
		tags.push_back(int(pnts[i][0]));
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
								   vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2, vector<int>& tags)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
				out_pnts2.push_back(Point2f(pnts2[j][1], pnts2[j][2]));
				tags.push_back(int(pnts1[i][0]));
			}
		}
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
								   vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2)
{
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		for (unsigned int j = 0; j < pnts2.size(); j++)
		{
			if (pnts1[i][0] == pnts2[j][0])
			{
				out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
				out_pnts2.push_back(Point2f(pnts2[j][1], pnts2[j][2]));
			}
		}
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
	}
}

void CoreAlgorithm::pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1,vector<int>& tags)
{
	//本部分仅仅为了去除标签信息
	for (unsigned int i = 0; i < pnts1.size(); i++)
	{
		out_pnts1.push_back(Point2f(pnts1[i][1], pnts1[i][2]));
		tags.push_back(int(pnts1[i][0]));
	}
}

void CoreAlgorithm::addPntsTag(const vector<vector<Point3f>> Pnts3fAll, const vector<int>& tags,vector<vector<TagPoint3f>> &TagPnts3fAll)
{
	TagPoint3f TagPoint;
	TagPnts3fAll.resize(Pnts3fAll.size());
	//vector<Point3f> pointtemp;
	for (unsigned int i = 0; i < Pnts3fAll.size(); i++)
	{
		for(unsigned int k = 0;k < Pnts3fAll[i].size();k++)
		{
			TagPoint[0] = float(tags[k]);
			TagPoint[1] = Pnts3fAll[i][k].x;
			TagPoint[2] = Pnts3fAll[i][k].y;
			TagPoint[3] = Pnts3fAll[i][k].z;
			TagPnts3fAll[i].push_back(TagPoint);
		}
	}
}

void CoreAlgorithm::addPntsTag(const vector<Point3f> Pnts3fAll, const vector<int>& tags,vector<TagPoint3f> &TagPnts3fAll)
{
	for (unsigned int i = 0; i < Pnts3fAll.size(); i++)
	{
		TagPoint3f TagPoint;
		TagPoint[0] = float(tags[i]);
		TagPoint[1] = Pnts3fAll[i].x;
		TagPoint[2] = Pnts3fAll[i].y;
		TagPoint[3] = Pnts3fAll[i].z;
		TagPnts3fAll.push_back(TagPoint);
	}
}

//////////////////////////////         周涛                   ///////////////////////////////////////////////////////////////////////    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//提取激光中心
bool CoreAlgorithm::LaserCenterDetector(const Mat& srcimage, const cv::Rect mask, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter)
{
	//将图像转化成灰度图
	Mat src;
	if (srcimage.channels() == 1)
	{
		src = srcimage;
	}
	else
	{
		cvtColor(srcimage, src, CV_BGR2GRAY);
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
		//根据左边界和右边界之和的奇偶，判断中间点所在位置
		//if ((maxLoc.x + k11 - 1 + maxLoc.x + k22 - 1) % 2 == 0)
		//{
		//	data.x = i;
		//	////data.y = (maxLoc.x + k11 - 1 + maxLoc.x + k22 - 1) / 2;
		//	data.y = (maxLoc.x + k11 + maxLoc.x + k22) / 2;  /// zhangxu modified
		//}
		//else
		//{
		//	data.x = i;
		//	data.y = (maxLoc.x + k11 - 1 + maxLoc.x + k22) / 2;
		//}
		data.x = i;
		data.y = (maxLoc.x + k11 + maxLoc.x - k22) / 2;
		/////  zhangxu  added  进行越界检查
		//if (data.y<windowwidth / 2 || data.y> src1.cols - windowwidth / 2)
		//{
		//	continue;
		//}

		////ZLL 越界检查本应该检查的是x，但是这里周涛把data的x，y写反了。应该是x行，y列
		if (data.y<windowwidth / 2 || data.y> src1.cols-1 - windowwidth / 2)
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
				sum2 += PixelValue;
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
	//Step 6:  换算重心坐标到图像坐标系

	//        //显示中心坐标
	//        for (int i=0;i<LaserCenter.size();i++)
	//             {
	//                src.row((int)LaserCenter[i].x).col((int)LaserCenter[i].y)=1;
	//             }
	//        namedWindow( "window" );
	//        imshow("window",src);
	//        waitKey( 0 );

	return true;
}

//////////////////////////////////////////////////////////////////
///////////////////////////////庄磊磊/////////////////////////////
bool CoreAlgorithm::LaserCenterDetector(const Mat& srcimage, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter)
{
	//将图像转化成灰度图
	Mat src;
	if (srcimage.channels() == 1)
	{
		src = srcimage;
	}
	else
	{
		cvtColor(srcimage, src, CV_BGR2GRAY);
	}
	//进行高斯滤波，去除噪声
	GaussianBlur(src, src, Size(5, 5), 0, 0);     //高斯滤波
	//Step 1：求该行图像中亮度最大值所在位置Maxc
	Point2f data;
	vector<Point2f> Maxc;
	Point2f center;
	for (int i = 0; i < src.rows; i++)
	{
		Mat src1 = src.row(i);
		double maxVal = 0;
		Point maxLoc;
		minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, Mat()); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置

		int GrayValue = src.at<uchar>(i, maxLoc.x);

		//根据行最大值以及条纹左边界和右边界确定条纹中心
		int borderL = 0, borderR = maxLoc.x+1;
		bool findL=0, findR = 0;
		for (; borderL < maxLoc.x;++borderL)
		{
			if (GrayValue==src.at<uchar>(i,borderL))
			{
				findL = 1;
				break;
			}
		}
		for (; borderR < src.cols;++borderR)
		{
			if (GrayValue==src.at<uchar>(i,borderR))
			{
				findR = 1;
				break;
			}
		}
		if (findL&&findR)
		{
			data.y = i;
			data.x = (borderL+borderR) / 2;
		}
		else
		{
			data.y = i;
			data.x = maxLoc.x;
		}
		/////  zhangxu  added  进行越界检查
		if (data.x<windowwidth / 2 || data.x> src1.cols-1 - windowwidth / 2)
		{
			continue;
		}
		//Step 2：确定前景亮度I1、背景亮度I2、条纹宽度w
		//Step 2.1：
		double I1 = 0, I2 = 0, w = 0;
		//窗口宽度的一半
		int width = (windowwidth - 1) / 2;
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
		//Step 2.2：
		double a = 0, b = 0, c = 0;
		a = m2 - m1*m1;
		b = m1*m2 - m3;
		c = m1*m3 - m2*m2;

		I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
		I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
		w = windowwidth*((m1 - I1) / (I2 - I1));

		//Step 3： I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
		if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold) 
		{
			Maxc.push_back(data);
		}
		else
		{
			continue;
		}
		//Step 5:  比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
		//zll		比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
		if (windowwidth < w)
		{
			continue;
		}
		//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
		float PixelValue = 0;
		float sum1 = 0;
		float sum2 = 0;
		for (int j = 0; j < windowwidth; j++)
		{
			PixelValue = src.at<uchar>(data.y, data.x - ((windowwidth - 1) / 2) + j);
			if (PixelValue>I1 + Threshold)  //改动
			{
				sum1 += PixelValue*(data.x - ((windowwidth - 1) / 2) + j);
				sum2 += PixelValue;
			}
		}
		if (sum1 != 0)
		{
			center.y = data.y;
			center.x = sum1 / sum2;
			LaserCenter.push_back(center);
		}
	}
	return 1;
}

bool CoreAlgorithm::LaserCenterDetector(const Rect ROI, Mat& src, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter, Mat& gaussK)
{
	//将图像转化成灰度图
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//截图原图像的roi部分作为中心提取的新图像
	src = src(ROI);

	//进行高斯滤波，去除噪声
	GaussianBlur(src, src, Size(5, 5), 0, 0);     //高斯滤波
	//Step 1：求该行图像中亮度最大值所在位置Maxc
	Point data;
	Point2f center;

	//参数初始化
	double maxVal = 0;

	double I1 = 0, I2 = 0, w = 0;
	double m1 = 0, m2 = 0, m3 = 0;
	double a = 0, b = 0, c = 0;
	float PixelValue = 0;
	float sum1 = 0;
	float sum2 = 0;
	for (int i = 2; i < src.rows-2; i++)
	{
		Mat src1 = src.row(i);
		Point maxLoc;
		minMaxLoc(src1, NULL, &maxVal, NULL, &maxLoc, Mat()); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
		data.y = i;
		data.x = maxLoc.x;
#if 0
		int GrayValue = src.at<uchar>(i, maxLoc.x);

		//根据行最大值以及条纹左边界和右边界确定条纹中心
		int borderL = 0, borderR = maxLoc.x + 1;
		bool findL = 0, findR = 0;
		for (; borderL < maxLoc.x; ++borderL)
		{
			if (GrayValue == src.at<uchar>(i, borderL))
			{
				findL = 1;
				break;
			}
		}
		for (; borderR < src.cols; ++borderR)
		{
			if (GrayValue == src.at<uchar>(i, borderR))
			{
				findR = 1;
				break;
			}
		}
		if (findL&&findR)
		{
			data.y = i;
			data.x = (borderL + borderR) / 2;
		}
		else
		{
			data.y = i;
			data.x = maxLoc.x;
		}

#endif
		/////  zhangxu  added  进行越界检查
		if (data.x<windowwidth / 2+2 || data.x> src1.cols - 1 - windowwidth / 2-2)
		{
			continue;
		}
		//Step 2：确定前景亮度I1、背景亮度I2、条纹宽度w
		//Step 2.1：
		I1 = 0, I2 = 0, w = 0;
		//窗口宽度的一半
		int width = (windowwidth - 1) / 2;
		m1 = 0, m2 = 0, m3 = 0;
		for (int j = 0; j < windowwidth; j++)
		{
			m1 += pow(src.at<uchar>(data.y, data.x - width + j), 1);
			m2 += pow(src.at<uchar>(data.y, data.x - width + j), 2);
			m3 += pow(src.at<uchar>(data.y, data.x - width + j), 3);
		}
		m1 = m1 / windowwidth;
		m2 = m2 / windowwidth;
		m3 = m3 / windowwidth;
		//Step 2.2：
		a = 0, b = 0, c = 0;
		a = m2 - m1*m1;
		b = m1*m2 - m3;
		c = m1*m3 - m2*m2;

		I1 = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);
		I2 = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);
		w = windowwidth*((m1 - I1) / (I2 - I1));

		//Step 3： I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
		//if (a >= 0 && b <= 0 && c >= 0 && I2 - I1 > Threshold)
		if (a< 0 || b > 0 || c < 0 || I2 - I1<= Threshold)
		{
			continue;
		}

		//Step 5:  比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
		//zll		比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
		if (windowwidth < w)
		{
			continue;
		}

		//PartGaussianBlur(src, i, data.x - (windowwidth - 1) / 2, data.x + (windowwidth - 1) / 2, gaussK);

		//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
		PixelValue = 0;
		sum1 = 0;
		sum2 = 0;
		for (int j = 0; j < windowwidth; j++)
		{
			PixelValue = src.at<uchar>(data.y, data.x - ((windowwidth - 1) / 2) + j);
			if (PixelValue>I1 + Threshold)  //改动
			{
				sum1 += PixelValue*(data.x - ((windowwidth - 1) / 2) + j);
				sum2 += PixelValue;
			}
		}
		if (sum1 != 0)
		{
			center.y = data.y;
			center.x = sum1 / sum2;
			LaserCenter.push_back(center);
		}
	}
	//将roi图像条纹中心坐标转换成原图像中的坐标
	for (int i = 0; i < LaserCenter.size();++i)
	{
		LaserCenter[i].x = LaserCenter[i].x + ROI.tl().x;
		LaserCenter[i].y = LaserCenter[i].y + ROI.tl().y;
	}
	return 1;
}

bool CoreAlgorithm::LaserCenterDetector(cv::Rect ROI, Mat& src, const int Threshold, const int windowwidth, vector<Point2f> &LaserCenter, int searchWinHei, Mat& gaussK)
{
	//判断输入图像是否是灰度图像
	if (src.channels() != 1)
	{
		cout << "Laser image channel erro." << endl;
		return false;
	}

	//截图原图像的roi部分作为中心提取的新图像
	src = src(ROI);

	//进行高斯滤波，去除噪声---------这一步放在了每个循环里面了
	//GaussianBlur(src, src, Size(5, 5), 0, 0);     //高斯滤波

	//定义是否是新条纹标识
	bool isNear = 0;

	//Step 1：求该行图像中亮度最大值所在位置Maxc
	Point data;
	Point2f center;
	int trapeHeiTem = 0;
	int colRangeHalf = 0;
	//max函数中mask的起始列和终止列
	int colStart = 0;
	int colEnd = 0;
	int colMax;

	int maxVal = 0;
	Point maxLoc;

	//窗口宽度的一半
	int width = (windowwidth - 1) / 2;

	//由于使用高斯滤波时没有对边界进行插值，所以行的开始值和结束值必须缩减
	for (int i = 2; i < src.rows - 2; i++)
	{
		Mat src1 = src.row(i);
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
			getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
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

			//I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
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
			PartGaussianBlur(src, i, data.x - width, data.x + width, gaussK);

			//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>(data.y, data.x - width + j);
				if (PixelValue>I1 + Threshold)
				{
					sum1 += PixelValue*(data.x - width + j);
					sum2 += PixelValue;
				}
			}
			if (sum1 != 0)
			{
				center.y = data.y;
				center.x = sum1 / sum2;
				LaserCenter.push_back(center);
				trapeHeiTem = 0;
			}

		}
		else
		{
			minMaxLoc(src1, NULL, NULL, NULL, &maxLoc); //函数MinMaxLoc 查找元素中的最小、大值以及他们的位置
			data.y = i;
			data.x = maxLoc.x;

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

			//I1-I2 与阈值T比较，若小于，则该行无激光条纹中心
			if (a < 0 || b > 0 || c < 0 || I2 - I1 <= Threshold)
			{
				continue;
			}
			// 比较重心计算的像素个数n1 与w，当n1大于等于w，则为有效重心值p
			//比较条纹宽度是否大于窗口宽度，若大于，则中心点无效
			if (windowwidth < w)
			{
				continue;
			}

			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur(src, i, data.x - width, data.x + width, gaussK);

			//Step 4：重心法求亚像素条纹中心，并统计重心计算的像素个数n1
			float PixelValue = 0;
			float sum1 = 0;
			float sum2 = 0;
			for (int j = 0; j < windowwidth; j++)
			{
				PixelValue = src.at<uchar>(data.y, data.x - width + j);
				if (PixelValue>I1 + Threshold)  //改动
				{
					sum1 += PixelValue*(data.x - width + j);
					sum2 += PixelValue;
				}
			}
			if (sum1 != 0)
			{
				center.y = data.y;
				center.x = sum1 / sum2;
				LaserCenter.push_back(center);
				isNear = 1;
			}
		}
	}
	//将roi图像条纹中心坐标转换成原图像中的坐标
	for (int num = 0; num < LaserCenter.size(); ++num)
	{
		LaserCenter[num].x = LaserCenter[num].x + ROI.tl().x;
		LaserCenter[num].y = LaserCenter[num].y + ROI.tl().y;
	}

	return 1;
}

bool CoreAlgorithm::getRowMaxLoc(Mat& rowMat, int colStart, int colEnd, int& maxValue, int& colMax)
{
	if (rowMat.rows != 1)
	{
		cout << "Row image error." << endl;
		return false;
	}

	uchar* rowPtr = rowMat.ptr<uchar>(0);
	maxValue = rowPtr[colStart];
	colMax = colStart;
	for (int col = colStart + 1; col < colEnd; ++col)
	{
		if (rowPtr[col]>maxValue)
		{
			maxValue = rowPtr[col];
			colMax = col;
		}
	}
	return true;
}

bool CoreAlgorithm::LaserCenterDetector(cv::Rect ROI, Mat& src, const int Threshold, const int windowwidth, int centerRange, vector<Point2f> &LaserCenter, int searchWinHei, Mat& gaussK)
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
			getRowMaxLoc(src1, colStart, colEnd, maxVal, colMax);
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
			
			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur(src, i, data.x - width, data.x + width, gaussK);
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
					sum1 += PixelValue*(data.x - width + j);
					sum2 += PixelValue;
				}
			}
			if (sum1 != 0)
			{
				center.y = data.y;
				center.x = sum1 / sum2;
				LaserCenter.push_back(center);
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
				getRowMaxLoc(src1, rangeColStart, rangeColEnd, maxVal, colMax);
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

			//对当前行窗口区域中的像素进行高斯滤波
			PartGaussianBlur(src, i, data.x - width, data.x + width, gaussK);
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
					sum1 += PixelValue*(data.x - width + j);
					sum2 += PixelValue;
				}
			}
			if (sum1 != 0)
			{
				center.y = data.y;
				center.x = sum1 / sum2;
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

			}
		}

	}
	//将roi图像条纹中心坐标转换成原图像中的坐标
	for (int num = 0; num < LaserCenter.size(); ++num)
	{
		LaserCenter[num].x = LaserCenter[num].x + ROI.tl().x;
		LaserCenter[num].y = LaserCenter[num].y + ROI.tl().y;
	}

	return 1;
}

bool CoreAlgorithm::PartGaussianBlur(Mat& srcImg, int rowCurrent, int colStart, int colEnd, Mat& gaussK)
{
	if (gaussK.rows != 5 || gaussK.cols != 5)
	{
		cout << "GaussBlur Kernal'size error." << endl;
		return false;
	}
	if (srcImg.empty())
	{
		cout << "GaussBlur Image is empty." << endl;
		return false;
	}
	for (int col = colStart; col < colEnd; ++col)
	{
		float numTem = 0;
		for (int rowTem = 0; rowTem < 5;++rowTem)
		{
			for (int colTem = 0; colTem < 5;++colTem)
			{
				numTem = numTem + srcImg.at<uchar>(rowCurrent - 2 + rowTem, col - 2 + colTem)*gaussK.at<float>(rowTem, colTem);
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


bool CoreAlgorithm::PartMeanBlur(Mat& srcImg, int rowCurrent, int colStart, int colEnd)
{
	if (srcImg.empty())
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
				numTem = numTem + srcImg.at<uchar>(rowCurrent - 2 + rowTem, col - 2 + colTem);
			}
		}
		srcImg.at<uchar>(rowCurrent, col) = numTem/25;
	}
	return true;
}

float CoreAlgorithm::Point2LineDistance(Vec3f& coeffs, Point2f& point)
{


	float distance;
	float a = coeffs[0];
	float b = coeffs[1];
	float c = coeffs[2];
	float x0 = point.x;
	float y0 = point.y;
	//float sign;
	//if (a*x0+b*y0+c>0)
	//{
	//	sign = 1;
	//}
	//else
	//{
	//	sign = -1;
	//}
	distance = (a*x0 + b*y0 + c) / sqrt(a*a + b*b);
	return distance;
}
