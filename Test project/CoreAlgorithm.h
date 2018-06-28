#pragma once
//#include"stdafx.h"
#include "SharedHead.h"
//#include "sharedmethod.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <io.h>
#include <list>
#include <map>
#include <stack>
#include <iterator>
#include <numeric>

#undef  ACCESS_MASK

//#define BLUR
#define CANNY
//#define THRESHOL

class CoreAlgorithm
{
public:
    ///使用迭代方法进行单个相机的标定
    ///@param imgsPath 输入 标定图像的绝对路径
    ///@param cData 输入 标定用数据
    ///@param camParaSrc 输入 由张正友计算出的相机参数初始值
    ///@param camParaDst 输出 迭代后计算后获得的相机参数
    ////  2014.11.6 updated by zhangxu  迭代特征点每次更新作为下次计算平行视图点的数据，最终特征点输出
    static void iterativeCameraCalibration(const vector<string>& imgsPath, CalibrationData &cData,
        const CamPara& camParaSrc, CamPara& camParaDst);

    ///获得畸变矫正后的图像
    ///@param src 输入 标定板原始图像
    ///@param dst 输出 标定板畸变校正后的图像
    ///@param cameraMatrix 输入 相机内参数
    ///@param distCoeffs 输入 相机畸变参数
    ///@param R 3x3输入 如果输入单位矩阵，则仅输出畸变校正后的图像
    ///					如果输入标定板平行视图的R，则输出矫正后的平行视图图像
    static void undistortImg(const Mat& src, Mat& dst,
        const Mat& cameraMatrix, const vector<double>& distCoeffs, const Mat& R);

    ///将某一相机图像坐标系下的标定板坐标转换到另一相机图像图像坐标系下
    ///@param src 输入 标定板原始视图上的特征点2d
    ///@param dst 输出 标定板平行视图上的特征点2d
    ///@param cameraMatrix12 输入 相机内参数
    ///@param distCoeffs12 输入 相机畸变参数
    ///@param R12 输入 标定板R 3x1
    ///@param T12 输入 标定板T 3x1
    static void undistortPoints2DifferentView(const vector<Point2f>& src, vector<Point2f>& dst,
        const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
        const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2);

    ///采用Random算法找出在长轴和短轴上的点,十字交叉点属于长轴
    ///长轴以y坐标排序，从十字交叉处开始；短轴按照光笔正方时十字交叉左右排序
    ///输入点的要求：1、十字叉形状;2、至少6个点
    ///@param pnts 输入 至少6个点
    ///@param L_pnts 输出 长轴点
    ///@param S_pnts 输出 短轴点
    static bool pointsSort2D_3(vector<Point2f>& pnts,vector<Point2f>& L_pnts,vector<Point2f>& S_pnts);

    ///根据光笔的具体形状（十字交叉），给输入的点加上标签
    ///输入点条件：1、十字叉形状;2、长短轴加起来至少6个点
    ///@param L_pnts 输入 长轴点，至少4个点
    ///@param S_pnts 输入 短轴点，至少1个点
    ///@param tagPnts 输出 带有标签的光斑点坐标,Vec3f 第一个数为标签，后两个数为点坐标
    static bool getPntsWithTag(const vector<Point2f>& L_pnts,const vector<Point2f>& S_pnts,vector<TagPoint2f>& tagPnts);

    ///红外LED光斑提取算法
    static void detectLightSpot_LED(const Mat& imgMat,vector<Point2f>& centerPnt,vector<float>& longaxisRadius);

    ///张正友相机标定方法
    ///@param cData 输入 标定数据
    ///@param campara 输出 相机标定参数
    static void ZhangCalibrationMethod_CPP( const CalibrationData& cData,CamPara& campara );

    ///立体标定方法,相机1为主相机
    ///@param camPara12 输入 单个相机标定参数
    ///@param CalibrationData12 输入 单个相机标定数据
    ///@param rt 输出 两相机相对位姿关系
    static void stereoCalibration(const CamPara& camPara1,const CamPara& camPara2,
        const CalibrationData& cornerDataVec1,const CalibrationData& cornerDataVec2,RT& rt);

    ///判断一个int形是否存在于vector中
    static bool isBelong2Vec(vector<int> vec,int index);

    ///三维重建
    //// 方案0  这个有问题，必须得重新整理一下
    /// 针对结构光系统，投影机只有一个位置对应信息，例如对应性上只有x坐标，没有y坐标
    /// 因为本系统中恢复三维形状，得到的是投影图像x坐标，宽度方向的，只有一个坐标没办法解算畸变模型，所以本函数计算三维点
    ///   投影机假设没有畸变，或者对投影机畸变已经校正考虑了， 摄像机可以有畸变。以后再编码时可以考虑把编码图像编制在矫正后的图像上，不过就现在看来，线性模型精度已经够了。
    ///   const vector<Point2f>& pointCam,<in> camera image point
    ///   const vector<float>& proImg_x <in> the corresponding image point in the projector image. the model of the  projector is linear,no lens distortion
    ///   vector<Point3f>& point3d <out>  the point cloud in the coordinate of camera
    ///   const CamPara& camParaLeft, 摄像机内参数
    ///   const CamPara& camParapro， 投影机内参数
    ///   const double rotVector[3] ,const double traVector[3] ,  投影机相对与摄像机的外参数
    static bool Cal3dPointStrLight(const vector<Point2f>& pointCam,const vector<float>& proImg_x,const CamPara& camParaLeft, const CamPara& camParapro,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d);

    //方案一：
    ///// 根据左右相机的对应性信息，内外参数值，对畸变进行校正，然后计算三维点
    /// const vector<Point2f> pointLeft <in>, 左相机的对应点信息，要求x坐标，y坐标都有
    /// const CamPara& camParaLeft  <in>  左相机的内参数，
    /// const vector<Point2f> pointRight <in>  右相机的对应点信息，要求x坐标，y坐标都有
    /// const CamPara& camParaRight <in> 右相机的内参数
    /// const double rotVector[3] <in>  右相机相对左相机的旋转向量
    /// const double traVector[3] <in>  右相机相对左相机的平移向量
    /// vector<Point3f>& point3d  <out>  左相机坐标系下的三维点
    static bool Cal3dPoint( const vector<Point2f> pointLeft ,const CamPara& camParaLeft
                    ,const vector<Point2f> pointRight ,const CamPara& camParaRight
                    ,const double rotVector[3] ,const double traVector[3] ,vector<Point3f>& point3d );

    ///方案二：opencv的triangulatePoints函数
    //// 根据左右相机的对应性信息，内外参数值，对畸变进行校正，然后计算三维点
    //// const vector<Point2f>& pnts2d1, <in> 左相机的对应点信息，要求x坐标，y坐标都有
    /// const vector<Point2f>& pnts2d2, <in> 右相机的对应点信息，要求x坐标，y坐标都有
    /// const Mat& cameraMatrix1,  <in>  左相机的内参数
    /// const Mat& R1, <in> 左相机相对世界坐标系的旋转矩阵，若世界坐标系建立在摄像机坐标1 上，其为单位矩阵
    /// const Mat& T1, <in> 左相机相对世界坐标系的平移向量，若是世界坐标系建立在摄像机坐标1上，其为0矩阵
    /// const vector<double>& distCoeffs1, <in>
    /// const Mat& cameraMatrix2, <in>  右相机的内参数
    /// const Mat& R2, <in> 右相机相对世界坐标系的旋转矩阵，若世界坐标系建立在摄像机坐标系1上，其为相对于摄像机1的旋转矩阵
    /// const Mat& T2, <in> 右相机相对世界坐标系的平移矩阵，若世界坐标系建立在摄像机坐标系1上，其为相对于摄像机1的平移矩阵
    /// const vector<double>& distCoeffs2, <in>
    ///	vector<Point3f>& pnts3d  <out>
    static bool triangulatePnts(const vector<Point2f>& pnts2d1,const vector<Point2f>& pnts2d2,
        const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
        const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
        vector<Point3f>& pnts3d);



    ///角点提取算法，按照个数均分，一行一行排序
    ///算法存在的问题，如果近距离，标定板倾斜较大，则初始角点不能均分，无法检测角点
    static vector<Point2f> extraCorner(const Mat& img,const vector<Point2f>& mousePnts,const int r,const int c);

    ///角点提取算法，按照像素搜索;row和 col为输出
    static bool extraCorner2(const Mat& img, const vector<Point2f>& mousePnts
        ,vector<Point2f>& ImageCorner, int &row, int &col);

    ///圆环中心提取算法，点选进行圆环四周
    ///整幅图像中进行圆环提取，无序，需要在提取后重新排序
    static bool extraRingCenter(const Mat& img,const vector<Point2f>& mousePnts,
        vector<Point2f>& ImageCorner,int& row,int& col);

    ///圆环中心提取算法，点选进行圆环四周
    ///整幅图像中进行圆环提取，无序，需要在提取后重新排序  张旭的老版
    static bool extraRingCenterXu(const Mat& img, const vector<Point2f>& mousePnts
        , vector<Point2f>& plane2dPnts, int& row, int& col);

    ///圆环中心提取算法，点选四个角圆环中心区域
    ///整幅图像中进行圆环提取，无序，需要在提取后重新排序
    static bool extraRingCenter2(const Mat& img,const vector<Point2f>& mousePnts,
        vector<Point2f>& ImageCorner,int& row,int& col);

    //至少3个点
    //oriPoints,起始位置三维坐标点，既X1
    //terminatePoints，结束位置三维坐标点，既X2
    //Rot，输出的旋转矩阵 3X3 double
    //Tra，输出的平移向量 3X1 double
    //X2=R*X1+T 起始位置点变换到结束位置点
    static bool rigidTransform(const std::vector<Point3f>& oriPoints,
                                const std::vector<Point3f>& terminatePoints,
                                cv::Mat& Rot,cv::Mat& Tra);

    ///测头标定算法
    ///@param 输入 光笔图片的R(3X3), T(3X1) double
    ///@param 输出 测头中心在测量坐标系下的三维坐标
    ///修改计算方法SVD，放到一个Mat 3xN
    static bool calculateProbeCenter(const vector<Mat>& R_vec,const vector<Mat>& T_vec,Point3f& center);

    ///建立光笔坐标系
    ///@param vector<TagPoint3f> tagPnts 输入/输出 测量坐标系下,7个光斑点坐标；输出光笔坐标系下的新坐标
    ///@param Point3f probeCenter 输入 光笔坐标系下,测头中心坐标
    ///建立方法：原点在测头中心，长轴5点到1点方向确定Z轴正方向，6点到7点方向确定x轴正方向，y轴方向根据右手法则确定
    ///tagPnts  Vec4f,第一个数为tag，后三个数为坐标
    static void createLightPenCoordinate(vector<TagPoint3f>& tagPnts,const Point3f& probeCenter);

    ///二次曲面模型，亚像素寻找
    ///@param 输入 win 3x3 峰值点位于窗口的中间
    ///@param 输出 dx  x方向偏移量
    ///@param 输出 dy  y方向偏移量
    static bool quadSubPixel(const Mat& win,float& dx,float& dy);

    ///由圆环中心点间距生成模板匹配的模板
    static Mat generateRingTemp(const vector<Point2f>& centers);

    ///求出标定板特征点的三维点平面点
    ///@param R 3x1的旋转向量 或3x3的旋转矩阵
    static void project2CalibrationBoard(const vector<Point2f>& src, vector<Point3f>& pnts3d,
        const Mat& cameraMatrix, vector<double> distCoeffs, const Mat& R,const Mat& T);

    /// 产生余弦条纹图像，I = bias + range*cos(2*PI*x/T - fai)  或者I = bias + range*cos(2*PI*y/T - fai)
    // Mat& Img, <in and out> 出入余弦条纹图像
    // float T, <in> 余弦条纹的周期
    // float fai,<in> 余弦的初始相位
    // float bias, <in> 余弦亮度均值
    // float range,<in> 余弦亮度的幅值
    // float gamma, <in> 余弦亮度条纹的非线性特性gamma值，若是线性的 gamma = 1 （默认值），若是非线性的，不等于1
    // bool, isX <in> 是否对X坐标进行余弦编码， true 是对X， 是对y进行编码
    // bool isGrid <in>  x y坐标是否垂直成正交grid， true 为是，false 表示为六边形排布，类似Lightcrafter的DMD形式
    static bool creatCosImg(Mat& Img, float T, float fai, float bias, float range, float gamma = 1, bool isX =true, bool isGrid = true);

    /// 求最大公约数
    /// 扩展的欧几里德算法，计算a  b 的最大公约数q，以及a b的 逆 x y，则满足  ax+by = q= gcd(a,b)
    /// 算法 采用递归的方法 参考 算法导论 528页 网页 http://baike.baidu.com/view/1478219.htm
    /// int a,<in> 输入整数 a
    /// int b,<in> 输入整数 b
    /// int& x,<in and out>  输出的参数，对应a的系数
    /// int& y, <in and out>  输出的参数，对应b的系数
    /// int& q <in and out>  输出的最大公约数
    static bool gcd(int a, int b, int& x, int& y, int& q);

    /// 计算N步相移的卷绕相位，只针对灰度图像序列
    /// vector<Mat> images   输入相同频率的图像序列
    /// float threshold      阈值，判断是否为合适的有效的计算结果，
    /// Mat& mod_phase,  卷绕相位，输出，浮点数矩阵 float，行列数等于图像行列数
    /// Mat& mean_lus,   均值亮度，输出，浮点数矩阵 float，行列数等于图像行列数
    /// Mat& lus_range,   亮度范围，，输出，浮点数矩阵 float，行列数等于图像行列数
    /// Mat& mask，   当前坐标位置的相位是否有效，1为有效，0为无效；输出，整数矩阵，行列数等于图像行列数
    static bool phaseshift (vector<Mat> images, float threshold, Mat& mod_phase, Mat& mean_lus, Mat& lus_range, Mat& mask);

    /// 根据中国剩余定理 计算卷绕相位的绝对相位，并转化为像素坐标
    /// vector<Mat> mod_phases,<in>  输入多个卷绕相位，float型
    /// vector<float> lamda <in>   输入多个频率的周期值，
    /// float m <in>  所有周期的最大公约数
    /// Mat& cor <out>
    static bool robustCRM(vector<Mat> mod_phases,vector<float> lamda, float m, Mat& cor);

    /// 计算无畸变的图像像素坐标，即输入是图像像素坐标，输出也是像素坐标，两者具有相同的摄像机内外参数，只是第二个没有镜头畸变
    /// const vector<Point2f>& src , <in> 输入图像点坐标
    /// vector<Point2f>& dst ,<out>  输出摄像机图像坐标下的像素坐标
    /// double fc[2] ,<in> 摄像机参数focal lengh
    /// double cc[2] ,<in> 摄像机参数主点位置
    /// double kc[5] ,<in> 摄像机畸变参数
    /// double alfpha_c  <in>  摄像机参数纵横比例因子
    static bool undistorPixcor(const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c );

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
    static bool creatdistorImg(Mat& img,double fc[2] ,double cc[2] ,double kc[5] ,
                               double alfpha_c, float meanLus, float rangelus,float T,
                               float fai, bool isGrid=true,int method =0);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// 程伟 ///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //根据检测出的轮廓中心点，找出需要的特征点并标出相应序号
    //输入：检测出的轮廓中心点；每个轮廓拟合椭圆的长轴半径,gamma代表圆斑距离除以圆斑半径
    //输出：带有标签信息，已排好序的特征点	一号点的圆斑区域宽度
    static bool findPntsWithTag(vector<Point2f>& centerPnt,vector<float>& longaxisRadius,vector<TagPoint2f>& tagPnts,float &firstFeaturelength,const double gamma);

    //// 创建一个圆环图像，改图像保存在正方形的矩阵中，正方形尺寸为dx，外圆环半径为 dx/3, 内圆环半径为  dx/6
    //// Mat& img,<out> 输出图像 double型
    /// int dx <in>  正方形矩阵的尺寸
    static bool creatRingImg(Mat& img,int dx);

    ///圆环中心提取算法，自动
    static bool findRingBoardCenter(const Mat& img, Size patternSize, vector<Point2f>& centers);

    //得到有序号的二维特征点集 基于opencv fittingellipse
    static bool getTagPoint2f(const Mat& img, vector<TagPoint2f>& tagPnts2f);

    //得到摄像机坐标系下的有序号的三维特征点集 基于opencv fittingellipse
    //static bool getTagPoint3f(const Mat& img1,const CamPara _campara1, const Mat& img2, const CamPara _campara2,
    //    const CamGroupPara _camgrouppara, vector<TagPoint3f>& tagPnts3f);

    //得到有序号的二维特征点集 基于dual fitting ellipse亚像素圆检测
    static bool getTagPoint2f2(const Mat& img, vector<TagPoint2f>& tagPnts2f);
    static bool getTagPoint2f2(const Mat& img, vector<TagPoint2f>& tagPnts2f,cv::Rect mask);
    //static bool getTagPoint2f2(const Mat& img, const Mat imgvector<TagPoint2f>& tagPnts2f);

    //输入已知世界坐标系三维特征点objectPoints
    //输入图像坐标系下的二维图像特征点imagePoints
    //输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
    //输出测量坐标系下的各个特征点的三维坐标PointToCam
    //输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
    //输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
    //标志位Flag 代表内部使用的解决PNP问题的方法
    // PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization.
    //In this case the function finds such a pose that minimizes reprojection error,
    //that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
    //以下两个需要opencv3.0以上版本才支持
    //PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
    //PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer.
    //“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
    //In this case the function also estimates the parameters f_x and f_y assuming that both have the same value.
    //Then the cameraMatrix is updated with the estimated focal length.
    static void PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
                        Mat cameraMatrix, Mat distCoeffs,
                        Mat &PoseR,Mat &PoseT,
                        vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag = PNP_DLS);

    //输入已知世界坐标系三维特征点objectPoints
    //输入图像坐标系下的二维图像特征点imagePoints
    //输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
    //输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
    //输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
    //标志位Flag 代表内部使用的解决PNP问题的方法
    // PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization.
    //In this case the function finds such a pose that minimizes reprojection error,
    //that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
    //以下两个需要opencv3.0以上版本才支持
    //PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
    //PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer.
    //“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
    //In this case the function also estimates the parameters f_x and f_y assuming that both have the same value.
    //Then the cameraMatrix is updated with the estimated focal length.
    static void PnPMethod(vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
                        Mat cameraMatrix, Mat distCoeffs,
                        Mat &PoseR,Mat &PoseT,
                        vector<double> &dpdrot,vector<double> &dpdt,int Flag = PNP_DLS);

    //输入拍摄特征点图像img
    //输入已知世界坐标系三维特征点objectPoints
    //输入图像坐标系下的二维图像特征点imagePoints
    //输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
    //输入迭代次数iterativeTimes，默认为10次
    //输出测量坐标系下的各个特征点的三维坐标PointToCam
    //输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
    //输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
    //标志位Flag 代表内部使用的解决PNP问题的方法
    // 特征点是否是圆环
    // PNP_ITERATIVE Iterative method is based on Levenberg-Marquardt optimization.
    //In this case the function finds such a pose that minimizes reprojection error,
    //that is the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints() ) objectPoints .
    //PNP_EPNP  F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    // PNP_P3P   X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
    //以下两个需要opencv3.0以上版本才支持
    //PNP_DLS   Method is based on the paper of Joel A. Hesch and Stergios I. Roumeliotis. “A Direct Least-Squares (DLS) Method for PnP”.
    //PNP_UPNP  Method is based on the paper of A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer.
    //“Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation”.
    //In this case the function also estimates the parameters f_x and f_y assuming that both have the same value.
    //Then the cameraMatrix is updated with the estimated focal length.
    static bool iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
                        Mat cameraMatrix, Mat distCoeffs,
                        Mat &PoseR,Mat &PoseT,
                        vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam,int Flag = PNP_DLS ,int iterativeTimes =10);

    static bool iterativePnPMethod(Mat &img,vector<Point3f> objectPoints,
                        vector<Point2f>imagePoints,
                        Mat cameraMatrix, Mat distCoeffs,
                        Mat &PoseR,Mat &PoseT,
                        vector<double> &dpdrot,vector<double> &dpdt,int Flag = PNP_DLS ,int iterativeTimes =10);

    //利用三点法2求表面法线
    static void GetSurfaceNormal(Point3f point1,Point3f point2,Point3f point3,SurfaceNormal &SurfaceNormal);

    //平面拟合
    //输入：obsPoints  平面上的观测点
    //输出：Plane
    static bool PlaneFitting(vector<Point3f> obsPoints,SPlane& model);

    //柱面拟合
    //输入 obsPoints 柱面上的观测点
    //输出 柱面模型参数
    static bool CircleCylinderFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank =0);

    //球面拟合
    //输入 obsPoints 柱面上的观测点
    //输出 球面模型参数
    //static bool sphereFitting(vector<Point3f> obsPoints,CircleCylinder& models,int errorRank =0);

    //求出平面与直线交点
    //输入：直线参数 平面参数
    //输出：直线与平面的交点
    //拟合直线可以用opencv 中的fitLine函数
    static bool CalPlaneLineIntersectPoint(const SPlane _plane,const Line _line,Point3f &IntersectPoint);

    //根据文档《》生成平顶视图的转换矩阵
    //const Mat &PoseR,<in>  3X1
    //const Mat &PoseT,<in>  3X1
    //Mat &parallelR<in & out>  输入平顶视图相对于摄像机的外参数R，3X3矩阵，输出是文档要求的H1 * inv(H0), 3X3
    static void calculateParallelViewR(const Mat &PoseR,const Mat &PoseT,Mat &parallelR);

    ///圆环中心亚像素检测
    ///@param img 输入 平行视图图像
    ///@param centers 输入和输出
    ///@param temp 输入 图像模板
    static bool ringSubPix(const Mat& img,vector<Point2f>& centers,const Mat& temp);

    ///高级圆环中心亚像素检测,必须首先用ringSubPix函数获取平行视图上像素级精度圆环中心坐标
    ///@param img 输入 平行视图图像
    ///@param centers 输入和输出
    ///@param mode 模式 0为内圆 1为外圆 2为内外圆平均值 3为内外圆优化平均值
    static bool ringSubPixAdv(const Mat& img, vector<Point2f>& centers, uint mode);

    //用于返回单幅图像中包含有T型靶标的的ROI
    //输入检测的七个靶点特征的二维中心点坐标
    //输入相对于原图的ROI
    static void findTtypeROI(vector<Point2f>& imagePoints,cv::Rect& ROI);

    //计算两点之间的距离
    static double distancePoints(const Point2f pnt1,const Point2f pnt2);

    //计算两个三维点之间的距离
    static double distancePoints(const Point3f pnt1,const Point3f pnt2);
    static float distancePlanes(const SPlane plane1,const SPlane plane2);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// 郑泽龙 //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    //描述：从相机参数类CamPara中提取相机内参和畸变参数
    ///@param Camparam     输入 通过readcampara()函数读进来的相机参数
    ///@param CamMatrix    输出 相机内参数矩阵
    ///@param DistCoeffs   输出 相机畸变
    static bool ExtraCamMatrixAndDist(const CamPara& Camparam,Mat& CamMatrix,Mat& DistCoeffs);

    //描述：提取立体主动光标靶特征点圆心
    ///@param img 输入 所要提取特征点的图像
    ///@param Threshold	输入 图片二值化的阈值
    ///@param areamin	输入 圆斑筛选的最小面积（像素点数）
    ///@param areamax	输入 圆斑筛选的最大面积（像素点数）
    ///@param rate  	输入 大小圆判别比例系数
    ///@param LargeCircleP 输出 所提取的大圆的中心
    ///@param SmallCircleP 输出 所提取的小圆的中心
    ///@param imge_contour 输出 特征标记后的图片
    static bool DiscernLSCircle(Mat img,vector<SH_Ellipse> &Fea_circle,Mat &imge_contour,
        int Threshold=ThreshLevel,int areamin=AreaMin,int areamax=AreaMax,float rate=RATE, int method = 0);

	//重载上述函数
	static bool DiscernLSCircle(Mat img, Mat &imge_contour, vector<SH_Ellipse> &Fea_circle,
		int Threshold = ThreshLevel, float rate = RATE);

    //描述：对提取的立体标靶的特征点进行特征圆识别和面识别
    ///@param img 输入 所要编号图像
    ///@param outimage 输出 识别结果图片
    ///@param Fea_circle 输入 由函数DiscernLSCircle()所提取的特征圆
    ///@param MarkedArea   输出 所识别到的立体标靶的特征信息
    ///@param err_str   输出 识别错误提示信息
    static bool FindMarkArea(Mat img,Mat& outimage,vector<SH_Ellipse> Fea_circle,MarkArea& MarkedArea,string& err_str);

	//描述：提取图片中指定面的特征信息
	///@param img		 输入 待处理图片
	///@param MarkedArea 输出 识别结果
	///@param err_str    输出 识别错误提示信息
	///@param Face		 输入 所要识别的面（可指定多个面，如face中存放1，2表示提取图像中的1，2面，若图片中没有指定面返回false）
	static bool FindImgFeature(const Mat &img, MarkArea& MarkedArea, string& err_str, const vector<int> Face);

	//描述：提取图片中指定面的特征信息（重载函数，用于双相机）
	///@param Limg		 输入 左图片
	///@param Rimg		 输入 右图片
	///@param MarkedArea 输出 识别结果
	///@param err_str    输出 识别错误提示信息
	///@param Face		 输入 所要识别的面（可指定多个面，如face中存放1，2表示提取图像中的1，2面，若图片中没有指定面返回false）
	static bool FindImgFeature(const Mat &Limg, const Mat &Rimg,MarkArea &LMarkedArea, MarkArea &RMarkedArea,string& err_str, const vector<int> &Face);

    //描述：在图片中画箭头
    ///@param 输入：操作图片、起点、终点、箭头长度、箭头角度、颜色、粗细、线型
    static void drawArrow(cv::Mat &img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar& color, int thickness, int lineType);

    //描述：获得立体标靶（即标靶的一号面）在测量坐标系下的位姿
    ///@param markedarea 输入	通过FindMarkArea()函数识别出的标靶特征信息
    ///@param distCoeffs 输入	相机畸变参数
    ///@param cameraMatrix 输入 相机内参矩阵
    ///@param rtTOcam   输出	立体标靶（即标靶的一号面）在测量坐标系下的位姿（4*4矩阵表示）
    ///@param err_str   输出    位姿获取错误提示
    static bool TransferFrame(const MarkArea &markedarea,const Mat &distCoeffs,const Mat &cameraMatrix,const ObjectPara &objparam, Mat &rtTOcam,string &err_str);

    //描述：找出左右相机图片中同时都能看到的面
    ///@param LeftCamMarkFace 输入	由FindMarkArea()函数提取的标靶特征信息
    ///@param RightCamMarkFace 输入	由FindMarkArea()函数提取的标靶特征信息
    ///@param LeftSameFace	输出	提取到的特征面信息
    ///@param RightSameFace 输出	提取到的特征面信息
    static int FindObjectSameFace(const MarkArea LeftCamMarkFace, const MarkArea RightCamMarkFace,MarkArea &LeftSameFace,MarkArea &RightSameFace);

    //描述：提取左右相机标靶图像中指定的特征面上的特征点坐标
    ///@param LeftCamImagePath 输入 待处理左图像路径
    ///@param RightCamImagePath 输入 待处理右图像路径
    ///@param flag 输入 所要提取的特征面编号
    ///@param LeftImagePoints 输出 提取到的左图像特征面特征坐标
    ///@param RightImagePoints 输出 提取到的右图像特征面特征坐标
    ///@param err_inf 输出 错误信息输出
    static bool GetObjectFeatureData(const vector <std::string> LeftCamImagePath,const vector <std::string> RightCamImagePath,const FaceFlag flag,
        vector<vector<Point2f>> &LeftImagePoints,vector<vector<Point2f>> &RightImagePoints,string &err_inf);

    //上面函数的重载：用于标靶工作面相对位姿标定
    ///@param LeftCamImagePath 输入 待处理左图像路径，可以同时处理多张
    ///@param RightCamImagePath 输入 待处理右图像路径，可以同时处理多张
    ///@param flag 输入 所要提取的特征面编号
    ///@param LeftImagePoints 输出 提取到的左图像第一个特征面特征坐标
    ///@param RightImagePoints 输出 提取到的右图像第一个特征面特征坐标
    ///@param LeftImagePoints 输出 提取到的左图像第二个特征面特征坐标
    ///@param RightImagePoints 输出 提取到的右图像第二个特征面特征坐标
    ///@param err_inf 输出 错误信息输出
    static bool GetObjectFeatureData(const vector <std::string> LeftCamImagePath,const vector <std::string> RightCamImagePath,const int flag,
        vector<vector<Point2f>> &LeftImagePnt1s,vector<vector<Point2f>> &RightImagePnt1s,
        vector<vector<Point2f>> &LeftImagePnt2s,vector<vector<Point2f>> &RightImagePnt2s,string &err_inf);

    //描述：三角法获取特征点三维信息
    ///@param LeftImagePoints 输入 左相机图像上对应点坐标
    ///@param RightImagePoints 输入 右图图像上对应点坐标
    ///@param cameraMatrix1 输入 左相机内参
    ///@param R1 输入 左相机相对测量坐标系的旋转矩阵
    ///@param T1 输入 左相机相对测量坐标系的平移矩阵
    ///@param distCoeffs1 左相机畸变参数
    ///右相机相关参数
    static bool CalibrateFeatureDim(const vector<vector<Point2f>> LeftImagePoints,const vector<vector<Point2f>> RightImagePoints,
        const Mat& cameraMatrix1, const Mat& R1, const Mat& T1, const vector<double>& distCoeffs1,
        const Mat& cameraMatrix2, const Mat& R2, const Mat& T2, const vector<double>& distCoeffs2,
        vector <Point3f> &pnts3d);
        //以1号大圆的圆心为原点，以1号圆圆心和3,5号圆心的向量为x,y轴的坐标系下，各特征点相对坐标值

    //重载上面的函数
    static bool CalibrateFeatureDim(const vector<vector<Point2f>> LeftImagePoints,const vector<vector<Point2f>> RightImagePoints,
        const CamPara& camParaLeft,const CamPara& camParaRight,const double rotVector[3] ,const double traVector[3] ,vector <Point3f> &pnts3d);

    //描述：对输入的三维向量进行斯密特标准正交化
    ///@param vector <Vec3f> src 输入 2个或者3个的线性无关的三维向量，第一个向量为基向量
    ///@param vector <Vec3f> dst 输出 标准正交化后的向量，里面有三个向量
    static bool GramSchmidt(const vector <Vec3f> src,vector <Vec3f> dst);

    //重载GramSchmidt（）：对输入的三维向量进行斯密特标准正交化
    ///@param Mat src  输入 3*2或者3*3的矩阵，每行必须是线性无关的
    ///@param Mat &dst 输出 标准正交化后的矩阵（3*3）
    static bool GramSchmidt(const Mat src,Mat &dst);

    //描述：由特征点的在相机坐标系三维坐标信息，建立标靶工作面坐标系，得到标靶工作面坐标系在相机坐标系下的位姿
    ///@param  vector <point3f> src_pnts 输入 标靶上七个已排序大圆的圆心坐标（在相机坐标系下）
    ///@param  Mat dst_RT				 输出 标靶坐标系在相机坐标系下的位姿矩阵（4*4）
    ///note   输入的vector<point3f>中点的顺序必需和标靶工作面上特征圆的顺序标号是一致的
    static bool GetObjectFaceRT(const vector <Point3f> src_pnts, Mat &dst_RT);

    //描述：获取两个特征面的相对位姿（标靶标定用）
    static bool GetFaceToFaceRT(const vector<vector<Point2f>> &LeftImagePnt1s,const vector<vector<Point2f>> &RightImagePnt1s,
        const vector<vector<Point2f>> &LeftImagePnt2s,const vector<vector<Point2f>> &RightImagePnt2s,
        const CamPara& camParaLeft,const CamPara& camParaRight,const double rotVector[3] ,const double traVector[3] ,
        vector<Mat> &RTs,string err_str);

	//上面函数的重载：用于标靶工作面相对位姿标定
	///@param Limg  输入 左图片
	///@param Rimg  输入 右图片
	///@param flag  输入 所要提取的特征面编号
	///@param Lcam  输入 左相机参数
	///@param Rcam  输入 右相机参数
	///@param Camrt 输入 相机参数相对位姿
	///@param RTs   输出 两个面的相对位姿
	///@param err_inf 输出 错误信息输出
	static bool GetFaceToFaceRT(const Mat &Limg,const Mat &Rimg,const int flag, const CamPara& Lcam,const CamPara& Rcam,
		const Mat &Camrt ,Mat &RT,string &err_str);

    //描述：用双目的方法获取标靶在测量坐标系下的位姿
    ///input   LeftImage，RightImage:左右相机图片
    ///input   L_Camparam，R_Camparam:左右相机参数
    ///input   CamRT：相机外参
    ///input   objparam：标靶参数
    ///output  RT2cam:标靶一号面在左相机下的位姿
    ///output  err_inf：错误提示信息
    static bool GetObjectRT(const Mat &LeftImage,const Mat &RightImage,const CamPara &L_Camparam,
        const CamPara &R_Camparam,const Mat &CamRT,const ObjectPara &objparam,Mat &outL,Mat &outR, Mat &RT2cam,
        std::string &err_inf);

	//描述：单相机获取标靶位姿
	///input   image:图片
	///input   Camparam:相机内参数
	///input   objparam：标靶参数
	///output  RT2cam:标靶在相机下的位姿
	///output  err_inf：错误提示信息
	static bool GetObjectRT(const Mat &Image, const CamPara &Camparam, const ObjectPara &objparam, Mat &outImage, Mat &RT2cam, std::string &err_inf);

    //函数描述：用标靶的特征图形对评价双目系统的测量误差
    //@param Leftimage 输入 左相机图片
    //@param Rightimage 输入 右相机图片
    //@param camParaLeft 输入 左相机参数
    //@param camParaRight 输入 右相机参数
    //@param rotVector 输入 左相机相对右相机的旋转量
    //@param traVector 输入左相机相对右相机的平移量
    //备注：
    static bool StereMeasureAccuracy(const Mat &Leftimage, const Mat &Rightimage,
                                        const CamPara& camParaLeft, const CamPara& camParaRight,
                                        const double rotVector[3] , const double traVector[3],
                                        vector <Point3f> &pnts3d, vector <Point3f> &pntsToCam, vector <float> &_mean,
                                        float &meanerror, float &dis, string &err_str,
                                        const float &Xdis=70.432,const float &Ydis=70.432);
    //描述：标靶的特征图形
    //@param imgheight 输入 图片高
    //@param imgwidth 输入 图片宽
    //@param FeaSize 输入 特征尺寸
    //@param outimage 输出 输出特征图像
    //备注：
    static bool ShowFeaCircle(const int &imgheight, const int &imgwidth,const int &FeaSize, Mat &outimage);

	//描述：在图像上画实心圆
	//@param imgheight 输入/输出 所要画圆的图片
	//@param imgwidth 输入 圆心
	//@param FeaSize 输入 实心圆的半径
	//备注：
	static bool DrawCircle( Mat& inoutimage, const  Point2i & Center, const int & radius);

    //函数描述：机器人关节转角和关节脉冲比例关系标定函数
    //@param RT44 输入 机器人末端位姿
    //@param pose 输入 机器人关节脉冲
    //@param aixsflag 输入 要标定的轴编号
    //@param ratio 输出 标定结果
    //备注：
    static bool CalibrateAngerPulse(const vector <Mat>RT44,const vector<Vec6f> &pose,const int aixsflag,float &ratio);

	///遍历文件夹中图片
	//static bool ReadFileImg(CString filepath, CString fileFormat,vector<CString> &filename);//要加static的原因：非静态成员的访问必须以对象的形式进行访问

	///通过一张图片的绝对路径返回文件中下一张图片的路径
	///@param  CString &InOutFilePath 输入\输出 输入图片的绝对路径，输出下一张图片的绝对路径
	///note    图片的文件名需以"图片前缀+‘_’+图片序号"的形式存储，否者会出错
	//static bool GetNextImagePath(CString &InOutFilePath);

	///通过一张图片的绝对路径返回文件中上一张图片的路径
	///@param  CString &InOutFilePath 输入\输出 输入图片的绝对路径，输出上一张图片的绝对路径
	///note    图片的文件名需以"图片前缀+‘_’+图片序号"的形式存储，否者会出错
	//static  bool GetFormerImagePath(CString &InOutFilePath);

	///函数描述：空间点的圆拟合(无剔除离散点)
	//@param 
	//@param 
	//@param 
	//@param 
	//备注：
	static StereoEllipse fitStereoEllipse(vector<Point3f> points);

	///函数描述：空间点的圆拟合（剔除了离散点）
	//@param 
	//@param 
	//@param 
	//@param 
	//备注：
	static vector<Point3f> RansacMachine(vector<Point3f> SourcePoint);

	///函数描述：将相机坐标系下的三维点投影到图像坐标系上
	//@param 
	//@param 
	//@param 
	//@param 
	//备注：
	static void projectPoint3DInCamToImg(const vector<Point3f> &Point3DInCam, const CamPara &camParam, vector<Point2f>);


    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// 张召瑞 //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    /// 机器人手眼标定
    /// vector<Mat> ObjToCam, <in> 标定板到摄像机坐标变换
    /// vector<Mat> HandToBase, <in> 机器人手爪到机器人基座坐标变换
    /// vector<Mat>& Outputs, <out> 输出标定要求的变换矩阵
    /// bool type, 0为eye-in-hand模型，1为eye-to-hand模型
    static bool HandEyeCalibration(const vector<Mat> ObjToCamSrc, const vector<Mat> HandToBaseSrc, vector<Mat>& Outputs, bool type = false);

    /// 获得投影机对应性数据
    /// const vector<Mat>& Images, <in> 输入的一个位置姿态的多幅(12)图像
    /// const float Threshold, <in>
    /// const int N, <in> 歩数
    /// const vector<float> T, <in> 周期
    /// vector<Point2f>& pointCam, <out> 一个位置姿态的多幅(12)图像转化的点云
    /// vector<float>& proImg_x, <out> 投影机对应性数据,点云的X坐标
    static bool ObtainData4ProX(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Point2f>& pointCam, vector<float>& proImg_x);

    /// 得到单一标定板的投影仪坐标系的物理坐标X
    /// const vector<Mat>& Images， <in> 所有相机拍摄到的图像
    /// const float Threshold, <in>
    /// const int N, <in> 歩数
    /// const vector<float> T， <in> 周期
    /// Cor_X，<out> 所有图像上x的对应性
    /// MaskAll, <out> 所有条纹图像对应性的mask
    static bool GetCorrespondanceX(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Mat>& Cor_X, vector<Mat>& Mask_All);

    /// 保存成点云文件txt
    /// const string qfilename, <in> 要读取的文件名
    /// const vector<Point3f>& Points, <in> 点
    static bool WritePointsCloud(const string filename, const vector<Point3f>& Points, bool modeAdd = false);
    static bool WritePointsCloud(const string filename, const vector<Point2f>& Points, bool modeAdd = false);

    /// 读取点云文件txt
    /// const string qfilename, <in> 保存的文件名
    /// const vector<Point2f>& Points, <in> 点
    static bool ReadPointsCloud(const string filename, vector<Point3f> &Points);
    static bool ReadPointsCloud(const string filename, vector<Point2f>& Points);

    /// 获得投影机对应性数据
    /// const vector<Mat>& Images, <in> 输入的一个位置姿态的多幅(12)图像
    /// const float Threshold, <in>
    /// const int N, <in> 歩数
    /// const vector<float> T, <in> 周期
    /// vector<Point2f>& pointCam, <out> 一个位置姿态的多幅(12)图像转化的点云
    /// vector<Point2f>& pointPro, <out> 投影机对应性数据,点云的坐标
    static bool ObtainData4Pro(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Point2f>& pointCam, vector<Point2f>& pointPro);

    /// 得到单一标定板的投影仪坐标系的物理坐标(X,Y)
    /// const vector<Mat>& Images，<in> 所有相机拍摄到的图像
    /// const float Threshold, <in>
    /// const int N, <in> 歩数
    /// const vector<float> T， <in> 周期
    /// Cor_All，<out> 所有图像上x和y的对应性，在容器中的排序方式为第一个位姿对应性x,y，第二个x,y，第三个...以此类推
    /// MaskAll, <out> 所有条纹图像对应性的mask
    static bool GetCorrespondance(const vector<Mat>& Images, const float Threshold, const int N, const vector<float> T, vector<Mat>& Cor_X, vector<Mat>& Mask_All);

//    //输入已知世界坐标系三维特征点objectPoints
//    //输入图像坐标系下的二维图像特征点imagePoints
//    //输入相机内参数CameraIntrinsic[3][3],畸变参数DistortionCoeffs[4]
//    //输出测量坐标系下的各个特征点的三维坐标PointToCam
//    //输出世界坐标系相对于测量坐标系的旋转变量和平移变量rvec,tvec
//    //输出旋转变量的三倍标准差，平移向量三倍标准差dpdrot,dpdt
//    static void PnPMethod(vector<Point3f> objectPoints,
//                        vector<Point2f>imagePoints,
//                        Mat cameraMatrix, Mat distCoeffs,
//                        Mat &PoseR,Mat &PoseT,
//                        vector<double> &dpdrot,vector<double> &dpdt,vector<Point3f> &PointToCam);

    //椭圆亚像素级中心检测
    //亚像素圆斑检测
    //输出：输入图像的亚像素检测出的椭圆拟合出来的相关要素
    //输入：图像ROI
    //输入kenelsize必须是大于O的奇数
    //nocation:返回长轴为RotatedRect.width,短轴为RotatedRect.height
    //bool multi:是否为同心圆检测,默认false表示不调用同心圆检测函数MultiEllipseFitting()
    static bool findEllipses(const Mat img,const Rect mask,vector<RotatedRect>& findResults,const double precisionlevel,bool multi=false,int kenelsize=5);

    //输入同心圆的对偶椭圆拟合算子,暂时只能针对迭代标定,因为需要给出每个同心圆环的精确边界区域ROI
    //Input:  areaGrads    vector containing the horizontal and vertical image gradient
    //        areaPoses     corresponding coordinates of the image gradient in Ix,Iy
    //Output: dCs          the fitted dual ellipse in the form L'*dC*L = 0 where L is a line on the dual conic L = [a b c]'
    //        precisions   estimated uncertainty on the center of the dual ellipse (in pixel)
    //        angleIncertitudes   angle of the center uncertainty covariance ellipse
    static bool MultiEllipseFitting(vector<vector<cv::Point2d>>areaGrads,
                                    vector<vector<cv::Point2d>>areaPoses,
                                    vector<Mat>& dCs, vector<Mat>& precisions,
                                    vector<double> angleIncertitudes);

    //根据已经标定好的相机内部参数和畸变参数将圆环特征点二维坐标投影到平行视图上进行圆环匹配的特征点检测,从而得到更为精确的圆环特征点
    //input:  imgsPath 输入的圆环标定板图像路径
    //        cData输入的圆环中心点二维坐标和三维坐标
    //        camPara输入的摄像机参数
    //outout: cData输出的圆环中心点二维坐标和三维坐标
    static bool undistortRingCenter(const vector<string> &imgsPath, CalibrationData &cData, const CamPara& camPara);

    //求矩阵的秩
    static uint rankCompute(const Mat& M);

    //对Mat类型的数据按照某一列数据按大小进行排序  从小到大冒泡  注意只支持CV_8UC1 CV_32FC1 CV_64FC1
    //Input:  Matrix        待排序的Mat矩阵
    //        Col           按照该列数据排序
    //Output: Matrix        将输入的矩阵排序后的矩阵
    static bool SortInMatByCol(Mat& Matrix, const int Col);

    //计算两个点形成的直线与X轴正向形成的角度  逆时针为正
    static bool CalculateLineAngle(Point2f inputPnt1, Point2f inputPnt2, float& angleInDeg);

    //自动探测所有椭圆封闭轮廓  并输出其中心点  注意已排除了重复点
    static bool AutoDetectRingCenters(const Mat& srcImg, vector<Point2f>& centerPointsVec);

    //判断某一点是否在一系列点所组成的直线上  返回true表示在直线上
    static bool IsOnline(const vector<Point2f>& pntsLine, Point2f pnt);

    //对给定的一系列点去除孤立点
    static void RemoveIsolatePoints(vector<Point2f>& Pnts);

    //将有序点集按行或列排列  并输出所有行或列的点集
    //输入:  const Mat& PointsMat 按某个坐标排序的有序点集  其中每一行为一个点的各个坐标值
    //输出:  vector<Mat>& correctLinesVec 每行或每列的点集
    static bool SortPointsLineByLine(const Mat& PointsMat, vector<Mat>& correctLinesVec);

    //自动识别圆环中心点阵  并按一定顺序给出圆环中心点阵区域ROI的四个角点  要求所有图片最上一行外法线始终向上
    static bool AutoDetermineRingsROI(const Mat& srcImg, vector<Point2f>& mousePnts);

    //获取指定目录下所有指定扩展名的所有文件路径
    static void GetFiles(string path, string exd, std::vector<string>& files);


private:
    ///移除重复的中心点
    ///这里认为，一个位置一定会有两个较接近的中心坐标点  注意有错
    static void removeRepeatPoints( vector<Point2f>& points );

    /// 对含有畸变的图像像素坐标进行畸变校正，并求取在摄像机坐标下的 normalize 坐标
    /// const vector<Point2f>& src , <in> 输入图像点坐标
    /// vector<Point2f>& dst ,<out>  输出摄像机坐标下的normalized 坐标
    /// double fc[2] ,<in> 摄像机参数focal lengh
    /// double cc[2] ,<in> 摄像机参数主点位置
    /// double kc[5] ,<in> 摄像机畸变参数
    /// double alfpha_c  <in>  摄像机参数纵横比例因子
    static bool normalizPixel( const vector<Point2f>& src ,vector<Point2f>& dst ,double fc[2] ,double cc[2] ,double kc[5] ,double alfpha_c );

    static bool GetCornerAtRow(const Mat& ImageGray,Point2f imgpnt_left, Point2f imgpnt_right, vector<Point2f> &cornerrow);

    ///对圆环中心进行排序
    ///从左上角开始，按行排序
    static bool sortRingCenter(const vector<Point2f>& mousePnts,vector<Point2f>& center,int& row,int& col);

    ///判断pnt3是否在pnt1和pnt2的连线上  注意貌似也不对
    static bool isOnLine(const Point2f& pnt1,const Point2f& pnt2,const Point2f& pnt3);

    ///按照x坐标由小到大排序
    static void sortByXMin2Max(vector<Point2f>& pnts);

    ///按照y坐标由小到大排序
    static void sortByYMin2Max(vector<Point2f>& pnts);

    ///颠倒点的排序
    static void reverse(vector<Point2f>& pnts);

    ///返回两点间距离
    static float distance(const Point2f& pnt1,const Point2f& pnt2);

    ///计算平行视图标定板图像的R，3x3
    ///标定板坐标系建立在lefttop点，按行排列
    static void calculateParallelViewR(const vector<RT>& rt_vec, vector<Mat>& parallelR);

    ///生成平行视图相机的R
    ///标定板坐标系建立在lefttop点，按行排列
    static void generateParallelCamR(Mat& R);

    ///判断一个点是否在一个轮廓内,返回值大于0则在内部
    static double containsPnt(vector<Point2f> mousePnts, Point2f pnt);   

    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// 张召瑞 //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ///通过判断pnt3是否在pnt1和pnt2的连线上的方法  根据行或列信息建立点阵表型结构  按照Mat类型存储
    static Mat CreatTabByIsOnLine(const vector<Point2f>& Pnts);

    ///根据点阵的行表型结构信息  去除每行或每列上多余的边界点  以保证点阵刚好是m行n列的  即每列对应有m个点  每行对应有n个点
    ///暂时要求图片的最上一行以上和最底一行以下没有多余的点或行
    static bool RemoveRedundantPoints(vector<Point2f>& Pnts);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////// 程伟 ///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ///圆度评判，越小，圆度越好
    static float roundness(const Point2f& pnt1,const vector<Point>& circle,const float r);

    ///对圆环中心进行排序
    ///按照XY坐标，均从小到大排列，未考虑标定板颠倒的情况
    static bool sortRingCenter2(vector<Point2f>& center,const int row,const int col);

    //判断三个点中哪个点分别是2，6，7点
    //输出容器中一次存放这1，2，6，7点
    static bool findshortAxisPoints(const Point2f footpointNearPnts1,const Point2f footpointNearPnts2,const Point2f footpointNearPnts3,const Point2f footPoint,vector<TagPoint2f>& TagPoints);

    //对偶椭圆拟合算子
    //Input:  areaGrad    vector containing the horizontal and vertical image gradient
    //        areaPos     corresponding coordinates of the image gradient in Ix,Iy
    //Output: dC          the fitted dual ellipse in the form L'*dC*L = 0 where L is a line on the dual conic L = [a b c]'
    //        precision   estimated uncertainty on the center of the dual ellipse (in pixel)
    //        angleIncertitude   angle of the center uncertainty covariance ellipse
    static bool DualConicFitting(vector<cv::Point2d>areaGrad,vector<cv::Point2d>areaPos,Mat& dC,Mat& precision,double& angleIncertitude);

    //输入一般椭圆参数方程参数，求解标准椭圆参数
    static bool conicaEllipseTostd(const Mat ellipsePara,RotatedRect& result);

    //将三维坐标点转换到新的坐标系下
    //输入：三维坐标点集合，两坐标系之间的旋转矩阵，要求是3X3的矩阵，平移向量
    static void pointPosetranslation(const vector<Point3f> PointSrc,vector<Point3f>& PointDst,Mat R,Mat t);

    //用来求出图像中包含有特征的ROI区域
    //输入特征点图像坐标信息集合
    static void getFeatureROI(const vector<Point2f> pnts2fVec,cv::Rect& ROI,vector<Point2f>& pnts2fVecNew,double lengthOffset);

	///模板函数
	///进行三维重建前确认点的tag匹配性，并进行数据类型转换
	///进行刚体变换前确认点的tag匹配性，并进行数据类型转换
	static void pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
		vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2, vector<int>& tags);
	static void pntsTagConfirm(const vector<TagPoint3f>& pnts1, const vector<TagPoint3f>& pnts2,
		vector<Point3f>& out_pnts1, vector<Point3f>& out_pnts2);

	static void pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts);//chengwei added
	static void pntsTagConfirm(const vector<TagPoint3f>& pnts,vector<Point3f>& out_pnts,vector<int>& tags);//chengwei added

	static void pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
		vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2, vector<int>& tags);
	static void pntsTagConfirm(const vector<TagPoint2f>& pnts1, const vector<TagPoint2f>& pnts2,
		vector<Point2f>& out_pnts1, vector<Point2f>& out_pnts2);
	static void pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1,vector<int>& tags);//chengwei added
	static void pntsTagConfirm(const vector<TagPoint2f>& pnts1,vector<Point2f>& out_pnts1);//chengwei added
	//给没有标签信息的三维点集增加标签信息
	static void addPntsTag(const vector<vector<Point3f>> Pnts3fAll, const vector<int>& tags,vector<vector<TagPoint3f>> &TagPnts3fAll);//chengwei addded
	static void addPntsTag(const vector<Point3f> Pnts3fAll, const vector<int>& tags,vector<TagPoint3f> &TagPnts3fAll);//chengwei addded

    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// 郑泽龙 //////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    static bool creatCircleImg(Mat& img,int dx,float ratio);
};

