
//#include "stdafx.h"
#include "xmlwriter.h"

void XMLWriter::writeCamPara( std::string filename, const CamPara& camparameter )
{
    char text[100];
    char* segmentation=" ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );

    //创建内参数元素
    TiXmlElement *intrinsic_parameters_element = new TiXmlElement( "Intrinsic_parameters" );
    doc.LinkEndChild( intrinsic_parameters_element );

    TiXmlElement * fc_element = new TiXmlElement( "fc" );
    intrinsic_parameters_element->LinkEndChild(fc_element);

    TiXmlElement * cc_element = new TiXmlElement( "cc" );
    intrinsic_parameters_element->LinkEndChild(cc_element);

    sprintf(text, "%.9lf%s%.9lf", camparameter.CameraIntrinsic[0][0]
                            ,segmentation
                            ,camparameter.CameraIntrinsic[1][1]);

    TiXmlText * fc_text = new TiXmlText( text );
    fc_element->LinkEndChild( fc_text );

    sprintf(text, "%.9lf%s%.9lf", camparameter.CameraIntrinsic[0][2]
                            ,segmentation
                            ,camparameter.CameraIntrinsic[1][2]);

    TiXmlText * cc_text = new TiXmlText( text );
    cc_element->LinkEndChild( cc_text );

    //创建畸变参数元素
    TiXmlElement * Distortions_element = new TiXmlElement( "Distortions" );
    doc.LinkEndChild( Distortions_element );

    TiXmlElement * kc_element = new TiXmlElement( "kc" );
    Distortions_element->LinkEndChild(kc_element);

    sprintf(text, "%.9lf%s%.9lf%s%.9lf%s%.9lf", camparameter.DistortionCoeffs[0]
                            ,segmentation
                            ,camparameter.DistortionCoeffs[1]
                            ,segmentation
                            ,camparameter.DistortionCoeffs[2]
                            ,segmentation
                            ,camparameter.DistortionCoeffs[3]);

    TiXmlText * kc_text = new TiXmlText( text );
    kc_element->LinkEndChild( kc_text );

    //创建误差元素
    TiXmlElement * error_element = new TiXmlElement( "Error" );
    doc.LinkEndChild( error_element );

    //创建重投影误差元素
    TiXmlElement * pixel_norm_error_element = new TiXmlElement( "Repro_Norm_error" );
    error_element->LinkEndChild(pixel_norm_error_element);

    sprintf(text, "%lf", camparameter.totalReproNormErr);

    TiXmlText * pixel_norm_error_text = new TiXmlText( text );
    pixel_norm_error_element->LinkEndChild( pixel_norm_error_text );

    TiXmlElement * pixel_error_element = new TiXmlElement( "Repro_error" );
    error_element->LinkEndChild(pixel_error_element);

    sprintf(text, "%lf%s%lf", camparameter.ReprojectionError[0]
                        ,segmentation
                        ,camparameter.ReprojectionError[1]);

    TiXmlText * pixel_error_text = new TiXmlText( text );
    pixel_error_element->LinkEndChild( pixel_error_text );

    //创建内参数误差元素
    TiXmlElement * fc_error_element = new TiXmlElement( "fc_error" );
    error_element->LinkEndChild(fc_error_element);

    sprintf(text, "%lf%s%lf", camparameter.fcError[0]
                        ,segmentation
                        ,camparameter.fcError[1]);

    TiXmlText * fc_error_text = new TiXmlText( text );
    fc_error_element->LinkEndChild( fc_error_text );

    TiXmlElement * cc_error_element = new TiXmlElement( "cc_error" );
    error_element->LinkEndChild(cc_error_element);

    sprintf(text, "%lf%s%lf", camparameter.ccError[0]
                        ,segmentation
                        ,camparameter.ccError[1]);

    TiXmlText * cc_error_text = new TiXmlText( text );
    cc_error_element->LinkEndChild( cc_error_text );

    TiXmlElement * kc_error_element = new TiXmlElement( "kc_error" );
    error_element->LinkEndChild(kc_error_element);

    sprintf(text, "%lf%s%lf%s%lf%s%lf", camparameter.kcError[0]
                        ,segmentation,camparameter.kcError[1]
                        ,segmentation,camparameter.kcError[2]
                        ,segmentation,camparameter.kcError[3]);

    TiXmlText * kc_error_text = new TiXmlText( text );
    kc_error_element->LinkEndChild( kc_error_text );

    //创建标定板外参数
    writeImageRT(doc,camparameter);

    doc.SaveFile( filename.c_str() );
}
/*
void XMLWriter::writeCamGroupPara(std::string filename, const CamGroupPara& camGroupParameter)
{
    char text[100];
    char* segmentation = " ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );

    TiXmlElement *left_mid_element = new TiXmlElement( "Left_Mid" );
    doc.LinkEndChild( left_mid_element );
    TiXmlElement *right_mid_element = new TiXmlElement( "Right_Mid" );
    doc.LinkEndChild( right_mid_element );
    TiXmlElement *right_left_element = new TiXmlElement( "Right_Left" );
    doc.LinkEndChild( right_left_element );

    writeGroupRT(left_mid_element,camGroupParameter.left2MidRotVector,camGroupParameter.left2MidTraVector);

    writeGroupRT(right_mid_element,camGroupParameter.right2MidRotVector,camGroupParameter.right2MidTraVector);

    writeGroupRT(right_left_element,camGroupParameter.right2LeftRotVector,camGroupParameter.right2LeftTraVector);
    doc.SaveFile( filename.c_str() );
}
*/
void XMLWriter::writeCornerData( std::string filename, const CalibrationData& m_cornerData )
{
    char text[100];
    char* segmentation = " ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0","","" );
    doc.LinkEndChild( decl );

    //保存图像尺寸
    TiXmlElement *ImageSize_element = new TiXmlElement( "ImageSize" );
    doc.LinkEndChild( ImageSize_element );
    TiXmlElement *Image_Height_element = new TiXmlElement( "Height" );
    ImageSize_element->LinkEndChild( Image_Height_element );
    sprintf(text, "%d", m_cornerData.imgHeight);
    TiXmlText * Height_text = new TiXmlText( text );
    Image_Height_element->LinkEndChild( Height_text );

    TiXmlElement *Image_Width_element = new TiXmlElement( "Width" );
    ImageSize_element->LinkEndChild( Image_Width_element );

    sprintf(text, "%d", m_cornerData.imgWidth);
    TiXmlText * width_text = new TiXmlText( text );
    Image_Width_element->LinkEndChild( width_text );

    //保存角点二维数据
    TiXmlElement *Corner_Points_element = new TiXmlElement( "Corner_Points" );
    doc.LinkEndChild( Corner_Points_element );
    for(int i=0;i<m_cornerData.plane2dPntsVec.size();i++)
    {
        for (int j = 0; j < m_cornerData.plane2dPntsVec[i].size(); j++)
        {
            sprintf(text, "%s%d", "p" ,i);
            TiXmlElement * index_element = new TiXmlElement( text );
            Corner_Points_element->LinkEndChild(index_element);
            sprintf(text, "%f%s%f", m_cornerData.plane2dPntsVec[i][j].x
                                    ,segmentation
                                    ,m_cornerData.plane2dPntsVec[i][j].y);
            TiXmlText * width_text = new TiXmlText( text );
            index_element->LinkEndChild( width_text );
        }
    }

    //保存角点特征点
    TiXmlElement *Feature_Points_element = new TiXmlElement( "Feature_Points" );
    doc.LinkEndChild( Feature_Points_element );
    for(int i=0;i<m_cornerData.plane3dPntsVec.size();i++)
    {
        for (int j = 0; j < m_cornerData.plane3dPntsVec[i].size(); j++)
        {
            sprintf(text, "%s%d", "p" ,i);
            TiXmlElement * index_element = new TiXmlElement( text );
            Feature_Points_element->LinkEndChild(index_element);
            sprintf(text, "%.f%s%.f%s%.f", m_cornerData.plane3dPntsVec[i][j].x
                                ,segmentation,m_cornerData.plane3dPntsVec[i][j].y
                                ,segmentation,m_cornerData.plane3dPntsVec[i][j].z);
            TiXmlText * FeaturePoint_text = new TiXmlText( text );
            index_element->LinkEndChild( FeaturePoint_text );
        }
    }

    //保存每帧角点数
    TiXmlElement *CornerNumPerFrame_element = new TiXmlElement( "CornerNumPerFrame" );
    CornerNumPerFrame_element->SetAttribute("size",m_cornerData.plane2dPntsVec.size());
    doc.LinkEndChild( CornerNumPerFrame_element );
    for(int i=0;i<m_cornerData.plane2dPntsVec.size();i++)
    {
        sprintf(text, "%s%d", "n" ,i);
        TiXmlElement * index_element = new TiXmlElement( text );
        CornerNumPerFrame_element->LinkEndChild(index_element);

        sprintf(text, "%d", m_cornerData.plane2dPntsVec[i].size());
        TiXmlText * CornerNumPerFrame_text = new TiXmlText( text );
        index_element->LinkEndChild( CornerNumPerFrame_text );
    }

    //保存图像的索引
    TiXmlElement *FrameNumList_element = new TiXmlElement( "FrameNumList" );
    FrameNumList_element->SetAttribute("size",m_cornerData.frameNumList.size());
    doc.LinkEndChild( FrameNumList_element );
    for(int i=0;i<m_cornerData.frameNumList.size();i++)
    {
        sprintf(text, "%s%d", "n" ,i);
        TiXmlElement * index_element = new TiXmlElement( text );
        FrameNumList_element->LinkEndChild(index_element);

        sprintf(text, "%d", m_cornerData.frameNumList[i]);
        TiXmlText * FrameNumList_text = new TiXmlText( text );
        index_element->LinkEndChild( FrameNumList_text );
    }

    doc.SaveFile( filename.c_str() );
}

void XMLWriter::writePointsCloud( std::string filename, const std::vector<cv::Point3f>& points )
{
    char text[100];
    char* segmentation = " ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );

    TiXmlElement *Points_element = new TiXmlElement( "Points" );
    Points_element->SetAttribute("size",points.size());
    doc.LinkEndChild( Points_element );

    for(int i=0;i<points.size();i++)
    {
        sprintf(text, "%s%d", "p",i+1);
        TiXmlElement * index_element = new TiXmlElement( text );
        Points_element->LinkEndChild(index_element);

        sprintf(text, "%f%s%f%s%f", points[i].x,segmentation
                                    ,points[i].y,segmentation
                                    ,points[i].z,segmentation);
        TiXmlText *point_text = new TiXmlText( text );
        index_element->LinkEndChild( point_text );
    }

    doc.SaveFile( filename.c_str() );
}

void XMLWriter::writeImageRT( TiXmlDocument& doc,const CamPara& camparameter )
{
    char text[100];
    char* segmentation = " ";
    TiXmlElement * Image_RT_element = new TiXmlElement( "Image_RT" );
    Image_RT_element->SetAttribute("size",camparameter.imgRTVec.size());
    doc.LinkEndChild( Image_RT_element );

    for(int i=0;i<camparameter.imgRTVec.size();i++)
    {
        sprintf(text, "%s%d", "n" ,i);
        TiXmlElement * index_element = new TiXmlElement( text );
        Image_RT_element->LinkEndChild(index_element);

        TiXmlElement * R_element = new TiXmlElement( "R" );
        index_element->LinkEndChild(R_element);

        sprintf(text, "%f%s%f%s%f", camparameter.imgRTVec[i].R.at<double>(0,0)
                                ,segmentation
                                ,camparameter.imgRTVec[i].R.at<double>(1,0)
                                ,segmentation
                                ,camparameter.imgRTVec[i].R.at<double>(2,0));


        TiXmlText * R_text = new TiXmlText( text );
        R_element->LinkEndChild( R_text );

        TiXmlElement * T_element = new TiXmlElement( "T" );
        index_element->LinkEndChild(T_element);

        sprintf(text, "%f%s%f%s%f", camparameter.imgRTVec[i].T.at<double>(0,0)
                                ,segmentation
                                ,camparameter.imgRTVec[i].T.at<double>(1,0)
                                ,segmentation
                                ,camparameter.imgRTVec[i].T.at<double>(2,0));

        TiXmlText * T_text = new TiXmlText( text );
        T_element->LinkEndChild( T_text );

        //单张图片重投影误差
        TiXmlElement * Repro_Norm_error_element = new TiXmlElement( "Repro_Norm_error" );
        index_element->LinkEndChild(Repro_Norm_error_element);

        sprintf(text, "%f", camparameter.reprojectNormErr[i]);

        TiXmlText * Repro_Norm_error_text = new TiXmlText( text );
        Repro_Norm_error_element->LinkEndChild( Repro_Norm_error_text );

    }
}

void XMLWriter::writeGroupRT( TiXmlElement* element,const double RotVector[3],const double TraVector[3] )
{
    char text[100];
    char* segmentation = " ";
    sprintf(text, "%f%s%f%s%f", RotVector[0],segmentation
                                ,RotVector[1],segmentation
                                ,RotVector[2]);
    TiXmlElement * R_element = new TiXmlElement( "R" );
    element->LinkEndChild(R_element);
    TiXmlText * R_text = new TiXmlText( text );
    R_element->LinkEndChild( R_text );

    sprintf(text, "%f%s%f%s%f", TraVector[0],segmentation
                                ,TraVector[1],segmentation
                                ,TraVector[2]);
    TiXmlElement * T_element = new TiXmlElement( "T" );
    element->LinkEndChild(T_element);
    TiXmlText * T_text = new TiXmlText( text );
    T_element->LinkEndChild( T_text );
}

void XMLWriter::writeLightPenPara(std::string filename, const std::vector<Vec4f>& pnts,const Point3f probeCenter)
{
    char text[100];
    char* segmentation = " ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);

    TiXmlElement *Points_element = new TiXmlElement("Points_LightPenCoordinate");
    Points_element->SetAttribute("size", pnts.size());
    doc.LinkEndChild(Points_element);

    for (int i = 0; i < pnts.size(); i++)
    {
        sprintf(text, "p%d",pnts[i][0]);
        TiXmlElement * index_element = new TiXmlElement(text);
        Points_element->LinkEndChild(index_element);

        sprintf(text, "%f%s%f%s%f", pnts[i][1], segmentation
            , pnts[i][2], segmentation
            , pnts[i][3], segmentation);
        TiXmlText *point_text = new TiXmlText(text);
        index_element->LinkEndChild(point_text);
    }

    TiXmlElement *probeCenter_element = new TiXmlElement("probeCenter");
    sprintf(text, "_");
    TiXmlElement * _element = new TiXmlElement(text);
    probeCenter_element->LinkEndChild(_element);
    sprintf(text, "%f%s%f%s%f", probeCenter.x, segmentation
        , probeCenter.y, segmentation
        , probeCenter.z, segmentation);
    TiXmlText *point_text = new TiXmlText(text);
    _element->LinkEndChild(point_text);

    doc.LinkEndChild(probeCenter_element);

    doc.SaveFile(filename.c_str());
}

void XMLWriter::WriteCamProPara(std::string filename, const CamPara& camParameter, const CamPara& proParameter, const struct RT& camproRT)
{
    char text[100];
    char* segmentation=" ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );


    //保存相机内参数
    //创建内参数元素
    TiXmlElement * intrinsic_parameters_element = new TiXmlElement( "Intrinsic_Cam_Parameters" );
    doc.LinkEndChild( intrinsic_parameters_element );

    TiXmlElement * fc_element = new TiXmlElement( "fc" );
    intrinsic_parameters_element->LinkEndChild(fc_element);

    TiXmlElement * cc_element = new TiXmlElement( "cc" );
    intrinsic_parameters_element->LinkEndChild(cc_element);

    sprintf(text, "%.9lf%s%.9lf", camParameter.CameraIntrinsic[0][0]
                            ,segmentation
                            ,camParameter.CameraIntrinsic[1][1]);

    TiXmlText * fc_text = new TiXmlText( text );
    fc_element->LinkEndChild( fc_text );

    sprintf(text, "%.9lf%s%.9lf", camParameter.CameraIntrinsic[0][2]
                            ,segmentation
                            ,camParameter.CameraIntrinsic[1][2]);

    TiXmlText * cc_text = new TiXmlText( text );
    cc_element->LinkEndChild( cc_text );

    //创建畸变参数元素
    TiXmlElement * Distortions_element = new TiXmlElement( "Cam_Distortions" );
    doc.LinkEndChild( Distortions_element );

    TiXmlElement * kc_element = new TiXmlElement( "kc" );
    Distortions_element->LinkEndChild(kc_element);

    sprintf(text, "%.9lf%s%.9lf%s%.9lf%s%.9lf", camParameter.DistortionCoeffs[0]
                            ,segmentation
                            ,camParameter.DistortionCoeffs[1]
                            ,segmentation
                            ,camParameter.DistortionCoeffs[2]
                            ,segmentation
                            ,camParameter.DistortionCoeffs[3]);

    TiXmlText * kc_text = new TiXmlText( text );
    kc_element->LinkEndChild( kc_text );


    //保存投影机内参数
    //创建内参数元素
    TiXmlElement * intrinsic_parameters_element2 = new TiXmlElement( "Intrinsic_Pro_Parameters" );
    doc.LinkEndChild( intrinsic_parameters_element2 );

    TiXmlElement * fc_element2 = new TiXmlElement( "fc" );
    intrinsic_parameters_element2->LinkEndChild(fc_element2);

    TiXmlElement * cc_element2 = new TiXmlElement( "cc" );
    intrinsic_parameters_element2->LinkEndChild(cc_element2);

    sprintf(text, "%.9lf%s%.9lf", proParameter.CameraIntrinsic[0][0]
                            ,segmentation
                            ,proParameter.CameraIntrinsic[1][1]);

    TiXmlText * fc_text2 = new TiXmlText( text );
    fc_element2->LinkEndChild( fc_text2 );

    sprintf(text, "%.9lf%s%.9lf", proParameter.CameraIntrinsic[0][2]
                            ,segmentation
                            ,proParameter.CameraIntrinsic[1][2]);

    TiXmlText * cc_text2 = new TiXmlText( text );
    cc_element2->LinkEndChild( cc_text2 );

    //创建畸变参数元素
    TiXmlElement * Distortions_element2 = new TiXmlElement( "Pro_Distortions" );
    doc.LinkEndChild( Distortions_element2);

    TiXmlElement * kc_element2 = new TiXmlElement( "kc" );
    Distortions_element2->LinkEndChild(kc_element2);

    sprintf(text, "%.9lf%s%.9lf%s%.9lf%s%.9lf", proParameter.DistortionCoeffs[0]
                            ,segmentation
                            ,proParameter.DistortionCoeffs[1]
                            ,segmentation
                            ,proParameter.DistortionCoeffs[2]
                            ,segmentation
                            ,proParameter.DistortionCoeffs[3]);

    TiXmlText * kc_text2 = new TiXmlText( text );
    kc_element2->LinkEndChild( kc_text2 );


    //保存联合标定参数
    //创建元素
    TiXmlElement * stero_parameters_element = new TiXmlElement( "Stero_RT_Parameters" );
    doc.LinkEndChild( stero_parameters_element );

    TiXmlElement * R_element = new TiXmlElement( "R" );
    stero_parameters_element->LinkEndChild(R_element);

    TiXmlElement * T_element = new TiXmlElement( "T" );
    stero_parameters_element->LinkEndChild(T_element);

    sprintf(text, "%.9lf%s%.9lf%s%.9lf", camproRT.R.at<double>(0,0)
                            ,segmentation
                            ,camproRT.R.at<double>(1,0)
                            ,segmentation
                            ,camproRT.R.at<double>(2,0));

    TiXmlText * R_text = new TiXmlText( text );
    R_element->LinkEndChild( R_text );

    sprintf(text, "%.9lf%s%.9lf%s%.9lf", camproRT.T.at<double>(0,0)
                            ,segmentation
                            ,camproRT.T.at<double>(1,0)
                            ,segmentation
                            ,camproRT.T.at<double>(2,0));

    TiXmlText * T_text = new TiXmlText( text );
    T_element->LinkEndChild( T_text );

    doc.SaveFile( filename.c_str() );

    return;
}

void XMLWriter::WriteStereoPara(std::string filename, const CamPara& camLPara, const CamPara& camRPara, const struct RT& relativeRT,  const Size imgSize)
{
	char text[100];		//用于把文本写到文件中的临时字符串
	char* segmentation = " ";		//文本内容之间的分隔符
	TiXmlDocument doc;		//新建一个xml文本类
	TiXmlDeclaration *decl = new TiXmlDeclaration("1.0","","");		//xml文件第一行的声明部分，必须要有的
	doc.LinkEndChild(decl);		//把声明decl放到文本doc的末尾
	
	//*************插入左相机元素******************//

	TiXmlElement *Stero_Paras_root = new TiXmlElement("StereoParas");
	doc.LinkEndChild(Stero_Paras_root);	

	//先把xml文件的元素名称写好

	TiXmlElement *camLParas = new TiXmlElement("CamL_Intrinsic_Parameters");
	Stero_Paras_root->LinkEndChild(camLParas);		//左相机内参元素名称

	TiXmlElement *fc_element = new TiXmlElement("fc");
	camLParas->LinkEndChild(fc_element);		//fc元素

	TiXmlElement *cc_element = new TiXmlElement("cc");
	camLParas->LinkEndChild(cc_element);		//cc元素

	TiXmlElement *distL_element = new TiXmlElement("CamL_Distortion");
	Stero_Paras_root->LinkEndChild(distL_element);

	TiXmlElement *kc_element1 = new TiXmlElement("kc");
	distL_element->LinkEndChild(kc_element1);

	//对左相机元素添加内容
	sprintf(text,"%.9f%s%.9f",camLPara.CameraIntrinsic[0][0],segmentation,camLPara.CameraIntrinsic[1][1]);
	TiXmlText *fc_text1 = new TiXmlText(text);
	fc_element->LinkEndChild(fc_text1);		//CamL  fc

	sprintf(text,"%.9f%s%.9f",camLPara.CameraIntrinsic[0][2],segmentation,camLPara.CameraIntrinsic[1][2]);
	TiXmlText *cc_text1 = new TiXmlText(text);
	cc_element->LinkEndChild(cc_text1);		//CamL cc

	sprintf(text,"%.9f%s%.9f%s%.9f%s%.9f",camLPara.DistortionCoeffs[0],segmentation,
		camLPara.DistortionCoeffs[1],segmentation,camLPara.DistortionCoeffs[2],segmentation,camLPara.DistortionCoeffs[3]);
	TiXmlText *distL_text = new TiXmlText(text);
	kc_element1->LinkEndChild(distL_text);		//DistortionL


	//*************右相机元素************//
	TiXmlElement *camRParas = new TiXmlElement("CamR_Intrinsic_Parameters");
	Stero_Paras_root->LinkEndChild(camRParas);

	TiXmlElement *fc_element2 = new TiXmlElement("fc");
	camRParas->LinkEndChild(fc_element2);

	TiXmlElement *cc_element2 = new TiXmlElement("cc");
	camRParas->LinkEndChild(cc_element2);

	TiXmlElement *distR_element = new TiXmlElement("CamR_Distortion");
	Stero_Paras_root->LinkEndChild(distR_element);

	TiXmlElement *kc_element2 = new TiXmlElement("kc");
	distR_element->LinkEndChild(kc_element2);

	//对右相机元素添加内容
	sprintf(text, "%.9f%s%.9f", camRPara.CameraIntrinsic[0][0], segmentation, camRPara.CameraIntrinsic[1][1]);
	TiXmlText *fc_text2 = new TiXmlText(text);
	fc_element2->LinkEndChild(fc_text2);		//CamR  fc

	sprintf(text, "%.9f%s%.9f", camRPara.CameraIntrinsic[0][2], segmentation, camRPara.CameraIntrinsic[1][2]);
	TiXmlText *cc_text2 = new TiXmlText(text);
	cc_element2->LinkEndChild(cc_text2);		//CamR cc

	sprintf(text, "%.9f%s%.9f%s%.9f%s%.9f", camRPara.DistortionCoeffs[0], segmentation,
		camRPara.DistortionCoeffs[1], segmentation, camRPara.DistortionCoeffs[2], segmentation, camRPara.DistortionCoeffs[3]);
	TiXmlText *distR_text = new TiXmlText(text);
	kc_element2->LinkEndChild(distR_text);		//DistortionR

	//*****************插入两相机相对位姿元素**********************//
	TiXmlElement *RT_element = new TiXmlElement("RT_Parameter");
	Stero_Paras_root->LinkEndChild(RT_element);

	TiXmlElement *R_element = new TiXmlElement("R");
	RT_element->LinkEndChild(R_element);

	TiXmlElement *T_element = new TiXmlElement("T");
	RT_element->LinkEndChild(T_element);


	//添加RT内容
	sprintf(text,"%.9f%s%.9f%s%.9f",relativeRT.R.at<double>(0,0),segmentation,relativeRT.R.at<double>(1,0),segmentation,relativeRT.R.at<double>(2,0));
	TiXmlText *R_text = new TiXmlText(text);
	R_element->LinkEndChild(R_text);		//R


	//test
	//double x1 = relativeRT.T.at<double>(0, 0);
	//double x2 = relativeRT.T.at<double>(1, 0);
	//double x3 = relativeRT.T.at<double>(2, 0);
	//test

	sprintf(text, "%.9f%s%.9f%s%.9f", relativeRT.T.at<double>(0, 0), segmentation, relativeRT.T.at<double>(1, 0), segmentation, relativeRT.T.at<double>(2, 0));
	TiXmlText *T_text = new TiXmlText(text);
	T_element->LinkEndChild(T_text);		//T

	//*************添加图像尺寸元素******************//
	TiXmlElement* size_element = new TiXmlElement("Image_Size");
	Stero_Paras_root->LinkEndChild(size_element);

	//添加图像尺寸内容
	sprintf(text,"%d%s%d",imgSize.width,segmentation,imgSize.height);
	TiXmlText* size_text = new TiXmlText(text);
	size_element->LinkEndChild(size_text);

	doc.SaveFile(filename.c_str());
}

//保存两点坐标--------庄磊磊
void XMLWriter::WritePoints(std::string filename, vector<Point>& points)
{
	char text[1000];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	doc.LinkEndChild(decl);

	TiXmlElement * root_element = new TiXmlElement("Points");
	root_element->SetAttribute("size", points.size());
	doc.LinkEndChild(root_element);

	for (int i = 0; i < points.size(); ++i)
	{
		sprintf(text, "%s%d", "Point", i);
		TiXmlElement *index_element = new TiXmlElement(text);
		root_element->LinkEndChild(index_element);

		sprintf(text, "%d%s%d", points[i].x, segmentation, points[i].y);
		TiXmlText * P_text = new TiXmlText(text);
		index_element->LinkEndChild(P_text);
	}

	doc.SaveFile(filename.c_str());

}

void XMLWriter::WriteRT44(std::string filename,const vector<Mat>& Outputs)
{
    char text[1000];
    char* segmentation = " ";
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );

    TiXmlElement * Image_RT_element = new TiXmlElement( "MatRT44" );
    Image_RT_element->SetAttribute("size",Outputs.size());
    doc.LinkEndChild( Image_RT_element );

    for(int i=0;i<Outputs.size();i++)
    {
        sprintf(text, "%s%d", "n" ,i);
        TiXmlElement * index_element = new TiXmlElement( text );
        Image_RT_element->LinkEndChild(index_element);

        TiXmlElement * R_element = new TiXmlElement( "RT" );
        index_element->LinkEndChild(R_element);

        sprintf(text, "%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f", Outputs[i].at<double>(0,0)
                                ,segmentation
                                ,Outputs[i].at<double>(0,1)
                                ,segmentation
                                ,Outputs[i].at<double>(0,2)
                                ,segmentation
                                ,Outputs[i].at<double>(0,3)
                                ,segmentation
                                ,Outputs[i].at<double>(1,0)
                                ,segmentation
                                ,Outputs[i].at<double>(1,1)
                                ,segmentation
                                ,Outputs[i].at<double>(1,2)
                                ,segmentation
                                ,Outputs[i].at<double>(1,3)
                                ,segmentation
                                ,Outputs[i].at<double>(2,0)
                                ,segmentation
                                ,Outputs[i].at<double>(2,1)
                                ,segmentation
                                ,Outputs[i].at<double>(2,2)
                                ,segmentation
                                ,Outputs[i].at<double>(2,3)
                                ,segmentation
                                ,Outputs[i].at<double>(3,0)
                                ,segmentation
                                ,Outputs[i].at<double>(3,1)
                                ,segmentation
                                ,Outputs[i].at<double>(3,2)
                                ,segmentation
                                ,Outputs[i].at<double>(3,3));

        TiXmlText * R_text = new TiXmlText( text );
        R_element->LinkEndChild( R_text );

//        TiXmlElement * T_element = new TiXmlElement( "T" );
//        index_element->LinkEndChild(T_element);

//        sprintf(text, "%f%s%f%s%f", camparameter.imgRTVec[i].T.at<double>(0,0)
//                                ,segmentation
//                                ,camparameter.imgRTVec[i].T.at<double>(1,0)
//                                ,segmentation
//                                ,camparameter.imgRTVec[i].T.at<double>(2,0));

//        TiXmlText * T_text = new TiXmlText( text );
//        T_element->LinkEndChild( T_text );

    }

    doc.SaveFile( filename.c_str() );
}



#if 0
//*************************zzl add*****************************//
void XMLWriter::WriteObjData(std::string filename, const ObjectPara& ObjParam)
{
	char text[1000];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );
	//创建标靶特征点尺寸元素
	TiXmlElement * FacePnts = new TiXmlElement( "FacePnts" );
	doc.LinkEndChild( FacePnts );
	for (int i=0;i<5;i++)
	{
		sprintf(text, "%s%d", "Face_" ,i+1);
		TiXmlElement * Face_index = new TiXmlElement( text );
		FacePnts->LinkEndChild(Face_index);
		for (int j=0;j<7;j++)
		{
			sprintf(text, "%s%d", "Pnt_" ,j);
			TiXmlElement *Pnt_index = new TiXmlElement(text);
			Face_index->LinkEndChild(Pnt_index);
			sprintf(text,"%f%s%f%s%f",ObjParam.FacePnts[i][j].x,segmentation,
									ObjParam.FacePnts[i][j].y,segmentation,
									ObjParam.FacePnts[i][j].z);
			TiXmlText *Pnt = new TiXmlText(text);
			Pnt_index->LinkEndChild(Pnt);
		}
	}
	//创建标靶面相对位姿关系元素
	TiXmlElement * RT2one = new TiXmlElement( "RT2one" );
	doc.LinkEndChild( RT2one );
	for(int i=0;i<4;i++)
	{
		sprintf(text, "%s%d", "RT_" ,i);
		TiXmlElement * index_element = new TiXmlElement( text );
		RT2one->LinkEndChild(index_element);

		TiXmlElement * R_element = new TiXmlElement( "RT" );
		index_element->LinkEndChild(R_element);

		sprintf(text, "%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f", ObjParam.RT2one[i].at<double>(0,0)
			,segmentation
			,ObjParam.RT2one[i].at<double>(0,1)
			,segmentation
			,ObjParam.RT2one[i].at<double>(0,2)
			,segmentation
			,ObjParam.RT2one[i].at<double>(0,3)
			,segmentation
			,ObjParam.RT2one[i].at<double>(1,0)
			,segmentation
			,ObjParam.RT2one[i].at<double>(1,1)
			,segmentation
			,ObjParam.RT2one[i].at<double>(1,2)
			,segmentation
			,ObjParam.RT2one[i].at<double>(1,3)
			,segmentation
			,ObjParam.RT2one[i].at<double>(2,0)
			,segmentation
			,ObjParam.RT2one[i].at<double>(2,1)
			,segmentation
			,ObjParam.RT2one[i].at<double>(2,2)
			,segmentation
			,ObjParam.RT2one[i].at<double>(2,3)
			,segmentation
			,ObjParam.RT2one[i].at<double>(3,0)
			,segmentation
			,ObjParam.RT2one[i].at<double>(3,1)
			,segmentation
			,ObjParam.RT2one[i].at<double>(3,2)
			,segmentation
			,ObjParam.RT2one[i].at<double>(3,3));

		TiXmlText * R_text = new TiXmlText( text );
		R_element->LinkEndChild( R_text );
	}
	doc.SaveFile( filename.c_str() );
}

void XMLWriter::WriteRobotPluse(std::string filename, const vector<vector<double>> plu_pos)
{
	char text[1000];
	char* segmentation = " ";
	TiXmlDocument doc;
	TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
	doc.LinkEndChild( decl );
	//创建标靶特征点尺寸元素
	TiXmlElement * PlusPos = new TiXmlElement( "Pluse_Poso" );
	PlusPos->SetAttribute("size",plu_pos.size());
	doc.LinkEndChild( PlusPos );
	for (int i=0;i<plu_pos.size();i++)
	{
		sprintf(text, "%s%d", "POS_" ,i+1);
		TiXmlElement * POS_index = new TiXmlElement( text );
		PlusPos->LinkEndChild(POS_index);
		sprintf(text,"%f%s%f%s%f%s%f%s%f%s%f",
			plu_pos[i][0],segmentation,
			plu_pos[i][1],segmentation,
			plu_pos[i][2],segmentation,
			plu_pos[i][3],segmentation,
			plu_pos[i][4],segmentation,
			plu_pos[i][5],segmentation);
		TiXmlText *POS = new TiXmlText(text);
		POS_index->LinkEndChild(POS);
	}
	doc.SaveFile( filename.c_str() );
}

#endif