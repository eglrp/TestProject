//#include "stdafx.h"//在每个cpp文件中都要包含
#include "xmlreader.h"

bool XMLReader::readCamPara( std::string filename,CamPara& m_camPara )
{
    //加载文件
    TiXmlDocument doc;
    if( !doc.LoadFile( filename.c_str()) )
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);

    if( !parseIntrinsicparametersElement(hDoc,m_camPara) )
        return false;
    if( !parseDistortionsElement(hDoc,m_camPara) )
        return false;
    if( !parseImageRTElement(hDoc,m_camPara) )
        return false;

    return true;
}
/*
bool XMLReader::readCamGroupPara( std::string filename,CamGroupPara& m_camGroupPara )
{
    TiXmlDocument doc;
    if( !doc.LoadFile( filename.c_str()) )
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);

    if( !parseLeftMidElement(hDoc,m_camGroupPara) )
        return false;
    if( !parseRightMidElement(hDoc,m_camGroupPara) )
        return false;
    if (!parseRightLeftElement(hDoc, m_camGroupPara))
        return false;

    return true;
}
*/
bool XMLReader::readCornerData( std::string filename,CalibrationData& m_cornerData )
{
    TiXmlDocument doc;
    if( !doc.LoadFile( filename.c_str()) )
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);

    vector<int> CornerNumPerFrame;

    if( !parseImageSizeElement(hDoc,m_cornerData) )
        return false;
    if( !parseCornerNumPerFrameElement(hDoc,CornerNumPerFrame) )
        return false;
    if( !parseCornerPointsElement(hDoc,CornerNumPerFrame,m_cornerData) )
        return false;
    if( !parseFeaturePointsElement(hDoc,CornerNumPerFrame,m_cornerData) )
        return false;
    if( !parseFrameNumListElement(hDoc,m_cornerData) )
        return false;

    return true;
}

bool XMLReader::readPointsCloud( std::string filename,std::vector<cv::Point3f>& m_points )
{
    char* segmentation = " ";
    TiXmlDocument doc;
    if( !doc.LoadFile( filename.c_str()) )
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Points").Element();
    if( pElem->ValueTStr() == "Points" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        const char* jj = tElem->Value();
        for(tElem;tElem;tElem=tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float point[3];
            sscanf( nums[0].c_str(), "%f", &point[0] );
            sscanf( nums[1].c_str(), "%f", &point[1] );
            sscanf( nums[2].c_str(), "%f", &point[2] );
            m_points.push_back(cv::Point3f(point[0],point[1],point[2]));
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool XMLReader::parseIntrinsicparametersElement( TiXmlHandle hDoc, CamPara& m_camPara )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Intrinsic_parameters").Element();
    if( pElem->ValueTStr() == "Intrinsic_parameters" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("fc")->ToElement();
        if(  tElem->ValueTStr() == "fc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float fc0,fc1;
            sscanf( nums[0].c_str(), "%f", &fc0 );
            sscanf( nums[1].c_str(), "%f", &fc1 );
            m_camPara.CameraIntrinsic[0][0] = fc0;
            m_camPara.CameraIntrinsic[1][1] = fc1;
        }
        tElem = pElem->FirstChild("cc")->ToElement();
        if(  tElem->ValueTStr() == "cc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float cc0,cc1;
            sscanf( nums[0].c_str(), "%f", &cc0 );
            sscanf( nums[1].c_str(), "%f", &cc1 );
            m_camPara.CameraIntrinsic[0][2] = cc0;
            m_camPara.CameraIntrinsic[1][2] = cc1;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseDistortionsElement( TiXmlHandle hDoc, CamPara& m_camPara )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Distortions").Element();
    if( pElem->ValueTStr() == "Distortions" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("kc")->ToElement();
        if(  tElem->ValueTStr() == "kc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float kc[4];
            for(int i=0;i<4;i++)
            {
                sscanf( nums[i].c_str(), "%f", &kc[i] );
                m_camPara.DistortionCoeffs[i] = kc[i];
            }
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseImageRTElement(TiXmlHandle hDoc, CamPara& m_camPara)
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Image_RT").Element();
    if( pElem->ValueTStr() == "Image_RT" )
    {
        int num = pElem->FirstAttribute()->IntValue();
        TiXmlElement* tElem;
        tElem = pElem->FirstChild()->ToElement();
        for (int i = 0; i < num; i++)
        {
            RT rt; rt.R = Mat(3, 1, CV_64F); rt.T = Mat(3, 1, CV_64F);
            TiXmlElement* _tElem;
            _tElem = tElem->FirstChild()->ToElement();
            for(_tElem;_tElem;_tElem=_tElem->NextSiblingElement())
            {
                std::vector<std::string> nums = split( _tElem->GetText(),segmentation);
                float vec[3];
                for( int j=0;j<3;j++ )
                {
                    sscanf( nums[j].c_str(), "%f", &vec[j] );
                    if (_tElem->ValueTStr() == "R")
                        rt.R.at<double>(j, 0) = vec[j];
                    else if (_tElem->ValueTStr() == "T")
                        rt.T.at<double>(j, 0) = vec[j];
                    else
                        break;
                }
            }
            m_camPara.imgRTVec.push_back(rt);
            tElem = tElem->NextSiblingElement();
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseErrorElement( TiXmlHandle hDoc, CamPara& m_camPara )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Error").Element();
    std::string error_tag_string[4];
    error_tag_string[0]="pixel_error";error_tag_string[1]="fc_error";
    error_tag_string[2]="cc_error";error_tag_string[3]="kc_error";
    if( pElem->ValueTStr() == "Error" )
    {
        for(int i=0;i<4;i++)
        {
            TiXmlElement* tElem;
            tElem = pElem->FirstChild(error_tag_string[i].c_str())->ToElement();
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            int n=0;

            //判断是否为kc_error,因为它有4个参数，其他均两个参数
            if( tElem->ValueTStr() == "kc_error" ) n=4;
            else n=2;

            float error[5];
            for(int j=0;j<n;j++)
            {
                sscanf( nums[j].c_str(), "%f", &error[j] );
            }

            if(  tElem->ValueTStr() == "pixel_error" )
            {
                m_camPara.ReprojectionError[0] = error[0];m_camPara.ReprojectionError[1] = error[1];
            }
            else if( tElem->ValueTStr() == "fc_error" )
            {
                m_camPara.fcError[0]=error[0];m_camPara.fcError[1]=error[1];
            }
            else if( tElem->ValueTStr() == "cc_error" )
            {
                m_camPara.ccError[0]=error[0];m_camPara.ccError[1]=error[1];
            }
            else if( tElem->ValueTStr() == "kc_error" )
            {
                for(int j=0;j<4;j++)
                    m_camPara.kcError[j]=error[j];
            }
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool XMLReader::parseImageSizeElement( TiXmlHandle hDoc, CalibrationData& m_cornerData )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("ImageSize").Element();
    if( pElem->ValueTStr() == "ImageSize" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("Height")->ToElement();
        if(  tElem->ValueTStr() == "Height" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float height;
            sscanf( nums[0].c_str(), "%f", &height );
            m_cornerData.imgHeight = height;
        }

        tElem = pElem->FirstChild("Width")->ToElement();
        if(  tElem->ValueTStr() == "Width" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float width;
            sscanf( nums[0].c_str(), "%f", &width );
            m_cornerData.imgWidth = width;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseCornerPointsElement( TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame, CalibrationData& m_cornerData )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Corner_Points").Element();
    if( pElem->ValueTStr() == "Corner_Points" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        const char* jj = tElem->Value();
        for (int i = 0; i < CornerNumPerFrame.size();i++)
        {
            vector<Point2f> pnts2d;
            for (int j = 0; j<CornerNumPerFrame[i]; tElem = tElem->NextSiblingElement(),j++)
            {
                vector<string> nums = split( tElem->GetText(),segmentation);
                float point[2];
                sscanf( nums[0].c_str(), "%f", &point[0] );
                sscanf( nums[1].c_str(), "%f", &point[1] );
                pnts2d.push_back(cv::Point2f(point[0],point[1]));
            }
            m_cornerData.plane2dPntsVec.push_back(pnts2d);
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool XMLReader::parseFeaturePointsElement( TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame, CalibrationData& m_cornerData )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Feature_Points").Element();
    if( pElem->ValueTStr() == "Feature_Points" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        for (int i = 0; i < CornerNumPerFrame.size();i++)
        {
            vector<Point3f> pnts3d;
            for (int j = 0; j<CornerNumPerFrame[i]; tElem = tElem->NextSiblingElement(),j++)
            {
                vector<string> nums = split( tElem->GetText(),segmentation);
                float point[3];
                sscanf( nums[0].c_str(), "%f", &point[0] );
                sscanf( nums[1].c_str(), "%f", &point[1] );
                sscanf( nums[2].c_str(), "%f", &point[2] );
                pnts3d.push_back(Point3f(point[0],point[1],point[2]));
            }
            m_cornerData.plane3dPntsVec.push_back(pnts3d);
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool XMLReader::parseCornerNumPerFrameElement( TiXmlHandle hDoc, vector<int>& CornerNumPerFrame )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("CornerNumPerFrame").Element();
    if( pElem->ValueTStr() == "CornerNumPerFrame" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        const char* jj = tElem->Value();
        for(tElem;tElem;tElem=tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            int num;
            sscanf( nums[0].c_str(), "%d", &num );
            CornerNumPerFrame.push_back(num);
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool XMLReader::parseFrameNumListElement( TiXmlHandle hDoc, CalibrationData& m_cornerData )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("FrameNumList").Element();
    int n = pElem->FirstAttribute()->IntValue();
    if( pElem->ValueTStr() == "FrameNumList" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        const char* jj = tElem->Value();
        for(tElem;tElem;tElem=tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            int index;
            sscanf( nums[0].c_str(), "%d", &index );
            m_cornerData.frameNumList.push_back(index);
        }
    }
    else
    {
        return false;
    }
    return true;
}
/*
bool XMLReader::parseLeftMidElement( TiXmlHandle hDoc, CamGroupPara& m_camGroupPara )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Left_Mid").Element();
    if( pElem->ValueTStr() == "Left_Mid" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        std::vector<std::string> nums = split( tElem->GetText(),segmentation);
        for(tElem;tElem;tElem=tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float vec[3];
            for( int j=0;j<3;j++ )
            {
                sscanf( nums[j].c_str(), "%f", &vec[j] );
                if( tElem->ValueTStr() == "R")
                    m_camGroupPara.left2MidRotVector[j] = vec[j];
                else if( tElem->ValueTStr() == "T")
                    m_camGroupPara.left2MidTraVector[j] = vec[j];
            }
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseRightMidElement( TiXmlHandle hDoc, CamGroupPara& m_camGroupPara )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Right_Mid").Element();
    if( pElem->ValueTStr() == "Right_Mid" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        std::vector<std::string> nums = split( tElem->GetText(),segmentation);
        for(tElem;tElem;tElem=tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float vec[3];
            for( int j=0;j<3;j++ )
            {
                sscanf( nums[j].c_str(), "%f", &vec[j] );
                if( tElem->ValueTStr() == "R")
                    m_camGroupPara.right2MidRotVector[j] = vec[j];
                else if( tElem->ValueTStr() == "T")
                    m_camGroupPara.right2MidTraVector[j] = vec[j];
            }
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseRightLeftElement( TiXmlHandle hDoc, CamGroupPara& m_camGroupPara )
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Right_Left").Element();
    if( pElem->ValueTStr() == "Right_Left" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        std::vector<std::string> nums = split( tElem->GetText(),segmentation);
        for(tElem;tElem;tElem=tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float vec[3];
            for( int j=0;j<3;j++ )
            {
                sscanf( nums[j].c_str(), "%f", &vec[j] );
                if( tElem->ValueTStr() == "R")
                    m_camGroupPara.right2LeftRotVector[j] = vec[j];
                else if( tElem->ValueTStr() == "T")
                    m_camGroupPara.right2LeftTraVector[j] = vec[j];
            }
        }
    }
    else
    {
        return false;
    }

    return true;
}
*/
bool XMLReader::readLightPenPara(std::string filename, std::vector<TagPoint3f>& pnts, cv::Point3f& pnt)
{
    char* segmentation = " ";
    TiXmlDocument doc;
    if (!doc.LoadFile(filename.c_str()))
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);

    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Points_LightPenCoordinate").Element();
    if (pElem->ValueTStr() == "Points_LightPenCoordinate")
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        std::vector<std::string> nums = split(tElem->GetText(), segmentation);
        for (tElem; tElem; tElem = tElem->NextSiblingElement())
        {
            std::vector<std::string> nums = split(tElem->GetText(), segmentation);
            float vec[3];
            for (int j = 0; j < 3; j++)
            {
                sscanf(nums[j].c_str(), "%f", &vec[j]);
            }
            TagPoint3f _pnt; int _tag;
            string str(tElem->Value());
            char o = str.at(1);
            sscanf(&o, "%d",&_tag);
            _pnt[0] = _tag;
            _pnt[1] = vec[0];
            _pnt[2] = vec[1];
            _pnt[3] = vec[2];
            pnts.push_back(_pnt);
        }
    }
    else
    {
        return false;
    }

    pElem = hDoc.FirstChild("probeCenter").Element();
    if (pElem->ValueTStr() == "probeCenter")
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChildElement();
        std::vector<std::string> nums = split(tElem->GetText(), segmentation);
        float vec[3];
        for (int j = 0; j < 3; j++)
        {
            sscanf(nums[j].c_str(), "%f", &vec[j]);
        }
        pnt = cv::Point3f(vec[0], vec[1], vec[2]);
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::ReadCamProPara(std::string filename, CamPara& camPara, CamPara& proPara, RT& camproRT)
{
    //加载文件
    TiXmlDocument doc;
    if( !doc.LoadFile( filename.c_str()) )
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);

    if( !parseIntrinsicCamParametersElement(hDoc,camPara) )
        return false;
    if( !parseCamDistortionsElement(hDoc,camPara) )
        return false;
    if( !parseIntrinsicProParametersElement(hDoc,proPara) )
        return false;
    if( !parseProDistortionsElement(hDoc,proPara) )
        return false;
    if( !parsesteroRTElement(hDoc,camproRT) )
        return false;

    return true;
}

bool XMLReader::parseIntrinsicCamParametersElement( TiXmlHandle hDoc, CamPara& camPara)
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Intrinsic_Cam_Parameters").Element();
    if( pElem->ValueTStr() == "Intrinsic_Cam_Parameters" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("fc")->ToElement();
        if(  tElem->ValueTStr() == "fc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float fc0,fc1;
            sscanf( nums[0].c_str(), "%f", &fc0 );
            sscanf( nums[1].c_str(), "%f", &fc1 );
            camPara.CameraIntrinsic[0][0] = fc0;
            camPara.CameraIntrinsic[1][1] = fc1;
        }
        tElem = pElem->FirstChild("cc")->ToElement();
        if(  tElem->ValueTStr() == "cc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float cc0,cc1;
            sscanf( nums[0].c_str(), "%f", &cc0 );
            sscanf( nums[1].c_str(), "%f", &cc1 );
            camPara.CameraIntrinsic[0][2] = cc0;
            camPara.CameraIntrinsic[1][2] = cc1;
        }
    }
    else
    {
        return false;
    }

    return true;
}



//***********************************庄磊磊********************************//
bool XMLReader::parsePoints(TiXmlHandle hDoc, vector<Point>& points)
{
	char* segmentation = " ";

	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("Points").Element();
	if (rElem->ValueTStr() == "Points")
	{

		TiXmlElement* pElem;
		pElem = rElem->FirstChild("Point0")->ToElement();
		if (pElem->ValueTStr() == "Point0")
		{
			std::vector<std::string> nums = split(pElem->GetText(), segmentation);
			Point pnt1;
			sscanf(nums[0].c_str(), "%d", &pnt1.x);
			sscanf(nums[1].c_str(), "%d", &pnt1.y);
			points.push_back(pnt1);
		}
		pElem = rElem->FirstChild("Point1")->ToElement();
		if (pElem->ValueTStr() == "Point1")
		{
			std::vector<std::string> nums = split(pElem->GetText(), segmentation);
			Point pnt2;
			sscanf(nums[0].c_str(), "%d", &pnt2.x);
			sscanf(nums[1].c_str(), "%d", &pnt2.y);
			points.push_back(pnt2);
		}
	    else
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

bool XMLReader::parseCamLIntrinsicparametersElement(TiXmlHandle hDoc, CamPara& camPara)
{
	char* segmentation = " ";
	
	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("StereoParas").Element();
	if (rElem->ValueTStr() == "StereoParas")
	{

		TiXmlElement* pElem;
		pElem = rElem->FirstChild("CamL_Intrinsic_Parameters")->ToElement();
		if (pElem->ValueTStr() == "CamL_Intrinsic_Parameters")
		{
			TiXmlElement* tElem;
			tElem = pElem->FirstChild("fc")->ToElement();
			if (tElem->ValueTStr() == "fc")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float fc0, fc1;
				sscanf(nums[0].c_str(), "%f", &fc0);
				sscanf(nums[1].c_str(), "%f", &fc1);
				camPara.CameraIntrinsic[0][0] = fc0;
				camPara.CameraIntrinsic[1][1] = fc1;
			}
			tElem = pElem->FirstChild("cc")->ToElement();
			if (tElem->ValueTStr() == "cc")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float cc0, cc1;
				sscanf(nums[0].c_str(), "%f", &cc0);
				sscanf(nums[1].c_str(), "%f", &cc1);
				camPara.CameraIntrinsic[0][2] = cc0;
				camPara.CameraIntrinsic[1][2] = cc1;
			}
		}
		else
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

bool XMLReader::parseCamRIntrinsicparametersElement(TiXmlHandle hDoc, CamPara& camPara)
{
	char* segmentation = " ";

	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("StereoParas").Element();
	if (rElem->ValueTStr() == "StereoParas")
	{
		TiXmlElement* pElem;
		pElem = rElem->FirstChild("CamR_Intrinsic_Parameters")->ToElement();
		//pElem = hDoc.FirstChildElement("CamR_Intrinsic_Parameters").Element();

		if (pElem->ValueTStr() == "CamR_Intrinsic_Parameters")
		{
			TiXmlElement* tElem;
			tElem = pElem->FirstChild("fc")->ToElement();
			if (tElem->ValueTStr() == "fc")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float fc0, fc1;
				sscanf(nums[0].c_str(), "%f", &fc0);
				sscanf(nums[1].c_str(), "%f", &fc1);
				camPara.CameraIntrinsic[0][0] = fc0;
				camPara.CameraIntrinsic[1][1] = fc1;
			}
			tElem = pElem->FirstChild("cc")->ToElement();
			if (tElem->ValueTStr() == "cc")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float cc0, cc1;
				sscanf(nums[0].c_str(), "%f", &cc0);
				sscanf(nums[1].c_str(), "%f", &cc1);
				camPara.CameraIntrinsic[0][2] = cc0;
				camPara.CameraIntrinsic[1][2] = cc1;
			}
		}
		else
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

bool XMLReader::parseCamLDistortionsElement(TiXmlHandle hDoc, CamPara& camPara)
{
    char* segmentation = " ";
	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("StereoParas").Element();
	if (rElem->ValueTStr() == "StereoParas")
	{

		TiXmlElement* pElem;
		pElem = rElem->FirstChild("CamL_Distortion")->ToElement();
		if (pElem->ValueTStr() == "CamL_Distortion")
		{
			TiXmlElement* tElem;
			tElem = pElem->FirstChild("kc")->ToElement();
			if (tElem->ValueTStr() == "kc")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float kc[4];
				for (int i = 0; i < 4; i++)
				{
					sscanf(nums[i].c_str(), "%f", &kc[i]);
					camPara.DistortionCoeffs[i] = kc[i];
				}
			}
		}
		else
		{
			return false;
		}
	}

    return true;
}

bool XMLReader::parseCamRDistortionsElement(TiXmlHandle hDoc, CamPara& camPara)
{
	char* segmentation = " ";
	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("StereoParas").Element();
	if (rElem->ValueTStr() == "StereoParas")
	{
		TiXmlElement* pElem;
		pElem = rElem->FirstChild("CamR_Distortion")->ToElement();
		if (pElem->ValueTStr() == "CamR_Distortion")
		{
			TiXmlElement* tElem;
			tElem = pElem->FirstChild("kc")->ToElement();
			if (tElem->ValueTStr() == "kc")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float kc[4];
				for (int i = 0; i < 4; i++)
				{
					sscanf(nums[i].c_str(), "%f", &kc[i]);
					camPara.DistortionCoeffs[i] = kc[i];
				}
			}
		}
		else
		{
			return false;
		}
	}

	return true;
}

bool XMLReader::parseRTElement(TiXmlHandle hDoc, struct RT& steroRTPara)
{
	char* segmentation = " ";
	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("StereoParas").Element();
	if (rElem->ValueTStr() == "StereoParas")
	{
		TiXmlElement* pElem;
		pElem = rElem->FirstChild("RT_Parameter")->ToElement();
		if (pElem->ValueTStr() == "RT_Parameter")
		{
			TiXmlElement* tElem;
			tElem = pElem->FirstChild("R")->ToElement();
			if (tElem->ValueTStr() == "R")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float r0, r1, r2;
				steroRTPara.R.create(3, 1, CV_64FC1);
				sscanf(nums[0].c_str(), "%f", &r0);
				sscanf(nums[1].c_str(), "%f", &r1);
				sscanf(nums[2].c_str(), "%f", &r2);
				steroRTPara.R.at<double>(0, 0) = r0;
				steroRTPara.R.at<double>(1, 0) = r1;
				steroRTPara.R.at<double>(2, 0) = r2;
			}
			tElem = pElem->FirstChild("T")->ToElement();
			if (tElem->ValueTStr() == "T")
			{
				std::vector<std::string> nums = split(tElem->GetText(), segmentation);
				float t0, t1, t2;
				steroRTPara.T.create(3, 1, CV_64FC1);
				sscanf(nums[0].c_str(), "%f", &t0);
				sscanf(nums[1].c_str(), "%f", &t1);
				sscanf(nums[2].c_str(), "%f", &t2);
				steroRTPara.T.at<double>(0, 0) = t0;
				steroRTPara.T.at<double>(1, 0) = t1;
				steroRTPara.T.at<double>(2, 0) = t2;
			}
		}
		else
		{
			return false;
		}
	}

	return true;
}

bool XMLReader::parseStereoParaSize(TiXmlHandle hDoc, Size& imgSize)
{
	char* segmentation = " ";
	TiXmlElement* rElem;
	rElem = hDoc.FirstChild("StereoParas").Element();
	if (rElem->ValueTStr()=="StereoParas")
	{
		TiXmlElement* pElem;
		pElem = rElem->FirstChild("Image_Size")->ToElement();
		if (pElem->ValueTStr()=="Image_Size")
		{
			std::vector<std::string> nums = split(pElem->GetText(), segmentation);
			int width, height;
			sscanf(nums[0].c_str(), "%d", &width);
			sscanf(nums[1].c_str(), "%d", &height);
			imgSize.width = width;
			imgSize.height = height;
		}
		else
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

bool XMLReader::ReadStereoPara(std::string filename, CamPara& camLPara, CamPara& camRPara, struct RT& relativeRT,Size& imgSize)
{
	//加载文件
	TiXmlDocument doc;
	if (!doc.LoadFile(filename.c_str()))
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	if (!parseCamLIntrinsicparametersElement(hDoc, camLPara))
	{
		return false;
	}

	if (!parseCamRIntrinsicparametersElement(hDoc, camRPara))
	{
		return false;
	}

	if (!parseCamLDistortionsElement(hDoc, camLPara))
	{
		return false;
	}

	if (!parseCamRDistortionsElement(hDoc, camRPara))
	{
		return false;
	}

	if (!parseRTElement(hDoc, relativeRT))
	{
		return false;
	}

	if (!parseStereoParaSize(hDoc,imgSize))
	{
		return false;
	}

	return true;
}

bool XMLReader::ReadPoints(std::string filename, vector<Point>& points)
{
	//加载文件
	TiXmlDocument doc;
	if (!doc.LoadFile(filename.c_str()))
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	if (!parsePoints(hDoc, points))
	{
		return false;
	}
	return true;
}

//*********************蔡永凯********************//
bool XMLReader::ReadSettingPara(std::string filename, laserScannerSetting& setParam)
{
	TiXmlDocument doc;
	if (!doc.LoadFile(filename.c_str()))
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	if (!parseSettingParaElement(hDoc, setParam))
		return false;
	

	return true;
}

//*********************庄磊磊********************//


bool XMLReader::parseCamDistortionsElement(TiXmlHandle hDoc, CamPara& camPara)
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Cam_Distortions").Element();
	if (pElem->ValueTStr() == "Cam_Distortions")
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild("kc")->ToElement();
		if (tElem->ValueTStr() == "kc")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			float kc[4];
			for (int i = 0; i < 4; i++)
			{
				sscanf(nums[i].c_str(), "%f", &kc[i]);
				camPara.DistortionCoeffs[i] = kc[i];
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}


bool XMLReader::parseIntrinsicProParametersElement( TiXmlHandle hDoc, CamPara& proPara)
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Intrinsic_Pro_Parameters").Element();
    if( pElem->ValueTStr() == "Intrinsic_Pro_Parameters" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("fc")->ToElement();
        if(  tElem->ValueTStr() == "fc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float fc0,fc1;
            sscanf( nums[0].c_str(), "%f", &fc0 );
            sscanf( nums[1].c_str(), "%f", &fc1 );
            proPara.CameraIntrinsic[0][0] = fc0;
            proPara.CameraIntrinsic[1][1] = fc1;
        }
        tElem = pElem->FirstChild("cc")->ToElement();
        if(  tElem->ValueTStr() == "cc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float cc0,cc1;
            sscanf( nums[0].c_str(), "%f", &cc0 );
            sscanf( nums[1].c_str(), "%f", &cc1 );
            proPara.CameraIntrinsic[0][2] = cc0;
            proPara.CameraIntrinsic[1][2] = cc1;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parseProDistortionsElement(TiXmlHandle hDoc, CamPara& proPara)
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Pro_Distortions").Element();
    if( pElem->ValueTStr() == "Pro_Distortions" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("kc")->ToElement();
        if(  tElem->ValueTStr() == "kc" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float kc[4];
            for(int i=0;i<4;i++)
            {
                sscanf( nums[i].c_str(), "%f", &kc[i] );
                proPara.DistortionCoeffs[i] = kc[i];
            }
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::parsesteroRTElement(TiXmlHandle hDoc, struct RT& steroRTPara)
{
    char* segmentation = " ";
    TiXmlElement* pElem;
    pElem = hDoc.FirstChild("Stero_RT_Parameters").Element();
    if( pElem->ValueTStr() == "Stero_RT_Parameters" )
    {
        TiXmlElement* tElem;
        tElem = pElem->FirstChild("R")->ToElement();
        if(  tElem->ValueTStr() == "R" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float r0,r1,r2;
            steroRTPara.R.create(3,1,CV_64FC1);
            sscanf( nums[0].c_str(), "%f", &r0 );
            sscanf( nums[1].c_str(), "%f", &r1 );
            sscanf( nums[2].c_str(), "%f", &r2 );
            steroRTPara.R.at<double>(0,0) = r0;
            steroRTPara.R.at<double>(1,0) = r1;
            steroRTPara.R.at<double>(2,0) = r2;
        }
        tElem = pElem->FirstChild("T")->ToElement();
        if(  tElem->ValueTStr() == "T" )
        {
            std::vector<std::string> nums = split( tElem->GetText(),segmentation);
            float t0,t1,t2;
            steroRTPara.T.create(3,1,CV_64FC1);
            sscanf( nums[0].c_str(), "%f", &t0 );
            sscanf( nums[1].c_str(), "%f", &t1 );
            sscanf( nums[2].c_str(), "%f", &t2 );
            steroRTPara.T.at<double>(0,0) = t0;
            steroRTPara.T.at<double>(1,0) = t1;
            steroRTPara.T.at<double>(2,0) = t2;
        }
    }
    else
    {
        return false;
    }

    return true;
}

//*********************蔡永凯********************//
bool XMLReader::parseSettingParaElement(TiXmlHandle hDoc, laserScannerSetting& setParam)
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("SettingPara").Element();
	if (pElem->ValueTStr() == "SettingPara")
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild("exposureTime")->ToElement();
		if (tElem->ValueTStr() == "exposureTime")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			double exposureTime;
			sscanf(nums[0].c_str(), "%lf", &exposureTime);
			setParam.exposureTime = exposureTime;
		}

		tElem = pElem->FirstChild("recontructionTimeOut")->ToElement();
		if (tElem->ValueTStr() == "recontructionTimeOut")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			int recontructionTimeOut;
			sscanf(nums[0].c_str(), "%d", &recontructionTimeOut);
			setParam.recontructionTimeOut = recontructionTimeOut;
		}

		tElem = pElem->FirstChild("FrameRate")->ToElement();
		if (tElem->ValueTStr() == "FrameRate")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			int FrameRate;
			sscanf(nums[0].c_str(), "%d", &FrameRate);
			setParam.FrameRate = FrameRate;
		}

		tElem = pElem->FirstChild("captureTime")->ToElement();
		if (tElem->ValueTStr() == "captureTime")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			double captureTime;
			sscanf(nums[0].c_str(), "%lf", &captureTime);
			setParam.captureTime = captureTime;
		}

		tElem = pElem->FirstChild("moveSpeed")->ToElement();
		if (tElem->ValueTStr() == "moveSpeed")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			double moveSpeed;
			sscanf(nums[0].c_str(), "%lf", &moveSpeed);
			setParam.moveSpeed = moveSpeed;
		}

		tElem = pElem->FirstChild("encoderSpace")->ToElement();
		if (tElem->ValueTStr() == "encoderSpace")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			int encoderSpace;
			sscanf(nums[0].c_str(), "%d", &encoderSpace);
			setParam.encoderSpace = encoderSpace;
		}

		tElem = pElem->FirstChild("stepDis")->ToElement();
		if (tElem->ValueTStr() == "stepDis")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			double stepDis;
			sscanf(nums[0].c_str(), "%lf", &stepDis);
			setParam.stepDis = stepDis;
		}

		tElem = pElem->FirstChild("triggerNum")->ToElement();
		if (tElem->ValueTStr() == "triggerNum")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			int triggerNum;
			sscanf(nums[0].c_str(), "%d", &triggerNum);
			setParam.triggerNum = triggerNum;
		}

		tElem = pElem->FirstChild("startPositon")->ToElement();
		if (tElem->ValueTStr() == "startPositon")
		{
			std::vector<std::string> nums = split(tElem->GetText(), segmentation);
			int startPositon;
			sscanf(nums[0].c_str(), "%d", &startPositon);
			setParam.startPositon = startPositon;
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool XMLReader::ReadRT44(std::string filename, vector<Mat>& Inputs)
{
    char* segmentation = " ";
    TiXmlDocument doc;
    if( !doc.LoadFile( filename.c_str()) )
    {
        return false;
    }
    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;

    pElem = hDoc.FirstChild("MatRT44").Element();
    if( pElem->ValueTStr() == "MatRT44" )
    {
        int num = pElem->FirstAttribute()->IntValue();
        TiXmlElement* tElem;
        tElem = pElem->FirstChild()->ToElement();
        for (int i = 0; i < num; i++)
        {
            Mat CamRT44 = Mat::zeros(4, 4, CV_64F);
            //rt.R = Mat(3, 1, CV_64F); rt.T = Mat(3, 1, CV_64F);
            TiXmlElement* _tElem;
            _tElem = tElem->FirstChild()->ToElement();
            for(_tElem;_tElem;_tElem=_tElem->NextSiblingElement())
            {
                std::vector<std::string> nums = split( _tElem->GetText(),segmentation);
                float vec[16];
                for( int j=0;j<16;j++ )
                {
                    sscanf( nums[j].c_str(), "%f", &vec[j] );
                    if (_tElem->ValueTStr() == "RT")
                    {
                        int l,k;
                        l=j%4; //// 除以 代表列
                        k=int(j/4); //// mode 代表行
                        CamRT44.at<double>(k, l) = vec[j];
                    }
                    else
                        break;
                }
            }
            Inputs.push_back(CamRT44);
            tElem = tElem->NextSiblingElement();
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool XMLReader::ReadRT33(std::string filename, vector<Mat>& Inputs)
{
	char* segmentation = " ";
	TiXmlDocument doc;
	if (!doc.LoadFile(filename.c_str()))
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);
	TiXmlElement* pElem;

	pElem = hDoc.FirstChild("MatRT33").Element();
	if (pElem->ValueTStr() == "MatRT33")
	{
		int num = pElem->FirstAttribute()->IntValue();
		TiXmlElement* tElem;
		tElem = pElem->FirstChild()->ToElement();
		for (int i = 0; i < num; i++)
		{
			Mat RT33 = Mat::zeros(3, 3, CV_64FC1);
			//rt.R = Mat(3, 1, CV_64F); rt.T = Mat(3, 1, CV_64F);
			TiXmlElement* _tElem;
			_tElem = tElem->FirstChild()->ToElement();
			for (_tElem; _tElem; _tElem = _tElem->NextSiblingElement())
			{
				std::vector<std::string> nums = split(_tElem->GetText(), segmentation);
				float vec[9];
				for (int j = 0; j < 9; j++)
				{
					sscanf(nums[j].c_str(), "%f", &vec[j]);
					if (_tElem->ValueTStr() == "RT")
					{
						int l, k;
						l = j % 3; //// 除以 代表列
						k = int(j / 3); //// mode 代表行
						RT33.at<double>(k, l) = vec[j];
					}
					else
						break;
				}
			}
			Inputs.push_back(RT33);
			tElem = tElem->NextSiblingElement();
		}
	}
	else
	{
		return false;
	}

	return true;
}

#if 0
bool XMLReader::readObjPara( std::string filename,ObjectPara& ObjParam)
{
	//加载文件
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);

	if( !parseFacePntsElement(hDoc,ObjParam) )
		return false;
	if( !parseRT2oneElement(hDoc,ObjParam) )
		return false;
	return true;
}

bool XMLReader::readpos61(std::string filename,vector<Vec6f>& pose)
{
	TiXmlDocument doc;
	if( !doc.LoadFile( filename.c_str()) )
	{
		return false;
	}
	TiXmlHandle hDoc(&doc);
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("Pluse_Poso").Element();
	if (pElem->ValueTStr()=="Pluse_Poso")
	{
		TiXmlElement * tElem;
		int num = pElem->FirstAttribute()->IntValue();
		tElem = pElem->FirstChild()->ToElement();
		for (int i =0;i<num;i++)
		{
			Vec6f _pose;
			std::vector<std::string> nums = split( tElem->GetText(),segmentation);//split()用于将字符串分割成一个个的片段
			sscanf(nums[0].c_str(),"%f",&_pose.val[0]);
			sscanf(nums[1].c_str(),"%f",&_pose.val[1]);
			sscanf(nums[2].c_str(),"%f",&_pose.val[2]);
			sscanf(nums[3].c_str(),"%f",&_pose.val[3]);
			sscanf(nums[4].c_str(),"%f",&_pose.val[4]);
			sscanf(nums[5].c_str(),"%f",&_pose.val[5]);
			pose.push_back(_pose);
			tElem=tElem->NextSiblingElement();
		}

	}
	else
	{
		return false;
	}
	return true;
}
#endif

#if 0
bool XMLReader::parseFacePntsElement( TiXmlHandle hDoc, ObjectPara& ObjParam )
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("FacePnts").Element();
	if( pElem->ValueTStr() == "FacePnts" )
	{
		TiXmlElement * tElem;
		tElem = pElem->FirstChild()->ToElement();
		for (int i = 0;i<5;i++)
		{
			vector<Point3f> _facepnt;
			TiXmlElement* _tElem;
			_tElem = tElem->FirstChild()->ToElement();
			for(_tElem;_tElem;_tElem=_tElem->NextSiblingElement())
			{
				std::vector<std::string> nums = split( _tElem->GetText(),segmentation);//split()用于将字符串分割成一个个的片段
				Point3f _pnt;
				sscanf(nums[0].c_str(),"%f",&_pnt.x);
				sscanf(nums[1].c_str(),"%f",&_pnt.y);
				sscanf(nums[2].c_str(),"%f",&_pnt.z);
				_facepnt.push_back(_pnt);
			}
			ObjParam.FacePnts[i]=_facepnt;
			_facepnt.clear();
			tElem = tElem->NextSiblingElement();
		}
	}
	else
	{
		return false;
	}
	return true;
}
bool XMLReader::parseRT2oneElement(TiXmlHandle hDoc, ObjectPara& ObjParam)
{
	char* segmentation = " ";
	TiXmlElement* pElem;
	pElem = hDoc.FirstChild("RT2one").Element();
	if( pElem->ValueTStr() == "RT2one" )
	{
		TiXmlElement* tElem;
		tElem = pElem->FirstChild()->ToElement();
		for (int i = 0; i < 4; i++)
		{
			Mat CamRT44 = Mat::zeros(4, 4, CV_64F);
			//rt.R = Mat(3, 1, CV_64F); rt.T = Mat(3, 1, CV_64F);
			TiXmlElement* _tElem;
			_tElem = tElem->FirstChild()->ToElement();
			for(_tElem;_tElem;_tElem=_tElem->NextSiblingElement())
			{
				std::vector<std::string> nums = split( _tElem->GetText(),segmentation);
				float vec[16];
				for( int j=0;j<16;j++ )
				{
					sscanf( nums[j].c_str(), "%f", &vec[j] );
					if (_tElem->ValueTStr() == "RT")
					{
						int l,k;
						l=j%4; //// 除以 代表列
						k=int(j/4); //// mode 代表行
						CamRT44.at<double>(k, l) = vec[j];
					}
					else
						break;
				}
			}
			ObjParam.RT2one[i]=CamRT44;
			tElem = tElem->NextSiblingElement();
		}
	}
	else
	{
		return false;
	}

	return true;
}

#endif