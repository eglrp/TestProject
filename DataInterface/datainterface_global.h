//*************************************************************************
//说明：用于生成各平台之间数据转化的动态库
//作者：ZZL
//发布时间：2018/02/01
//备注：
//*************************************************************************
#ifndef DATAINTERFACE_GLOBAL_H
#define DATAINTERFACE_GLOBAL_H

#include <QtCore/qglobal.h>

#ifdef DATAINTERFACE_LIB
# define DATAINTERFACE_EXPORT Q_DECL_EXPORT
#else
# define DATAINTERFACE_EXPORT Q_DECL_IMPORT
#endif

#endif // DATAINTERFACE_GLOBAL_H
