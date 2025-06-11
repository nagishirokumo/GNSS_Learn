//
// Created by asus on 2025/4/17.
//

#ifndef POS_NAV_PROJECT_DATA_RINEX303_H
#define POS_NAV_PROJECT_DATA_RINEX303_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>
#include <cctype>
#include <cmath>
#include "GNSS_Coord_System.h"
#include "GNSS_Time_System.h"

#define GPS_GM 3.986005e14
#define BDS_GM 3.986004418e14
#define GPS_OMEGAE 7.2921150e-5
#define BDS_OMEGAE 7.2921150e-5
#define PI 3.1415926535898
#define DELTAT_CORR_F -4.442807633e-10
#define IF_GPS_M 2.545727780163160
#define IF_GPS_N -1.545727780163160
#define IF_BDS_M 2.843645083932853
#define IF_BDS_N -1.843645083932853
#define C_LIGHT 299792458.0
#define FREQ_BDS_B1I 1.561098e9
#define FREQ_BDS_B3I 1.26852e9
#define FREQ_GPS_L1 1.57542e9
#define FREQ_GPS_L2 1.22760e9

class Rinex_Navigation_Message_Data;
class Navi_Data_Head;
class Navi_Data_Body;
class Rinex_Navigation_Message_Result;

class Navi_Data_Head
{
    friend void Rinex_Navigation_Message_Header_Read(std::ifstream& file, Navi_Data_Head& header);
    friend void Positioning(Rinex_Navigation_Message_Data& data);
    friend void GPS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
    friend void BDS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
private:
    double version;//rinex 版本号
    char type[20];//文件数据类型
    double BDT_CORR[4];//北斗时修正数
    double GPST_CORR[4];//GPS时修正数
    int leap_seconds;//跳秒
};

class Navi_Data_Body
{
    //友元函数以访问私有成员
    friend void Rinex_NavBlock_Read(std::ifstream& file, std::vector<Navi_Data_Body>& navs);
    friend void Positioning(Rinex_Navigation_Message_Data& data);
    friend void GPS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
    friend void BDS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
private:
    //数据块第一行内容：
    std::string sPRN;//卫星PRN号
    //历元：TOC中卫星钟的参考时刻
    int TOC_Y=0;//年
    int TOC_M=0;//月
    int TOC_D=0;//日
    int TOC_H=0;//时
    int TOC_Min=0;//分
    int TOC_Sec=0;//秒
    double sa0=0;//卫星钟差
    double sa1=0;//卫星钟偏
    double sa2=0;//卫星钟漂

    //数据块第二行内容：
    double IODE=0;//数据、星历发布时间(数据期龄)
    double Crs=0;//轨道半径的正弦调和改正项的振幅（单位：m）
    double deltan=0;//卫星平均运动速率与计算值之差(rad/s)
    double M0=0;//参考时间的平近点角(rad)

    //数据块第三行内容：
    double Cuc=0;//维度幅角的余弦调和改正项的振幅(rad)
    double e=0;//轨道偏心率
    double Cus=0;//轨道幅角的正弦调和改正项的振幅(rad)
    double sqrtA=0;//长半轴平方根

    //数据块第四行内容：
    double TOE=0;//星历的参考时刻(GPS周内秒)
    double Cic=0;//轨道倾角的余弦调和改正项的振幅(rad)
    double OMEGA0=0;//参考时刻的升交点赤经
    double Cis=0;//维度倾角的正弦调和改正项的振幅(rad)

    //数据块第五行内容：
    double i0=0;//参考时间的轨道倾角(rad)
    double Crc=0;//轨道平径的余弦调和改正项的振幅(m)
    double omega=0;//近地点角距
    double OMEGADOT=0;//升交点赤经变化率(rad)

    //数据块第六行内容：
    double IDOT=0;//近地点角距(rad/s)
    double L2code=0;//L2上的码
    double GPSweek=0;//GPS周,于TOE一同表示
    double L2Pflag=0;//L2,p码数据标记

    //数据块第七行内容
    double sACC=0;//卫星精度
    double sHEA=0;//卫星健康状态
    double TGD=0;//sec
    double IODC=0;//钟的数据龄期

    //数据块第八行内容
    double TTN=0;//电文发送时间
    double fit=0;//拟合区间，可为保留字
    double spare1=0;//保留字
    double spare2=0;//保留字
};

class Rinex_Navigation_Message_Result//:private Cartesian_XYZ_Coord
{
    friend void Positioning(Rinex_Navigation_Message_Data& data);
    friend void GPS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
    friend void BDS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
    friend void Write2CSV(Rinex_Navigation_Message_Data &data);
public:
    Rinex_Navigation_Message_Result(std::string sPRN_);
private:
    //存储卫星的编号以及一个历元内的位置XYZ坐标
    std::string sPRN;
    std::vector<Cartesian_XYZ_Coord> vXYZ_Coordinate;
    std::vector<int> vTimeStamp;
};

class Rinex_Navigation_Message_Data
{
public:
    Navi_Data_Head Data_Head;
    std::vector<Navi_Data_Body> vData_Body;
    std::vector<Rinex_Navigation_Message_Result> vResult;

    double GM;//地球引力常数 WGS-84
public:
    Rinex_Navigation_Message_Data(std::ifstream& file, int time_cur, double GM=3.986005e+14);

};

class Rinex_Observation_Head
{
public:
    double version;                             //rinex 版本号
    char type[20];                              //文件数据类型
    Cartesian_XYZ_Coord Approximate_Position;   //测站近似坐标
    //key-系统类型 value-系统观测值类型
    std::map<char,std::vector<std::string>> Sys_Obs_Type;
    Rinex_Observation_Head(std::ifstream& file);
};
//某一卫星的观测值
class Rinex_Single_Satellite_Observation_Data
{
public:
    double Pseudorange_1;//伪距观测值
    double Pseudorange_2;
    double Carrier_Phase_1;//载波相位观测值
    double Carrier_Phase_2;
    double Pseudorange_Geometry_Free=0;//消几何距离组合
    double Carrier_Phase_Geometry_Free=0;
    double Melbourne_Wubbena=0;//MW组合
    //GPS使用C1C、C2W、L1C、L2W；BDS使用C2I、C6I、L2I、L6I
};
//单历元观测值
class Rinex_Single_Epoch_Observation_Data
{
public:
    UTC Epoch_Time;
    int Epoch_Flag;
    int Num_Of_Satellites;
    std::map<std::string,Rinex_Single_Satellite_Observation_Data> Satellites_Obs;
    Rinex_Single_Epoch_Observation_Data(std::ifstream& file, Rinex_Observation_Head* pHeader);
    Rinex_Single_Epoch_Observation_Data(){};
};
//观测值文件数据体
class Rinex_Observation_Data
{
public:
    Rinex_Observation_Head* pHeader;
    std::vector<Rinex_Single_Epoch_Observation_Data> Rinex_Obs_Data;
    std::map<std::string,std::vector<std::pair<int,int>>> Time_Range;
    Rinex_Observation_Data(std::ifstream& file,Rinex_Observation_Head* pHeader_);
};

class Epoch_Differential_Data
{
public:
    UTC Epoch_Time;
    UTC Refer_Epoch_Time;
    std::map<std::string,double> P_GF_Epoch_Differential;
    std::map<std::string,double> L_GF_Epoch_Differential;
    std::map<std::string,double> MW_Epoch_Differential;
};

class Rinex_Observation_Epoch_Differential
{
public:
    std::vector<Epoch_Differential_Data> Epoch_Differential;

    Rinex_Observation_Epoch_Differential(Rinex_Observation_Data Obs_Data);
};







void Rinex_Navigation_Message_Header_Read(std::ifstream& file, Navi_Data_Head& header);

double Read_Double_Data(const std::string& line, int offset,int length);
int Read_Int_Data(const std::string& line, int offset,int length);

// 提取每一行中固定宽度字段的通用函数（每个字段宽度为19）
double Read_Field(const std::string& line, int idx) ;

bool isAllSpaces(const std::string& str);

// 解析每组导航数据（共8行），填入 Navi_Data_Body
void Rinex_NavBlock_Read(std::ifstream& file, std::vector<Navi_Data_Body>& navs);

//定位解算
void Positing(Rinex_Navigation_Message_Data& data, int time_cur);
void GPS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
void BDS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);


void Positioning(Rinex_Navigation_Message_Data& data);
void GPS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
void BDS_Positioning(Rinex_Navigation_Message_Data &data, std::vector<Navi_Data_Body>::iterator &it, Rinex_Navigation_Message_Result &result);
void Write2CSV(Rinex_Navigation_Message_Data &data);
void Write2TXT(Rinex_Observation_Data &data);
void Write2TXT(Rinex_Observation_Epoch_Differential &data);

#endif //POS_NAV_PROJECT_DATA_RINEX303_H

