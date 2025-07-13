//
// Created by asus on 2025/4/17.
//

#ifndef POS_NAV_PROJECT_DATA_RINEX_H
#define POS_NAV_PROJECT_DATA_RINEX_H

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
#define GPS_OMEGAE 7.2921151467e-5
#define BDS_OMEGAE 7.2921150e-5
#define PI 3.1415926535898
#define DELTAT_CORR_F -4.442807633e-10

#define IF_GPS_M 2.545727780163160
#define IF_GPS_N -1.545727780163160
#define IF_BDS_M 2.843645083932853
#define IF_BDS_N -1.843645083932853

#define C_LIGHT 299792458.0
#define FREQ_BDS_B1I 1.561098e9
#define FREQ_BDS_B2I 1.20714e9
#define FREQ_BDS_B3I 1.26852e9
#define FREQ_GPS_L1 1.57542e9
#define FREQ_GPS_L2 1.22760e9

#define DEG2RAD 3.1415926535898/180
#define RAD2DEG 180/3.1415926535898


class Navi_Data_Head;
class Navi_Data_Body;
class Rinex_Navigation_Message_Data;
class Rinex_Navigation_Message_Result;

class Navi_Data_Head
{
public:
    double version;//rinex 版本号
    char type[20];//文件数据类型
    double BDT_CORR[4];//北斗时修正数
    double GPST_CORR[4];//GPS时修正数
    int leap_seconds;//跳秒
};
class Navi_Data_Body
{
public:
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

class Rinex_Navigation_Message_Data
{
public:
    Navi_Data_Head Data_Head;
    std::vector<Navi_Data_Body> vData_Body;
    std::vector<Rinex_Navigation_Message_Result> vResult;
    double GM;//地球引力常数 WGS-84
    Rinex_Navigation_Message_Data(std::ifstream& file, int time_cur, double GM=3.986005e+14);
};

class Rinex_Navigation_Message_Result
{
public:
    Rinex_Navigation_Message_Result(std::string sPRN_);//存储卫星的编号以及一个历元内的位置XYZ坐标

    std::string sPRN;
    std::vector<Cartesian_XYZ_Coord> vXYZ_Coordinate;
    std::vector<int> vTimeStamp;
};


//Rinex304
class Rinex304_Observation_Head
{
public:
    double version;                             //rinex 版本号
    char type[20];                              //文件数据类型
    Cartesian_XYZ_Coord Approximate_Position;   //测站近似坐标
    GPST First_Obs_Time;
    GPST Last_Obs_Time;
    //key-系统类型 value-系统观测值类型
    std::map<char,std::vector<std::string>> Sys_Obs_Type;
    Rinex304_Observation_Head(std::ifstream& file);
    Rinex304_Observation_Head(){}
};
//某一卫星的观测值
class Rinex304_Single_Satellite_Observation_Data
{
public:
    //     key为观测值类型 前两位含义如下
    //    类型：
    //    C = 测距码/伪距 L = 载波相位 D = 多普勒频移 S = 原始信号强度（载噪比） I = 电离层相位延迟 X = 接收机通道号
    //    频段：
    //    1 = L1（GPS、QZSS、SBAS） G1（GLONASS） E1（Galileo）
    //    2 = L2（GPS、QZSS） G2（GLONASS） B1（北斗）
    //    5 = L5（GPS、QZSS、SBAS） E5a（Galileo） L5（IRNSS）
    //    6 = E6（Galileo） LEX（QZSS） B3（北斗）
    //    7 = E5b（Galileo） B2（北斗）
    //    8 = E5a+b（Galileo）
    //    9 = S波段（IRNSS）
    //    0 = 类型X专用（全频段）
    std::map<std::string,double> Single_Satellite_Obs;
    double Melbourne_Wubbena=0;
    double Carrier_Phase_Geometry_Free=0;
    double Pseudorange_Geometry_Free=0;

    double Pseudorange1=0;
    double Carrier_Phase1=0;
    double Pseudorange2=0;
    double Carrier_Phase2=0;
};
//单历元观测值
class Rinex304_Single_Epoch_Observation_Data
{
public:
    UTC Epoch_Time;
    int Epoch_Flag;
    int Num_Of_Satellites;
    std::map<std::string,Rinex304_Single_Satellite_Observation_Data> Satellites_Obs;
    Rinex304_Single_Epoch_Observation_Data(std::ifstream& file, Rinex304_Observation_Head* pHeader);
    Rinex304_Single_Epoch_Observation_Data(){};
};
//观测值文件数据体
class Rinex304_Observation_Data
{
public:
    Rinex304_Observation_Head* pHeader= nullptr;
    std::vector<Rinex304_Single_Epoch_Observation_Data> Rinex_Obs_Data;
    std::map<std::string,std::vector<std::pair<int,int>>> Time_Range;
    Rinex304_Observation_Data(std::ifstream& file, Rinex304_Observation_Head* pHeader_);
    Rinex304_Observation_Data(){}
};

//观测值文件
class Rinex304_Observation_File
{
public:
    Rinex304_Observation_Head Obs_File_Head;
    Rinex304_Observation_Data Obs_File_Data;

    Rinex304_Observation_File(){};
    Rinex304_Observation_File(std::string filepath);

    Rinex304_Observation_File(const Rinex304_Observation_File& other)
    {
        Obs_File_Head = other.Obs_File_Head;
        Obs_File_Data = other.Obs_File_Data;
        Obs_File_Data.pHeader = &Obs_File_Head;
    }

    Rinex304_Observation_File& operator=(const Rinex304_Observation_File& other)
    {
        if (this != &other) {
            Obs_File_Head = other.Obs_File_Head;
            Obs_File_Data = other.Obs_File_Data;
            Obs_File_Data.pHeader = &Obs_File_Head;
        }
        return *this;
    }


};

//Rinex303
class Rinex303_Observation_Head
{
public:
    double version;                             //rinex 版本号
    char type[20];                              //文件数据类型
    Cartesian_XYZ_Coord Approximate_Position;   //测站近似坐标
    GPST First_Obs_Time;
    GPST Last_Obs_Time;
    //key-系统类型 value-系统观测值类型
    std::map<char,std::vector<std::string>> Sys_Obs_Type;
    Rinex303_Observation_Head(std::ifstream& file);
};
//某一卫星的观测值
class Rinex303_Single_Satellite_Observation_Data
{
public:
    //     key为观测值类型 前两位含义如下
    //    类型：
    //    C = 测距码/伪距 L = 载波相位 D = 多普勒频移 S = 原始信号强度（载噪比） I = 电离层相位延迟 X = 接收机通道号
    //    频段：
    //    1 = L1（GPS、QZSS、SBAS） G1（GLONASS） E1（Galileo）
    //    2 = L2（GPS、QZSS） G2（GLONASS） B1（北斗）
    //    5 = L5（GPS、QZSS、SBAS） E5a（Galileo） L5（IRNSS）
    //    6 = E6（Galileo） LEX（QZSS） B3（北斗）
    //    7 = E5b（Galileo） B2（北斗）
    //    8 = E5a+b（Galileo）
    //    9 = S波段（IRNSS）
    //    0 = 类型X专用（全频段）
    std::map<std::string,double> Single_Satellite_Obs;
};
//单历元观测值
class Rinex303_Single_Epoch_Observation_Data
{
public:
    UTC Epoch_Time;
    int Epoch_Flag;
    int Num_Of_Satellites;
    std::map<std::string,Rinex303_Single_Satellite_Observation_Data> Satellites_Obs;
    Rinex303_Single_Epoch_Observation_Data(std::ifstream& file, Rinex303_Observation_Head* pHeader);
    Rinex303_Single_Epoch_Observation_Data(){};
};
//观测值文件数据体
class Rinex303_Observation_Data
{
public:
    Rinex303_Observation_Head* pHeader;
    std::vector<Rinex303_Single_Epoch_Observation_Data> Rinex_Obs_Data;
    std::map<std::string,std::vector<std::pair<int,int>>> Time_Range;
    Rinex303_Observation_Data(std::ifstream& file, Rinex303_Observation_Head* pHeader_);
};

//数据完整率
class Rinex_Observation_Completeness_Rate
{
public:
    //单频观测数据完整率
    std::map<char,std::map<char,std::pair<int,int> > > Freq_Observation_Completeness_Rate;
    //系统观测数据完整率
    std::map<char,std::pair<int,int> > Sys_Observation_Completeness_Rate;

    Rinex_Observation_Completeness_Rate(){}
    Rinex_Observation_Completeness_Rate(Rinex304_Observation_Data &Observation_Data,
                                        double Freq_Of_Obs);

};
//周跳计算
class Rinex_Observation_Cycle_Slip_Detection
{
public:
    std::map<std::string,std::map<int,double> > MW_Mean;
    std::map<std::string,std::map<int,double> > MW_Sigma;

    std::map<std::string,std::map<int,double> > MW_Diff;
    std::map<std::string,std::map<int,double> > GF_Carrier_Phase;//载波
    std::map<std::string,std::map<int,double> > GF_Pseudorange;//伪距

    std::map<std::string,std::vector<int> > Cycle_Slip_List;
    std::map<std::string,std::vector<int> > Gross_Error_List;

    //外层key为历元index 内层是prn和钟跳类型
    std::map<int,std::map<std::string,int> > Receiver_Clock_Slip;

    Rinex_Observation_Cycle_Slip_Detection(){}
    Rinex_Observation_Cycle_Slip_Detection(Rinex304_Observation_Data &Observation_Data,
                                           double Freq_Of_Obs);
};
bool Linear_Combine_Calc(Rinex304_Single_Satellite_Observation_Data &Sat_Obs,std::string PRN);
void MW_Cycle_Slip_Detection(Rinex304_Observation_Data &Observation_Data,
                             Rinex_Observation_Cycle_Slip_Detection &Cycle_Slip_Detection);
void GF_Cycle_Slip_Detection(Rinex304_Observation_Data &Observation_Data,
                             Rinex_Observation_Cycle_Slip_Detection &Cycle_Slip_Detection);
Eigen::VectorXd Poly_Fit(const std::vector<double>& x, const std::vector<double>& y, int degree);
double Poly_Value(const Eigen::VectorXd& coeffs, double x);
void Receiver_Clock_Slip_Detection(Rinex304_Observation_Data &Observation_Data,
                                   Rinex_Observation_Cycle_Slip_Detection &Cycle_Slip_Detection);

class Rinex_Observation_MultiPath_Detection
{
public:
    std::map<std::string,std::map<int,double> > MultiPath_Freq1;
    std::map<std::string,std::map<int,double> > MultiPath_Freq2;
    std::map<std::string,std::map<int,double> > MultiPath_Freq1_PrefixSum;
    std::map<std::string,std::map<int,double> > MultiPath_Freq2_PrefixSum;

    std::map<std::string,std::map<int,double> > MultiPath_Freq1_Assessment;
    std::map<std::string,std::map<int,double> > MultiPath_Freq2_Assessment;


    Rinex_Observation_MultiPath_Detection(){}
    Rinex_Observation_MultiPath_Detection(Rinex304_Observation_Data &Observation_Data,
                                           double Freq_Of_Obs);
};

//导航电文
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


#endif //POS_NAV_PROJECT_DATA_RINEX_H

