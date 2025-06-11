//
// Created by asus on 2025/4/29.
//

#ifndef POS_NAV_PROJECT_DATA_NOVATEL_H
#define POS_NAV_PROJECT_DATA_NOVATEL_H

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
#include <cstdint>
#include "GNSS_Coord_System.h"
#include "GNSS_Time_System.h"
#include "Matrix_Calculate.h"



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
#define IF_BDS_K13 1.5424037460978147
#define C_LIGHT 299792458.0
#define GPS_L1_LAMDA 0.190293672798
#define GPS_L2_LAMDA 0.244210213424
#define BDS_B1_LAMDA 0.192039486310
#define BDS_B3_LAMDA 0.236332464604
#define DEG2RAD 3.1415926535898/180
#define RAD2DEG 180/3.1415926535898

class Novatel_Data;


//读取无符号8位整数0-255 1字节/8bit 传入参数 文件 数据存储的指针
void Uint8_read(std::ifstream& file, uint8_t* puint8);
//读取有符号整数-128-127 1字节/8bit 传入参数 文件 数据存储的指针
void Int8_read(std::ifstream& file, int8_t* pint8);
//读取无符号16位整数类型 2字节/16bit 传入参数 文件 数据存储的指针
void Uint16_read(std::ifstream& file, uint16_t* puint16);
//读取无符号16位整数类型 2字节/16bit 传入参数 文件 数据存储的指针
void Int16_read(std::ifstream& file, uint16_t* pint16);
//读取无符号整数 4字节/32bit 传入参数 文件 数据存储的指针
void Uint32_read(std::ifstream& file, uint32_t* puint32);
//读取有符号整数 4字节/32bit 传入参数 文件 数据存储的指针
void Int32_read(std::ifstream& file, int32_t* pint32);
//读取单精度浮点 4字节/32bit 传入参数 文件 数据存储的指针
void Float32_read(std::ifstream& file, float_t* pfloat32);
//读取有符号整数 8字节/64bit 传入参数 文件 数据存储的指针
void Int64_read(std::ifstream& file, int64_t* pint64);
//读取双精度浮点 8字节/64bit 传入参数 文件 数据存储的指针
void Double64_read(std::ifstream& file, double_t* pdouble64);



//二进制文件头校验
bool Novatel_Binary_Header_SyncCheck(std::ifstream& file,uint8_t* pSync);
//CRC校验
unsigned long CRC32Value(int i);
bool CalcMessageCRC32(std::ifstream &file, uint16_t Message_Length);

class Novatel_Data_Binary_Header
{
public:
    Novatel_Data_Binary_Header(){};
    Novatel_Data_Binary_Header(std::ifstream& file);
public:
    uint8_t Sync[3];              // 同步头，固定为0xAA 0x44 0x12，用于标识二进制消息的起始位置
    uint8_t Hearder_Length;       // 头部长度，通常为28（表示从Sync开始到Message_Length字段结束的长度）
    uint16_t Message_ID;          // 消息ID，标识消息的类型，例如 RANGE 是 43，BESTPOS 是 42 等
    int8_t Message_Type;          // 消息类型：0-原始日志（binary），1-ASCII，2-Abbreviated ASCII，3-NMEA，4-无效，5-二进制NMEA
    uint8_t Port_Address;         // 端口地址：表示从哪个接收机端口输出的，比如 COM1、USB1 等
    uint16_t Message_Length;      // 消息体（payload）长度，不包括头部和CRC
    uint16_t Sequence;            // 消息序号，用于识别是否有消息丢失或乱序
    int8_t Idle_Time;             // 接收机空闲时间（百分比）
    int8_t Time_Status;           // 表示GPS参考时间的质量（参见第50页的表11：GPS参考时间状态）
    uint16_t GPS_Week;            // GPS周
    uint32_t Milliseconds;        // 周内毫秒数，表示GPS时间（精确时间戳）
    uint32_t Receiver_Status;     // 接收机状态，表示各种系统状态位，比如是否锁定、是否有差分等
    uint16_t Reserved;            // 保留字段，当前未使用，通常为0
    uint16_t Receiver_Version;    // 接收机软件版本号（主版本号），用于识别固件版本
};

//Message_ID 43 RANGE 详见p723
class Novatel_Message_RANGE_DataBlock
{
public:
    Novatel_Message_RANGE_DataBlock(){};
    Novatel_Message_RANGE_DataBlock(std::ifstream& file);
public:
    uint16_t PRN;                   //PRN号
    uint16_t GLONASS_Frequency;     //GLONASS Frequency+7
    double_t Psr;                   //伪橙色测量值(m)
    float_t Psr_Sigma;              //伪橙色测量标准偏差(m)
    double_t Adr;                   //载波相位，以周期为单位（累积多普勒范围）
    float_t Adr_Sigma;              //估计的载波相位标准偏差（周期）
    float_t Dopp;                   //瞬时载波多普勒频率（Hz）
    float_t C_No;                   //载流子与噪声密度比
    float_t Locktime;               //连续跟踪秒数（无周期滑移）
    uint32_t Ch_tr_status;          //跟踪状态（见下页表149：通道跟踪状态和下页图15：通道跟踪示例）

};
class Novatel_OBSERVATION
{
public:
    Novatel_OBSERVATION(){};
    Novatel_OBSERVATION(Novatel_Data_Binary_Header* pHeader,Novatel_Message_RANGE_DataBlock& DataBlock);
    int32_t System_Flag;            //0 GPS  4 BDS
//    uint16_t GLONASS_Frequency;     //GLONASS Frequency+7
    double_t Psr;                   //伪橙色测量值(m)
    float_t Psr_Sigma;              //伪橙色测量标准偏差(m)
    double_t Adr;                   //载波相位，以周期为单位（累积多普勒范围）
    float_t Adr_Sigma;              //估计的载波相位标准偏差（周期）
    float_t Dopp;                   //瞬时载波多普勒频率（Hz）
    float_t C_No;                   //载流子与噪声密度比
//    float_t Locktime;               //连续跟踪秒数（无周期滑移）
//    uint32_t Ch_tr_status;          //跟踪状态（见下页表149：通道跟踪状态和下页图15：通道跟踪示例）
    int32_t Signal_Type;
};
class Novatel_Single_Epoch_OBSERVATION
{
public:
    GPST GPS_Time;
    std::map<uint32_t,std::vector<Novatel_OBSERVATION> > Single_Epoch_GPS_Observation;
    std::map<uint32_t,std::vector<Novatel_OBSERVATION> > Single_Epoch_BDS_Observation;
    std::map<uint32_t,double > Ionospheric_Free_GPS_Observation;
    std::map<uint32_t,double > Ionospheric_Free_BDS_Observation;
    std::map<uint32_t,double > Ionospheric_Free_Sigma_GPS_Observation;
    std::map<uint32_t,double > Ionospheric_Free_Sigma_BDS_Observation;
    std::map<uint32_t,double > Doppler_GPS_Observation;
    std::map<uint32_t,double > Doppler_BDS_Observation;
    std::map<uint32_t,double > Doppler_CNO_GPS_Observation;
    std::map<uint32_t,double > Doppler_CNO_BDS_Observation;
};
class Novatel_Message_RANGE
{
public:
    Novatel_Message_RANGE(){};
    Novatel_Message_RANGE(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_);
    Novatel_Message_RANGE(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_,Novatel_Data &Novatel_Data);

public:
    Novatel_Data_Binary_Header* pHeader;
    uint32_t Num_Of_Observations;
    std::vector<Novatel_Message_RANGE_DataBlock> vRange_DataBlock;
    uint32_t CRC;
};

//Message_ID 47 PSRPOS 详见p696
//此日志包含接收器计算出的位置，以及三个状态标志。
//此外，它还报告其他状态指示符，包括差分年龄，这在预测由差分校正中断引起的异常行为时非常有用。
class Novatel_Message_PSRPOS
{
public:
    Novatel_Message_PSRPOS(){};
    Novatel_Message_PSRPOS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_);
public:
    Novatel_Data_Binary_Header* pHeader;
    int32_t Sol_Status;         //
    int32_t Pos_Type;           //位置类型
    double_t Latitude;          //纬度(角度制)
    double_t Longitude;         //经度(角度制)
    double_t Height;            //大地高（m）
    float_t Undulation;         //波纹——大地水准面与WGS84椭球体之间的关系(m)
    int32_t Datum_ID;           //数据ID编号 61 = WGS84 63 = USER
    float_t Latitude_Sigma;     //纬度标准差（m）
    float_t Longitude_Sigma;    //进度标准差（m）
    float_t Height_Sigma;       //大地高标准差（m）
    int8_t Base_Station_ID[4];  //基站ID
    float_t Differential_Age;   //秒龄差
    float_t Solution_Age;       //
    uint8_t SVs;                //跟踪的卫星数量
    uint8_t SolnSVs;            //在解决方案中使用的卫星数量
    uint8_t Reserved[3];        //保留字 Uchar Uchar Hex
    uint8_t Ext_Sol_Stat;       //扩展解决方案状态（参见第457页的表84：扩展解决方案状态）
    uint8_t Gal_BD_Sig_mask;    //伽利略和北斗信号使用的掩码（见第457页表83：伽利略和北斗信号使用的掩码）
    uint8_t GPS_GLO_Sig_mask;   //使用了GPS和格洛纳斯信号掩码（见第456页表82：GPS和格洛纳斯信号-使用的掩码）
    uint32_t CRC;
};

//Message_ID 42 BESTPOS 详见p450
/*
 * 系统以RTK模式运行时，BESTPOS反映接收最后一个基站观测后最多60秒内的最新低延迟解决方案。
 * 在此60秒之后，位置恢复到可用的最佳解决方案，精度下降则体现在标准差字段中。
 * 如果系统未以RTK模式运行，伪距差分解决方案将持续到PSRDIFFTIMEOUT命令指定的时间（见第296页）。
 * 如果接收器启用SPAN，GNSS+INS组合解决方案也是BESTPOS输出的候选方案。
 */
class Novatel_Message_BESTPOS
{
public:
    Novatel_Message_BESTPOS(){};
    Novatel_Message_BESTPOS(std::ifstream &file, Novatel_Data_Binary_Header *pHeader_);
public:
    Novatel_Data_Binary_Header* pHeader;
    int32_t Sol_status;             //解决方案状态
    int32_t Pos_Type;               //位置类型
    double_t Latitude;              //纬度(角度制)
    double_t Longitude;             //经度(角度制)
    double_t Height;                //大地高（m）
    float_t Undulation;             //波纹——大地水准面与WGS84椭球体之间的关系(m)
    int32_t Datum_ID;               //数据ID编号 61 = WGS84 63 = USER
    float_t Latitude_Sigma;         //纬度标准差（m）
    float_t Longitude_Sigma;        //进度标准差（m）
    float_t Height_Sigma;           //大地高标准差（m）
    int8_t Base_Station_ID[4];      //基站ID
    float_t Differential_Age;       //秒龄差
    float_t Solution_Age;           //
    uint8_t SVs;                    //跟踪的卫星数量
    uint8_t SolnSVs;                //在解决方案中使用的卫星数量
    uint8_t SolnL1SVs;              //解决方案中使用L1/E1/B1信号的卫星数量
    uint8_t SolnMultiSVs;           //解决方案中使用的多频信号卫星数量
    uint8_t Reserved;               //保留字 Hex
    uint8_t Ext_Sol_Stat;           //扩展解决方案状态（参见第457页的表84：扩展解决方案状态）
    uint8_t Gal_BD_Sig_mask;        //伽利略和北斗信号使用的掩码（见第457页表83：伽利略和北斗信号使用的掩码）
    uint8_t GPS_GLO_Sig_mask;       //使用了GPS和格洛纳斯信号掩码（见第456页表82：GPS和格洛纳斯信号-使用的掩码）
    uint32_t CRC;
};

//Message_ID 7 GPSEPHEM 详见p569
//GPSEPHEM日志可用于监测GPS卫星轨道的变化。
class Novatel_Message_GPSEPHEM
{
public:
    Novatel_Message_GPSEPHEM(){};
    Novatel_Message_GPSEPHEM(std::ifstream &file,Novatel_Data_Binary_Header *pHeader_);
public:
    Novatel_Data_Binary_Header *pHeader;
    uint32_t PRN;               //卫星PRN编号
    double_t Tow;               //时间戳
    uint32_t Health;            //健康状况——IS-GPS-200中定义的6位健康代码
    uint32_t IDOE1;             //流星数据的发布1
    uint32_t IDOE2;             //流星数据的发布2
    uint32_t Week;              //周数（根据Z计数周计算）
    uint32_t Z_Week;            //Z计数周数。这是星历表子帧1中的周数。从该值推导出“趾周”（字段#7)，以考虑翻转
    double_t Toe;               //星历参考时间（秒）
    double_t A;                 //半长轴(m)
    double_t Delta_N;           //平均运动差（rad/s）              //RINEX卫星平均运动速率与计算值之差(rad/s)
    double_t M0;                //参考时间的平均异常值（弧度）        //RINEX参考时间的平近点角(rad)
    double_t e;                 //偏心率，无量纲                   //RINEX偏心率
    double_t omega;             //近地点角（弧度）                  //RINEX近地点角距
    double_t Cuc;               //维度幅角的余弦调和改正项的振幅(rad)
    double_t Cus;               //轨道幅角的正弦调和改正项的振幅(rad)
    double_t Crc;               //轨道平径的余弦调和改正项的振幅(m)
    double_t Crs;               //轨道半径的正弦调和改正项的振幅（单位：m）
    double_t Cic;               //轨道倾角的余弦调和改正项的振幅(rad)
    double_t Cis;               //维度倾角的正弦调和改正项的振幅(rad)
    double_t I0;                //参考时间的轨道倾角（弧度）
    double_t I_Dot;             //近地点角距(rad/s)
    double_t OMEGA0;            //天赤角（弧度）                   //RINEX参考时刻的升交点赤经
    double_t OMEGADOT;          //赤经速率（弧度/秒）              //RINEX升交点赤经变化率(rad)
    uint32_t Idoc;              //数据钟问题
    double_t Toc;               //SV时钟校正项(s)                //
    double_t Tgd;               //估计的组延迟差(s)
    double_t sa0;               //卫星钟差
    double_t sa1;               //卫星钟偏
    double_t sa2;               //卫星钟漂
    uint32_t AS;                //防欺骗：0=FALSE 1=TRUE
    double_t N;                 //校正后的平均运动（rad/s）
    double_t URA;               //用户范围精度方差（米）2 ICD规定，
                                // 星历中传输的URA指数可以使用该表中列出的算法转换为名义标准偏差值。
                                // 我们发布的是名义值的平方（方差）。
                                // 原始URA指数与输出值之间的对应关系见下表115：URA方差
    uint32_t CRC;

    double_t sqrtA;

};

//Message_ID 1696 BDSEPHEMERIS 详见p443
class Novatel_Message_BDSEPHEMERIS
{
public:
    Novatel_Message_BDSEPHEMERIS(){};
    Novatel_Message_BDSEPHEMERIS(std::ifstream &file,Novatel_Data_Binary_Header *pHeader_);
public:
    Novatel_Data_Binary_Header *pHeader;
    uint32_t Satellite_ID;      //ID/测距码
    uint32_t Week;              //北斗周数
    double_t URA;               //用户范围精度(m)。这是评估的URAI/URA查找表值
    uint32_t Health1;           //自主卫星健康标志。0表示广播卫星状态良好，1表示不正常。
    double_t Tgd1;              //B1信号的设备组延迟差(s)
    double_t Tgd2;              //B2信号的设备组延迟差(s)
    uint32_t AODC;              //时钟数据期龄
    uint32_t Toc;               //时钟参数参考时间(s)
    double_t sa0;               //卫星钟差
    double_t sa1;               //卫星钟偏
    double_t sa2;               //卫星钟漂
    uint32_t AODE;              //历表数据年龄
    uint32_t Toe;               //星历参数参考时间(s)
    double_t Root_A;            //半长轴平方根(m)
    double_t e;                 //偏心率，无量纲                   //RINEX偏心率
    double_t omega;             //近地点角（弧度）                  //RINEX近地点角距
    double_t Delta_N;           //平均运动差（rad/s）              //RINEX卫星平均运动速率与计算值之差(rad/s)
    double_t M0;                //参考时间的平均异常值（弧度）        //RINEX参考时间的平近点角(rad)
    double_t OMEGA0;            //天赤角（弧度）                   //RINEX参考时刻的升交点赤经
    double_t OMEGADOT;          //赤经速率（弧度/秒）              //RINEX升交点赤经变化率(rad)
    double_t I0;                //参考时间的轨道倾角（弧度）
    double_t I_Dot;             //近地点角距(rad/s)
    double_t Cuc;               //维度幅角的余弦调和改正项的振幅(rad)
    double_t Cus;               //轨道幅角的正弦调和改正项的振幅(rad)
    double_t Crc;               //轨道平径的余弦调和改正项的振幅(m)
    double_t Crs;               //轨道半径的正弦调和改正项的振幅（单位：m）
    double_t Cic;               //轨道倾角的余弦调和改正项的振幅(rad)
    double_t Cis;               //维度倾角的正弦调和改正项的振幅(rad)
    uint32_t CRC;

    double_t A;
};

//Message_ID 172 AVEPOS 详见p437
class Novatel_Message_AVEPOS
{
public:
    Novatel_Message_AVEPOS(){};
    Novatel_Message_AVEPOS(std::ifstream &file,Novatel_Data_Binary_Header *pHeader_);
public:
    Novatel_Data_Binary_Header *pHeader;
    double_t Latitude;              //纬度(角度制)
    double_t Longitude;             //经度(角度制)
    double_t Height;                //大地高（m）
    float_t Latitude_Sigma;         //纬度标准差（m）
    float_t Longitude_Sigma;        //进度标准差（m）
    float_t Height_Sigma;           //大地高标准差（m）
    int32_t Pos_Ave;                //位置平均状态（见下表77：位置平均状态）
    uint32_t Ave_Time;              //平均时间(s)
    uint32_t Samples;               //平均样品数量
    uint32_t CRC;
};

//Message_ID 469 PDPPOS 详见p673
class Novatel_Message_PDPPOS
{
public:
    Novatel_Message_PDPPOS(){};
    Novatel_Message_PDPPOS(std::ifstream &file,Novatel_Data_Binary_Header *pHeader_);
public:
    Novatel_Data_Binary_Header* pHeader;
    int32_t Sol_status;             //解决方案状态
    int32_t Pos_Type;               //位置类型
    double_t Latitude;              //纬度(角度制)
    double_t Longitude;             //经度(角度制)
    double_t Height;                //大地高（m）
    float_t Undulation;             //波纹——大地水准面与WGS84椭球体之间的关系(m)
    int32_t Datum_ID;               //数据ID编号 61 = WGS84 63 = USER
    float_t Latitude_Sigma;         //纬度标准差（m）
    float_t Longitude_Sigma;        //进度标准差（m）
    float_t Height_Sigma;           //大地高标准差（m）
    int8_t Base_Station_ID[4];      //基站ID
    float_t Differential_Age;       //秒龄差
    float_t Solution_Age;           //
    uint8_t Sats;                   //跟踪的卫星数目
    uint8_t SolnSats;               //在解决方案中使用的卫星数量
    uint8_t SolnL1SVs;              //解决方案中使用L1/E1/B1信号的卫星数量
    uint8_t Reserved[3];            //保留字 Uchar Uchar Hex
    uint8_t Ext_Sol_Stat;           //扩展解决方案状态（参见第457页的表84：扩展解决方案状态）
    uint8_t Gal_BD_Sig_mask;        //伽利略和北斗信号使用的掩码（见第457页表83：伽利略和北斗信号使用的掩码）
    uint8_t GPS_GLO_Sig_mask;       //使用了GPS和格洛纳斯信号掩码（见第456页表82：GPS和格洛纳斯信号-使用的掩码）
    uint32_t CRC;
};

class GPS_Position
{
public:
    GPS_Position(){}

    Cartesian_XYZ_Coord Position;
    UTC Time;
    GPST GPS_Time;
    double v_X,v_Y,v_Z;
    double Clkdelta;
    double Clkdelta_Dot;
};

class BDS_Position
{
public:
    BDS_Position(){}

    Cartesian_XYZ_Coord Position;
    UTC Time;
    BDT BDS_Time;
    double v_X,v_Y,v_Z;
    double Clkdelta;
    double Clkdelta_Dot;
};

class Receiver_Position
{
public:
    Receiver_Position(){}

    Cartesian_XYZ_Coord Position;
    UTC Time;
    GPST GPS_Time;
    double v_X,v_Y,v_Z;
};


class Novatel_Data
{
public:
    Novatel_Data(){};
    Novatel_Data(std::ifstream &file);
public:
    std::vector<Novatel_Data_Binary_Header> vData_Header;
    std::vector<Novatel_Message_RANGE> vMessage_RANGE;

    std::vector<Novatel_Message_PSRPOS> vMessage_PSRPOS;
    std::vector<Novatel_Message_BESTPOS> vMessage_BESTPOS;
    std::vector<Novatel_Message_AVEPOS> vMessage_AVEPOS;
    std::vector<Novatel_Message_PDPPOS> vMessage_PDPPOS;

    //星历
    std::vector<Novatel_Message_GPSEPHEM> vMessage_GPSEPHEM;
    std::map<uint32_t,std::vector<GPS_Position> >GPS_Position;
    std::vector<Novatel_Message_BDSEPHEMERIS> vMessage_BDSEPHEMERIS;
    std::map<uint32_t,std::vector<BDS_Position> >BDS_Position;

    //双频观测值
    std::vector<Novatel_Single_Epoch_OBSERVATION> Observation;

    //SPP结果
    std::vector<Receiver_Position> SPP_Result;

//    std::map<uint32_t,std::vector<Novatel_Message_BDSEPHEMERIS> > BDS_EPHEM_DATA;
//    std::map<uint32_t,std::vector<Novatel_Message_GPSEPHEM> > GPS_EPHEM_DATA;

};

bool Novatel_Binary_Header_SyncSearch(std::ifstream& file);
bool Novatel_Binary_MessageLengthCheck(std::ifstream& file,unsigned short length);

//sort函数
bool GPS_Ephem_Compare(const Novatel_Message_GPSEPHEM& a, const Novatel_Message_GPSEPHEM& b);
bool BDS_Ephem_Compare(const Novatel_Message_BDSEPHEMERIS& a, const Novatel_Message_BDSEPHEMERIS& b);
bool GPS_Position_Compare(const GPS_Position& a, const GPS_Position& b);
bool BDS_Position_Compare(const BDS_Position& a, const BDS_Position& b);

//广播星历解算
void GPS_PVT(const Novatel_Message_GPSEPHEM &GPSEPHEM, Novatel_Data &Novatel_Data);
GPS_Position GPS_PVT(const Novatel_Message_GPSEPHEM &GPSEPHEM, Novatel_Data &Novatel_Data, GPST Positioning_Time, double Psr);
GPS_Position GPS_PVT(const Novatel_Message_GPSEPHEM &GPSEPHEM, Novatel_Data &Novatel_Data, GPST Positioning_Time, double Psr,double Rou);
void BDS_PVT(const Novatel_Message_BDSEPHEMERIS& BDSEPHEM,Novatel_Data& Novatel_Data);
BDS_Position BDS_PVT(const Novatel_Message_BDSEPHEMERIS &BDSEPHEM, Novatel_Data &Novatel_Data, BDT Positioning_Time, double Psr);


void Export_Position2CSV(const std::map<uint32_t, std::vector<GPS_Position>>& data, const std::string& filename);


//单点定位
double Hopfield(double Elevation_Angle,double Height);
void Single_Point_Pusitioning(Novatel_Data& Novatel_Data,
                              Novatel_Single_Epoch_OBSERVATION& Single_Epoch_Obs);




#endif //POS_NAV_PROJECT_DATA_NOVATEL_H
