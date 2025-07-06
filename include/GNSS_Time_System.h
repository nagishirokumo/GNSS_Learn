#ifndef  _GNSS_TIME_SYSTEM_H_ //防止头文件被重复包含
#define  _GNSS_TIME_SYSTEM_H_

#include <ctime>
#include <cmath>
#include <iostream>

class UTC;
class MJD;

class MJD// 简化的儒略日
{
    friend UTC MJD2UTC();
    friend void output();
private:
    unsigned long mjd;
public:
    MJD(unsigned long mjd_)
    :mjd(mjd_)
    {}
    UTC MJD2UTC();
    void output() const;
};

class UTC//协调世界时
{
public:
    unsigned short year;
    char month;
    char day;
    char hour;
    char min;
    double sec;
    MJD UTC2MJD();
    UTC(unsigned short year_,char month_,char day_,char hour_,
        char min_,double sec_)
            :year(year_)
            ,month(month_)
            ,day(day_)
            ,hour(hour_)
            ,min(min_)
            ,sec(sec_)
    {}
    UTC(){}
    void output();

    bool operator< (const UTC& other) const
    {
      if(year!=other.year)
            return year<other.year;
        if(month!=other.month)
            return month<other.month;
        if(day!=other.day)
            return day<other.day;
        if(hour!=other.hour)
            return hour<other.hour;
        if(min!=other.min)
            return min<other.min;
        return sec<other.sec;
    }

    bool operator> (const UTC& other) const
    {
        if(year!=other.year)
            return year>other.year;
        if(month!=other.month)
            return month>other.month;
        if(day!=other.day)
            return day>other.day;
        if(hour!=other.hour)
            return hour>other.hour;
        if(min!=other.min)
            return min>other.min;
        return sec>other.sec;
    }

};

class TAI//国际原子时
{
private:
    int year;
    int month;
    int day;
    int hour;
    int min;
    int sec_int;
    double sec_frac;
public:
    TAI(int year_,int month_,int day_,int hour_,
        int min_,int sec_int_,double sec_frac_)
            :year(year_)
            ,month(month_)
            ,day(day_)
            ,hour(hour_)
            ,min(min_)
            ,sec_int(sec_int_)
            ,sec_frac(sec_frac_)
    {}
};

class GPST//GPS时间
{
public:
    int GPS_Week;
    double GPS_Second;
public:
    GPST(int GPS_Week_,double GPS_Second_)
            :GPS_Week(GPS_Week_)
            ,GPS_Second(GPS_Second_)
    {}
    GPST(unsigned short year_,char month_,char day_,char hour_,
         char min_,double sec_);

    GPST()
    {
        GPS_Week=0;
        GPS_Second=0;
    }

    // 减法
    double operator-(const GPST& other) const {
        return 604800*(GPS_Week-other.GPS_Week)+GPS_Second-other.GPS_Second;
    }

};

class BDT//BEIDOU时间
{
public:
    int BDS_Week;
    double BDS_Second;
public:
    BDT(int BDS_Week_,double BDS_Second_)
            :BDS_Week(BDS_Week_)
            ,BDS_Second(BDS_Second_)
    {}
    BDT()
    {
        BDS_Week=0;
        BDS_Second=0;
    }

    // 减法
    double operator-(const BDT& other) const {
        return 604800*(BDS_Week-other.BDS_Week)+BDS_Second-other.BDS_Second;
    }

};


GPST BDT2GPST(BDT BDTime);
BDT GPST2BDT(GPST GPSTime);


#endif