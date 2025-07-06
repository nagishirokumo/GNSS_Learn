//
// Created by asus on 2025/4/17.
//
#include "GNSS_Time_System.h"

void MJD::output() const
{
    std::wcout<<this->mjd<<std::endl;
}

UTC MJD::MJD2UTC()
{
    unsigned long jd = this->mjd + 2400000.5;

    int Z = static_cast<int>(jd + 0.5);
    double F = jd + 0.5 - Z;

    int A = Z;
    if (Z >= 2299161) {
        int alpha = (Z - 1867216.25) / 36524.25;
        A += 1 + alpha - alpha / 4;
    }

    int B = A + 1524;
    int C = (B - 122.1) / 365.25;
    int D = 365.25 * C;
    int E = (B - D) / 30.6001;

    double day = B - D - static_cast<int>(30.6001 * E) + F;
    int day_int = static_cast<int>(day);

    int month = (E < 14) ? (E - 1) : (E - 13);
    int year = (month > 2) ? (C - 4716) : (C - 4715);

    double day_frac = day - day_int;
    double total_seconds = day_frac * 86400.0;

    int hour = total_seconds / 3600;
    int min = (total_seconds - hour * 3600) / 60;
    double sec = total_seconds - hour * 3600 - min * 60;

    UTC temp(year,month,day_int,hour,min,sec);
    temp.output();
    return temp;
}

void UTC::output()
{
    printf("%d/%d/%d/%d/%d/%d.%lf",year,month,day,hour,min,sec);
}

GPST::GPST(unsigned short year_, char month_, char day_, char hour_,
     char min_, double sec_)
{
    std::tm utc_tm = {};
    utc_tm.tm_year = year_ - 1900;  // 年份从1900开始
    utc_tm.tm_mon = month_ - 1;     // 月份从0开始
    utc_tm.tm_mday = day_;
    utc_tm.tm_hour = hour_;
    utc_tm.tm_min = min_;
    utc_tm.tm_sec = static_cast<int>(sec_);  // 小数部分暂不算

    std::time_t t = std::mktime(&utc_tm);
    if (t == -1) {
        GPS_Week = 0;
        GPS_Second = 0.0;
        return;  // 转换失败
    }

    // Step 3: 计算 GPS 时间起点（1980-01-06 00:00:00 UTC）
    std::tm gps_epoch = {};
    gps_epoch.tm_year = 1980 - 1900;
    gps_epoch.tm_mon = 0;
    gps_epoch.tm_mday = 6;
    gps_epoch.tm_hour = 0;
    gps_epoch.tm_min = 0;
    gps_epoch.tm_sec = 0;

    std::time_t gps_epoch_time = std::mktime(&gps_epoch);
    if (gps_epoch_time == -1) {
        GPS_Week = 0;
        GPS_Second = 0.0;
        return;  // 转换失败
    }

    // Step 4: 计算时间差（单位：秒）
    double total_seconds = std::difftime(t, gps_epoch_time) + (sec_ - static_cast<int>(sec_));

    // Step 5: 转为 GPS 周和周内秒
    GPS_Week = static_cast<int>(total_seconds / 604800);  // 每周 604800 秒
    GPS_Second = std::fmod(total_seconds, 604800.0);
    if (GPS_Second < 0) GPS_Second += 604800.0;
}


GPST BDT2GPST(BDT BDTime)
{
    GPST GPSTime;
    GPSTime.GPS_Week=BDTime.BDS_Week+1356;
    GPSTime.GPS_Second=BDTime.BDS_Second+14;
    if (GPSTime.GPS_Second>=604800)
    {
        GPSTime.GPS_Second-=604800;
        GPSTime.GPS_Week+=1;
    }
    return GPSTime;
}

BDT GPST2BDT(GPST GPSTime)
{
    BDT BDTime;
    BDTime.BDS_Week=GPSTime.GPS_Week-1356;
    BDTime.BDS_Second=GPSTime.GPS_Second-14;
    if (GPSTime.GPS_Second<=0)
    {
        GPSTime.GPS_Second+=604800;
        GPSTime.GPS_Week-=1;
    }
    return BDTime;
}


